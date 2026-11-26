"""Embedded DAVID camera receiver. No external runtime files or ROS context."""
import json
import socket
import struct
import threading
import time
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, Vt

BIND_IP = '169.254.89.190'
DAVID_IP = '169.254.40.222'
PORTS = {'left': 5510, 'right': 5511}
DOWNSAMPLE = 10
UPDATE_INTERVAL_S = 1.0 / 30.0
MIN_DEPTH_M = 0.15
MAX_DEPTH_M = 4.0
ROLL_180 = True  # Initial correction for the reported upside-down reconstruction.
CAMERA_HINT = 'lens_cover_awg_rgb'
MESH_ROOT = '/World/DAVID_Ethernet_'


def _read(sock, size):
    data = bytearray(size)
    view = memoryview(data)
    offset = 0
    while offset < size:
        n = sock.recv_into(view[offset:])
        if not n:
            raise ConnectionError('sender disconnected')
        offset += n
    return data


def _receive(s, side, port):
    while not s['stop'].is_set():
        try:
            with socket.socket() as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((BIND_IP, port))
                server.listen(1)
                server.settimeout(0.5)
                print(f'[DAVID RGBD] {side}: listening {BIND_IP}:{port}')
                while not s['stop'].is_set():
                    try:
                        conn, addr = server.accept()
                    except socket.timeout:
                        continue
                    with conn:
                        if addr[0] != DAVID_IP:
                            continue
                        conn.settimeout(1.0)
                        print(f'[DAVID RGBD] {side}: connected')
                        try:
                            while not s['stop'].is_set():
                                magic, nm, nr, nd = struct.unpack('!4sIII', _read(conn, 16))
                                if magic != b'DRB2' or not (0 < nm < 100000 and 0 < nr < 16000000 and 0 < nd < 16000000):
                                    raise ValueError('invalid packet')
                                meta = json.loads(_read(conn, nm))
                                rgb = _read(conn, nr)
                                depth = _read(conn, nd)
                                if meta.get('side') != side:
                                    continue
                                with s['lock']:
                                    s['count'][side] += 1
                                    s['latest'][side] = (s['count'][side], time.monotonic(), meta, rgb, depth)
                        except (OSError, ValueError, ConnectionError) as exc:
                            if not s['stop'].is_set():
                                print(f'[DAVID RGBD] {side}: {exc}; waiting for sender')
                        finally:
                            with s['lock']:
                                s['latest'].pop(side, None)
                                s['geometry'].pop(side, None)
        except OSError as exc:
            if not s['stop'].is_set():
                print(f'[DAVID RGBD] {side}: bind error: {exc}')
                s['stop'].wait(2.0)


def _geometry(meta, rgb_bytes, depth_bytes):
    r, d = meta['rgb'], meta['depth']
    w, h = int(d['width']), int(d['height'])
    if (int(r['width']), int(r['height'])) != (w, h):
        raise ValueError('unaligned RGB/depth')
    enc = r['encoding'].lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
    if channels is None or d['encoding'].lower() not in ('16uc1', 'mono16'):
        raise ValueError('unsupported encoding')
    rgb = np.frombuffer(rgb_bytes, np.uint8).reshape(h, int(r['step']))[:, :w*channels]
    rgb = rgb.reshape(h, w, channels)[::DOWNSAMPLE, ::DOWNSAMPLE, :3]
    if enc.startswith('bgr'):
        rgb = rgb[:, :, ::-1]
    depth = np.ndarray((h, w), dtype='>u2' if d['bigendian'] else '<u2', buffer=depth_bytes, strides=(int(d['step']), 2))
    z = depth[::DOWNSAMPLE, ::DOWNSAMPLE].astype(np.float32) * float(d.get('unit_m', .001))
    k = meta['camera_info']['k']
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError('invalid intrinsics')
    hh, ww = z.shape
    u = np.arange(ww, dtype=np.float32)[None, :] * DOWNSAMPLE
    v = np.arange(hh, dtype=np.float32)[:, None] * DOWNSAMPLE
    x, y = (u-cx)*z/fx, (v-cy)*z/fy
    if ROLL_180:
        x, y = -x, -y
    points = np.ascontiguousarray(np.stack(np.broadcast_arrays(x, y, z), axis=-1).reshape(-1, 3), dtype=np.float32)
    valid = np.isfinite(z) & (z >= MIN_DEPTH_M) & (z <= MAX_DEPTH_M)
    ids = np.arange(hh*ww, dtype=np.int32).reshape(hh, ww)
    a, b, c, d = ids[:-1,:-1], ids[:-1,1:], ids[1:,1:], ids[1:,:-1]
    good = valid.reshape(-1)
    mask = good[a] & good[b] & good[c] & good[d]
    jumps = np.maximum.reduce([np.abs(z[:-1,:-1]-z[:-1,1:]), np.abs(z[:-1,:-1]-z[1:,:-1]), np.abs(z[1:,1:]-z[:-1,1:]), np.abs(z[1:,1:]-z[1:,:-1])])
    mask &= jumps < .08
    faces = np.ascontiguousarray(np.stack([a[mask], b[mask], c[mask], a[mask], c[mask], d[mask]], axis=1).reshape(-1), dtype=np.int32)
    counts = np.full(len(faces)//3, 3, dtype=np.int32)
    colors = np.ascontiguousarray(rgb.reshape(-1, 3), dtype=np.float32) / 255.0
    return points, faces, counts, colors


def _prepare(s):
    done = {}
    while not s['stop'].is_set():
        with s['lock']:
            packets = dict(s['latest'])
        for side, (seq, received, meta, rgb, depth) in packets.items():
            if done.get(side) == seq:
                continue
            done[side] = seq
            try:
                geom = _geometry(meta, rgb, depth)
                with s['lock']:
                    if side in s['latest']:
                        s['geometry'][side] = (seq, received, geom)
            except Exception as exc:
                print(f'[DAVID RGBD] {side}: frame error: {exc}')
        s['stop'].wait(UPDATE_INTERVAL_S)


def _camera(stage, side):
    for prim in stage.Traverse():
        path = str(prim.GetPath()).lower()
        if CAMERA_HINT in path and side + '_mount' in path:
            return prim
    return None


def _show(stage, s, side, geometry):
    points, faces, counts, colors = geometry
    cam = s['cams'].get(side)
    if cam is None or not cam.IsValid():
        cam = _camera(stage, side)
        if cam is None:
            return False
        s['cams'][side] = cam
        print(f'[DAVID RGBD] {side}: camera {cam.GetPath()}')
    matrix = UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    m = np.asarray(matrix, dtype=np.float64)
    world = np.ascontiguousarray((points / UsdGeom.GetStageMetersPerUnit(stage)) @ m[:3,:3] + m[3,:3], dtype=np.float32)
    path = MESH_ROOT + side + '_mesh'
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreateSubdivisionSchemeAttr().Set('none')
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(world))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(counts))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(faces))
    pv = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec3fArray.FromNumpy(colors))
    mesh.MakeVisible()
    return True


def _hide(stage, side):
    prim = stage.GetPrimAtPath(MESH_ROOT + side + '_mesh')
    if prim.IsValid():
        UsdGeom.Imageable(prim).MakeInvisible()


def setup(db):
    old = getattr(db.per_instance_state, 'rgbd', None)
    if old is not None and not old['stop'].is_set():
        return
    s = {'stop': threading.Event(), 'lock': threading.Lock(), 'latest': {}, 'geometry': {}, 'count': dict.fromkeys(PORTS, 0), 'previous': dict.fromkeys(PORTS, 0), 'cams': {}, 'shown': {}, 'threads': [], 'last_update': 0.0, 'last_log': time.monotonic()}
    db.per_instance_state.rgbd = s
    for side, port in PORTS.items():
        t = threading.Thread(target=_receive, args=(s, side, port), daemon=True)
        s['threads'].append(t)
        t.start()
    t = threading.Thread(target=_prepare, args=(s,), daemon=True)
    s['threads'].append(t)
    t.start()
    print('[DAVID RGBD] Optimized inline receiver started')


def compute(db):
    s = getattr(db.per_instance_state, 'rgbd', None)
    if s is None or s['stop'].is_set():
        setup(db)
        s = db.per_instance_state.rgbd
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return True
    now = time.monotonic()
    if now - s['last_update'] < UPDATE_INTERVAL_S:
        return True
    s['last_update'] = now
    with s['lock']:
        geometry, count = dict(s['geometry']), dict(s['count'])
    # Author only transient reconstruction geometry to the session layer.
    # Never change the saved robot or camera graph's layer.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for side in PORTS:
            packet = geometry.get(side)
            if packet is None or now - packet[1] > 1.0:
                if s['shown'].get(side) != -1:
                    _hide(stage, side)
                    s['shown'][side] = -1
                continue
            seq, received, geom = packet
            if s['shown'].get(side) == seq:
                continue
            try:
                if _show(stage, s, side, geom):
                    s['shown'][side] = seq
                else:
                    _hide(stage, side)
            except Exception as exc:
                if now - s['last_log'] >= 5:
                    print(f'[DAVID RGBD] {side}: render error: {exc}')
    elapsed = now - s['last_log']
    if elapsed >= 5.0:
        for side in PORTS:
            print(f'[DAVID RGBD] {side}: RX {(count[side]-s["previous"][side])/elapsed:.1f} FPS; camera found={side in s["cams"]}')
        s['previous'], s['last_log'] = count, now
    return True


def cleanup(db):
    s = getattr(db.per_instance_state, 'rgbd', None)
    if s is not None:
        s['stop'].set()
        for t in s['threads']:
            t.join(timeout=1.5)
        db.per_instance_state.rgbd = None
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for side in PORTS:
                _hide(stage, side)
    print('[DAVID RGBD] Receiver stopped')
