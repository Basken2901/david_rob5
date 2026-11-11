import json
import socket
import time

from isaacsim import SimulationApp
app = SimulationApp({'headless': True})

import omni.usd
from pxr import Usd, UsdGeom

SCENE = '/home/bjarke/Downloads/Saves/DAVID_VR_UR5.usd'
PORT = 15151

ctx = omni.usd.get_context()
if not ctx.open_stage(SCENE):
    raise RuntimeError(f'Cannot open {SCENE}')
for _ in range(20):
    app.update()
stage = ctx.get_stage()
paths = sorted(str(p.GetPath()) for p in stage.Traverse() if p.GetName() == 'LENS_COVER_AWG_RGB')
if len(paths) != 2:
    raise RuntimeError(f'Expected 2 camera meshes, found {len(paths)}: {paths}')
print('Camera paths:', *paths, sep='\n', flush=True)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    while app.is_running():
        app.update()
        poses = []
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = matrix.ExtractTranslation()
            q = matrix.ExtractRotationQuat()
            v = q.GetImaginary()
            poses.append({'position': [float(t[i]) for i in range(3)],
                          'quaternion': [float(v[0]), float(v[1]), float(v[2]), float(q.GetReal())]})
        sock.sendto(json.dumps({'poses': poses}).encode(), ('127.0.0.1', PORT))
        time.sleep(1 / 30)
finally:
    sock.close()
    app.close()
