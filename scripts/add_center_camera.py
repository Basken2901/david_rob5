from pathlib import Path
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf

home = Path.home()

source = home / "Downloads/DAVID_RGB_Output_V2/DAVID_Textured.usda"
output = home / "Downloads/DAVID_3_Cameras.usda"

if not source.exists():
    raise SystemExit(f"Missing source scene: {source}")

original = Usd.Stage.Open(str(source))
root = original.GetDefaultPrim()

if not root or not root.IsValid():
    raise SystemExit("Source scene has no valid default prim")

stage = Usd.Stage.CreateNew(str(output))
stage.GetRootLayer().subLayerPaths.append(str(source))

root_path = str(root.GetPath())
stage.SetDefaultPrim(stage.GetPrimAtPath(root_path))

path = root_path + "/Center_Camera"

camera = UsdGeom.Xform.Define(stage, path)
camera.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.0))
camera.AddRotateXYZOp().Set(Gf.Vec3f(-90, 0, 0))

sphere = UsdGeom.Sphere.Define(stage, path + "/Marker")
sphere.CreateRadiusAttr(0.025)
sphere.CreateDisplayColorAttr([Gf.Vec3f(1, 0.8, 0)])

for name, endpoint, color in [
    ("X_Right", (0.15, 0, 0), (1, 0, 0)),
    ("Y_Down", (0, 0.15, 0), (0, 1, 0)),
    ("Z_Forward", (0, 0, 0.15), (0, 0, 1)),
]:
    axis = UsdGeom.BasisCurves.Define(stage, path + "/" + name)
    axis.CreateTypeAttr("linear")
    axis.CreateCurveVertexCountsAttr([2])
    axis.CreatePointsAttr([
        Gf.Vec3f(0, 0, 0),
        Gf.Vec3f(*endpoint)
    ])
    axis.CreateWidthsAttr([0.006])
    axis.CreateDisplayColorAttr([Gf.Vec3f(*color)])

stage.GetRootLayer().Save()
app.close()

print(output)
