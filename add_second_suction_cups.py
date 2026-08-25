from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, Gf, UsdShade, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")
world = World()
world.reset()
stage = omni.usd.get_context().get_stage()

for path in ["/World/suction_cup_1", "/World/suction_cup_2"]:
    old = stage.GetPrimAtPath(path)
    if old.IsValid():
        stage.RemovePrim(path)

def make_cup(path):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(0.03)
    cyl.CreateHeightAttr(0.03)
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    mat = UsdShade.Material.Define(stage, path + "_mat")
    sh = UsdShade.Shader.Define(stage, path + "_mat/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.1, 0.1, 0.1))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(mat)

make_cup("/World/suction_cup_1")
make_cup("/World/suction_cup_2")

omni.usd.get_context().save_stage()
print("Two suction cups added")
simulation_app.close()