from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics, UsdShade, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")
world = World()
world.reset()

stage = omni.usd.get_context().get_stage()

# Remove any existing obstacle first, so this script is safe to rerun
for path in ["/World/static_obstacle", "/World/static_obstacle_2"]:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)

cube = UsdGeom.Cube.Define(stage, "/World/static_obstacle")
cube.CreateSizeAttr(1.0)
xform = UsdGeom.Xformable(cube)
xform.AddTranslateOp().Set(Gf.Vec3d(1.3, -0.3, 0.15))
xform.AddScaleOp().Set(Gf.Vec3f(0.4, 0.05, 0.3))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

material = UsdShade.Material.Define(stage, "/World/static_obstacle_mat")
shader = UsdShade.Shader.Define(stage, "/World/static_obstacle_mat/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.1, 0.1))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

omni.usd.get_context().save_stage()
print("Static obstacle added (red wall)")
simulation_app.close()