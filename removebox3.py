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

cube = UsdGeom.Cube.Define(stage, "/World/box_3")
cube.CreateSizeAttr(1.0)
xform = UsdGeom.Xformable(cube)
xform.AddTranslateOp().Set(Gf.Vec3d(1.3, -0.8, 0.5599999999999999))
xform.AddScaleOp().Set(Gf.Vec3f(0.08, 0.08, 0.08))
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.RigidBodyAPI(cube.GetPrim()).CreateKinematicEnabledAttr().Set(True)

material = UsdShade.Material.Define(stage, "/World/box_3_mat_blue")
shader = UsdShade.Shader.Define(stage, "/World/box_3_mat_blue/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.15, 0.3, 0.7))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

omni.usd.get_context().save_stage()
print("box_3 recreated and saved")
simulation_app.close()