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

cylinder = UsdGeom.Cylinder.Define(stage, "/World/suction_cup")
cylinder.CreateRadiusAttr(0.03)
cylinder.CreateHeightAttr(0.03)
xform = UsdGeom.Xformable(cylinder)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))

material = UsdShade.Material.Define(stage, "/World/suction_cup_mat")
shader = UsdShade.Shader.Define(stage, "/World/suction_cup_mat/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.1, 0.1, 0.1))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(cylinder.GetPrim()).Bind(material)

omni.usd.get_context().save_stage()
print("Suction cup added")
simulation_app.close()