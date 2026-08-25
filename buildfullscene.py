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

def make_box(path, position, scale, color):
    old = stage.GetPrimAtPath(path)
    if old.IsValid():
        stage.RemovePrim(path)
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    material = UsdShade.Material.Define(stage, path + "_mat")
    shader = UsdShade.Shader.Define(stage, path + "_mat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)
    return cube.GetPrim()

make_box("/World/conveyor", (1.3, 0.7, 0.03), (0.4, 1.8, 0.03), (0.9, 0.8, 0.1))

box_half_size = 0.08
for i in range(4):
    z = box_half_size + i * (2 * box_half_size)
    box_prim = make_box(f"/World/box_{i}", (1.3, -0.8, z), (box_half_size, box_half_size, box_half_size), (0.15, 0.3, 0.7))
    UsdPhysics.RigidBodyAPI.Apply(box_prim)
    UsdPhysics.RigidBodyAPI(box_prim).CreateKinematicEnabledAttr().Set(True)

omni.usd.get_context().save_stage()
print("Scene rebuilt: conveyor + 4 boxes, saved")
simulation_app.close()