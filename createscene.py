import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics, UsdShade, Sdf

stage = omni.usd.get_context().get_stage()

# Remove any old objects first (clean slate)
for path in ["/World/conveyor", "/World/box_0", "/World/box_1", "/World/box_2", "/World/box_3"]:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)

def make_box(path, position, scale, color):
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

# Conveyor: flat, yellow
conveyor_half_height = 0.03
make_box("/World/conveyor", (1.3, 0.7, conveyor_half_height), (0.4, 1.8, conveyor_half_height), (0.9, 0.8, 0.1))

# Boxes: stacked, blue
box_half_size = 0.08
box_x, box_y = 1.3, -0.8
num_boxes = 4
for i in range(num_boxes):
    z = box_half_size + i * (2 * box_half_size)
    box_prim = make_box(f"/World/box_{i}", (box_x, box_y, z), (box_half_size, box_half_size, box_half_size), (0.15, 0.3, 0.7))
    UsdPhysics.RigidBodyAPI.Apply(box_prim)
    UsdPhysics.RigidBodyAPI(box_prim).CreateKinematicEnabledAttr().Set(True)

print("Scene rebuilt: conveyor + 4 stacked boxes")