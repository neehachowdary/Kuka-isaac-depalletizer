from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, Gf, UsdPhysics, UsdShade, Sdf, UsdLux
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

def make_cylinder(path, position, radius, height, color):
    old = stage.GetPrimAtPath(path)
    if old.IsValid():
        stage.RemovePrim(path)
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    xform = UsdGeom.Xformable(cyl)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    material = UsdShade.Material.Define(stage, path + "_mat")
    shader = UsdShade.Shader.Define(stage, path + "_mat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(material)
    return cyl.GetPrim()

# --- Lighting ---
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1500)
distant_light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
distant_light.CreateIntensityAttr(3000)

CONCRETE = (0.85, 0.85, 0.85)
BLACK_BELT = (0.05, 0.05, 0.05)
YELLOW = (0.9, 0.75, 0.1)
GREEN = (0.3, 0.8, 0.4)
STEEL = (0.4, 0.4, 0.45)
ORANGE = (0.9, 0.5, 0.1)

# --- Floor ---
make_box("/World/floor", (1.3, 0.0, -0.01), (4.0, 4.0, 0.01), CONCRETE)

# --- 3 vertical conveyors (matching diagram: left, center, right) ---
make_box("/World/conveyor_center", (1.3, -0.8, 0.03), (0.25, 1.8, 0.03), BLACK_BELT)
make_box("/World/conveyor_left", (0.0, -0.8, 0.03), (0.25, 1.8, 0.03), BLACK_BELT)
make_box("/World/conveyor_right", (2.6, -0.8, 0.03), (0.25, 1.8, 0.03), BLACK_BELT)

# --- Dropzone 1 (Cages) - left of center conveyor ---
make_box("/World/dropzone1_marker", (0.65, -0.4, 0.005), (0.3, 0.6, 0.005), GREEN)
make_box("/World/cage_1", (0.65, -0.4, 0.15), (0.15, 0.15, 0.15), YELLOW)

# --- Dropzone 2 (Pallets) - right of center conveyor ---
make_box("/World/dropzone2_marker", (1.95, -0.4, 0.005), (0.3, 0.6, 0.005), GREEN)
make_box("/World/pallet_1", (1.95, -0.4, 0.05), (0.35, 0.3, 0.05), (0.5, 0.35, 0.15))

# --- Overhead camera gantry ---
pole_positions = [(-0.3, -1.8, 1.2), (-0.3, 0.3, 1.2), (2.9, -1.8, 1.2), (2.9, 0.3, 1.2)]
for i, pos in enumerate(pole_positions):
    make_cylinder(f"/World/camera_pole_{i}", pos, 0.03, 2.4, STEEL)
make_box("/World/gantry_beam_front", (1.3, -1.8, 2.4), (1.7, 0.03, 0.03), STEEL)
make_box("/World/gantry_beam_back", (1.3, 0.3, 2.4), (1.7, 0.03, 0.03), STEEL)
make_box("/World/overhead_camera", (1.3, -0.8, 2.3), (0.06, 0.06, 0.06), (0.1, 0.1, 0.1))

# --- Forklift placeholder ---
make_box("/World/forklift", (0.65, -2.3, 0.3), (0.3, 0.5, 0.3), ORANGE)

# --- Inbound loading zone (bottom of scene) ---
make_box("/World/inbound_zone", (1.3, -2.8, 0.005), (1.5, 0.2, 0.005), STEEL)

omni.usd.get_context().save_stage()
print("Full warehouse scene rebuilt from scratch: 3 conveyors, dropzones, cages, pallets, gantry, forklift, inbound zone")
simulation_app.close()










