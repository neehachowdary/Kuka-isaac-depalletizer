from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time
import omni.usd
from pxr import UsdLux, UsdGeom, Gf, UsdShade, Sdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.viewports import set_camera_view

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")
world = World()
world.reset()
stage = omni.usd.get_context().get_stage()

dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1000)
set_camera_view(eye=[1.3, -3, 1.0], target=[1.3, -0.3, 0.5], camera_prim_path="/OmniverseKit_Persp")

robot_art = SingleArticulation("/kr50_r2500")
robot_art.initialize()
num_dofs = robot_art.num_dof
robot_art.get_articulation_controller().set_gains(kps=np.array([1e6]*num_dofs), kds=np.array([1e4]*num_dofs))
controller = robot_art.get_articulation_controller()

# Collision sphere definitions - copied directly from kr50_r2500.yml
collision_spheres = {
    "base_link": [([0.0, 0.0, 0.1], 0.15)],
    "link_1": [([0.0, 0.0, 0.0], 0.15)],
    "link_2": [([0.0, 0.0, 0.3], 0.13), ([0.0, 0.0, 0.6], 0.13)],
    "link_3": [([0.0, 0.0, 0.0], 0.12)],
    "link_4": [([0.0, 0.0, 0.2], 0.1)],
    "link_5": [([0.0, 0.0, 0.0], 0.09)],
    "link_6": [([0.0, 0.0, 0.0], 0.08)],
    "tool0": [([0.0, 0.0, 0.0], 0.06)],
}

link_prefix = "/kr50_r2500/Geometry/world/base_link"
link_paths = {
    "base_link": link_prefix,
    "link_1": link_prefix + "/link_1",
    "link_2": link_prefix + "/link_1/link_2",
    "link_3": link_prefix + "/link_1/link_2/link_3",
    "link_4": link_prefix + "/link_1/link_2/link_3/link_4",
    "link_5": link_prefix + "/link_1/link_2/link_3/link_4/link_5",
    "link_6": link_prefix + "/link_1/link_2/link_3/link_4/link_5/link_6",
    "tool0": link_prefix + "/link_1/link_2/link_3/link_4/link_5/link_6/tool0",
}

# Create visible debug spheres, one per collision sphere
sphere_prims = {}
sphere_id = 0
for link_name, spheres in collision_spheres.items():
    for i, (center, radius) in enumerate(spheres):
        path = f"/World/debug_sphere_{sphere_id}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.CreateRadiusAttr(radius)
        xform = UsdGeom.Xformable(sphere)
        xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        material = UsdShade.Material.Define(stage, path + "_mat")
        shader = UsdShade.Shader.Define(stage, path + "_mat/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.2, 1.0, 0.2))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.4)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(sphere.GetPrim()).Bind(material)
        sphere_prims[sphere_id] = (link_name, center, path)
        sphere_id += 1

print(f"Created {len(sphere_prims)} debug collision spheres")

positions = np.load("depalletize_trajectory.npy")
segment_info = np.load("depalletize_segments.npy", allow_pickle=True)

idx = 0
grasp_indices = []
release_indices = []
box_order = []
for box_idx, stage_name, length in segment_info:
    if stage_name == "approach_and_grasp":
        grasp_indices.append(idx + length - 1)
        box_order.append(box_idx)
    if stage_name == "descend_place":
        release_indices.append(idx + length - 1)
    idx += length

box_half_size = 0.08
held_box_prim = None
held_box_idx = None
grip_offset = None

for i, waypoint in enumerate(positions):
    action = ArticulationAction(joint_positions=waypoint)
    controller.apply_action(action)
    world.step(render=True)

    if i in grasp_indices:
        box_num = grasp_indices.index(i)
        held_box_idx = box_order[box_num]
        held_box_prim = stage.GetPrimAtPath(f"/World/box_{held_box_idx}")
        grip_offset = Gf.Vec3d(0.0, 0.0, -box_half_size)

    if i in release_indices:
        held_box_prim = None
        held_box_idx = None
        grip_offset = None

    if held_box_prim is not None and grip_offset is not None:
        ee_prim = stage.GetPrimAtPath(link_paths["tool0"])
        if ee_prim.IsValid():
            ee_pos = UsdGeom.Xformable(ee_prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
            ops = UsdGeom.Xformable(held_box_prim).GetOrderedXformOps()
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(ee_pos[0] + grip_offset[0], ee_pos[1] + grip_offset[1], ee_pos[2] + grip_offset[2]))

    # Update every debug sphere's world position based on its link's real current transform
    for sid, (link_name, center, path) in sphere_prims.items():
        link_prim = stage.GetPrimAtPath(link_paths[link_name])
        if link_prim.IsValid():
            link_pos = UsdGeom.Xformable(link_prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
            sphere_prim = stage.GetPrimAtPath(path)
            ops = UsdGeom.Xformable(sphere_prim).GetOrderedXformOps()
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(link_pos[0] + center[0], link_pos[1] + center[1], link_pos[2] + center[2]))

    time.sleep(0.02 if held_box_prim is None else 0.06)

print("Playback with visible collision spheres complete! Window stays open.")
while simulation_app.is_running():
    world.step(render=True)
