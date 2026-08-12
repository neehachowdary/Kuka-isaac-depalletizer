from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time
import omni.usd
from pxr import UsdLux, UsdGeom, Gf
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

set_camera_view(eye=[3, 0, 3], target=[1.3, 0, 0.5], camera_prim_path="/OmniverseKit_Persp")

robot_art = SingleArticulation("/kr50_r2500")
robot_art.initialize()

num_dofs = robot_art.num_dof
stiffness = np.array([1e6] * num_dofs)
damping = np.array([1e4] * num_dofs)
robot_art.get_articulation_controller().set_gains(kps=stiffness, kds=damping)
print("Drive gains set")

controller = robot_art.get_articulation_controller()

positions = np.load("C:/Users/NehaaChowdary/warehouse/depalletize_trajectory.npy")
segment_info = np.load("C:/Users/NehaaChowdary/warehouse/depalletize_segments.npy", allow_pickle=True)
print(f"Loaded {len(positions)} waypoints across {len(segment_info)} segments")

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

print("Grasp at waypoints:", grasp_indices, "for boxes:", box_order)
print("Release at waypoints:", release_indices)

ee_link_path = "/kr50_r2500/Geometry/world/base_link/link_1/link_2/link_3/link_4/link_5/link_6/tool0"
conveyor_place_ys = [0.3, 0.55, 0.8, 1.05]
conveyor_x = 1.3
conveyor_top_z = 0.06 + 0.08

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

        for _ in range(100):
            world.step(render=True)

        ee_prim = stage.GetPrimAtPath(ee_link_path)
        ee_xform = UsdGeom.Xformable(ee_prim)
        ee_pos = ee_xform.ComputeLocalToWorldTransform(0).ExtractTranslation()

        box_xform = UsdGeom.Xformable(held_box_prim)
        box_pos = box_xform.ComputeLocalToWorldTransform(0).ExtractTranslation()

        grip_offset = Gf.Vec3d(box_pos[0] - ee_pos[0], box_pos[1] - ee_pos[1], box_pos[2] - ee_pos[2])
        print(f">>> GRASPING box_{held_box_idx}")
        print(f"    RAW ee_pos={ee_pos}")
        print(f"    RAW box_pos={box_pos}")
        print(f"    offset={grip_offset}")

    if i in release_indices:
        box_num = release_indices.index(i)
        final_pos = Gf.Vec3d(conveyor_x, conveyor_place_ys[box_num], conveyor_top_z)
        box_xform = UsdGeom.Xformable(held_box_prim)
        ops = box_xform.GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(final_pos)
        print(f">>> RELEASING box_{held_box_idx} at exact position {final_pos}")
        held_box_prim = None
        held_box_idx = None
        grip_offset = None

    if held_box_prim is not None and grip_offset is not None:
        ee_prim = stage.GetPrimAtPath(ee_link_path)
        if ee_prim.IsValid():
            ee_xform = UsdGeom.Xformable(ee_prim)
            ee_pos = ee_xform.ComputeLocalToWorldTransform(0).ExtractTranslation()

            box_xform = UsdGeom.Xformable(held_box_prim)
            ops = box_xform.GetOrderedXformOps()
            for op in ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    op.Set(Gf.Vec3d(ee_pos[0] + grip_offset[0], ee_pos[1] + grip_offset[1], ee_pos[2] + grip_offset[2]))

    time.sleep(0.02 if held_box_prim is None else 0.06)

print("Depalletization complete! Window stays open - close manually.")

while simulation_app.is_running():
    world.step(render=True)
