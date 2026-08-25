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
robot_art.get_articulation_controller().set_gains(kps=np.array([1e6]*num_dofs), kds=np.array([1e4]*num_dofs))
controller = robot_art.get_articulation_controller()

positions = np.load("C:/Users/NehaaChowdary/warehouse/dual_trajectory.npy")
segment_info = np.load("C:/Users/NehaaChowdary/warehouse/dual_segments.npy", allow_pickle=True)
pair_gaps = np.load("C:/Users/NehaaChowdary/warehouse/dual_pair_gaps.npy")
print(f"Loaded {len(positions)} waypoints, pair_gaps={pair_gaps}")

pairs = [(0, 1), (2, 3)]
grasp_idx_map = {}
release_idx_map = {}
idx = 0
for tag, stage_name, length in segment_info:
    if stage_name == "approach_and_grasp":
        grasp_idx_map[tuple(tag)] = idx + length - 1
    for box_idx in (tag if isinstance(tag, tuple) else []):
        if stage_name == f"descend_place_{box_idx}":
            release_idx_map[int(box_idx)] = idx + length - 1
    idx += length

print(f"Grasp indices: {grasp_idx_map}")
print(f"Release indices: {release_idx_map}")

ee_link_path = "/kr50_r2500/Geometry/world/base_link/link_1/link_2/link_3/link_4/link_5/link_6/tool0"
box_prims = {i: stage.GetPrimAtPath(f"/World/box_{i}") for i in range(4)}
holding = {i: False for i in range(4)}
box_to_pair = {}
box_to_side = {}
for p_num, (i0, i1) in enumerate(pairs):
    box_to_pair[i0] = p_num
    box_to_pair[i1] = p_num
    box_to_side[i0] = -1
    box_to_side[i1] = 1

grip_offset_z = -0.08

for i, waypoint in enumerate(positions):
    action = ArticulationAction(joint_positions=waypoint)
    controller.apply_action(action)
    world.step(render=True)

    for pair, g_idx in grasp_idx_map.items():
        if i == g_idx:
            for bi in pair:
                holding[bi] = True
            print(f">>> GRASPING pair {pair}")

    for bi, r_idx in release_idx_map.items():
        if i == r_idx and holding[bi]:
            holding[bi] = False
            print(f">>> RELEASING box_{bi}")

    ee_prim = stage.GetPrimAtPath(ee_link_path)
    if ee_prim.IsValid():
        ee_pos = UsdGeom.Xformable(ee_prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
        for bi in range(4):
            if holding[bi]:
                p_num = box_to_pair[bi]
                half_gap = pair_gaps[p_num]
                i0, i1 = pairs[p_num]
                other = i1 if bi == i0 else i0
                offset = box_to_side[bi] * half_gap if holding[other] else 0.0
                ops = UsdGeom.Xformable(box_prims[bi]).GetOrderedXformOps()
                for op in ops:
                    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        op.Set(Gf.Vec3d(ee_pos[0], ee_pos[1] + float(offset), ee_pos[2] + grip_offset_z))

    time.sleep(0.02 if not any(holding.values()) else 0.06)

print("Two pairs, sequential pick-place complete! Window stays open - close manually.")
while simulation_app.is_running():
    world.step(render=True)