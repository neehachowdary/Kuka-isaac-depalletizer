import torch
import numpy as np
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.rollout.cost.pose_cost import PoseCostMetric
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util_file import load_yaml, get_robot_configs_path, join_path
from curobo.geom.types import WorldConfig, Cuboid
from pxr import Usd, UsdGeom

torch.manual_seed(42)
np.random.seed(42)

print("Loading robot config...")
tensor_args = TensorDeviceType()
config_file = load_yaml(join_path(get_robot_configs_path(), "kr50_r2500.yml"))
robot_cfg = RobotConfig.from_dict(config_file["robot_cfg"])

world_cfg = WorldConfig(
    cuboid=[Cuboid(name="dummy", pose=[10, 10, 10, 1, 0, 0, 0], dims=[0.01, 0.01, 0.01])]
)
motion_gen_config = MotionGenConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args=tensor_args, high_precision=True, use_cuda_graph=False)
motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()

retract_cfg = motion_gen.get_retract_config()
start_state = JointState.from_position(retract_cfg.view(1, -1), joint_names=motion_gen.kinematics.joint_names)

def plan_and_extend(start, goal_pose_list, all_positions, label):
    goal_pose = Pose.from_list(goal_pose_list)
    result = motion_gen.plan_single(start, goal_pose, MotionGenPlanConfig(max_attempts=15))
    if not result.success.item():
        print(f"FAILED at {label}:", result.status)
        return None, None
    pos = result.get_interpolated_plan().position.cpu().numpy()
    all_positions.append(pos)
    new_start = JointState.from_position(torch.tensor(pos[-1], device="cuda:0").view(1, -1), joint_names=motion_gen.kinematics.joint_names)
    print(f"  {label}: {len(pos)} waypoints")
    return new_start, pos

usd_stage = Usd.Stage.Open("C:/Users/NehaaChowdary/Documents/newware_house.usd")
box_positions_real = []
box_half_size = 0.08
for i in range(4):
    prim = usd_stage.GetPrimAtPath(f"/World/box_{i}")
    pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    box_positions_real.append((pos[0], pos[1], pos[2]))
    print(f"Real box_{i} position: {pos}")

conveyor_x = 1.3
conveyor_place_ys = [0.3, 0.55, 0.8, 0.9]
conveyor_top_z = 0.06
approach_height = 0.15
place_clearance = 0.08

all_waypoints = []
segment_info = []
current_state = start_state
pick_order = [3, 2, 1, 0]

for box_num, box_idx in enumerate(pick_order):
    bx, by, bz = box_positions_real[box_idx]
    px, py, pz = conveyor_x, conveyor_place_ys[box_num], conveyor_top_z

    print(f"\n--- Box {box_idx} (pick #{box_num+1}) ---")

    grasp_pose_metric = PoseCostMetric.create_grasp_approach_metric(offset_position=approach_height, tstep_fraction=0.6, linear_axis=2)
    goal_pose = Pose.from_list([bx, by, bz + box_half_size, 0.0, 1.0, 0.0, 0.0])
    result = motion_gen.plan_single(current_state, goal_pose, MotionGenPlanConfig(max_attempts=15, pose_cost_metric=grasp_pose_metric))
    if not result.success.item():
        print("FAILED at approach_and_grasp:", result.status)
    else:
        pos = result.get_interpolated_plan().position.cpu().numpy()
        all_waypoints.append(pos)
        current_state = JointState.from_position(torch.tensor(pos[-1], device="cuda:0").view(1, -1), joint_names=motion_gen.kinematics.joint_names)
        segment_info.append((box_idx, "approach_and_grasp", len(pos)))
        print(f"  approach_and_grasp: {len(pos)} waypoints")

    current_state, seg = plan_and_extend(current_state, [bx, by, bz + approach_height, 0.0, 1.0, 0.0, 0.0], all_waypoints, "lift")
    segment_info.append((box_idx, "lift", len(seg) if seg is not None else 0))

    current_state, seg = plan_and_extend(current_state, [px, py, pz + approach_height, 0.0, 1.0, 0.0, 0.0], all_waypoints, "move_to_conveyor")
    segment_info.append((box_idx, "move_to_conveyor", len(seg) if seg is not None else 0))

    current_state, seg = plan_and_extend(current_state, [px, py, pz + place_clearance, 0.0, 1.0, 0.0, 0.0], all_waypoints, "descend_place")
    segment_info.append((box_idx, "descend_place", len(seg) if seg is not None else 0))

    current_state, seg = plan_and_extend(current_state, [px, py, pz + approach_height, 0.0, 1.0, 0.0, 0.0], all_waypoints, "retreat")
    segment_info.append((box_idx, "retreat", len(seg) if seg is not None else 0))

if all(w is not None for w in all_waypoints):
    full_trajectory = np.concatenate(all_waypoints, axis=0)
    np.save("C:/Users/NehaaChowdary/warehouse/01_basic_pick_and_place/depalletize_trajectory.npy", full_trajectory)
    np.save("C:/Users/NehaaChowdary/warehouse/01_basic_pick_and_place/depalletize_segments.npy", np.array(segment_info, dtype=object), allow_pickle=True)
    print(f"\nSaved: {full_trajectory.shape[0]} total waypoints")
else:
    print("\nOne or more stages FAILED - check output above")