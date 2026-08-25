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

tensor_args = TensorDeviceType()
config_file = load_yaml(join_path(get_robot_configs_path(), "kr50_r2500.yml"))
robot_cfg = RobotConfig.from_dict(config_file["robot_cfg"])
world_cfg = WorldConfig(cuboid=[Cuboid(name="dummy", pose=[10,10,10,1,0,0,0], dims=[0.01,0.01,0.01])])
motion_gen_config = MotionGenConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args=tensor_args, high_precision=True, use_cuda_graph=False)
motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()

retract_cfg = motion_gen.get_retract_config()
start_state = JointState.from_position(retract_cfg.view(1, -1), joint_names=motion_gen.kinematics.joint_names)

usd_stage = Usd.Stage.Open("C:/Users/NehaaChowdary/Documents/newware_house.usd")
box_positions_real = []
box_half_size = 0.08
for i in range(4):
    prim = usd_stage.GetPrimAtPath(f"/World/box_{i}")
    pos = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    box_positions_real.append((pos[0], pos[1], pos[2]))
    print(f"Real box_{i} position: {pos}")

approach_height = 0.15
place_clearance = 0.08
conveyor_top_z = 0.06
conveyor_x = 1.3

all_waypoints = []
segment_info = []
pair_half_gaps = {}
current_state = start_state

def plan_and_extend(start, goal_pose_list, label, tag):
    if start is None:
        return None
    goal_pose = Pose.from_list(goal_pose_list)
    result = motion_gen.plan_single(start, goal_pose, MotionGenPlanConfig(max_attempts=15))
    if not result.success.item():
        print(f"FAILED at {label}:", result.status)
        return None
    pos = result.get_interpolated_plan().position.cpu().numpy()
    all_waypoints.append(pos)
    new_start = JointState.from_position(torch.tensor(pos[-1], device="cuda:0").view(1, -1), joint_names=motion_gen.kinematics.joint_names)
    segment_info.append((tag, label, len(pos)))
    print(f"  {label}: {len(pos)} waypoints")
    return new_start

def plan_goalset_place(start, label, tag):
    if start is None:
        return None
    ys_candidates = np.linspace(region_min_y, region_max_y, 4)
    positions_list = [[float(conveyor_x), float(y), float(conveyor_top_z + place_clearance)] for y in ys_candidates]
    quats_list = [[0.0, 1.0, 0.0, 0.0]] * 4
    pos_tensor = torch.tensor(positions_list, dtype=torch.float32, device="cuda:0").unsqueeze(0)
    quat_tensor = torch.tensor(quats_list, dtype=torch.float32, device="cuda:0").unsqueeze(0)
    goalset_pose = Pose(position=pos_tensor, quaternion=quat_tensor)
    result = motion_gen.plan_goalset(start, goalset_pose, MotionGenPlanConfig(max_attempts=15))
    if not result.success.item():
        print(f"FAILED at {label} (goalset):", result.status)
        return None, None
    pos = result.get_interpolated_plan().position.cpu().numpy()
    all_waypoints.append(pos)
    new_start = JointState.from_position(torch.tensor(pos[-1], device="cuda:0").view(1, -1), joint_names=motion_gen.kinematics.joint_names)
    chosen_y = ys_candidates[result.goalset_index.item()]
    segment_info.append((tag, label, len(pos)))
    print(f"  {label} (goalset): {len(pos)} waypoints, chose y={chosen_y:.3f}")
    return new_start, chosen_y

pairs = [(0, 1), (2, 3)]

for pair_num, pair in enumerate(pairs):
    i0, i1 = pair
    if pair_num == 0:
        region_min_y, region_max_y = 0.3, 0.6
    else:
        region_min_y, region_max_y = 0.7, 1.0

    b0 = box_positions_real[i0]
    b1 = box_positions_real[i1]
    midpoint_x = b0[0]
    midpoint_y = (b0[1] + b1[1]) / 2
    midpoint_z = b0[2]
    half_gap = abs(b1[1] - b0[1]) / 2
    pair_half_gaps[pair] = half_gap

    print(f"\n--- Grasping pair {pair}, midpoint y={midpoint_y}, half_gap={half_gap} ---")

    grasp_pose_metric = PoseCostMetric.create_grasp_approach_metric(offset_position=approach_height, tstep_fraction=0.6, linear_axis=2)
    goal_pose = Pose.from_list([midpoint_x, midpoint_y, midpoint_z + box_half_size, 0.0, 1.0, 0.0, 0.0])
    result = motion_gen.plan_single(current_state, goal_pose, MotionGenPlanConfig(max_attempts=15, pose_cost_metric=grasp_pose_metric))
    if not result.success.item():
        print(f"FAILED at approach_and_grasp for {pair}:", result.status)
        current_state = None
        break
    pos = result.get_interpolated_plan().position.cpu().numpy()
    all_waypoints.append(pos)
    current_state = JointState.from_position(torch.tensor(pos[-1], device="cuda:0").view(1, -1), joint_names=motion_gen.kinematics.joint_names)
    segment_info.append((pair, "approach_and_grasp", len(pos)))
    print(f"  approach_and_grasp: {len(pos)} waypoints")

    current_state = plan_and_extend(current_state, [midpoint_x, midpoint_y, midpoint_z + approach_height, 0.0, 1.0, 0.0, 0.0], "lift", pair)
    current_state = plan_and_extend(current_state, [conveyor_x, (region_min_y+region_max_y)/2, conveyor_top_z + approach_height, 0.0, 1.0, 0.0, 0.0], f"move_to_conveyor_{i0}", pair)
    current_state, chosen_y0 = plan_goalset_place(current_state, f"descend_place_{i0}", pair)
    if current_state is None:
        break
    current_state = plan_and_extend(current_state, [conveyor_x, chosen_y0, conveyor_top_z + approach_height, 0.0, 1.0, 0.0, 0.0], f"lift_after_{i0}", pair)
    current_state, chosen_y1 = plan_goalset_place(current_state, f"descend_place_{i1}", pair)
    if current_state is None:
        break
    current_state = plan_and_extend(current_state, [conveyor_x, chosen_y1, conveyor_top_z + approach_height, 0.0, 1.0, 0.0, 0.0], "retreat", pair)

if current_state is not None and all(w is not None for w in all_waypoints):
    full_trajectory = np.concatenate(all_waypoints, axis=0)
    np.save("C:/Users/NehaaChowdary/warehouse/dual_trajectory.npy", full_trajectory)
    np.save("C:/Users/NehaaChowdary/warehouse/dual_segments.npy", np.array(segment_info, dtype=object), allow_pickle=True)
    np.save("C:/Users/NehaaChowdary/warehouse/dual_pair_gaps.npy", np.array(list(pair_half_gaps.values())))
    print(f"\nSaved: {full_trajectory.shape[0]} total waypoints")
else:
    print("\nFAILED - not saved")