import torch
import numpy as np
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util_file import load_yaml, get_robot_configs_path, join_path
from curobo.geom.types import WorldConfig, Cuboid

tensor_args = TensorDeviceType()
config_file = load_yaml(join_path(get_robot_configs_path(), "kr50_r2500.yml"))
robot_cfg = RobotConfig.from_dict(config_file["robot_cfg"])
world_cfg = WorldConfig(cuboid=[Cuboid(name="dummy", pose=[10,10,10,1,0,0,0], dims=[0.01,0.01,0.01])])
motion_gen_config = MotionGenConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args=tensor_args, high_precision=True, use_cuda_graph=False)
motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()

retract_cfg = motion_gen.get_retract_config()
start_state = JointState.from_position(retract_cfg.view(1,-1), joint_names=motion_gen.kinematics.joint_names)

for test_z in [0.8, 1.5, 1.68, 2.0, 2.5]:
    goal_pose = Pose.from_list([1.3, -0.3, test_z, 0.0, 1.0, 0.0, 0.0])
    result = motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=10))
    print(f"Z={test_z}: {'REACHABLE' if result.success.item() else 'NOT REACHABLE (' + str(result.status) + ')'}")
