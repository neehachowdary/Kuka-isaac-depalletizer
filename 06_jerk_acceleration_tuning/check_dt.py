from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util_file import load_yaml
from curobo.geom.types import WorldConfig, Cuboid
import torch

tensor_args = TensorDeviceType()
config_file = load_yaml('kr50_r2500_low_jerk.yml')
robot_cfg = RobotConfig.from_dict(config_file['robot_cfg'])
world_cfg = WorldConfig(cuboid=[Cuboid(name='dummy', pose=[10,10,10,1,0,0,0], dims=[0.01,0.01,0.01])])
motion_gen_config = MotionGenConfig.load_from_robot_config(robot_cfg, world_cfg, tensor_args=tensor_args, high_precision=True, use_cuda_graph=False)
motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()
retract_cfg = motion_gen.get_retract_config()
start_state = JointState.from_position(retract_cfg.view(1,-1), joint_names=motion_gen.kinematics.joint_names)
goal_pose = Pose.from_list([1.3, -0.8, 0.64, 0.0, 1.0, 0.0, 0.0])
result = motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=15))
print('motion_time (seconds):', result.motion_time)
print('interpolation_dt (seconds/waypoint):', result.interpolation_dt)
print('total_time:', result.total_time)
