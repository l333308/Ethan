"""
控制模块

第二阶段实现内容:
- posture_controller: 姿态控制器 (PID平衡)
- inverse_kinematics: 逆运动学求解 (2-link IK)
- gait_generator:     步态生成器 (状态机+轨迹)
"""

__version__ = '0.2.0'

from src.control.posture_controller import (
    PIDGains,
    PostureController,
    CenterOfMassController,
    StandingController,
)
from src.control.inverse_kinematics import (
    LegGeometry,
    solve_leg_ik,
    compute_leg_joints,
)
from src.control.gait_generator import (
    GaitPhase,
    GaitParams,
    GaitGenerator,
)
