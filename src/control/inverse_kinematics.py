"""
逆运动学求解器

针对本机器人的2D平面腿部逆运动学（解析法）。

腿部运动学链（每条腿）:
  hip_roll  → 侧向摆动 (x轴旋转)
  hip_pitch → 前后摆动 (y轴旋转)
  knee      → 弯曲      (y轴旋转)
  ankle_pitch → 脚踝   (y轴旋转)

简化: 在sagittal平面（前后方向）内做2-link IK，
      roll方向单独处理。
"""

import numpy as np
from typing import Dict, Optional
from dataclasses import dataclass


@dataclass
class LegGeometry:
    """腿部几何参数（来自URDF）"""
    thigh_length: float = 0.12   # 大腿长度 (m)
    calf_length:  float = 0.10   # 小腿长度 (m)
    hip_offset_y: float = 0.07   # 髋关节侧向偏移 (m)
    hip_offset_z: float = 0.075  # 髋关节相对躯干的下偏移 (m)


def solve_leg_ik(target_x: float, target_z: float,
                 geom: LegGeometry = None) -> Optional[Dict[str, float]]:
    """2-link 平面逆运动学

    在 sagittal 平面内求解 hip_pitch 和 knee 角度，
    使脚踝(ankle)到达相对于髋关节的目标位置 (target_x, target_z)。

    坐标系定义（相对于髋关节）:
      x: 前方为正
      z: 下方为负

    Args:
        target_x: 脚踝相对于髋关节的前方偏移 (m)
        target_z: 脚踝相对于髋关节的垂直偏移 (m)，向下为负
    
    Returns:
        {'hip_pitch': deg, 'knee': deg, 'ankle_pitch': deg} 或 None（不可达）
    """
    geom = geom or LegGeometry()
    L1 = geom.thigh_length
    L2 = geom.calf_length

    # 脚踝到髋关节距离
    dx, dz = target_x, target_z  # dz < 0 (脚在髋关节下方)
    d = np.sqrt(dx**2 + dz**2)

    # 可达性检查（留足够余量以包含边界）
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)
    if d > max_reach:
        # 超出范围，截断到最大可达
        d = max_reach - 1e-6
    if d < min_reach + 1e-6:
        d = min_reach + 1e-6

    # 余弦定理求膝关节角度
    cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee_angle = np.pi - np.arccos(cos_knee)  # 0=全伸直, >0=弯曲

    # 求 hip_pitch
    alpha = np.arctan2(-dx, -dz)  # 目标方向角（注意z向下为负）
    cos_beta = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    hip_pitch = alpha - beta  # 负值=前倾

    # ankle_pitch 保持脚底水平
    ankle_pitch = -(hip_pitch + knee_angle)

    return {
        'hip_pitch':    np.rad2deg(hip_pitch),
        'knee':         np.rad2deg(knee_angle),
        'ankle_pitch':  np.rad2deg(ankle_pitch),
    }


def compute_leg_joints(foot_x: float, foot_z: float, foot_roll: float = 0.0,
                       side: str = 'left',
                       geom: LegGeometry = None) -> Optional[Dict[str, float]]:
    """计算单腿全部关节角度

    Args:
        foot_x:    脚相对于髋关节的前方偏移 (m)
        foot_z:    脚相对于髋关节的垂直偏移 (m)，向下为负
        foot_roll: 脚的侧倾角（度），用于重心转移
        side:      'left' 或 'right'

    Returns:
        {'{side}_hip_roll': deg, '{side}_hip_pitch': deg,
         '{side}_knee': deg, '{side}_ankle_pitch': deg}
        如果不可达返回 None
    """
    ik = solve_leg_ik(foot_x, foot_z, geom)
    if ik is None:
        return None

    prefix = side
    return {
        f'{prefix}_hip_roll':    foot_roll,
        f'{prefix}_hip_pitch':   ik['hip_pitch'],
        f'{prefix}_knee':        ik['knee'],
        f'{prefix}_ankle_pitch': ik['ankle_pitch'],
    }
