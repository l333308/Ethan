"""
姿态控制器

实现PID姿态平衡控制，保持机器人直立姿态。

设计原则:
  - 保守控制：小增益、有限幅，宁可反应慢也不要发散
  - 分层架构：姿态控制 + 重心高度控制 → 站立控制器
  - 基准姿态：所有校正量都是在基准姿态上叠加的偏移
"""

import numpy as np
from typing import Dict, Tuple
from dataclasses import dataclass


@dataclass
class PIDGains:
    """PID控制器增益参数"""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.1


class PostureController:
    """姿态控制器 — 基于IMU反馈的Roll/Pitch PID平衡"""

    def __init__(self,
                 roll_gains: PIDGains = None,
                 pitch_gains: PIDGains = None,
                 target_roll: float = 0.0,
                 target_pitch: float = 0.0):
        # 保守增益（经诊断验证：直腿站立本身稳定，只需微调）
        self.roll_gains  = roll_gains  or PIDGains(kp=0.3, ki=0.01, kd=0.05)
        self.pitch_gains = pitch_gains or PIDGains(kp=0.3, ki=0.01, kd=0.05)

        self.target_roll  = target_roll
        self.target_pitch = target_pitch

        # PID 内部状态
        self._ri = 0.0   # roll 积分
        self._pi = 0.0   # pitch 积分
        self._re = 0.0   # roll 上次误差
        self._pe = 0.0   # pitch 上次误差

        self.integral_limit = 5.0  # 积分限幅（度·秒）

    def reset(self):
        self._ri = self._pi = 0.0
        self._re = self._pe = 0.0

    def update(self, roll: float, pitch: float, dt: float) -> Tuple[float, float]:
        """返回 (roll_correction, pitch_correction)（度）"""
        re = self.target_roll  - roll
        pe = self.target_pitch - pitch

        self._ri = np.clip(self._ri + re * dt, -self.integral_limit, self.integral_limit)
        self._pi = np.clip(self._pi + pe * dt, -self.integral_limit, self.integral_limit)

        rd = (re - self._re) / dt if dt > 0 else 0.0
        pd = (pe - self._pe) / dt if dt > 0 else 0.0

        rc = self.roll_gains.kp  * re + self.roll_gains.ki  * self._ri + self.roll_gains.kd  * rd
        pc = self.pitch_gains.kp * pe + self.pitch_gains.ki * self._pi + self.pitch_gains.kd * pd

        self._re, self._pe = re, pe
        return rc, pc

    @staticmethod
    def compute_joint_corrections(roll_corr: float, pitch_corr: float) -> Dict[str, float]:
        """姿态校正 → 关节偏移（度）

        策略：
          roll  → 两侧 hip_roll 反向调整
          pitch → 髋关节 pitch + 踝关节 pitch 协同
        """
        return {
            'left_hip_roll':     roll_corr  *  0.3,
            'right_hip_roll':    roll_corr  * -0.3,
            'left_hip_pitch':    pitch_corr *  0.3,
            'right_hip_pitch':   pitch_corr *  0.3,
            'left_ankle_pitch':  pitch_corr * -0.2,
            'right_ankle_pitch': pitch_corr * -0.2,
        }


class CenterOfMassController:
    """重心高度控制器 — 通过膝关节弯曲/伸展调整高度"""

    def __init__(self, target_height: float = 0.24):
        self.target = target_height
        self.gains  = PIDGains(kp=8.0, ki=0.5, kd=1.0)
        self._i = 0.0
        self._e = 0.0

    def reset(self):
        self._i = self._e = 0.0

    def update(self, height: float, dt: float) -> float:
        e  = self.target - height
        self._i = np.clip(self._i + e * dt, -0.05, 0.05)
        d  = (e - self._e) / dt if dt > 0 else 0.0
        self._e = e
        return self.gains.kp * e + self.gains.ki * self._i + self.gains.kd * d

    @staticmethod
    def compute_leg_extension(h_corr: float) -> Dict[str, float]:
        """高度校正(m) → 关节偏移(度)  简化线性映射"""
        # 正的校正 = 需要升高 = 膝盖伸直 = 膝关节角度减小
        knee = -h_corr * 150.0   # 每米 → 150度
        hip  =  h_corr *  75.0
        return {
            'left_knee':       knee,
            'right_knee':      knee,
            'left_hip_pitch':  hip,
            'right_hip_pitch': hip,
        }


class StandingController:
    """站立控制器 — 组合姿态 + 重心控制，输出关节目标角度"""

    def __init__(self,
                 target_height: float = 0.24,
                 target_roll:   float = 0.0,
                 target_pitch:  float = 0.0):
        self.posture = PostureController(target_roll=target_roll,
                                         target_pitch=target_pitch)
        self.com     = CenterOfMassController(target_height=target_height)

        # 基准姿态: 直腿零位（诊断证明零位即可稳定站立）
        self.base_pose = {
            'head_pitch':        0.0,
            'left_hip_roll':     0.0,
            'left_hip_pitch':    0.0,
            'left_knee':         0.0,
            'left_ankle_pitch':  0.0,
            'right_hip_roll':    0.0,
            'right_hip_pitch':   0.0,
            'right_knee':        0.0,
            'right_ankle_pitch': 0.0,
        }

        # 每个控制周期单关节最大校正（度）
        self.max_correction = 3.0

    def reset(self):
        self.posture.reset()
        self.com.reset()

    def set_base_pose(self, pose: Dict[str, float]):
        self.base_pose.update(pose)

    def compute_control(self, height: float, roll: float, pitch: float,
                        dt: float) -> Dict[str, float]:
        """计算关节目标角度（度）"""
        # 姿态补偿
        rc, pc = self.posture.update(roll, pitch, dt)
        p_corr = self.posture.compute_joint_corrections(rc, pc)

        # 高度补偿
        hc = self.com.update(height, dt)
        h_corr = self.com.compute_leg_extension(hc)

        # 合并：基准 + 姿态偏移 + 高度偏移（带限幅）
        out = {}
        for jname, base_val in self.base_pose.items():
            delta = p_corr.get(jname, 0.0) + h_corr.get(jname, 0.0)
            delta = np.clip(delta, -self.max_correction, self.max_correction)
            out[jname] = base_val + delta

        return out
