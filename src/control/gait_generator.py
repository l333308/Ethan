"""
步态生成器

基于有限状态机的步态生成，输出双腿的足端轨迹。

状态机:
  DOUBLE_SUPPORT → LEFT_SWING → DOUBLE_SUPPORT → RIGHT_SWING → ...

足端轨迹使用正弦曲线生成，保证连续平滑。
"""

import numpy as np
from enum import Enum, auto
from typing import Dict, Tuple
from dataclasses import dataclass

from src.control.inverse_kinematics import compute_leg_joints, LegGeometry


class GaitPhase(Enum):
    """步态阶段"""
    DOUBLE_SUPPORT = auto()   # 双足支撑
    LEFT_SWING     = auto()   # 左脚摆动（右脚支撑）
    RIGHT_SWING    = auto()   # 右脚摆动（左脚支撑）


@dataclass
class GaitParams:
    """步态参数"""
    step_height: float = 0.02    # 抬脚高度 (m)
    step_length: float = 0.00    # 步长 (m)，0=原地踏步
    step_period: float = 2.0     # 单步周期 (s)
    double_support_ratio: float = 0.3  # 双足支撑占比
    com_shift_y: float = 0.02    # 重心侧向转移量 (m)
    turn_rate: float = 0.0       # 转向速率 (度/步), 正=逆时针
    
    # 腿部默认站立位置（相对于髋关节）
    foot_x_offset: float = 0.0   # 脚前后偏移 (m)
    foot_z_stand:  float = -0.22 # 站立时脚到髋关节的垂直距离 (m)


class GaitGenerator:
    """步态生成器
    
    根据当前时间输出双腿关节目标角度。
    """
    
    def __init__(self, params: GaitParams = None):
        self.params = params or GaitParams()
        self.geom = LegGeometry()
        
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self.phase_time = 0.0       # 当前阶段已持续时间
        self.total_time = 0.0       # 总时间
        self.step_count = 0
        
        # 每个阶段的持续时间
        self._update_durations()
    
    def _update_durations(self):
        """根据参数计算各阶段时长"""
        T = self.params.step_period
        dr = self.params.double_support_ratio
        self.double_dur = T * dr       # 双足支撑时长
        self.swing_dur  = T * (1 - dr) # 摆动时长
    
    def reset(self):
        """重置状态机"""
        self.phase = GaitPhase.DOUBLE_SUPPORT
        self.phase_time = 0.0
        self.total_time = 0.0
        self.step_count = 0
    
    def set_velocity(self, forward: float = 0.0, turn: float = 0.0):
        """设置行走速度命令
        
        Args:
            forward: 前进步长 (m), 0=原地踏步
            turn: 转向速率 (度/步), 正=逆时针
        """
        self.params.step_length = forward
        self.params.turn_rate = turn
    
    def stop(self):
        """停止行走（完成当前步后停下）"""
        self.params.step_length = 0.0
        self.params.turn_rate = 0.0
        self.params.step_height = 0.0  # 不再抬脚
    
    def update(self, dt: float) -> Dict[str, float]:
        """步进一步，返回关节目标角度
        
        Args:
            dt: 时间步长 (s)
            
        Returns:
            所有关节名 → 目标角度（度）
        """
        self.phase_time += dt
        self.total_time += dt
        
        # 状态转换
        self._check_transition()
        
        # 直接规划关节角度（避免IK的大角度hip_pitch导致前倾）
        joints = self._compute_joint_angles_direct()
        return joints
    
    def _check_transition(self):
        """检查是否需要切换阶段"""
        if self.phase == GaitPhase.DOUBLE_SUPPORT:
            if self.phase_time >= self.double_dur:
                self.phase_time = 0.0
                # 交替左右摆动
                if self.step_count % 2 == 0:
                    self.phase = GaitPhase.LEFT_SWING
                else:
                    self.phase = GaitPhase.RIGHT_SWING
                    
        elif self.phase in (GaitPhase.LEFT_SWING, GaitPhase.RIGHT_SWING):
            if self.phase_time >= self.swing_dur:
                self.phase_time = 0.0
                self.phase = GaitPhase.DOUBLE_SUPPORT
                self.step_count += 1
    
    def _compute_foot_targets(self) -> Tuple[Tuple[float, float],
                                              Tuple[float, float],
                                              float]:
        """计算双脚目标位置和重心侧移

        Returns:
            (left_foot_xz, right_foot_xz, com_roll_deg)
            foot_xz = (x, z) 相对于各自髋关节
        """
        p = self.params
        x0 = p.foot_x_offset
        z0 = p.foot_z_stand
        
        # 摆动进度 [0, 1]
        if self.phase in (GaitPhase.LEFT_SWING, GaitPhase.RIGHT_SWING):
            progress = min(self.phase_time / self.swing_dur, 1.0)
        else:
            progress = 0.0
        
        # 足端高度轨迹：正弦曲线
        lift = p.step_height * np.sin(np.pi * progress)
        
        # 足端前后轨迹：余弦曲线（半步长）
        forward = p.step_length * 0.5 * (1 - np.cos(np.pi * progress))
        
        # 双足支撑期的重心转移（为下一次摆动做准备）
        if self.phase == GaitPhase.DOUBLE_SUPPORT:
            ds_progress = min(self.phase_time / self.double_dur, 1.0) if self.double_dur > 0 else 1.0
            # 下一步是哪只脚摆动？
            if self.step_count % 2 == 0:
                # 下一步左脚摆动 → 重心移向右侧
                com_roll = -p.com_shift_y * 100.0 * np.sin(np.pi * ds_progress * 0.5)
            else:
                # 下一步右脚摆动 → 重心移向左侧
                com_roll = p.com_shift_y * 100.0 * np.sin(np.pi * ds_progress * 0.5)
        elif self.phase == GaitPhase.LEFT_SWING:
            # 左脚摆动，重心在右侧
            com_roll = -p.com_shift_y * 100.0
        else:
            # 右脚摆动，重心在左侧
            com_roll = p.com_shift_y * 100.0
        
        # 根据阶段设置足端位置
        if self.phase == GaitPhase.DOUBLE_SUPPORT:
            left_foot  = (x0, z0)
            right_foot = (x0, z0)
            
        elif self.phase == GaitPhase.LEFT_SWING:
            left_foot  = (x0 + forward, z0 + lift)
            right_foot = (x0, z0)
            
        elif self.phase == GaitPhase.RIGHT_SWING:
            left_foot  = (x0, z0)
            right_foot = (x0 + forward, z0 + lift)
        
        return left_foot, right_foot, com_roll

    def _apply_pitch_compensation(self, joints: Dict[str, float],
                                   swing_side: str) -> Dict[str, float]:
        """补偿摆动腿弯曲引起的躯干前倾
        
        摆动腿的 hip_pitch 会产生一个前倾力矩，
        需要通过支撑腿的 ankle_pitch 来反向补偿。
        """
        if swing_side == 'none':
            return joints
        
        # 获取摆动腿的hip_pitch（负值=前倾）
        swing_hip = joints.get(f'{swing_side}_hip_pitch', 0.0)
        
        # 支撑腿侧
        support = 'right' if swing_side == 'left' else 'left'
        
        # 支撑腿ankle往正方向推（后仰），补偿前倾
        # 补偿比例: 摆动腿每前倾1°, 支撑腿ankle补偿约0.3°
        compensation = -swing_hip * 0.3
        joints[f'{support}_ankle_pitch'] = joints.get(f'{support}_ankle_pitch', 0.0) + compensation
        
        return joints

    def _compute_joint_angles_direct(self) -> Dict[str, float]:
        """直接规划关节角度（不使用IK）
        
        策略: 
          - 摆动腿只弯曲膝盖，hip_pitch极小变化
          - 支撑腿保持直立
          - 通过hip_roll做侧向重心转移
        
        这样避免了IK产生大角度hip_pitch导致躯干前倾的问题。
        """
        p = self.params
        
        # 摆动进度 [0, 1]
        if self.phase in (GaitPhase.LEFT_SWING, GaitPhase.RIGHT_SWING):
            progress = min(self.phase_time / self.swing_dur, 1.0)
        else:
            progress = 0.0
        
        # 摆动轮廓：正弦曲线（平滑上升下降）
        swing_profile = np.sin(np.pi * progress)  # 0→1→0
        
        # 抬脚高度转换为膝盖弯曲角度
        # knee 每弯曲10°约抬起: calf*(1-cos(10°))≈0.1*0.015=0.15cm
        # 所以抬2cm需要约 knee=35°
        # 但同时hip_pitch需要一点点补偿（约knee角度的1/4）
        knee_target = 35.0 * (p.step_height / 0.02)  # 按2cm为基准缩放
        hip_pitch_target = -5.0 * (p.step_height / 0.02)  # 微小的髋关节前倾
        ankle_target = -(knee_target + hip_pitch_target)  # 保持脚底水平
        # 限制ankle在合理范围
        ankle_target = np.clip(ankle_target, -30.0, 30.0)
        
        # 重心侧移
        if self.phase == GaitPhase.DOUBLE_SUPPORT:
            ds_progress = min(self.phase_time / self.double_dur, 1.0) if self.double_dur > 0 else 1.0
            if self.step_count % 2 == 0:
                # 下一步左脚摆动 → 重心向右
                com_roll = -p.com_shift_y * 100 * np.sin(np.pi * ds_progress * 0.5)
            else:
                com_roll = p.com_shift_y * 100 * np.sin(np.pi * ds_progress * 0.5)
        elif self.phase == GaitPhase.LEFT_SWING:
            com_roll = -p.com_shift_y * 100  # 重心在右侧
        else:
            com_roll = p.com_shift_y * 100  # 重心在左侧
        
        # 持续前进偏置 (作用于所有阶段)
        # 正 hip_pitch → 机器人向 -x 移动 (实验确认)
        fwd_bias = p.step_length * 10.0   # 10cm → 1度的持续偏置
        
        # 构建关节角度
        joints = {
            'head_pitch': 0.0,
            'left_hip_roll': 0.0,  'left_hip_pitch': fwd_bias,
            'left_knee': 0.0,      'left_ankle_pitch': -fwd_bias * 0.5,
            'right_hip_roll': 0.0, 'right_hip_pitch': fwd_bias,
            'right_knee': 0.0,     'right_ankle_pitch': -fwd_bias * 0.5,
        }
        
        # 侧向重心转移
        joints['left_hip_roll']  =  com_roll * 0.3
        joints['right_hip_roll'] = -com_roll * 0.3
        
        # 转向: 左右腿 hip_pitch 不对称 → 产生 yaw 力矩
        # 正turn_rate = 逆时针
        # 左脚多推(正hip_pitch) + 右脚少推 → 身体逆时针转
        if p.turn_rate != 0.0 and self.phase != GaitPhase.DOUBLE_SUPPORT:
            turn_bias = p.turn_rate * 0.5  # 保守映射
            if self.phase == GaitPhase.LEFT_SWING:
                joints['right_hip_pitch'] += turn_bias * swing_profile
                joints['left_hip_roll']   += turn_bias * 0.3 * swing_profile
            elif self.phase == GaitPhase.RIGHT_SWING:
                joints['left_hip_pitch']  -= turn_bias * swing_profile
                joints['right_hip_roll']  -= turn_bias * 0.3 * swing_profile
        
        # 支撑腿额外推力
        ankle_push = p.step_length * 15.0
        hip_push   = p.step_length * 20.0

        if self.phase == GaitPhase.LEFT_SWING:
            joints['left_knee']         = knee_target * swing_profile
            joints['left_hip_pitch']    = hip_pitch_target * swing_profile + fwd_bias
            joints['left_ankle_pitch']  = ankle_target * swing_profile - fwd_bias * 0.5
            # 支撑腿(右)额外推力
            joints['right_ankle_pitch'] += -ankle_push * progress
            joints['right_hip_pitch']   += hip_push * progress
            
        elif self.phase == GaitPhase.RIGHT_SWING:
            joints['right_knee']        = knee_target * swing_profile
            joints['right_hip_pitch']   = hip_pitch_target * swing_profile + fwd_bias
            joints['right_ankle_pitch'] = ankle_target * swing_profile - fwd_bias * 0.5
            joints['left_ankle_pitch']  += -ankle_push * progress
            joints['left_hip_pitch']    += hip_push * progress
        
        return joints

    @property
    def phase_name(self) -> str:
        return {
            GaitPhase.DOUBLE_SUPPORT: "双足支撑",
            GaitPhase.LEFT_SWING:     "左脚摆动",
            GaitPhase.RIGHT_SWING:    "右脚摆动",
        }[self.phase]
