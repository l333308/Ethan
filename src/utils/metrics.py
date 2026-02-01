"""
稳定性评估指标

用于量化评估机器人的站立和行走稳定性
"""

import numpy as np
from typing import Dict, List


class StabilityMetrics:
    """稳定性指标计算器"""
    
    def __init__(self):
        """初始化"""
        self.history = {
            'time': [],
            'height': [],
            'roll': [],
            'pitch': [],
            'yaw': [],
            'position': [],
            'velocity': []
        }
        
        # 稳定性阈值
        self.thresholds = {
            'min_height': 0.15,      # 最小高度（米）
            'max_roll': 30,          # 最大侧倾角度（度）
            'max_pitch': 30,         # 最大俯仰角度（度）
            'max_position_drift': 0.5  # 最大位置漂移（米）
        }
        
        self.start_position = None
        self.current_time = 0
        
    def update(self, base_state: Dict, joint_states: Dict, imu_data: Dict):
        """更新数据
        
        Args:
            base_state: 基座状态
            joint_states: 关节状态
            imu_data: IMU数据
        """
        pos = base_state['position']
        euler = base_state['orientation_euler']
        vel = base_state['linear_velocity']
        
        if self.start_position is None:
            self.start_position = pos.copy()
        
        self.history['time'].append(self.current_time)
        self.history['height'].append(pos[2])
        self.history['roll'].append(euler[0])
        self.history['pitch'].append(euler[1])
        self.history['yaw'].append(euler[2])
        self.history['position'].append(pos.copy())
        self.history['velocity'].append(np.linalg.norm(vel))
        
        self.current_time += 0.001  # 假设1ms更新
        
    def is_stable(self) -> bool:
        """判断当前是否稳定
        
        Returns:
            是否稳定
        """
        if len(self.history['height']) == 0:
            return True
            
        # 检查高度
        current_height = self.history['height'][-1]
        if current_height < self.thresholds['min_height']:
            return False
            
        # 检查倾斜角度
        current_roll = abs(self.history['roll'][-1])
        current_pitch = abs(self.history['pitch'][-1])
        
        if current_roll > self.thresholds['max_roll']:
            return False
        if current_pitch > self.thresholds['max_pitch']:
            return False
            
        # 检查位置漂移
        current_pos = self.history['position'][-1]
        drift = np.linalg.norm(current_pos[:2] - self.start_position[:2])
        
        if drift > self.thresholds['max_position_drift']:
            return False
            
        return True
        
    def get_stability_score(self) -> float:
        """计算稳定性得分（0-100）
        
        Returns:
            稳定性得分
        """
        if len(self.history['height']) == 0:
            return 100.0
            
        scores = []
        
        # 高度得分（越高越好，但有上限）
        avg_height = np.mean(self.history['height'])
        height_score = min(100, (avg_height / 0.3) * 100)
        scores.append(height_score)
        
        # 姿态稳定性得分
        roll_std = np.std(self.history['roll'])
        pitch_std = np.std(self.history['pitch'])
        orientation_score = max(0, 100 - (roll_std + pitch_std) * 5)
        scores.append(orientation_score)
        
        # 位置稳定性得分
        positions = np.array(self.history['position'])
        if len(positions) > 1:
            position_std = np.std(positions[:, :2], axis=0)
            position_score = max(0, 100 - np.mean(position_std) * 200)
            scores.append(position_score)
        
        return np.mean(scores)
        
    def get_summary(self) -> Dict:
        """获取统计摘要
        
        Returns:
            统计数据字典
        """
        if len(self.history['height']) == 0:
            return {}
            
        positions = np.array(self.history['position'])
        
        return {
            'duration': self.current_time,
            'avg_height': np.mean(self.history['height']),
            'min_height': np.min(self.history['height']),
            'max_height': np.max(self.history['height']),
            'avg_roll': np.mean(np.abs(self.history['roll'])),
            'max_roll': np.max(np.abs(self.history['roll'])),
            'avg_pitch': np.mean(np.abs(self.history['pitch'])),
            'max_pitch': np.max(np.abs(self.history['pitch'])),
            'position_drift': np.linalg.norm(positions[-1][:2] - positions[0][:2]),
            'stability_score': self.get_stability_score(),
            'is_stable': self.is_stable()
        }
        
    def print_summary(self):
        """打印摘要"""
        summary = self.get_summary()
        
        if not summary:
            print("⚠️ 没有数据")
            return
            
        print(f"测试时长: {summary['duration']:.2f}秒")
        print(f"\n高度统计:")
        print(f"  - 平均: {summary['avg_height']:.3f}m")
        print(f"  - 最小: {summary['min_height']:.3f}m")
        print(f"  - 最大: {summary['max_height']:.3f}m")
        
        print(f"\n姿态统计:")
        print(f"  - Roll平均: {summary['avg_roll']:.2f}°")
        print(f"  - Roll最大: {summary['max_roll']:.2f}°")
        print(f"  - Pitch平均: {summary['avg_pitch']:.2f}°")
        print(f"  - Pitch最大: {summary['max_pitch']:.2f}°")
        
        print(f"\n位置漂移: {summary['position_drift']:.3f}m")
        
        print(f"\n稳定性得分: {summary['stability_score']:.1f}/100")
        
        if summary['is_stable']:
            print("\n✅ 测试通过 - 机器人保持稳定")
        else:
            print("\n❌ 测试失败 - 机器人失去平衡")


class GaitMetrics:
    """步态评估指标（为后续步态控制准备）"""
    
    def __init__(self):
        """初始化"""
        self.steps_count = 0
        self.step_history = []
        
    def detect_step(self, foot_contact: Dict[str, bool]) -> bool:
        """检测步态
        
        Args:
            foot_contact: 脚底接触状态
            
        Returns:
            是否检测到新的一步
        """
        # TODO: 实现步态检测逻辑
        pass
        
    def calculate_step_length(self) -> float:
        """计算步长"""
        # TODO: 实现
        pass
        
    def calculate_step_frequency(self) -> float:
        """计算步频"""
        # TODO: 实现
        pass
