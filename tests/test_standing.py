"""
站立稳定性测试

测试机器人在不同初始姿态下的站立稳定性
"""

import sys
import time
import numpy as np
from pathlib import Path

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment
from src.utils.metrics import StabilityMetrics


def test_standing_stability(duration: float = 5.0):
    """测试站立稳定性
    
    Args:
        duration: 测试时长（秒）
    """
    print("=" * 60)
    print("🧍 站立稳定性测试")
    print("=" * 60)
    
    # 文件路径
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # 创建仿真环境
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    # 创建稳定性评估器
    metrics = StabilityMetrics()
    
    # 设置初始姿态（双腿微屈，更稳定）
    initial_pose = {
        'left_hip_pitch': 10,
        'left_knee': 20,
        'left_ankle_pitch': -10,
        'right_hip_pitch': 10,
        'right_knee': 20,
        'right_ankle_pitch': -10,
    }
    env.set_joint_positions(initial_pose)
    
    # 稳定一段时间让机器人settle
    print("\n⏳ 初始化姿态...")
    for _ in range(500):
        env.step()
        time.sleep(1/240)
    
    # 开始测试
    print(f"\n🚀 开始测试 (时长={duration}秒)")
    print("-" * 60)
    
    time_step = env.config['simulation']['physics']['time_step']
    steps = int(duration / time_step)
    
    for i in range(steps):
        env.step()
        
        # 收集数据
        base_state = env.get_base_state()
        joint_states = env.get_joint_states()
        imu_data = env.get_imu_data()
        
        metrics.update(base_state, joint_states, imu_data)
        
        # 每0.5秒打印一次
        if i % int(0.5 / time_step) == 0:
            pos = base_state['position']
            euler = base_state['orientation_euler']
            print(f"t={i*time_step:5.2f}s | "
                  f"高度={pos[2]:.3f}m | "
                  f"Roll={euler[0]:6.2f}° | "
                  f"Pitch={euler[1]:6.2f}° | "
                  f"稳定={metrics.is_stable()}")
        
        time.sleep(time_step)
        
        # 检查是否摔倒
        if not metrics.is_stable():
            print("\n❌ 机器人失去平衡!")
            break
    
    # 打印测试结果
    print("\n" + "=" * 60)
    print("📊 测试结果")
    print("=" * 60)
    metrics.print_summary()
    
    # 保持窗口打开
    print("\n按Enter键关闭...")
    input()
    
    env.close()


def test_with_disturbance(duration: float = 10.0):
    """测试扰动下的稳定性
    
    Args:
        duration: 测试时长（秒）
    """
    print("=" * 60)
    print("💨 扰动测试")
    print("=" * 60)
    
    # 文件路径
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # 创建仿真环境
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    metrics = StabilityMetrics()
    
    # 设置初始姿态
    initial_pose = {
        'left_hip_pitch': 10,
        'left_knee': 20,
        'left_ankle_pitch': -10,
        'right_hip_pitch': 10,
        'right_knee': 20,
        'right_ankle_pitch': -10,
    }
    env.set_joint_positions(initial_pose)
    
    # 稳定
    print("\n⏳ 初始化...")
    for _ in range(500):
        env.step()
        time.sleep(1/240)
    
    # 测试
    print(f"\n🚀 开始测试")
    print("   - 在t=3秒时施加侧向力")
    print("-" * 60)
    
    time_step = env.config['simulation']['physics']['time_step']
    steps = int(duration / time_step)
    disturbance_applied = False
    
    for i in range(steps):
        current_time = i * time_step
        
        # 在3秒时施加侧向扰动
        if current_time >= 3.0 and not disturbance_applied:
            import pybullet as p
            force = [0, 50, 0]  # Y方向50N
            position = [0, 0, 0.2]
            p.applyExternalForce(
                env.robot_id,
                -1,  # 作用在base上
                force,
                position,
                p.WORLD_FRAME
            )
            print(f"\n💥 施加扰动: {force}")
            disturbance_applied = True
        
        env.step()
        
        # 收集数据
        base_state = env.get_base_state()
        joint_states = env.get_joint_states()
        imu_data = env.get_imu_data()
        
        metrics.update(base_state, joint_states, imu_data)
        
        # 打印
        if i % int(0.5 / time_step) == 0:
            pos = base_state['position']
            euler = base_state['orientation_euler']
            print(f"t={current_time:5.2f}s | "
                  f"高度={pos[2]:.3f}m | "
                  f"Roll={euler[0]:6.2f}° | "
                  f"Pitch={euler[1]:6.2f}° | "
                  f"稳定={metrics.is_stable()}")
        
        time.sleep(time_step)
        
        if not metrics.is_stable():
            print("\n❌ 机器人失去平衡!")
            break
    
    # 结果
    print("\n" + "=" * 60)
    print("📊 测试结果")
    print("=" * 60)
    metrics.print_summary()
    
    print("\n按Enter键关闭...")
    input()
    
    env.close()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='站立稳定性测试')
    parser.add_argument('--mode', type=str, default='basic',
                       choices=['basic', 'disturbance'],
                       help='测试模式: basic(基础站立) 或 disturbance(扰动测试)')
    parser.add_argument('--duration', type=float, default=5.0,
                       help='测试时长（秒）')
    
    args = parser.parse_args()
    
    if args.mode == 'basic':
        test_standing_stability(args.duration)
    else:
        test_with_disturbance(args.duration)
