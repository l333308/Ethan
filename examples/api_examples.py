"""
API使用示例

演示如何使用项目提供的各种API
"""

import sys
import time
import numpy as np
from pathlib import Path

# 添加项目路径
sys.path.append(str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment
from src.utils.metrics import StabilityMetrics
from src.utils.visualization import Plotter


def example_1_basic_simulation():
    """示例1: 基础仿真"""
    print("\n" + "="*60)
    print("示例1: 基础仿真")
    print("="*60)
    
    # 获取路径
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # 创建环境
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    # 运行2秒仿真
    print("\n运行2秒仿真...")
    env.run(duration=2.0, real_time=True)
    
    env.close()
    print("✅ 示例1完成\n")


def example_2_joint_control():
    """示例2: 关节控制"""
    print("\n" + "="*60)
    print("示例2: 关节控制")
    print("="*60)
    
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    print("\n执行关节动作序列...")
    
    # 动作序列
    actions = [
        {"name": "站直", "angles": {}},
        {"name": "微屈", "angles": {
            "left_knee": 20, "right_knee": 20,
            "left_hip_pitch": 10, "right_hip_pitch": 10
        }},
        {"name": "深蹲", "angles": {
            "left_knee": 40, "right_knee": 40,
            "left_hip_pitch": 20, "right_hip_pitch": 20,
            "left_ankle_pitch": -10, "right_ankle_pitch": -10
        }},
    ]
    
    for action in actions:
        print(f"\n▶ {action['name']}")
        env.set_joint_positions(action['angles'])
        
        # 执行1秒
        for _ in range(1000):
            env.step()
            time.sleep(0.001)
    
    env.close()
    print("\n✅ 示例2完成\n")


def example_3_state_monitoring():
    """示例3: 状态监测"""
    print("\n" + "="*60)
    print("示例3: 状态监测")
    print("="*60)
    
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    print("\n监测2秒机器人状态...")
    
    for i in range(200):  # 2秒 @ 100Hz
        env.step()
        
        # 每0.5秒打印一次
        if i % 50 == 0:
            # 获取基座状态
            base_state = env.get_base_state()
            pos = base_state['position']
            euler = base_state['orientation_euler']
            
            # 获取IMU数据
            imu = env.get_imu_data()
            
            print(f"\nt={i*0.01:.2f}s:")
            print(f"  位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            print(f"  姿态: Roll={euler[0]:.2f}° Pitch={euler[1]:.2f}° Yaw={euler[2]:.2f}°")
            print(f"  IMU加速度: {imu['linear_acceleration']}")
        
        time.sleep(0.01)
    
    env.close()
    print("\n✅ 示例3完成\n")


def example_4_stability_evaluation():
    """示例4: 稳定性评估"""
    print("\n" + "="*60)
    print("示例4: 稳定性评估")
    print("="*60)
    
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    # 创建评估器
    metrics = StabilityMetrics()
    
    print("\n收集3秒数据...")
    
    for i in range(300):  # 3秒
        env.step()
        
        # 获取状态
        base_state = env.get_base_state()
        joint_states = env.get_joint_states()
        imu_data = env.get_imu_data()
        
        # 更新指标
        metrics.update(base_state, joint_states, imu_data)
        
        time.sleep(0.01)
    
    # 打印评估结果
    print("\n" + "="*60)
    print("评估结果:")
    print("="*60)
    metrics.print_summary()
    
    env.close()
    print("\n✅ 示例4完成\n")


def example_5_custom_test():
    """示例5: 自定义测试"""
    print("\n" + "="*60)
    print("示例5: 自定义测试 - 摆动测试")
    print("="*60)
    
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    print("\n让机器人摆动膝关节...")
    
    # 摆动参数
    duration = 5.0  # 5秒
    frequency = 0.5  # 0.5 Hz
    amplitude = 15  # 15度
    
    time_step = env.config['simulation']['physics']['time_step']
    steps = int(duration / time_step)
    
    for i in range(steps):
        t = i * time_step
        
        # 正弦波摆动
        angle = amplitude * np.sin(2 * np.pi * frequency * t)
        
        # 应用到两个膝盖
        env.set_joint_positions({
            'left_knee': angle,
            'right_knee': angle
        })
        
        env.step()
        
        # 每秒打印一次
        if i % int(1.0 / time_step) == 0:
            print(f"t={t:.2f}s | 膝关节角度={angle:.2f}°")
        
        time.sleep(time_step)
    
    env.close()
    print("\n✅ 示例5完成\n")


def main():
    """主菜单"""
    print("\n" + "="*60)
    print("🤖 API使用示例")
    print("="*60)
    print("\n请选择要运行的示例:")
    print("  1. 基础仿真")
    print("  2. 关节控制")
    print("  3. 状态监测")
    print("  4. 稳定性评估")
    print("  5. 自定义测试")
    print("  0. 全部运行")
    
    choice = input("\n请输入选择 (0-5): ").strip()
    
    examples = {
        '1': example_1_basic_simulation,
        '2': example_2_joint_control,
        '3': example_3_state_monitoring,
        '4': example_4_stability_evaluation,
        '5': example_5_custom_test,
    }
    
    if choice == '0':
        for func in examples.values():
            func()
    elif choice in examples:
        examples[choice]()
    else:
        print("\n❌ 无效选择")
        return
    
    print("\n" + "="*60)
    print("🎉 示例演示完成!")
    print("="*60)
    print("\n💡 提示:")
    print("  - 查看源代码: examples/api_examples.py")
    print("  - 阅读API文档: docs/architecture.md")
    print("  - 创建你自己的脚本!")


if __name__ == '__main__':
    main()
