"""
简化版站立控制测试

使用固定姿态和简单的姿态补偿来实现站立
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment


def test_simple_standing(duration: float = 30.0, gui: bool = True):
    """测试简单的站立（固定姿态 + 轻微补偿）
    
    Args:
        duration: 测试时长（秒）
        gui: 是否显示GUI
    """
    print("=" * 60)
    print("简化版站立测试")
    print("=" * 60)
    print(f"测试时长: {duration}秒")
    print(f"GUI模式: {'开启' if gui else '关闭'}")
    print()
    
    # 文件路径
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # 创建仿真环境
    print("1. 初始化仿真环境...")
    env = SimulationEnvironment(str(config_path), gui=gui)
    env.setup_world()
    env.load_robot(str(urdf_path))
    print()
    
    # 设置站立姿态（尝试不同的姿态）
    print("2. 设置站立姿态...")
    standing_pose = {
        'left_hip_roll': 0.0,
        'left_hip_pitch': -10.0,  # 髋关节轻微前倾
        'left_knee': 20.0,  # 膝关节弯曲20度
        'left_ankle_pitch': -10.0,  # 踝关节补偿
        'right_hip_roll': 0.0,
        'right_hip_pitch': -10.0,
        'right_knee': 20.0,
        'right_ankle_pitch': -10.0,
        'head_pitch': 0.0
    }
    
    # 用力让机器人保持姿态
    env.set_joint_positions(standing_pose)
    
    # 稳定一会
    print("   稳定初始姿态...")
    for _ in range(1000):
        env.set_joint_positions(standing_pose)
        env.step()
    
    print("   ✅ 初始姿态已设置")
    print()
    
    # 记录数据
    time_history = []
    height_history = []
    roll_history = []
    pitch_history = []
    
    print("3. 开始测试...")
    print()
    
    try:
        start_time = time.time()
        step_count = 0
        
        while time.time() - start_time < duration:
            # 获取当前状态
            base_state = env.get_base_state()
            current_height = base_state['position'][2]
            current_roll, current_pitch, current_yaw = base_state['orientation_euler']
            
            # 简单的姿态补偿
            compensation = standing_pose.copy()
            
            # 根据Roll角度微调hip_roll
            roll_correction = -current_roll * 0.1  # 很小的反馈增益
            compensation['left_hip_roll'] = roll_correction
            compensation['right_hip_roll'] = -roll_correction
            
            # 根据Pitch角度微调hip_pitch和ankle_pitch
            pitch_correction = -current_pitch * 0.1
            compensation['left_hip_pitch'] += pitch_correction
            compensation['right_hip_pitch'] += pitch_correction
            compensation['left_ankle_pitch'] -= pitch_correction * 0.5
            compensation['right_ankle_pitch'] -= pitch_correction * 0.5
            
            # 应用补偿后的姿态
            env.set_joint_positions(compensation)
            
            # 执行仿真步
            env.step()
            step_count += 1
            
            # 记录数据
            elapsed_time = time.time() - start_time
            time_history.append(elapsed_time)
            height_history.append(current_height)
            roll_history.append(current_roll)
            pitch_history.append(current_pitch)
            
            # 每秒打印一次状态
            if step_count % 1000 == 0:
                print(f"   t={elapsed_time:.1f}s | "
                      f"高度={current_height:.3f}m | "
                      f"Roll={current_roll:.1f}° | "
                      f"Pitch={current_pitch:.1f}°")
            
            # 检查是否跌倒
            if current_height < 0.10:
                print("\n⚠️ 机器人跌倒！测试终止")
                break
            
            # 短暂延迟以避免过快
            time.sleep(0.001)
                
    except KeyboardInterrupt:
        print("\n⏹️ 用户手动停止测试")
    
    # 计算并显示结果
    print("\n" + "=" * 60)
    print("测试结果")
    print("=" * 60)
    
    if len(time_history) > 0:
        height_std = np.std(height_history)
        roll_std = np.std(roll_history)
        pitch_std = np.std(pitch_history)
        test_duration = time_history[-1]
        
        print(f"\n详细指标:")
        print(f"  - 高度标准差: {height_std:.4f}m")
        print(f"  - Roll标准差: {roll_std:.2f}°")
        print(f"  - Pitch标准差: {pitch_std:.2f}°")
        print(f"  - 平均高度: {np.mean(height_history):.3f}m")
        print(f"  - 测试时长: {test_duration:.1f}s")
        
        # 判断成功标准
        print(f"\nM1成功标准检查:")
        
        checks = [
            ("重心控制精度 ±1cm", height_std < 0.01),
            ("姿态角度 ±3°", max(roll_std, pitch_std) < 3.0),
            ("持续时间 ≥30秒", test_duration >= 30.0),
        ]
        
        for check_name, passed in checks:
            status = "✅" if passed else "❌"
            print(f"  {status} {check_name}")
        
        all_passed = all(check[1] for check in checks)
        
        if all_passed:
            print("\n🎉 恭喜！M1（稳定站立）测试通过！")
        else:
            print("\n⚠️ M1测试未完全通过")
            
            # 给出建议
            print("\n💡 调优建议:")
            if height_std >= 0.01:
                print("  - 调整膝关节和髋关节角度以获得更稳定的高度")
            if max(roll_std, pitch_std) >= 3.0:
                print("  - 增加反馈增益或调整基准姿态")
            if test_duration < 30.0:
                print("  - 机器人过早跌倒，需要调整基准姿态")
    
    # 关闭仿真
    env.close()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='简化版站立测试')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='测试时长（秒）')
    parser.add_argument('--no-gui', action='store_true',
                       help='不显示GUI')
    
    args = parser.parse_args()
    
    test_simple_standing(
        duration=args.duration,
        gui=not args.no_gui
    )


if __name__ == '__main__':
    main()
