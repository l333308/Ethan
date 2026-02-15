"""
姿态探索工具

使用GUI滑块找到机器人的静态稳定站立姿态
"""

import sys
import time
import numpy as np
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment


def explore_standing_pose():
    """使用GUI探索稳定的站立姿态"""
    
    print("=" * 60)
    print("姿态探索工具")
    print("=" * 60)
    print()
    print("使用滑块调整关节角度，找到稳定的站立姿态")
    print()
    print("提示:")
    print("  1. 保持左右腿对称")
    print("  2. 膝关节适当弯曲(20-40度)可降低重心")
    print("  3. 髋关节和踝关节需要配合保持平衡")
    print("  4. 找到稳定姿态后，记录下关节角度")
    print()
    print("按Ctrl+C退出并保存当前姿态")
    print("=" * 60)
    print()
    
    # 文件路径
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # 创建仿真环境
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    # 添加调试滑块
    debug_params = env.add_debug_parameters()
    
    # 记录最佳姿态
    best_pose = None
    max_duration = 0
    stable_start_time = None
    last_height = None
    
    try:
        while True:
            # 读取滑块值
            joint_values = env.read_debug_parameters(debug_params)
            
            if not joint_values:  # 如果GUI被关闭
                break
            
            # 转换为位置控制命令
            joint_positions = {}
            for joint_name, angle_rad in joint_values.items():
                joint_positions[joint_name] = np.rad2deg(angle_rad)
            
            # 应用关节位置
            env.set_joint_positions(joint_positions)
            
            # 执行仿真步
            env.step()
            
            # 获取当前状态
            base_state = env.get_base_state()
            current_height = base_state['position'][2]
            current_roll, current_pitch, current_yaw = base_state['orientation_euler']
            
            # 检查是否稳定
            is_stable = (
                current_height > 0.15 and
                abs(current_roll) < 15 and
                abs(current_pitch) < 15
            )
            
            if is_stable:
                if stable_start_time is None:
                    stable_start_time = time.time()
                    print(f"\n⏱️  开始计时... 高度={current_height:.3f}m")
                
                stable_duration = time.time() - stable_start_time
                
                # 每5秒显示一次
                if int(stable_duration) % 5 == 0 and int(stable_duration) > max_duration:
                    print(f"   保持稳定 {int(stable_duration)}秒 | "
                          f"高度={current_height:.3f}m | "
                          f"Roll={current_roll:.1f}° | "
                          f"Pitch={current_pitch:.1f}°")
                    
                    # 更新最佳姿态
                    if stable_duration > max_duration:
                        max_duration = int(stable_duration)
                        best_pose = joint_positions.copy()
                        
            else:
                if stable_start_time is not None:
                    duration = time.time() - stable_start_time
                    if duration > 3:  # 只记录超过3秒的稳定期
                        print(f"\n❌ 失去稳定 (保持了{duration:.1f}秒)")
                    stable_start_time = None
            
            # 控制帧率
            time.sleep(1/240)
            
    except KeyboardInterrupt:
        print("\n\n⏹️  用户停止探索")
    except Exception as e:
        print(f"\n⚠️  程序异常: {e}")
    
    # 显示结果
    print("\n" + "=" * 60)
    print("探索结果")
    print("=" * 60)
    
    if best_pose and max_duration > 0:
        print(f"\n🎯 找到稳定姿态！最长保持时间: {max_duration}秒")
        print("\n关节角度配置:")
        print("```python")
        print("standing_pose = {")
        for joint_name, angle in best_pose.items():
            print(f"    '{joint_name}': {angle:.1f},")
        print("}")
        print("```")
        
        # 保存到文件
        output_file = base_path / 'config' / 'standing_pose.txt'
        with open(output_file, 'w') as f:
            f.write("# 稳定站立姿态配置\n")
            f.write(f"# 测试时长: {max_duration}秒\n")
            f.write(f"# 生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write("standing_pose = {\n")
            for joint_name, angle in best_pose.items():
                f.write(f"    '{joint_name}': {angle:.1f},\n")
            f.write("}\n")
        
        print(f"\n✅ 配置已保存到: {output_file}")
        
    else:
        print("\n⚠️ 未找到稳定姿态")
        print("\n建议:")
        print("  - 尝试调整膝关节角度(20-40度)")
        print("  - 髋关节pitch略微前倾(-5到-15度)")
        print("  - 踝关节pitch用于平衡(-5到-15度)")
    
    # 关闭仿真
    try:
        env.close()
    except:
        pass


if __name__ == '__main__':
    explore_standing_pose()
