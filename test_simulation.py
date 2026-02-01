"""
快速仿真测试（3秒自动退出）

验证PyBullet安装和基本仿真功能
"""

import sys
import time
import signal
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent
sys.path.append(str(project_root))

from src.simulation.environment import SimulationEnvironment


def timeout_handler(signum, frame):
    """超时处理"""
    raise TimeoutError("测试超时")


def main():
    """主函数"""
    print("=" * 60)
    print("快速仿真测试（3秒后自动退出）")
    print("=" * 60)
    
    # 文件路径
    config_path = project_root / 'config' / 'robot_config.yaml'
    urdf_path = project_root / 'models' / 'humanoid_v1.urdf'
    
    try:
        # 创建仿真环境（无GUI模式）
        print("\n1. 创建仿真环境（无GUI）...")
        env = SimulationEnvironment(str(config_path), gui=False)
        print("   ✅ 环境创建成功")
        
        # 设置世界
        print("\n2. 设置物理世界...")
        env.setup_world()
        print("   ✅ 世界设置完成")
        
        # 加载机器人
        print("\n3. 加载机器人模型...")
        env.load_robot(str(urdf_path))
        print(f"   ✅ 机器人已加载")
        print(f"   - 机器人ID: {env.robot_id}")
        print(f"   - 关节数: {len(env.joint_names)}")
        
        # 设置姿态
        print("\n4. 设置初始姿态...")
        initial_pose = {
            'left_knee': 20,
            'right_knee': 20,
            'left_hip_pitch': 10,
            'right_hip_pitch': 10,
        }
        env.set_joint_positions(initial_pose)
        print("   ✅ 姿态设置完成")
        
        # 运行仿真
        print("\n5. 运行仿真（3秒）...")
        start_time = time.time()
        steps = 0
        
        while time.time() - start_time < 3.0:
            env.step()
            steps += 1
            
            # 每秒打印一次状态
            if steps % 1000 == 0:
                base_state = env.get_base_state()
                pos = base_state['position']
                print(f"   t={time.time()-start_time:.1f}s | 高度={pos[2]:.3f}m | 步数={steps}")
        
        print(f"   ✅ 仿真完成 (总步数: {steps})")
        
        # 获取最终状态
        print("\n6. 最终状态:")
        base_state = env.get_base_state()
        pos = base_state['position']
        euler = base_state['orientation_euler']
        print(f"   - 位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        print(f"   - 姿态: Roll={euler[0]:.2f}° Pitch={euler[1]:.2f}° Yaw={euler[2]:.2f}°")
        
        # 关闭
        print("\n7. 关闭仿真...")
        env.close()
        print("   ✅ 仿真已关闭")
        
        print("\n" + "=" * 60)
        print("🎉 所有测试通过！")
        print("=" * 60)
        print("\n✅ PyBullet 安装正常")
        print("✅ 仿真环境可用")
        print("✅ 机器人模型加载正常")
        print("✅ 物理引擎运行正常")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
