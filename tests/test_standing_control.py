"""
站立控制测试 (第二阶段 M1)

验证机器人能否使用PID控制器稳定站立≥30秒
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment
from src.control.posture_controller import StandingController, PIDGains


def test_standing(duration: float = 30.0, gui: bool = True):
    """测试站立控制
    
    Args:
        duration: 测试时长（秒）
        gui: 是否显示GUI
    """
    print("=" * 60)
    print("  第二阶段 M1: 站立控制测试")
    print("=" * 60)
    
    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # ── 1. 初始化仿真 ──
    print("\n[1/4] 初始化仿真环境...")
    env = SimulationEnvironment(str(config_path), gui=gui)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    # ── 2. 初始化控制器 ──
    print("\n[2/4] 初始化站立控制器...")
    controller = StandingController(
        target_height=0.24,   # 诊断得出的稳定高度
        target_roll=0.0,
        target_pitch=0.0
    )
    # 基准姿态：直腿站立（与URDF零位一致）
    controller.set_base_pose({
        'head_pitch': 0.0,
        'left_hip_roll': 0.0,
        'left_hip_pitch': 0.0,
        'left_knee': 0.0,
        'left_ankle_pitch': 0.0,
        'right_hip_roll': 0.0,
        'right_hip_pitch': 0.0,
        'right_knee': 0.0,
        'right_ankle_pitch': 0.0,
    })
    print("   ✅ 控制器就绪 (直腿基准姿态)")
    
    # ── 3. 控制循环 ──
    print(f"\n[3/4] 开始控制循环 (目标: {duration}秒)...")
    
    dt = 0.01          # 控制周期 10ms (100Hz)
    sim_dt = 0.001     # 仿真步长 1ms
    steps_per_ctrl = int(dt / sim_dt)
    
    time_log = []
    height_log = []
    roll_log = []
    pitch_log = []
    
    fallen = False
    sim_time = 0.0
    
    try:
        while sim_time < duration:
            # 获取状态
            base = env.get_base_state()
            h = base['position'][2]
            roll, pitch, yaw = base['orientation_euler']
            
            # 计算控制
            targets = controller.compute_control(h, roll, pitch, dt)
            
            # 发送控制命令
            env.set_joint_positions(targets)
            
            # 步进仿真
            for _ in range(steps_per_ctrl):
                env.step()
            
            sim_time += dt
            
            # 记录
            time_log.append(sim_time)
            height_log.append(h)
            roll_log.append(roll)
            pitch_log.append(pitch)
            
            # 每秒打印
            if len(time_log) % 100 == 0:
                print(f"   t={sim_time:5.1f}s  h={h:.4f}m  "
                      f"roll={roll:+6.1f}°  pitch={pitch:+6.1f}°")
            
            # 跌倒检测
            if h < 0.10:
                print(f"\n   ⚠️  机器人跌倒 (h={h:.3f}m < 0.10m)")
                fallen = True
                break
                
    except KeyboardInterrupt:
        print("\n   ⏹️  用户中断")
    
    actual_duration = time_log[-1] if time_log else 0.0
    
    # ── 4. 评估结果 ──
    print(f"\n[4/4] 评估结果")
    print("=" * 60)
    
    if len(time_log) < 2:
        print("❌ 数据不足，无法评估")
        env.close()
        return
    
    h_arr = np.array(height_log)
    r_arr = np.array(roll_log)
    p_arr = np.array(pitch_log)
    
    h_mean = np.mean(h_arr)
    h_std  = np.std(h_arr)
    r_std  = np.std(r_arr)
    p_std  = np.std(p_arr)
    
    print(f"\n  测试时长:  {actual_duration:.1f}s / {duration:.1f}s")
    print(f"  跌倒:     {'是 ❌' if fallen else '否 ✅'}")
    print(f"\n  高度:  均值={h_mean:.4f}m  标准差={h_std:.4f}m")
    print(f"  Roll:  标准差={r_std:.2f}°")
    print(f"  Pitch: 标准差={p_std:.2f}°")
    
    # M1 成功标准
    print(f"\n  ╔══════════════════════════════════════╗")
    
    c1 = h_std < 0.01
    c2 = max(r_std, p_std) < 3.0
    c3 = actual_duration >= min(duration, 30.0)
    
    print(f"  ║ 重心精度 ±1cm : h_std={h_std*100:.2f}cm  {'✅' if c1 else '❌'} ║")
    print(f"  ║ 姿态角度 ±3°  : max_std={max(r_std,p_std):.2f}°  {'✅' if c2 else '❌'} ║")
    print(f"  ║ 持续 ≥30s     : {actual_duration:.1f}s       {'✅' if c3 else '❌'} ║")
    
    if c1 and c2 and c3:
        print(f"  ║                                      ║")
        print(f"  ║   🎉 M1 里程碑达成！                 ║")
    
    print(f"  ╚══════════════════════════════════════╝")
    
    # 保存结果图表
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        axes[0].plot(time_log, height_log, 'b-', linewidth=0.8)
        axes[0].axhline(0.24, color='r', ls='--', alpha=0.5, label='目标')
        axes[0].set_ylabel('高度 (m)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f'站立控制测试  (时长={actual_duration:.1f}s)')
        
        axes[1].plot(time_log, roll_log, 'g-', linewidth=0.8)
        axes[1].axhline(0, color='r', ls='--', alpha=0.5)
        axes[1].set_ylabel('Roll (°)')
        axes[1].grid(True, alpha=0.3)
        
        axes[2].plot(time_log, pitch_log, 'm-', linewidth=0.8)
        axes[2].axhline(0, color='r', ls='--', alpha=0.5)
        axes[2].set_ylabel('Pitch (°)')
        axes[2].set_xlabel('时间 (s)')
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        out = base_path / 'results'
        out.mkdir(exist_ok=True)
        path = out / 'standing_control.png'
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"\n  📊 结果图已保存: {path}")
    except Exception as e:
        print(f"\n  ⚠️  图表保存失败: {e}")
    
    env.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--no-gui', action='store_true')
    args = ap.parse_args()
    test_standing(duration=args.duration, gui=not args.no_gui)
