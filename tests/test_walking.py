"""
行走测试 (第二阶段 M3)

验证机器人能否直线前进1米。
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment
from src.control.gait_generator import GaitGenerator, GaitParams
from src.control.posture_controller import PostureController, PIDGains


def test_walking(duration: float = 60.0, gui: bool = True):
    print("=" * 60)
    print("  第二阶段 M3: 直线行走测试")
    print("=" * 60)

    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path   = base_path / 'models' / 'humanoid_v1.urdf'

    print("\n[1/4] 初始化仿真环境...")
    env = SimulationEnvironment(str(config_path), gui=gui)
    env.setup_world()
    env.load_robot(str(urdf_path))

    print("\n[2/4] 初始化控制器...")
    gait = GaitGenerator(GaitParams(
        step_height=0.015,
        step_length=0.10,          # 10cm步长（更大以加速）
        step_period=2.0,
        double_support_ratio=0.3,
        com_shift_y=0.015,
        foot_x_offset=0.0,
        foot_z_stand=-0.22,
    ))
    
    posture_pid = PostureController(
        roll_gains=PIDGains(kp=0.2, ki=0.01, kd=0.03),
        pitch_gains=PIDGains(kp=0.2, ki=0.01, kd=0.03),
    )
    
    print(f"   步长={gait.params.step_length*100:.0f}cm, "
          f"周期={gait.params.step_period}s, "
          f"预计速度={gait.params.step_length/gait.params.step_period*100:.1f}cm/s")
    print("   ✅ 控制器就绪")

    # 预热
    dt = 0.01
    sim_dt = 0.001
    steps_per_ctrl = int(dt / sim_dt)
    
    print("\n   预热: 稳定站立2秒...")
    stand_pose = {k: 0.0 for k in [
        'head_pitch', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
        'left_ankle_pitch', 'right_hip_roll', 'right_hip_pitch',
        'right_knee', 'right_ankle_pitch'
    ]}
    for _ in range(200):
        env.set_joint_positions(stand_pose)
        for _ in range(steps_per_ctrl):
            env.step()

    # 记录初始位置
    init_base = env.get_base_state()
    init_x = init_base['position'][0]
    print(f"   初始位置: x={init_x:.4f}m")

    print(f"\n[3/4] 开始行走 (目标: 1米)...")
    
    time_log, x_log, height_log, roll_log, pitch_log = [], [], [], [], []
    fallen = False
    sim_time = 0.0

    try:
        while sim_time < duration:
            base = env.get_base_state()
            h = base['position'][2]
            x = base['position'][0]
            roll, pitch, yaw = base['orientation_euler']

            gait_joints = gait.update(dt)
            
            rc, pc = posture_pid.update(roll, pitch, dt)
            pid_corr = posture_pid.compute_joint_corrections(rc, pc)

            final = {}
            for jname, gval in gait_joints.items():
                corr = np.clip(pid_corr.get(jname, 0.0), -2.0, 2.0)
                final[jname] = gval + corr

            env.set_joint_positions(final)
            for _ in range(steps_per_ctrl):
                env.step()

            sim_time += dt

            time_log.append(sim_time)
            x_log.append(x)
            height_log.append(h)
            roll_log.append(roll)
            pitch_log.append(pitch)

            dist = abs(x - init_x)
            if len(time_log) % 100 == 0:
                print(f"   t={sim_time:5.1f}s  dist={dist:.3f}m  h={h:.4f}m  "
                      f"roll={roll:+5.1f}°  pitch={pitch:+5.1f}°  "
                      f"[{gait.phase_name}]  steps={gait.step_count}")

            if h < 0.10:
                print(f"\n   ⚠️  跌倒 (h={h:.3f}m)")
                fallen = True
                break

            if dist >= 1.0:
                print(f"\n   🎉 已行走 {dist:.3f}m, 到达目标!")
                break

    except KeyboardInterrupt:
        print("\n   ⏹️  用户中断")

    actual_duration = time_log[-1] if time_log else 0.0
    total_dist = abs(x_log[-1] - init_x) if x_log else 0.0

    print(f"\n[4/4] 评估结果")
    print("=" * 60)
    
    h_arr = np.array(height_log)
    r_arr = np.array(roll_log)
    p_arr = np.array(pitch_log)

    print(f"\n  测试时长:  {actual_duration:.1f}s")
    print(f"  行走距离: {total_dist:.3f}m")
    print(f"  跌倒:     {'是 ❌' if fallen else '否 ✅'}")
    print(f"  总步数:   {gait.step_count}")
    if actual_duration > 0:
        print(f"  平均速度: {total_dist/actual_duration*100:.1f}cm/s")
    print(f"  高度: 均值={np.mean(h_arr):.4f}m  std={np.std(h_arr):.4f}m")
    print(f"  Roll:  max={np.max(np.abs(r_arr)):.2f}°")
    print(f"  Pitch: max={np.max(np.abs(p_arr)):.2f}°")

    c1 = total_dist >= 1.0
    c2 = not fallen
    c3 = gait.step_count >= 4

    print(f"\n  ╔══════════════════════════════════════╗")
    print(f"  ║ 行走≥1m     : {total_dist:.2f}m  {'✅' if c1 else '❌'}             ║")
    print(f"  ║ 不跌倒      : {'✅' if c2 else '❌'}                      ║")
    print(f"  ║ 持续步态    : {gait.step_count}步  {'✅' if c3 else '❌'}               ║")
    if c1 and c2 and c3:
        print(f"  ║                                      ║")
        print(f"  ║   🎉 M3 里程碑达成！                 ║")
    print(f"  ╚══════════════════════════════════════╝")

    # 图表
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        axes[0].plot(time_log, [xi - init_x for xi in x_log], 'b-', lw=0.8)
        axes[0].axhline(1.0, color='r', ls='--', alpha=0.5, label='Target 1m')
        axes[0].set_ylabel('Distance (m)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f'Walking Test ({total_dist:.2f}m in {actual_duration:.1f}s)')
        axes[1].plot(time_log, height_log, 'g-', lw=0.8)
        axes[1].set_ylabel('Height (m)')
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(time_log, roll_log, 'c-', lw=0.8, label='Roll')
        axes[2].plot(time_log, pitch_log, 'm-', lw=0.8, label='Pitch')
        axes[2].set_ylabel('Angle (deg)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        axes[3].set_xlabel('Time (s)')
        plt.tight_layout()
        out = base_path / 'results'
        out.mkdir(exist_ok=True)
        path = out / 'walking_test.png'
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"\n  📊 结果图: {path}")
    except Exception as e:
        print(f"\n  ⚠️  图表失败: {e}")

    env.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--no-gui', action='store_true')
    args = ap.parse_args()
    test_walking(duration=args.duration, gui=not args.no_gui)
