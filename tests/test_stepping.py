"""
原地踏步测试 (第二阶段 M2)

策略: 步态生成器输出基准轨迹 + 姿态PID补偿叠加
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


def test_stepping(duration: float = 20.0, gui: bool = True):
    print("=" * 60)
    print("  第二阶段 M2: 原地踏步测试")
    print("=" * 60)

    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path   = base_path / 'models' / 'humanoid_v1.urdf'

    # ── 1. 仿真初始化 ──
    print("\n[1/4] 初始化仿真环境...")
    env = SimulationEnvironment(str(config_path), gui=gui)
    env.setup_world()
    env.load_robot(str(urdf_path))

    # ── 2. 控制器初始化 ──
    print("\n[2/4] 初始化控制器...")
    
    # 步态生成器（非常保守的参数）
    gait = GaitGenerator(GaitParams(
        step_height=0.015,         # 只抬1.5cm
        step_length=0.0,           # 原地踏步
        step_period=2.5,           # 2.5秒一步（很慢）
        double_support_ratio=0.4,  # 40%双足支撑（更多时间稳定）
        com_shift_y=0.01,          # 重心侧移1cm（轻微）
        foot_x_offset=0.0,
        foot_z_stand=-0.22,
    ))
    
    # 姿态补偿PID（叠加在步态输出之上）
    posture_pid = PostureController(
        roll_gains=PIDGains(kp=0.2, ki=0.01, kd=0.03),
        pitch_gains=PIDGains(kp=0.2, ki=0.01, kd=0.03),
    )
    
    print(f"   步态: 抬脚={gait.params.step_height*100:.1f}cm, "
          f"周期={gait.params.step_period}s")
    print("   ✅ 控制器就绪")

    # ── 预热: 先稳定站立2秒 ──
    print("\n   预热: 稳定站立2秒...")
    dt = 0.01
    sim_dt = 0.001
    steps_per_ctrl = int(dt / sim_dt)
    
    for _ in range(200):  # 2秒 @100Hz
        env.set_joint_positions({
            'head_pitch': 0, 'left_hip_roll': 0, 'left_hip_pitch': 0,
            'left_knee': 0, 'left_ankle_pitch': 0, 'right_hip_roll': 0,
            'right_hip_pitch': 0, 'right_knee': 0, 'right_ankle_pitch': 0,
        })
        for _ in range(steps_per_ctrl):
            env.step()
    
    base = env.get_base_state()
    print(f"   预热完成: h={base['position'][2]:.4f}m")

    # ── 3. 控制循环 ──
    print(f"\n[3/4] 开始原地踏步 (目标: {duration}秒)...")
    
    time_log, height_log, roll_log, pitch_log = [], [], [], []
    phase_log, step_count_log = [], []

    fallen = False
    sim_time = 0.0

    try:
        while sim_time < duration:
            # 读取状态
            base = env.get_base_state()
            h = base['position'][2]
            roll, pitch, yaw = base['orientation_euler']

            # (a) 步态生成 → 基准关节角度
            gait_joints = gait.update(dt)

            # (b) 姿态PID → 补偿偏移
            rc, pc = posture_pid.update(roll, pitch, dt)
            pid_corr = posture_pid.compute_joint_corrections(rc, pc)

            # (c) 合并: 步态 + PID补偿（限幅）
            final = {}
            for jname, gait_val in gait_joints.items():
                corr = pid_corr.get(jname, 0.0)
                corr = np.clip(corr, -2.0, 2.0)  # 每步最多补偿2°
                final[jname] = gait_val + corr

            # 发送
            env.set_joint_positions(final)

            # 仿真步进
            for _ in range(steps_per_ctrl):
                env.step()

            sim_time += dt

            # 记录
            time_log.append(sim_time)
            height_log.append(h)
            roll_log.append(roll)
            pitch_log.append(pitch)
            phase_log.append(gait.phase.value)
            step_count_log.append(gait.step_count)

            if len(time_log) % 100 == 0:
                print(f"   t={sim_time:5.1f}s  h={h:.4f}m  "
                      f"roll={roll:+6.1f}°  pitch={pitch:+6.1f}°  "
                      f"[{gait.phase_name}]  steps={gait.step_count}")

            if h < 0.10:
                print(f"\n   ⚠️  机器人跌倒 (h={h:.3f}m)")
                fallen = True
                break

    except KeyboardInterrupt:
        print("\n   ⏹️  用户中断")

    actual_duration = time_log[-1] if time_log else 0.0
    total_steps = gait.step_count

    # ── 4. 评估 ──
    print(f"\n[4/4] 评估结果")
    print("=" * 60)
    
    h_arr = np.array(height_log)
    r_arr = np.array(roll_log)
    p_arr = np.array(pitch_log)

    print(f"\n  测试时长:  {actual_duration:.1f}s")
    print(f"  跌倒:     {'是 ❌' if fallen else '否 ✅'}")
    print(f"  总步数:   {total_steps}")
    print(f"  高度:  均值={np.mean(h_arr):.4f}m  标准差={np.std(h_arr):.4f}m")
    print(f"  Roll:  std={np.std(r_arr):.2f}°  max={np.max(np.abs(r_arr)):.2f}°")
    print(f"  Pitch: std={np.std(p_arr):.2f}°  max={np.max(np.abs(p_arr)):.2f}°")

    c1 = not fallen and actual_duration >= min(duration, 10.0)
    c2 = total_steps >= 4
    c3 = np.max(np.abs(r_arr)) < 30

    print(f"\n  ╔══════════════════════════════════════╗")
    print(f"  ║ 保持不倒    : {'✅' if c1 else '❌'}                      ║")
    print(f"  ║ 左右脚交替  : {total_steps}步  {'✅' if c2 else '❌'}                ║")
    print(f"  ║ 姿态可控    : max_roll={np.max(np.abs(r_arr)):.1f}°  {'✅' if c3 else '❌'}  ║")
    if c1 and c2 and c3:
        print(f"  ║                                      ║")
        print(f"  ║   🎉 M2 里程碑达成！                 ║")
    print(f"  ╚══════════════════════════════════════╝")

    # 图表
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        axes[0].plot(time_log, height_log, 'b-', lw=0.8)
        axes[0].set_ylabel('Height (m)')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f'Stepping Test  ({actual_duration:.1f}s, {total_steps} steps)')
        axes[1].plot(time_log, roll_log, 'g-', lw=0.8)
        axes[1].set_ylabel('Roll (deg)')
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(time_log, pitch_log, 'm-', lw=0.8)
        axes[2].set_ylabel('Pitch (deg)')
        axes[2].grid(True, alpha=0.3)
        axes[3].plot(time_log, phase_log, 'k-', lw=1.0)
        axes[3].set_ylabel('Phase')
        axes[3].set_xlabel('Time (s)')
        axes[3].set_yticks([1, 2, 3])
        axes[3].set_yticklabels(['Double', 'L-Swing', 'R-Swing'])
        axes[3].grid(True, alpha=0.3)
        plt.tight_layout()
        out = base_path / 'results'
        out.mkdir(exist_ok=True)
        path = out / 'stepping_test.png'
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"\n  📊 结果图: {path}")
    except Exception as e:
        print(f"\n  ⚠️  图表失败: {e}")

    env.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=20.0)
    ap.add_argument('--no-gui', action='store_true')
    args = ap.parse_args()
    test_stepping(duration=args.duration, gui=not args.no_gui)
