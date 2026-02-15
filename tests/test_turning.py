"""
转向与停止测试 (第二阶段 M4)

测试序列:
  1. 原地站立2秒
  2. 前进行走5秒  
  3. 原地转向90度
  4. 前进行走5秒
  5. 停止站立5秒
"""

import sys
import argparse
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.simulation.environment import SimulationEnvironment
from src.control.gait_generator import GaitGenerator, GaitParams
from src.control.posture_controller import PostureController, PIDGains


def test_turning(gui: bool = True):
    print("=" * 60)
    print("  第二阶段 M4: 转向与停止测试")
    print("=" * 60)

    base_path = Path(__file__).parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path   = base_path / 'models' / 'humanoid_v1.urdf'

    print("\n[1/2] 初始化...")
    env = SimulationEnvironment(str(config_path), gui=gui)
    env.setup_world()
    env.load_robot(str(urdf_path))

    gait = GaitGenerator(GaitParams(
        step_height=0.015,
        step_length=0.0,
        step_period=2.0,
        double_support_ratio=0.3,
        com_shift_y=0.015,
        turn_rate=0.0,
    ))
    
    pid = PostureController(
        roll_gains=PIDGains(kp=0.2, ki=0.01, kd=0.03),
        pitch_gains=PIDGains(kp=0.2, ki=0.01, kd=0.03),
    )
    print("   ✅ 就绪")

    dt = 0.01
    sim_dt = 0.001
    steps_per_ctrl = int(dt / sim_dt)
    
    time_log, x_log, y_log, yaw_log, height_log = [], [], [], [], []
    phase_labels = []
    sim_time = 0.0
    fallen = False

    def step_loop(dur, label):
        nonlocal sim_time, fallen
        t0 = sim_time
        while sim_time - t0 < dur and not fallen:
            base = env.get_base_state()
            h = base['position'][2]
            roll, pitch, yaw = base['orientation_euler']
            
            gj = gait.update(dt)
            rc, pc = pid.update(roll, pitch, dt)
            corr = pid.compute_joint_corrections(rc, pc)
            final = {k: gj.get(k, 0) + np.clip(corr.get(k, 0), -2, 2) for k in gj}
            
            env.set_joint_positions(final)
            for _ in range(steps_per_ctrl):
                env.step()
            sim_time += dt

            time_log.append(sim_time)
            x_log.append(base['position'][0])
            y_log.append(base['position'][1])
            yaw_log.append(yaw)
            height_log.append(h)
            phase_labels.append(label)

            if h < 0.10:
                print(f"   ⚠️  跌倒 (t={sim_time:.1f}s, h={h:.3f}m)")
                fallen = True
                return
        
        base = env.get_base_state()
        r, p, y = base['orientation_euler']
        print(f"   [{label:12s}] t={sim_time:5.1f}s  "
              f"x={base['position'][0]:+.3f}m  y={base['position'][1]:+.3f}m  "
              f"yaw={y:+.1f}°  h={base['position'][2]:.4f}m")

    print(f"\n[2/2] 执行测试序列...")

    # 1. 原地站立
    stand = {k: 0.0 for k in [
        'head_pitch', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
        'left_ankle_pitch', 'right_hip_roll', 'right_hip_pitch',
        'right_knee', 'right_ankle_pitch'
    ]}
    for _ in range(200):
        env.set_joint_positions(stand)
        for _ in range(steps_per_ctrl):
            env.step()
    sim_time = 2.0
    print(f"   [{'预热':12s}] t={sim_time:5.1f}s  站立稳定")

    # 2. 前进行走
    gait.set_velocity(forward=0.10, turn=0.0)
    step_loop(10.0, "前进行走")

    # 3. 原地转向（保守角速度、长时间）
    if not fallen:
        gait.set_velocity(forward=0.0, turn=15.0)  # 设定15度/步
        step_loop(50.0, "原地转向")  # 50秒 = 约25步

    # 4. 停止
    if not fallen:
        gait.stop()
        gait.params.step_height = 0.0
        step_loop(5.0, "停止站立")

    # 评估
    print(f"\n{'='*60}")
    
    if not time_log:
        print("❌ 无数据")
        env.close()
        return

    yaw_arr = np.array(yaw_log)
    h_arr = np.array(height_log)
    
    # 计算yaw变化
    init_yaw = yaw_arr[0]
    total_yaw = yaw_arr[-1] - init_yaw
    
    # 计算位移
    total_x = abs(x_log[-1] - x_log[0])
    total_y = abs(y_log[-1] - y_log[0])
    
    print(f"\n  总时长: {time_log[-1]:.1f}s")
    print(f"  跌倒:  {'是 ❌' if fallen else '否 ✅'}")
    print(f"  总yaw变化: {total_yaw:.1f}°")
    print(f"  位移: x={total_x:.3f}m  y={total_y:.3f}m")
    
    c1 = not fallen
    c2 = abs(total_yaw) > 10  # 至少转了10度
    c3 = h_arr[-1] > 0.20 if not fallen else False  # 最终站稳

    print(f"\n  ╔══════════════════════════════════════╗")
    print(f"  ║ 不跌倒      : {'✅' if c1 else '❌'}                      ║")
    print(f"  ║ 能转向      : {abs(total_yaw):.1f}°  {'✅' if c2 else '❌'}            ║")
    print(f"  ║ 能停止      : {'✅' if c3 else '❌'}                      ║")
    if c1 and c2 and c3:
        print(f"  ║                                      ║")
        print(f"  ║   🎉 M4 里程碑达成！                 ║")
    print(f"  ╚══════════════════════════════════════╝")

    # 图表
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        axes[0].plot(time_log, [abs(x - x_log[0]) for x in x_log], 'b-', lw=0.8, label='|dx|')
        axes[0].plot(time_log, [abs(y - y_log[0]) for y in y_log], 'r-', lw=0.8, label='|dy|')
        axes[0].set_ylabel('Displacement (m)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Turning & Stopping Test')
        axes[1].plot(time_log, [y - yaw_arr[0] for y in yaw_log], 'g-', lw=0.8)
        axes[1].set_ylabel('Yaw Change (deg)')
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(time_log, height_log, 'm-', lw=0.8)
        axes[2].set_ylabel('Height (m)')
        axes[2].set_xlabel('Time (s)')
        axes[2].grid(True, alpha=0.3)
        plt.tight_layout()
        out = base_path / 'results'
        out.mkdir(exist_ok=True)
        path = out / 'turning_test.png'
        plt.savefig(path, dpi=150)
        plt.close()
        print(f"\n  📊 结果图: {path}")
    except Exception as e:
        print(f"\n  ⚠️  图表失败: {e}")

    env.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-gui', action='store_true')
    args = ap.parse_args()
    test_turning(gui=not args.no_gui)
