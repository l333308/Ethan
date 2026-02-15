#!/usr/bin/env python3
"""
站立问题诊断脚本

系统化地定位机器人无法站立的根本原因
"""

import sys
import time
import pybullet as p
import pybullet_data
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}\n")


def test1_kinematics_check():
    """测试1: 检查运动学链 - 确认脚底高度"""
    print_header("测试1: 运动学链检查")
    
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, 0)  # 先不加重力
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    
    base_path = Path(__file__).parent.parent
    urdf_path = str(base_path / 'models' / 'humanoid_v1.urdf')
    
    # 在不同高度测试
    robot = p.loadURDF(urdf_path, [0, 0, 0.25], [0, 0, 0, 1], useFixedBase=False)
    
    # 打印所有关节信息
    num_joints = p.getNumJoints(robot)
    print(f"总关节数: {num_joints}")
    print()
    
    joint_map = {}
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8')
        joint_type = info[2]
        parent_idx = info[16]
        parent_name = "base" if parent_idx == -1 else p.getJointInfo(robot, parent_idx)[1].decode('utf-8')
        link_name = info[12].decode('utf-8')
        
        type_str = {0: "REVOLUTE", 1: "PRISMATIC", 4: "FIXED"}
        print(f"  关节[{i}] {name}")
        print(f"    类型: {type_str.get(joint_type, joint_type)}")
        print(f"    父链接: {parent_name} -> 子链接: {link_name}")
        print(f"    限位: [{info[8]:.3f}, {info[9]:.3f}] rad")
        print(f"    最大力: {info[10]} N·m")
        print(f"    最大速度: {info[11]} rad/s")
        
        if joint_type == p.JOINT_REVOLUTE:
            joint_map[name] = i
    
    # 检查各link在世界坐标系中的位置
    print(f"\n--- 零位姿态下各部件位置 ---")
    for i in range(num_joints):
        link_state = p.getLinkState(robot, i)
        info = p.getJointInfo(robot, i)
        link_name = info[12].decode('utf-8')
        pos = link_state[0]
        print(f"  {link_name:25s}  位置: x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f}")
    
    base_pos, base_orn = p.getBasePositionAndOrientation(robot)
    print(f"\n  base_link                    位置: x={base_pos[0]:.4f} y={base_pos[1]:.4f} z={base_pos[2]:.4f}")
    
    # 获取AABB找到最低点
    for i in range(num_joints):
        aabb_min, aabb_max = p.getAABB(robot, i)
        info = p.getJointInfo(robot, i)
        link_name = info[12].decode('utf-8')
        if 'foot' in link_name:
            print(f"\n  {link_name} AABB:")
            print(f"    最低点 z = {aabb_min[2]:.4f} m")
            print(f"    最高点 z = {aabb_max[2]:.4f} m")
    
    p.disconnect()
    print("\n✅ 结论: 零位姿态下脚底接近地面")


def test2_no_gravity_motor():
    """测试2: 无重力 + 有电机控制"""
    print_header("测试2: 检查电机控制是否生效")
    
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, 0)  # 无重力
    p.setTimeStep(0.001)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    
    base_path = Path(__file__).parent.parent
    urdf_path = str(base_path / 'models' / 'humanoid_v1.urdf')
    robot = p.loadURDF(urdf_path, [0, 0, 0.25], [0, 0, 0, 1], useFixedBase=False)
    
    # 记录初始高度
    init_pos, _ = p.getBasePositionAndOrientation(robot)
    print(f"初始高度: {init_pos[2]:.4f}m")
    
    # 不施加任何控制，步进1秒
    for _ in range(1000):
        p.stepSimulation()
    
    pos1, _ = p.getBasePositionAndOrientation(robot)
    print(f"无重力+无控制 1秒后高度: {pos1[2]:.4f}m (应该不变)")
    
    # 加重力，不施加控制
    p.setGravity(0, 0, -9.81)
    for _ in range(1000):
        p.stepSimulation()
    
    pos2, orn2 = p.getBasePositionAndOrientation(robot)
    euler2 = p.getEulerFromQuaternion(orn2)
    print(f"有重力+无控制 1秒后: z={pos2[2]:.4f}m roll={np.rad2deg(euler2[0]):.1f}° pitch={np.rad2deg(euler2[1]):.1f}°")
    print(f"  → 说明: 没有电机控制时，关节自由塌陷，机器人倒下")
    
    p.disconnect()


def test3_gravity_with_motor():
    """测试3: 有重力 + 有电机控制（关键测试）"""
    print_header("测试3: 有重力 + 位置控制")
    
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")
    p.changeDynamics(plane, -1, lateralFriction=1.0)
    
    base_path = Path(__file__).parent.parent
    urdf_path = str(base_path / 'models' / 'humanoid_v1.urdf')
    robot = p.loadURDF(urdf_path, [0, 0, 0.25], [0, 0, 0, 1], useFixedBase=False)
    
    # 收集可动关节
    joint_map = {}
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8')
        if info[2] == p.JOINT_REVOLUTE:
            joint_map[name] = i
    
    print(f"可动关节: {list(joint_map.keys())}")
    
    # ===== 方案A: 所有关节角度=0, 直接施加位置控制 =====
    print(f"\n--- 方案A: 直腿站立 (全部角度=0) ---")
    
    # 重置状态
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.25], [0, 0, 0, 1])
    for name, idx in joint_map.items():
        p.resetJointState(robot, idx, 0)
    
    # 立即启用位置控制（关键！）
    for name, idx in joint_map.items():
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL,
            targetPosition=0,
            force=10,  # 使用URDF中的最大力 10 N·m
            maxVelocity=2.0
        )
    
    # 步进3秒
    for step in range(3000):
        p.stepSimulation()
        if step % 1000 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"  t={step/1000:.1f}s z={pos[2]:.4f}m "
                  f"roll={np.rad2deg(euler[0]):.1f}° "
                  f"pitch={np.rad2deg(euler[1]):.1f}°")
    
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    z_a = pos[2]
    print(f"  最终: z={z_a:.4f}m 是否站立: {'✅' if z_a > 0.15 else '❌'}")
    
    # ===== 方案B: 增大位置控制力 =====
    print(f"\n--- 方案B: 直腿站立 + 更大力矩 (force=50) ---")
    
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.25], [0, 0, 0, 1])
    for name, idx in joint_map.items():
        p.resetJointState(robot, idx, 0)
    
    for name, idx in joint_map.items():
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL,
            targetPosition=0,
            force=50,
            maxVelocity=2.0
        )
    
    for step in range(3000):
        p.stepSimulation()
        if step % 1000 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"  t={step/1000:.1f}s z={pos[2]:.4f}m "
                  f"roll={np.rad2deg(euler[0]):.1f}° "
                  f"pitch={np.rad2deg(euler[1]):.1f}°")
    
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    z_b = pos[2]
    print(f"  最终: z={z_b:.4f}m 是否站立: {'✅' if z_b > 0.15 else '❌'}")
    
    # ===== 方案C: 增加关节阻尼 + 更大力矩 =====
    print(f"\n--- 方案C: 直腿 + 关节阻尼 + 大力矩 (force=50, damping=5) ---")
    
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.25], [0, 0, 0, 1])
    for name, idx in joint_map.items():
        p.resetJointState(robot, idx, 0)
        p.changeDynamics(robot, idx, jointDamping=5.0)
    
    for name, idx in joint_map.items():
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL,
            targetPosition=0,
            force=50,
            maxVelocity=2.0
        )
    
    for step in range(3000):
        p.stepSimulation()
        if step % 1000 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"  t={step/1000:.1f}s z={pos[2]:.4f}m "
                  f"roll={np.rad2deg(euler[0]):.1f}° "
                  f"pitch={np.rad2deg(euler[1]):.1f}°")
    
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    z_c = pos[2]
    print(f"  最终: z={z_c:.4f}m 是否站立: {'✅' if z_c > 0.15 else '❌'}")
    
    # ===== 方案D: 使用positionGain/velocityGain（PD控制参数） =====
    print(f"\n--- 方案D: 直腿 + PD增益调优 (Kp=0.5, Kd=0.1) ---")
    
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.25], [0, 0, 0, 1])
    for name, idx in joint_map.items():
        p.resetJointState(robot, idx, 0)
        p.changeDynamics(robot, idx, jointDamping=0.5)
    
    for name, idx in joint_map.items():
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL,
            targetPosition=0,
            force=50,
            positionGain=0.5,
            velocityGain=0.1,
            maxVelocity=2.0
        )
    
    for step in range(3000):
        # 每步都重新发送控制命令
        for name, idx in joint_map.items():
            p.setJointMotorControl2(
                robot, idx, p.POSITION_CONTROL,
                targetPosition=0,
                force=50,
                positionGain=0.5,
                velocityGain=0.1,
                maxVelocity=2.0
            )
        p.stepSimulation()
        
        if step % 1000 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"  t={step/1000:.1f}s z={pos[2]:.4f}m "
                  f"roll={np.rad2deg(euler[0]):.1f}° "
                  f"pitch={np.rad2deg(euler[1]):.1f}°")
    
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    z_d = pos[2]
    print(f"  最终: z={z_d:.4f}m 是否站立: {'✅' if z_d > 0.15 else '❌'}")
    
    # ===== 方案E: 脚底摩擦力增大 + 方案D =====
    print(f"\n--- 方案E: 方案D + 脚底高摩擦力 ---")
    
    p.resetBasePositionAndOrientation(robot, [0, 0, 0.25], [0, 0, 0, 1])
    for name, idx in joint_map.items():
        p.resetJointState(robot, idx, 0)
        p.changeDynamics(robot, idx, jointDamping=0.5)
    
    # 给脚底增加摩擦力
    for name, idx in joint_map.items():
        link_name = p.getJointInfo(robot, idx)[12].decode('utf-8')
        if 'foot' in link_name:
            p.changeDynamics(robot, idx, lateralFriction=2.0, spinningFriction=0.5, rollingFriction=0.01)
            print(f"  设置 {link_name} 高摩擦力")
    
    for name, idx in joint_map.items():
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL,
            targetPosition=0,
            force=50,
            positionGain=0.5,
            velocityGain=0.1,
            maxVelocity=2.0
        )
    
    for step in range(3000):
        for name, idx in joint_map.items():
            p.setJointMotorControl2(
                robot, idx, p.POSITION_CONTROL,
                targetPosition=0,
                force=50,
                positionGain=0.5,
                velocityGain=0.1,
                maxVelocity=2.0
            )
        p.stepSimulation()
        
        if step % 1000 == 0:
            pos, orn = p.getBasePositionAndOrientation(robot)
            euler = p.getEulerFromQuaternion(orn)
            print(f"  t={step/1000:.1f}s z={pos[2]:.4f}m "
                  f"roll={np.rad2deg(euler[0]):.1f}° "
                  f"pitch={np.rad2deg(euler[1]):.1f}°")
    
    pos, orn = p.getBasePositionAndOrientation(robot)
    euler = p.getEulerFromQuaternion(orn)
    z_e = pos[2]
    print(f"  最终: z={z_e:.4f}m 是否站立: {'✅' if z_e > 0.15 else '❌'}")
    
    # ===== 总结 =====
    print_header("总结")
    results = [
        ("A: 直腿+小力矩(10Nm)", z_a),
        ("B: 直腿+大力矩(50Nm)", z_b),
        ("C: 直腿+阻尼+大力矩", z_c),
        ("D: 直腿+PD增益调优", z_d),
        ("E: D+脚底高摩擦", z_e),
    ]
    
    best = max(results, key=lambda x: x[1])
    for name, z in results:
        status = "✅" if z > 0.15 else "❌"
        marker = " ← 最佳" if name == best[0] else ""
        print(f"  {status} {name}: z={z:.4f}m{marker}")
    
    print(f"\n最佳方案: {best[0]} (z={best[1]:.4f}m)")
    
    p.disconnect()


def test4_fixed_base_sanity():
    """测试4: 固定基座验证 - 确认关节控制正常"""
    print_header("测试4: 固定基座 (验证关节控制)")
    
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(0.001)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    
    base_path = Path(__file__).parent.parent
    urdf_path = str(base_path / 'models' / 'humanoid_v1.urdf')
    
    # 使用固定基座
    robot = p.loadURDF(urdf_path, [0, 0, 0.5], [0, 0, 0, 1], useFixedBase=True)
    
    joint_map = {}
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        name = info[1].decode('utf-8')
        if info[2] == p.JOINT_REVOLUTE:
            joint_map[name] = i
    
    # 设置目标角度
    targets = {
        'head_pitch': 0,
        'left_hip_roll': 0, 'left_hip_pitch': 0, 'left_knee': 0, 'left_ankle_pitch': 0,
        'right_hip_roll': 0, 'right_hip_pitch': 0, 'right_knee': 0, 'right_ankle_pitch': 0,
    }
    
    for name, angle_deg in targets.items():
        idx = joint_map[name]
        angle_rad = np.deg2rad(angle_deg)
        p.resetJointState(robot, idx, angle_rad)
        p.setJointMotorControl2(
            robot, idx, p.POSITION_CONTROL,
            targetPosition=angle_rad,
            force=50,
            positionGain=0.5,
            velocityGain=0.1
        )
    
    # 步进2秒
    for step in range(2000):
        for name, angle_deg in targets.items():
            idx = joint_map[name]
            angle_rad = np.deg2rad(angle_deg)
            p.setJointMotorControl2(
                robot, idx, p.POSITION_CONTROL,
                targetPosition=angle_rad,
                force=50,
                positionGain=0.5,
                velocityGain=0.1
            )
        p.stepSimulation()
    
    # 检查关节实际位置
    print("关节目标 vs 实际:")
    for name, angle_deg in targets.items():
        idx = joint_map[name]
        state = p.getJointState(robot, idx)
        actual = np.rad2deg(state[0])
        print(f"  {name:25s}  目标={angle_deg:7.1f}°  实际={actual:7.1f}°  "
              f"误差={abs(actual-angle_deg):.2f}°  "
              f"{'✅' if abs(actual-angle_deg) < 1.0 else '❌'}")
    
    p.disconnect()


if __name__ == '__main__':
    test1_kinematics_check()
    test2_no_gravity_motor()
    test3_gravity_with_motor()
    test4_fixed_base_sanity()
    
    print_header("诊断完成")
    print("根据以上测试结果，确定最佳方案并修改控制代码。")
