"""
PyBullet仿真环境

提供机器人加载、物理仿真、可视化等核心功能
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Optional


class SimulationEnvironment:
    """仿真环境管理器"""
    
    def __init__(self, config_path: str, gui: bool = True):
        """初始化仿真环境
        
        Args:
            config_path: 配置文件路径
            gui: 是否启用GUI
        """
        # 加载配置
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        self.gui = gui
        self.robot_id = None
        self.joint_indices = {}
        self.joint_names = []
        
        # 连接物理引擎
        if gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        else:
            self.client = p.connect(p.DIRECT)
            
        print(f"✅ PyBullet已连接 (client={self.client})")
        
    def setup_world(self):
        """设置世界环境"""
        # 设置重力
        gravity = self.config['simulation']['physics']['gravity']
        p.setGravity(0, 0, gravity)
        
        # 设置时间步长
        time_step = self.config['simulation']['physics']['time_step']
        p.setTimeStep(time_step)
        
        # 加载地面
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        
        # 设置地面摩擦力
        friction = self.config['simulation']['ground']['friction']
        p.changeDynamics(plane_id, -1, lateralFriction=friction)
        
        # 设置相机
        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.0,
                cameraYaw=45,
                cameraPitch=-20,
                cameraTargetPosition=[0, 0, 0.3]
            )
            
        print("✅ 世界环境设置完成")
        
    def load_robot(self, urdf_path: str):
        """加载机器人模型
        
        Args:
            urdf_path: URDF文件路径
        """
        # 获取初始位置和姿态
        init_pos = self.config['initial_pose']['position']
        init_orn = self.config['initial_pose']['orientation']
        
        # 加载机器人
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=init_pos,
            baseOrientation=init_orn,
            useFixedBase=False
        )
        
        # 获取关节信息
        self._build_joint_mapping()
        
        # 设置初始关节角度
        self._set_initial_pose()
        
        print(f"✅ 机器人已加载 (ID={self.robot_id})")
        print(f"   - 关节数: {len(self.joint_names)}")
        print(f"   - 关节列表: {self.joint_names}")
        
    def _build_joint_mapping(self):
        """构建关节映射"""
        num_joints = p.getNumJoints(self.robot_id)
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            # 只记录可动关节（revolute）
            if joint_type == p.JOINT_REVOLUTE:
                self.joint_indices[joint_name] = i
                self.joint_names.append(joint_name)
                
    def _set_initial_pose(self):
        """设置初始姿态"""
        joint_angles = self.config['initial_pose']['joint_angles']
        
        for joint_name, angle_deg in joint_angles.items():
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                angle_rad = np.deg2rad(angle_deg)
                p.resetJointState(self.robot_id, joint_idx, angle_rad)
                
    def set_joint_positions(self, joint_positions: Dict[str, float]):
        """设置关节位置（位置控制）
        
        Args:
            joint_positions: 关节名称->角度（度）的字典
        """
        for joint_name, angle_deg in joint_positions.items():
            if joint_name in self.joint_indices:
                joint_idx = self.joint_indices[joint_name]
                angle_rad = np.deg2rad(angle_deg)
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=angle_rad,
                    force=100  # 最大力矩
                )
                
    def get_joint_states(self) -> Dict[str, Dict[str, float]]:
        """获取所有关节状态
        
        Returns:
            关节名称->状态字典的字典
            状态包含: position, velocity, torque
        """
        states = {}
        
        for joint_name, joint_idx in self.joint_indices.items():
            joint_state = p.getJointState(self.robot_id, joint_idx)
            states[joint_name] = {
                'position': np.rad2deg(joint_state[0]),  # 转为度
                'velocity': np.rad2deg(joint_state[1]),
                'torque': joint_state[3]
            }
            
        return states
        
    def get_base_state(self) -> Dict[str, np.ndarray]:
        """获取基座（躯干）状态
        
        Returns:
            包含position, orientation, linear_vel, angular_vel的字典
        """
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
        
        # 将四元数转换为欧拉角
        euler = p.getEulerFromQuaternion(orn)
        
        return {
            'position': np.array(pos),
            'orientation_quat': np.array(orn),
            'orientation_euler': np.rad2deg(np.array(euler)),
            'linear_velocity': np.array(lin_vel),
            'angular_velocity': np.array(ang_vel)
        }
        
    def get_imu_data(self) -> Dict[str, np.ndarray]:
        """模拟IMU数据
        
        Returns:
            包含加速度和角速度的字典
        """
        base_state = self.get_base_state()
        
        # 添加噪声
        noise_level = self.config['sensors']['imu']['noise_level']
        
        # 线性加速度（包含重力）
        lin_acc = np.random.normal(0, noise_level, 3)
        lin_acc[2] += 9.81  # Z轴重力
        
        # 角速度
        ang_vel = base_state['angular_velocity'] + np.random.normal(0, noise_level, 3)
        
        return {
            'linear_acceleration': lin_acc,
            'angular_velocity': ang_vel,
            'orientation': base_state['orientation_euler']
        }
        
    def step(self):
        """执行一步仿真"""
        p.stepSimulation()
        
    def run(self, duration: float = 10.0, real_time: bool = True):
        """运行仿真
        
        Args:
            duration: 运行时长（秒）
            real_time: 是否实时运行
        """
        time_step = self.config['simulation']['physics']['time_step']
        steps = int(duration / time_step)
        
        print(f"🚀 开始仿真 (时长={duration}s, 步数={steps})")
        
        for i in range(steps):
            self.step()
            
            if real_time:
                time.sleep(time_step)
                
            # 每秒打印一次状态
            if i % int(1.0 / time_step) == 0:
                base_state = self.get_base_state()
                print(f"   t={i*time_step:.2f}s | "
                      f"pos={base_state['position']} | "
                      f"euler={base_state['orientation_euler']}")
                      
        print("✅ 仿真完成")
        
    def add_debug_parameters(self) -> Dict[str, int]:
        """添加调试滑块（GUI模式）
        
        Returns:
            参数名->参数ID的字典
        """
        if not self.gui:
            return {}
            
        params = {}
        
        for joint_name, joint_idx in self.joint_indices.items():
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            
            # 如果没有限制，设置默认范围
            if lower_limit >= upper_limit:
                lower_limit = -np.pi
                upper_limit = np.pi
                
            param_id = p.addUserDebugParameter(
                joint_name,
                lower_limit,
                upper_limit,
                0  # 初始值
            )
            
            if param_id != -1:
                params[joint_name] = param_id
            else:
                print(f"⚠️ 警告: 无法为关节 {joint_name} 创建调试滑块")
            
        return params
        
    def read_debug_parameters(self, params: Dict[str, int]) -> Dict[str, float]:
        """读取调试滑块值
        
        Args:
            params: 参数名->参数ID的字典
            
        Returns:
            关节名称->角度（弧度）的字典
        """
        values = {}
        
        # 检查连接状态
        if not p.isConnected():
            return values
        
        try:
            for joint_name, param_id in params.items():
                values[joint_name] = p.readUserDebugParameter(param_id)
        except:
            # 如果读取失败（可能是断开连接），返回空字典
            return {}
            
        return values
        
    def close(self):
        """关闭仿真"""
        p.disconnect()
        print("✅ 仿真已关闭")


def main():
    """主函数：测试仿真环境"""
    # 文件路径
    base_path = Path(__file__).parent.parent.parent
    config_path = base_path / 'config' / 'robot_config.yaml'
    urdf_path = base_path / 'models' / 'humanoid_v1.urdf'
    
    # 创建仿真环境
    env = SimulationEnvironment(str(config_path), gui=True)
    env.setup_world()
    env.load_robot(str(urdf_path))
    
    # 添加调试滑块
    debug_params = env.add_debug_parameters()
    
    print("\n💡 使用调试滑块调整关节角度")
    print("   - 拖动滑块实时调整机器人姿态")
    print("   - 关闭GUI窗口或按Ctrl+C退出\n")
    
    try:
        while True:
            # 读取滑块值并应用
            joint_values = env.read_debug_parameters(debug_params)
            
            # 转换为位置控制命令
            joint_positions = {}
            for joint_name, angle_rad in joint_values.items():
                joint_positions[joint_name] = np.rad2deg(angle_rad)
                
            env.set_joint_positions(joint_positions)
            
            # 执行仿真步
            env.step()
            time.sleep(1/240)  # 240Hz
            
    except KeyboardInterrupt:
        print("\n⏹️ 用户手动停止仿真")
    except Exception as e:
        print(f"\n⚠️ 仿真结束: {e}")
        
    finally:
        try:
            env.close()
        except:
            pass  # 忽略关闭时的错误


if __name__ == '__main__':
    main()
