"""
机器人URDF模型生成器

根据配置文件自动生成URDF描述文件
采用程序化方式便于快速迭代和参数调整
"""

import yaml
import math
from pathlib import Path


class URDFGenerator:
    """URDF生成器"""
    
    def __init__(self, config_path: str):
        """初始化
        
        Args:
            config_path: 配置文件路径
        """
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        self.robot = self.config['robot']
        self.urdf_lines = []
        
    def generate(self, output_path: str):
        """生成URDF文件
        
        Args:
            output_path: 输出文件路径
        """
        self._add_header()
        self._add_base_link()
        self._add_torso()
        self._add_head()
        self._add_legs()
        self._add_footer()
        
        # 写入文件
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(self.urdf_lines))
        
        print(f"✅ URDF文件已生成: {output_path}")
        
    def _add_header(self):
        """添加XML头部"""
        self.urdf_lines.extend([
            '<?xml version="1.0"?>',
            f'<robot name="{self.robot["name"]}">',
            ''
        ])
        
    def _add_footer(self):
        """添加XML尾部"""
        self.urdf_lines.append('</robot>')
        
    def _add_base_link(self):
        """添加基准链接（虚拟）"""
        self.urdf_lines.extend([
            '  <!-- 基准链接 -->',
            '  <link name="base_link">',
            '    <inertial>',
            '      <mass value="0.001"/>',
            '      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>',
            '    </inertial>',
            '  </link>',
            ''
        ])
        
    def _add_torso(self):
        """添加躯干"""
        dims = self.robot['link_dimensions']['torso']
        mass = self.robot['mass_distribution']['torso']
        
        self.urdf_lines.extend([
            '  <!-- 躯干 -->',
            '  <link name="torso">',
            '    <visual>',
            '      <geometry>',
            f'        <box size="{dims["width"]} {dims["depth"]} {dims["height"]}"/>',
            '      </geometry>',
            '      <material name="blue">',
            '        <color rgba="0.2 0.4 0.8 1.0"/>',
            '      </material>',
            '    </visual>',
            '    <collision>',
            '      <geometry>',
            f'        <box size="{dims["width"]} {dims["depth"]} {dims["height"]}"/>',
            '      </geometry>',
            '    </collision>',
            '    <inertial>',
            f'      <mass value="{mass}"/>',
            f'      <inertia ixx="{self._box_inertia(mass, dims["depth"], dims["height"])}" '
            f'ixy="0" ixz="0" '
            f'iyy="{self._box_inertia(mass, dims["width"], dims["height"])}" '
            f'iyz="0" '
            f'izz="{self._box_inertia(mass, dims["width"], dims["depth"])}"/>',
            '    </inertial>',
            '  </link>',
            '',
            '  <joint name="base_to_torso" type="fixed">',
            '    <parent link="base_link"/>',
            '    <child link="torso"/>',
            f'    <origin xyz="0 0 {dims["height"]/2}" rpy="0 0 0"/>',
            '  </joint>',
            ''
        ])
        
    def _add_head(self):
        """添加头部"""
        dims = self.robot['link_dimensions']['head']
        mass = self.robot['mass_distribution']['head']
        torso_height = self.robot['link_dimensions']['torso']['height']
        
        self.urdf_lines.extend([
            '  <!-- 头部 -->',
            '  <link name="head">',
            '    <visual>',
            '      <geometry>',
            f'        <sphere radius="{dims["radius"]}"/>',
            '      </geometry>',
            '      <material name="white">',
            '        <color rgba="0.9 0.9 0.9 1.0"/>',
            '      </material>',
            '    </visual>',
            '    <collision>',
            '      <geometry>',
            f'        <sphere radius="{dims["radius"]}"/>',
            '      </geometry>',
            '    </collision>',
            '    <inertial>',
            f'      <mass value="{mass}"/>',
            f'      <inertia ixx="{self._sphere_inertia(mass, dims["radius"])}" '
            f'ixy="0" ixz="0" '
            f'iyy="{self._sphere_inertia(mass, dims["radius"])}" '
            f'iyz="0" '
            f'izz="{self._sphere_inertia(mass, dims["radius"])}"/>',
            '    </inertial>',
            '  </link>',
            '',
            '  <joint name="head_pitch" type="revolute">',
            '    <parent link="torso"/>',
            '    <child link="head"/>',
            f'    <origin xyz="0 0 {torso_height/2 + dims["radius"]}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{math.radians(self.robot["joints"]["head_pitch"]["range"][0])}" '
            f'upper="{math.radians(self.robot["joints"]["head_pitch"]["range"][1])}" '
            f'effort="10" velocity="{math.radians(self.robot["joints"]["head_pitch"]["max_velocity"])}"/>',
            '  </joint>',
            ''
        ])
        
    def _add_legs(self):
        """添加双腿"""
        for side in ['left', 'right']:
            self._add_single_leg(side)
            
    def _add_single_leg(self, side: str):
        """添加单条腿
        
        Args:
            side: 'left' 或 'right'
        """
        sign = 1 if side == 'left' else -1
        torso_width = self.robot['link_dimensions']['torso']['width']
        hip_offset = torso_width / 2 + 0.02  # 髋关节偏移
        
        thigh_dims = self.robot['link_dimensions']['thigh']
        calf_dims = self.robot['link_dimensions']['calf']
        foot_dims = self.robot['link_dimensions']['foot']
        
        thigh_mass = self.robot['mass_distribution']['thigh']
        calf_mass = self.robot['mass_distribution']['calf']
        foot_mass = self.robot['mass_distribution']['foot']
        
        # 髋关节roll（左右摆动）
        self.urdf_lines.extend([
            f'  <!-- {side.capitalize()} Leg -->',
            f'  <link name="{side}_hip_roll_link">',
            '    <inertial>',
            '      <mass value="0.05"/>',
            '      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>',
            '    </inertial>',
            '  </link>',
            '',
            f'  <joint name="{side}_hip_roll" type="revolute">',
            '    <parent link="torso"/>',
            f'    <child link="{side}_hip_roll_link"/>',
            f'    <origin xyz="0 {sign * hip_offset} {-self.robot["link_dimensions"]["torso"]["height"]/2}" rpy="0 0 0"/>',
            '    <axis xyz="1 0 0"/>',
            f'    <limit lower="{math.radians(self.robot["joints"]["hip_roll"]["range"][0])}" '
            f'upper="{math.radians(self.robot["joints"]["hip_roll"]["range"][1])}" '
            f'effort="10" velocity="{math.radians(self.robot["joints"]["hip_roll"]["max_velocity"])}"/>',
            '  </joint>',
            ''
        ])
        
        # 髋关节pitch（前后摆动）+ 大腿
        self.urdf_lines.extend([
            f'  <link name="{side}_thigh">',
            '    <visual>',
            '      <geometry>',
            f'        <cylinder length="{thigh_dims["length"]}" radius="{thigh_dims["radius"]}"/>',
            '      </geometry>',
            f'      <origin xyz="0 0 {-thigh_dims["length"]/2}" rpy="0 0 0"/>',
            '      <material name="gray">',
            '        <color rgba="0.5 0.5 0.5 1.0"/>',
            '      </material>',
            '    </visual>',
            '    <collision>',
            '      <geometry>',
            f'        <cylinder length="{thigh_dims["length"]}" radius="{thigh_dims["radius"]}"/>',
            '      </geometry>',
            f'      <origin xyz="0 0 {-thigh_dims["length"]/2}" rpy="0 0 0"/>',
            '    </collision>',
            '    <inertial>',
            f'      <origin xyz="0 0 {-thigh_dims["length"]/2}" rpy="0 0 0"/>',
            f'      <mass value="{thigh_mass}"/>',
            f'      <inertia ixx="{self._cylinder_inertia(thigh_mass, thigh_dims["radius"], thigh_dims["length"])}" '
            f'ixy="0" ixz="0" '
            f'iyy="{self._cylinder_inertia(thigh_mass, thigh_dims["radius"], thigh_dims["length"])}" '
            f'iyz="0" '
            f'izz="{thigh_mass * thigh_dims["radius"]**2 / 2}"/>',
            '    </inertial>',
            '  </link>',
            '',
            f'  <joint name="{side}_hip_pitch" type="revolute">',
            f'    <parent link="{side}_hip_roll_link"/>',
            f'    <child link="{side}_thigh"/>',
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{math.radians(self.robot["joints"]["hip_pitch"]["range"][0])}" '
            f'upper="{math.radians(self.robot["joints"]["hip_pitch"]["range"][1])}" '
            f'effort="10" velocity="{math.radians(self.robot["joints"]["hip_pitch"]["max_velocity"])}"/>',
            '  </joint>',
            ''
        ])
        
        # 膝关节 + 小腿
        self.urdf_lines.extend([
            f'  <link name="{side}_calf">',
            '    <visual>',
            '      <geometry>',
            f'        <cylinder length="{calf_dims["length"]}" radius="{calf_dims["radius"]}"/>',
            '      </geometry>',
            f'      <origin xyz="0 0 {-calf_dims["length"]/2}" rpy="0 0 0"/>',
            '      <material name="gray"/>',
            '    </visual>',
            '    <collision>',
            '      <geometry>',
            f'        <cylinder length="{calf_dims["length"]}" radius="{calf_dims["radius"]}"/>',
            '      </geometry>',
            f'      <origin xyz="0 0 {-calf_dims["length"]/2}" rpy="0 0 0"/>',
            '    </collision>',
            '    <inertial>',
            f'      <origin xyz="0 0 {-calf_dims["length"]/2}" rpy="0 0 0"/>',
            f'      <mass value="{calf_mass}"/>',
            f'      <inertia ixx="{self._cylinder_inertia(calf_mass, calf_dims["radius"], calf_dims["length"])}" '
            f'ixy="0" ixz="0" '
            f'iyy="{self._cylinder_inertia(calf_mass, calf_dims["radius"], calf_dims["length"])}" '
            f'iyz="0" '
            f'izz="{calf_mass * calf_dims["radius"]**2 / 2}"/>',
            '    </inertial>',
            '  </link>',
            '',
            f'  <joint name="{side}_knee" type="revolute">',
            f'    <parent link="{side}_thigh"/>',
            f'    <child link="{side}_calf"/>',
            f'    <origin xyz="0 0 {-thigh_dims["length"]}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{math.radians(self.robot["joints"]["knee"]["range"][0])}" '
            f'upper="{math.radians(self.robot["joints"]["knee"]["range"][1])}" '
            f'effort="10" velocity="{math.radians(self.robot["joints"]["knee"]["max_velocity"])}"/>',
            '  </joint>',
            ''
        ])
        
        # 踝关节 + 脚
        self.urdf_lines.extend([
            f'  <link name="{side}_foot">',
            '    <visual>',
            '      <geometry>',
            f'        <box size="{foot_dims["length"]} {foot_dims["width"]} {foot_dims["height"]}"/>',
            '      </geometry>',
            f'      <origin xyz="{foot_dims["length"]/4} 0 {-foot_dims["height"]/2}" rpy="0 0 0"/>',
            '      <material name="dark_gray">',
            '        <color rgba="0.3 0.3 0.3 1.0"/>',
            '      </material>',
            '    </visual>',
            '    <collision>',
            '      <geometry>',
            f'        <box size="{foot_dims["length"]} {foot_dims["width"]} {foot_dims["height"]}"/>',
            '      </geometry>',
            f'      <origin xyz="{foot_dims["length"]/4} 0 {-foot_dims["height"]/2}" rpy="0 0 0"/>',
            '    </collision>',
            '    <inertial>',
            f'      <origin xyz="{foot_dims["length"]/4} 0 {-foot_dims["height"]/2}" rpy="0 0 0"/>',
            f'      <mass value="{foot_mass}"/>',
            f'      <inertia ixx="{self._box_inertia(foot_mass, foot_dims["width"], foot_dims["height"])}" '
            f'ixy="0" ixz="0" '
            f'iyy="{self._box_inertia(foot_mass, foot_dims["length"], foot_dims["height"])}" '
            f'iyz="0" '
            f'izz="{self._box_inertia(foot_mass, foot_dims["length"], foot_dims["width"])}"/>',
            '    </inertial>',
            '  </link>',
            '',
            f'  <joint name="{side}_ankle_pitch" type="revolute">',
            f'    <parent link="{side}_calf"/>',
            f'    <child link="{side}_foot"/>',
            f'    <origin xyz="0 0 {-calf_dims["length"]}" rpy="0 0 0"/>',
            '    <axis xyz="0 1 0"/>',
            f'    <limit lower="{math.radians(self.robot["joints"]["ankle_pitch"]["range"][0])}" '
            f'upper="{math.radians(self.robot["joints"]["ankle_pitch"]["range"][1])}" '
            f'effort="10" velocity="{math.radians(self.robot["joints"]["ankle_pitch"]["max_velocity"])}"/>',
            '  </joint>',
            ''
        ])
        
    @staticmethod
    def _box_inertia(mass: float, width: float, height: float) -> float:
        """计算长方体转动惯量"""
        return mass * (width**2 + height**2) / 12
        
    @staticmethod
    def _cylinder_inertia(mass: float, radius: float, length: float) -> float:
        """计算圆柱体转动惯量"""
        return mass * (3 * radius**2 + length**2) / 12
        
    @staticmethod
    def _sphere_inertia(mass: float, radius: float) -> float:
        """计算球体转动惯量"""
        return 2 * mass * radius**2 / 5


def main():
    """主函数"""
    # 获取配置文件路径
    config_path = Path(__file__).parent.parent.parent / 'config' / 'robot_config.yaml'
    output_path = Path(__file__).parent.parent.parent / 'models' / 'humanoid_v1.urdf'
    
    # 生成URDF
    generator = URDFGenerator(str(config_path))
    generator.generate(str(output_path))
    
    print(f"✅ 机器人模型生成完成!")
    print(f"   - 自由度: {generator.robot['dof']['total']} DOF")
    print(f"   - 高度: {generator.robot['dimensions']['height']}m")
    print(f"   - 重量: {generator.robot['dimensions']['weight']}kg")


if __name__ == '__main__':
    main()
