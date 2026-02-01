# 系统架构设计

## 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                    用户交互层                                 │
│  - quickstart.py (快速启动)                                  │
│  - test_*.py (测试脚本)                                      │
│  - 命令行工具                                                │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────────┐
│                  应用层 (Python)                             │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │   机器人    │  │   仿真环境   │  │   评估工具   │       │
│  │   建模      │  │   管理       │  │   可视化     │       │
│  └─────────────┘  └──────────────┘  └──────────────┘       │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────────┐
│                 仿真引擎层 (PyBullet)                        │
│  - 物理引擎                                                  │
│  - 碰撞检测                                                  │
│  - 渲染系统                                                  │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────────┐
│                  数据层                                      │
│  - config/ (配置文件)                                        │
│  - models/ (URDF模型)                                        │
│  - logs/ (日志数据)                                          │
└─────────────────────────────────────────────────────────────┘
```

## 模块详细设计

### 1. 机器人建模模块 (`src/robot/`)

```
urdf_generator.py
    │
    ├─► URDFGenerator (类)
    │   ├─► __init__(config_path)
    │   ├─► generate(output_path)
    │   ├─► _add_torso()
    │   ├─► _add_head()
    │   ├─► _add_legs()
    │   └─► _add_single_leg(side)
    │
    └─► 转动惯量计算函数
        ├─► _box_inertia()
        ├─► _cylinder_inertia()
        └─► _sphere_inertia()

输入: robot_config.yaml
输出: humanoid_v1.urdf
```

**职责**：
- 程序化生成URDF模型
- 自动计算物理参数
- 参数化配置

### 2. 仿真环境模块 (`src/simulation/`)

```
environment.py
    │
    └─► SimulationEnvironment (类)
        │
        ├─► 初始化
        │   ├─► __init__(config_path, gui)
        │   ├─► setup_world()
        │   └─► load_robot(urdf_path)
        │
        ├─► 控制接口
        │   ├─► set_joint_positions()
        │   ├─► get_joint_states()
        │   └─► get_base_state()
        │
        ├─► 传感器模拟
        │   └─► get_imu_data()
        │
        ├─► 仿真执行
        │   ├─► step()
        │   └─► run(duration, real_time)
        │
        └─► 调试工具
            ├─► add_debug_parameters()
            └─► read_debug_parameters()
```

**职责**：
- PyBullet API封装
- 关节控制接口
- 状态查询接口
- 传感器数据模拟

### 3. 评估工具模块 (`src/utils/`)

#### metrics.py
```
StabilityMetrics (类)
    │
    ├─► update(base_state, joint_states, imu_data)
    ├─► is_stable() → bool
    ├─► get_stability_score() → float
    ├─► get_summary() → dict
    └─► print_summary()

GaitMetrics (类) [为第二阶段准备]
    │
    ├─► detect_step()
    ├─► calculate_step_length()
    └─► calculate_step_frequency()
```

#### visualization.py
```
Plotter (类)
    │
    ├─► plot_stability_metrics(metrics)
    ├─► plot_joint_trajectories(joint_history)
    ├─► plot_trajectory_2d(positions)
    └─► create_animation(positions, output_path)
```

**职责**：
- 稳定性量化评估
- 数据可视化
- 动画生成

## 数据流图

```
┌──────────────┐
│ robot_config │
│   .yaml      │
└──────┬───────┘
       │
       ▼
┌──────────────┐      ┌──────────────┐
│    URDF      │─────►│  PyBullet    │
│  Generator   │      │  Simulation  │
└──────────────┘      └──────┬───────┘
                             │
                             │ 每步输出
                             ▼
                      ┌──────────────┐
                      │  State Data  │
                      │ - base_state │
                      │ - joint_state│
                      │ - imu_data   │
                      └──────┬───────┘
                             │
                             ▼
                      ┌──────────────┐
                      │   Metrics    │
                      │  Calculator  │
                      └──────┬───────┘
                             │
                    ┌────────┴────────┐
                    ▼                 ▼
            ┌───────────┐      ┌──────────┐
            │ Real-time │      │  Offline │
            │  Monitor  │      │  Analysis│
            └───────────┘      └──────────┘
```

## 控制流程（当前）

```
1. 初始化
   ├─► 加载配置
   ├─► 生成URDF
   ├─► 创建仿真环境
   └─► 加载机器人模型

2. 设置初始姿态
   ├─► 读取初始关节角度
   ├─► 设置关节位置
   └─► 稳定若干步

3. 仿真循环
   │
   ├─► [可选] 读取控制命令
   │   └─► set_joint_positions()
   │
   ├─► 执行物理步进
   │   └─► step()
   │
   ├─► 获取状态数据
   │   ├─► get_base_state()
   │   ├─► get_joint_states()
   │   └─► get_imu_data()
   │
   ├─► 更新评估指标
   │   └─► metrics.update()
   │
   ├─► [可选] 实时显示
   │   └─► print_status()
   │
   └─► 判断是否继续
       ├─► 时间未到 → 继续
       └─► 失去平衡 → 停止

4. 后处理
   ├─► 打印统计摘要
   ├─► 绘制数据图表
   └─► 保存结果
```

## 第二阶段扩展架构（预览）

```
┌─────────────────────────────────────────────────────────┐
│                   高层控制 (Python)                      │
│  ┌──────────────┐  ┌───────────────┐  ┌─────────────┐  │
│  │ 行为状态机   │  │  任务规划     │  │  AI决策     │  │
│  └──────────────┘  └───────────────┘  └─────────────┘  │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                 中层控制 (Python/C++)                    │
│  ┌──────────────┐  ┌───────────────┐  ┌─────────────┐  │
│  │ 步态生成器   │  │  ZMP控制      │  │ 姿态稳定    │  │
│  └──────────────┘  └───────────────┘  └─────────────┘  │
│  ┌──────────────┐  ┌───────────────┐                   │
│  │ 轨迹规划     │  │  逆运动学     │                   │
│  └──────────────┘  └───────────────┘                   │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│                  底层控制 (当前阶段)                     │
│  ┌──────────────┐  ┌───────────────┐  ┌─────────────┐  │
│  │ 位置控制     │  │  状态估计     │  │ 传感器融合  │  │
│  └──────────────┘  └───────────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────────┘
```

## 配置系统设计

```
robot_config.yaml
    │
    ├─► robot
    │   ├─► dimensions (尺寸)
    │   ├─► dof (自由度配置)
    │   ├─► joints (关节参数)
    │   ├─► mass_distribution (质量分布)
    │   └─► link_dimensions (链节尺寸)
    │
    ├─► simulation
    │   ├─► physics (物理参数)
    │   ├─► control (控制参数)
    │   └─► ground (地面属性)
    │
    ├─► sensors
    │   ├─► imu (IMU配置)
    │   ├─► joint_encoder
    │   ├─► camera [未启用]
    │   └─► force_sensor [未启用]
    │
    └─► initial_pose
        ├─► position
        ├─► orientation
        └─► joint_angles
```

## 测试架构

```
tests/
    │
    └─► test_standing.py
        │
        ├─► test_standing_stability()
        │   ├─► 创建环境
        │   ├─► 加载机器人
        │   ├─► 设置初始姿态
        │   ├─► 运行仿真
        │   ├─► 收集数据
        │   └─► 评估结果
        │
        └─► test_with_disturbance()
            ├─► 创建环境
            ├─► 稳定初始化
            ├─► 施加扰动
            ├─► 观察恢复
            └─► 评估结果
```

## 接口定义

### SimulationEnvironment接口

```python
class SimulationEnvironment:
    """仿真环境接口"""
    
    # 初始化
    def __init__(config_path: str, gui: bool)
    def setup_world()
    def load_robot(urdf_path: str)
    
    # 控制接口
    def set_joint_positions(positions: Dict[str, float])
    def set_joint_velocities(velocities: Dict[str, float])  # 未来
    def set_joint_torques(torques: Dict[str, float])        # 未来
    
    # 状态查询
    def get_joint_states() -> Dict[str, Dict]
    def get_base_state() -> Dict[str, np.ndarray]
    def get_imu_data() -> Dict[str, np.ndarray]
    
    # 仿真执行
    def step()
    def run(duration: float, real_time: bool)
    
    # 工具
    def add_debug_parameters() -> Dict[str, int]
    def close()
```

### StabilityMetrics接口

```python
class StabilityMetrics:
    """稳定性评估接口"""
    
    # 数据收集
    def update(base_state: Dict, joint_states: Dict, imu_data: Dict)
    
    # 评估
    def is_stable() -> bool
    def get_stability_score() -> float
    def get_summary() -> Dict
    
    # 输出
    def print_summary()
```

## 扩展点

### 1. 传感器扩展
```python
# 未来可添加
def get_camera_data() -> np.ndarray
def get_force_sensor_data() -> Dict[str, float]
def get_lidar_data() -> np.ndarray
```

### 2. 控制器扩展
```python
# 第二阶段添加
class GaitController:
    def generate_gait()
    def plan_trajectory()
    def execute_step()

class BalanceController:
    def compute_zmp()
    def adjust_posture()
```

### 3. ROS 2集成点
```python
# 第四阶段
class ROS2Bridge:
    def publish_state()
    def subscribe_command()
    def publish_imu()
```

---

**版本**: v0.1.0  
**日期**: 2026-02-01
