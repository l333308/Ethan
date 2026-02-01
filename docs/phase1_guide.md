# 第一阶段：仿真建模 - 完整指南

## 📋 目标

- ✅ 建立完整的机器人URDF模型
- ✅ 搭建PyBullet仿真环境
- ✅ 验证模型物理特性
- ✅ 测试准静态站立稳定性
- ✅ 为第二阶段（控制算法）打好基础

## 🛠️ 环境准备

### 1. 安装Python依赖

```bash
cd /Users/pika/www/deep-learning/Ethan
pip install -r requirements.txt
```

### 2. 验证安装

```bash
python -c "import pybullet; print('PyBullet版本:', pybullet.__version__)"
```

## 🚀 快速开始

### 步骤1: 生成机器人URDF模型

```bash
python src/robot/urdf_generator.py
```

**输出**:
- ✅ 生成 `models/humanoid_v1.urdf`
- 包含9自由度（DOF）配置
- 质量分布、转动惯量已自动计算

**关键参数**（可在`config/robot_config.yaml`中调整）:
- 高度: 40cm
- 重量: 2.0kg
- 下肢: 8 DOF (单腿4 DOF × 2)
- 头部: 1 DOF

### 步骤2: 加载仿真环境

```bash
python src/simulation/environment.py
```

**功能**:
- 启动PyBullet GUI
- 加载机器人模型
- 提供调试滑块（可手动调整关节角度）

**操作提示**:
- 使用滑块调整各关节角度
- 鼠标拖拽旋转视角
- 滚轮缩放
- 按`Ctrl+C`退出

### 步骤3: 运行站立稳定性测试

#### 基础站立测试

```bash
python tests/test_standing.py --mode basic --duration 5
```

**测试内容**:
- 机器人保持站立姿态5秒
- 监测高度、Roll/Pitch角度、位置漂移
- 输出稳定性得分

**通过标准**:
- ✅ 高度 > 0.15m
- ✅ Roll/Pitch < 30°
- ✅ 位置漂移 < 0.5m
- ✅ 稳定性得分 > 60/100

#### 扰动测试

```bash
python tests/test_standing.py --mode disturbance --duration 10
```

**测试内容**:
- t=3秒时施加50N侧向力
- 观察机器人恢复能力
- 评估抗扰动性能

## 📊 数据可视化

### 查看稳定性指标

```python
from src.utils.visualization import Plotter
from src.utils.metrics import StabilityMetrics

# 运行测试后，使用metrics对象
Plotter.plot_stability_metrics(metrics)
```

生成图表:
1. 基座高度曲线
2. Roll/Pitch角度
3. 线性速度

## 🔧 配置调整

### 修改机器人参数

编辑 `config/robot_config.yaml`:

```yaml
robot:
  dimensions:
    height: 0.40    # 调整高度
    weight: 2.0     # 调整重量
    
  link_dimensions:
    thigh:
      length: 0.12  # 调整大腿长度
      
  joints:
    knee:
      range: [0, 120]  # 调整膝关节活动范围
```

**重新生成模型**:
```bash
python src/robot/urdf_generator.py
```

### 修改仿真参数

```yaml
simulation:
  physics:
    time_step: 0.001      # 时间步长
    solver_iterations: 50  # 求解器迭代次数
    
  ground:
    friction: 0.8         # 地面摩擦系数
```

## 🎯 里程碑检查

完成以下任务后可进入第二阶段：

- [ ] 机器人模型生成成功
- [ ] 仿真环境正常运行
- [ ] 基础站立测试通过（稳定≥5秒）
- [ ] 理解URDF模型结构
- [ ] 熟悉PyBullet API
- [ ] 掌握关节控制基础

## 📝 常见问题

### Q1: 机器人一加载就倒下

**原因**: 初始姿态不稳定

**解决**:
1. 调整初始关节角度（`config/robot_config.yaml` -> `initial_pose`）
2. 增加settling时间（让机器人先稳定500步）
3. 检查质心是否在支撑多边形内

### Q2: 仿真运行很慢

**解决**:
1. 减少`solver_iterations`
2. 增大`time_step`（牺牲精度）
3. 使用`p.DIRECT`模式（无GUI）

### Q3: 关节活动范围不合理

**解决**:
1. 参考人体关节活动范围
2. 逐步测试，避免自碰撞
3. 使用`debug_params`滑块测试合理范围

## 🔍 代码架构

```
src/
├── robot/
│   └── urdf_generator.py    # URDF生成器
│
├── simulation/
│   └── environment.py        # 仿真环境封装
│
└── utils/
    ├── metrics.py           # 稳定性评估
    └── visualization.py     # 数据可视化
```

**设计理念**:
- 模块化：每个模块职责单一
- 可配置：参数外部化
- 可扩展：为后续阶段预留接口

## 📈 性能指标

当前阶段目标（基准）:

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 站立时长 | ≥5秒 | 无外力扰动 |
| 高度稳定性 | ±2cm | 高度波动范围 |
| 姿态稳定性 | ±5° | Roll/Pitch波动 |
| 位置漂移 | <5cm | 水平位移 |
| 抗扰动 | 50N侧推不倒 | 扰动恢复能力 |

## 🎓 学习资源

### 推荐阅读

1. **PyBullet文档**: https://pybullet.org/
2. **URDF格式规范**: http://wiki.ros.org/urdf/XML
3. **机器人动力学基础**: 《机器人学导论》John J. Craig

### 相关概念

- **URDF**: 统一机器人描述格式（XML）
- **转动惯量**: 物体旋转运动的惯性量度
- **准静态**: 运动速度极慢，忽略动态效应
- **ZMP**: 零力矩点（下阶段会用到）

## 🔗 下一步

完成第一阶段后，进入 **第二阶段：控制算法开发**

预览：
- 步态规划器
- ZMP稳定性判据
- IMU姿态修正
- 轨迹插值与平滑
- 预定义步态表

---

**更新日期**: 2026-02-01  
**版本**: v0.1.0
