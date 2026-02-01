# 快速入门教程

欢迎来到具身机器人仿真项目！这是一个**30分钟快速上手**教程。

## 🎯 学习目标

完成本教程后，你将：
- ✅ 理解项目结构
- ✅ 成功运行第一个仿真
- ✅ 调整机器人参数
- ✅ 完成稳定性测试

## 📋 前提条件

- Python 3.8+
- 基本的Python编程知识
- 30分钟空闲时间

## 第1步：环境准备（5分钟）

### 1.1 安装依赖

```bash
cd /Users/pika/www/deep-learning/Ethan

# 方式1: 使用pip
pip install -r requirements.txt

# 方式2: 使用Makefile
make install
```

### 1.2 验证安装

```bash
python -c "import pybullet as p; print('PyBullet版本:', p.__version__)"
```

看到版本号就说明安装成功了！

## 第2步：生成机器人模型（2分钟）

### 2.1 理解配置文件

打开 `config/robot_config.yaml`，这是机器人的"设计图纸"：

```yaml
robot:
  dimensions:
    height: 0.40  # 40厘米高
    weight: 2.0   # 2公斤重
    
  dof:
    total: 9      # 9个关节
```

**不要修改任何内容**，先用默认配置。

### 2.2 生成URDF模型

```bash
# 方式1: 直接运行
python src/robot/urdf_generator.py

# 方式2: 使用Makefile
make setup
```

你会看到：
```
✅ URDF文件已生成: models/humanoid_v1.urdf
✅ 机器人模型生成完成!
   - 自由度: 9 DOF
   - 高度: 0.4m
   - 重量: 2.0kg
```

**恭喜！** 你的机器人模型已经创建好了。

## 第3步：第一次仿真（5分钟）

### 3.1 启动交互式调试

```bash
python src/simulation/environment.py
```

你会看到一个3D窗口弹出，里面有你的机器人！

### 3.2 探索界面

**鼠标操作**：
- 左键拖拽：旋转视角
- 中键滚轮：缩放
- 右键拖拽：平移视角

**调试滑块**：
- 右侧有很多滑块
- 每个滑块控制一个关节
- 试着拖动 `left_knee` 滑块

### 3.3 让机器人"动起来"

试着调整这些滑块：
1. `left_knee`: 拉到 20°（膝盖微屈）
2. `right_knee`: 拉到 20°
3. `left_hip_pitch`: 拉到 10°
4. `right_hip_pitch`: 拉到 10°

观察机器人姿态变化！

**按 Ctrl+C 退出**

## 第4步：运行自动化测试（10分钟）

### 4.1 基础站立测试

这个测试会让机器人保持站立5秒，并评估稳定性。

```bash
# 方式1
python tests/test_standing.py --mode basic --duration 5

# 方式2
make test
```

**观察输出**：
```
🧍 站立稳定性测试
===================================================
⏳ 初始化姿态...
🚀 开始测试 (时长=5秒)
---------------------------------------------------
t= 0.00s | 高度=0.251m | Roll=  0.12° | Pitch= -0.08° | 稳定=True
t= 0.50s | 高度=0.250m | Roll=  0.15° | Pitch= -0.10° | 稳定=True
...
```

**理解指标**：
- **高度**: 机器人重心高度（应该保持在0.25m左右）
- **Roll**: 左右倾斜（应该接近0°）
- **Pitch**: 前后倾斜（应该接近0°）
- **稳定**: 是否满足稳定性条件

### 4.2 查看测试结果

测试结束后会显示：

```
📊 测试结果
===================================================
测试时长: 5.00秒

高度统计:
  - 平均: 0.250m
  - 最小: 0.248m
  - 最大: 0.252m

姿态统计:
  - Roll平均: 0.15°
  - Pitch平均: 0.12°

位置漂移: 0.003m

稳定性得分: 95.2/100

✅ 测试通过 - 机器人保持稳定
```

**解读**：
- 得分 > 80 → 优秀
- 得分 60-80 → 良好
- 得分 < 60 → 需要调整

### 4.3 扰动测试（进阶）

这个测试会在3秒时推一下机器人，看它能否恢复平衡。

```bash
python tests/test_standing.py --mode disturbance --duration 10
```

观察机器人如何应对50N的侧向推力！

## 第5步：调整参数（8分钟）

现在让我们修改机器人，看看会发生什么。

### 5.1 修改机器人高度

打开 `config/robot_config.yaml`，找到：

```yaml
robot:
  dimensions:
    height: 0.40  # 改为 0.35
    weight: 2.0
```

改为：
```yaml
    height: 0.35  # 降低5厘米
```

### 5.2 重新生成并测试

```bash
# 重新生成模型
python src/robot/urdf_generator.py

# 再次测试
python tests/test_standing.py --mode basic --duration 5
```

**思考**：
- 稳定性得分有变化吗？
- 更矮的机器人更稳定还是更不稳定？

### 5.3 修改关节角度

找到：
```yaml
initial_pose:
  joint_angles:
    left_knee: 0    # 改为 15
    right_knee: 0   # 改为 15
```

改为：
```yaml
    left_knee: 15   # 膝盖微屈
    right_knee: 15
```

再次测试，观察变化！

### 5.4 恢复默认配置

实验完成后，建议改回：
```yaml
height: 0.40
left_knee: 0
right_knee: 0
```

## 🎓 学到了什么？

### 核心概念

1. **URDF模型**: 机器人的描述文件
2. **PyBullet**: 物理仿真引擎
3. **关节控制**: 位置控制（告诉关节去哪个角度）
4. **稳定性**: 通过高度、姿态、漂移来评估

### 工作流程

```
配置文件 → 生成URDF → 加载到仿真 → 控制关节 → 评估稳定性
```

## 🚀 下一步

### 进阶任务

完成了基础教程？试试这些：

#### 任务1: 找到最稳定的姿态
调整 `initial_pose.joint_angles`，让稳定性得分达到95+

提示：
- 微屈膝盖降低重心
- 调整踝关节平衡

#### 任务2: 测试极限
逐步增加扰动力（修改 `test_standing.py` 中的 `force = [0, 50, 0]`），找到机器人能承受的最大力。

#### 任务3: 自定义测试
创建自己的测试脚本：
```python
# my_test.py
from src.simulation.environment import SimulationEnvironment

env = SimulationEnvironment('config/robot_config.yaml', gui=True)
env.setup_world()
env.load_robot('models/humanoid_v1.urdf')

# 你的测试代码...
```

### 深入学习

1. **阅读文档**：
   - `docs/phase1_guide.md` - 完整指南
   - `docs/architecture.md` - 系统架构
   - `docs/roadmap.md` - 开发路线图

2. **理解代码**：
   - 看 `src/robot/urdf_generator.py` 如何生成模型
   - 看 `src/simulation/environment.py` 如何封装PyBullet
   - 看 `src/utils/metrics.py` 如何评估稳定性

3. **准备第二阶段**：
   - 学习步态规划
   - 了解ZMP（零力矩点）
   - 研究逆运动学

## 🔧 常见问题

### Q: 仿真窗口打不开
A: 检查是否安装了GUI版本的PyBullet：
```bash
pip uninstall pybullet
pip install pybullet
```

### Q: 机器人一加载就倒了
A: 检查初始高度是否过低，应该 > 0.2m

### Q: 找不到URDF文件
A: 确保先运行 `python src/robot/urdf_generator.py` 生成模型

### Q: 测试一直失败
A: 
1. 检查配置文件格式是否正确
2. 尝试恢复默认配置
3. 查看错误信息

## 📞 获取帮助

- 查看文档：`docs/` 文件夹
- 运行 `make help` 查看可用命令
- 检查 `README.md`

## 🎉 完成！

恭喜完成入门教程！你现在已经：
- ✅ 搭建好了仿真环境
- ✅ 运行了第一个测试
- ✅ 理解了基本工作流程
- ✅ 学会了调整参数

**准备好进入第二阶段了吗？**

下一步将学习：
- 如何让机器人"走"起来
- 步态规划算法
- 平衡控制策略

继续前进，打造你的机器人！🤖

---

**教程版本**: v0.1.0  
**预计完成时间**: 30分钟  
**难度**: ⭐⭐☆☆☆
