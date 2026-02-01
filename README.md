# 具身机器人仿真项目

## 项目概述
30-50cm级小型类人/双足机器人仿真与控制系统

## 设计理念
- ✅ 准静态/慢速双足
- ✅ 位置控制为主，IMU做姿态修正
- ✅ 软件仿真先行（仿真≥80%成熟度再上实体）
- ✅ 模块化，可替换

## 开发阶段
- [x] 第一阶段：仿真建模（当前阶段）
- [ ] 第二阶段：控制算法开发
- [ ] 第三阶段：硬件集成
- [ ] 第四阶段：系统优化

## 技术栈
- **仿真引擎**: PyBullet
- **开发语言**: Python 3.8+
- **控制框架**: 自研分层控制架构
- **未来集成**: ROS 2

# 具身机器人仿真项目

30-50cm级小型类人/双足机器人仿真与控制系统

## ⚡ 5秒开始

```bash
source .venv/bin/activate          # 激活环境
python test_simulation.py          # 测试仿真
python src/simulation/environment.py  # GUI调试
```

## 📖 完整指南

**新手必读**: [SETUP.md](SETUP.md) - 包含所有你需要知道的

## 🎯 项目目标

- ✅ 准静态站立（第一阶段 - 已完成）
- 🔜 慢速行走（第二阶段）
- 📅 硬件集成（第三阶段）
- 📅 智能化（第四阶段）

## 🤖 机器人规格

- 高度: 40cm
- 重量: 2kg  
- 自由度: 9 DOF
- 预算: ¥2500-4500

## 📚 文档

| 文档 | 说明 |
|------|------|
| [SETUP.md](SETUP.md) | **快速上手指南** ⭐ |
| [docs/getting_started.md](docs/getting_started.md) | 30分钟入门教程 |
| [docs/roadmap.md](docs/roadmap.md) | 完整开发路线 |
| [docs/phase1_guide.md](docs/phase1_guide.md) | 第一阶段详细指南 |
| [docs/architecture.md](docs/architecture.md) | 系统架构 |

## 🛠️ 技术栈

- Python 3.9 + PyBullet 3.2.7
- NumPy, SciPy, Matplotlib
- URDF机器人建模
- 模块化控制架构

## ✅ 当前状态

- 环境: ✅ 已配置
- 代码: ✅ 已测试
- 文档: ✅ 已完善
- 仿真: ✅ 可运行

**可以开始开发了！**

## 📝 快速参考

```bash
# 验证环境
python verify_setup.py

# 生成模型
python src/robot/urdf_generator.py

# 运行测试
python tests/test_standing.py --mode basic

# 查看帮助
make help
```

## License

MIT

## 项目结构
```
.
├── config/              # 配置文件
├── models/              # 机器人模型文件(URDF)
├── src/
│   ├── robot/          # 机器人模型定义
│   ├── simulation/     # 仿真环境
│   ├── control/        # 控制算法
│   └── utils/          # 工具函数
├── tests/              # 测试脚本
└── docs/               # 文档
```

## 硬件规格（目标）
- **尺寸**: 35-45cm
- **重量**: ≤2.5kg
- **自由度**: 9-13 DOF
- **下肢**: 单腿4 DOF × 2 = 8 DOF
- **上肢**: 可选，2 DOF × 2 = 4 DOF
- **头部**: 1 DOF

## 预算评估
- **总成本**: ¥2500-4500
- **开发周期**: 2-3个月（单人）

## License
MIT
