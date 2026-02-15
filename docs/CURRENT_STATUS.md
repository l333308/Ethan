# 第二阶段控制算法开发 - 状态报告

## 执行摘要

已完成第二阶段的基础架构搭建，包括姿态控制器、重心控制器和测试框架。当前正在调试M1（稳定站立）里程碑。

## 已完成的工作

### 1. 控制算法模块 ✅

创建了 `src/control/posture_controller.py`，包含：

- **PIDGains**: PID增益参数配置
- **PostureController**: 姿态平衡控制器
  - Roll/Pitch轴独立PID控制
  - 积分饱和限制
  - 误差微分计算
- **CenterOfMassController**: 重心高度控制器
  - 高度PID控制
  - 腿部关节协调
- **StandingController**: 站立控制器组合
  - 整合姿态和重心控制
  - 基准姿态管理
  - 控制命令生成

### 2. 评估指标增强 ✅

扩展了 `src/utils/metrics.py`：

- 添加 `add_measurement()` 兼容接口
- 添加 `calculate_scores()` 详细评分
- 位置、姿态、能量效率三维评分体系

### 3. 测试框架 ✅

创建了三个测试脚本：

- `tests/test_standing_control.py`: 完整PID控制测试
- `tests/test_simple_standing.py`: 简化版站立测试
- `tools/explore_pose.py`: GUI姿态探索工具

### 4. 文档 ✅

- `docs/phase2_progress.md`: 进展跟踪文档
- `docs/CURRENT_STATUS.md`: 本文档

## 当前挑战

### 问题：机器人无法保持站立

**现象**: 
- 在所有测试中，机器人几乎立即倒下
- 即使使用固定姿态也无法稳定
- 高度迅速下降，Roll/Pitch角度剧烈变化

**可能原因**:

1. **初始姿态不在平衡点**
   - 需要找到零力矩点(ZMP)在支撑多边形内的姿态
   - 当前配置可能重心过高或过前/过后

2. **关节控制力不足**
   - PyBullet的 `POSITION_CONTROL` 模式可能需要更高的`force`参数
   - 当前设置为100，可能需要调整

3. **物理参数不合理**
   - 摩擦系数、阻尼等可能需要调整
   - 关节限制可能过于宽松

4. **控制器增益过大**
   - PID参数可能导致过度振荡
   - 需要系统化的参数调优

## 下一步行动计划

### 立即行动（今天完成）

1. **使用姿态探索工具找到静态稳定姿态** 🎯
   ```bash
   python tools/explore_pose.py
   ```
   - 手动调节GUI滑块
   - 找到能保持至少10秒的姿态配置
   - 记录该配置

2. **验证静态姿态**
   - 使用固定姿态（无控制器）测试30秒
   - 确认机器人能自然保持平衡

3. **在稳定姿态基础上添加控制**
   - 使用极小的PID增益（Kp=0.1）
   - 仅针对小扰动进行补偿

### 短期目标（本周）

4. **参数调优**
   - 系统化测试不同PID增益组合
   - 记录每组参数的稳定性得分
   - 找到最优参数集

5. **完成M1里程碑**
   - 重心控制精度 ±1cm
   - 姿态角度 ±3°
   - 持续时间 ≥30秒

### 中期目标（下周）

6. **开始M2：原地踏步**
   - 实现单脚抬起轨迹
   - 重心转移控制
   - 左右脚交替

## 技术建议

### 调试技巧

1. **从最简单开始**
   ```python
   # 步骤1: 纯固定姿态（无控制）
   # 步骤2: 固定姿态 + 重力补偿
   # 步骤3: 添加极小的姿态反馈
   # 步骤4: 逐步增加控制器增益
   ```

2. **使用数据记录**
   - 记录每次测试的关键参数
   - 绘制时间序列图表
   - 分析失败模式

3. **物理直觉检查**
   - 计算重心位置是否在支撑多边形内
   - 检查关节力矩是否合理
   - 验证能量守恒

### 参数调优策略

1. **Ziegler-Nichols方法**
   - 先设置Ki=Kd=0，只调Kp
   - 找到产生持续振荡的临界Kp
   - 根据规则计算Ki和Kd

2. **手动调优经验**
   - Kp过大：快速振荡
   - Ki过大：慢速振荡，超调
   - Kd过大：对噪声敏感，抖动

3. **保守起步**
   - 从极小的增益开始（Kp=0.1）
   - 逐步翻倍增加
   - 观察稳定性变化

## 代码示例

### 使用姿态探索工具

```bash
cd /Users/sevan/www/deep-learning/Ethan
source .venv/bin/activate
python tools/explore_pose.py
```

### 测试找到的姿态

```python
# 在test_simple_standing.py中使用
standing_pose = {
    'left_hip_pitch': -10.0,
    'left_knee': 25.0,
    'left_ankle_pitch': -12.0,
    'right_hip_pitch': -10.0,
    'right_knee': 25.0,
    'right_ankle_pitch': -12.0,
    # ... 其他关节
}
```

## 资源链接

- PyBullet文档: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/
- ZMP理论: https://en.wikipedia.org/wiki/Zero_moment_point
- PID控制: https://en.wikipedia.org/wiki/PID_controller

## 联系与支持

如果遇到技术问题：
1. 检查 `docs/phase2_progress.md` 了解已知问题
2. 运行 `python verify_setup.py` 验证环境
3. 查看测试日志和错误信息

---

**报告时间**: 2026-02-12  
**作者**: Codex AI Assistant  
**项目**: 具身机器人仿真 - 第二阶段控制算法开发  
**下次更新**: 找到稳定姿态后
