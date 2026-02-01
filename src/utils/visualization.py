"""
可视化工具

用于绘制机器人状态、轨迹等
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Dict, List


class Plotter:
    """数据可视化工具"""
    
    @staticmethod
    def plot_stability_metrics(metrics):
        """绘制稳定性指标
        
        Args:
            metrics: StabilityMetrics对象
        """
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('机器人稳定性分析', fontsize=16, fontweight='bold')
        
        times = metrics.history['time']
        
        # 高度曲线
        axes[0].plot(times, metrics.history['height'], 'b-', linewidth=2)
        axes[0].axhline(y=metrics.thresholds['min_height'], 
                       color='r', linestyle='--', label='最小高度阈值')
        axes[0].set_ylabel('高度 (m)', fontsize=12)
        axes[0].set_title('基座高度', fontsize=14)
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # 姿态角度
        axes[1].plot(times, metrics.history['roll'], 'r-', linewidth=2, label='Roll')
        axes[1].plot(times, metrics.history['pitch'], 'g-', linewidth=2, label='Pitch')
        axes[1].axhline(y=metrics.thresholds['max_roll'], 
                       color='r', linestyle='--', alpha=0.5)
        axes[1].axhline(y=-metrics.thresholds['max_roll'], 
                       color='r', linestyle='--', alpha=0.5)
        axes[1].set_ylabel('角度 (°)', fontsize=12)
        axes[1].set_title('姿态角度', fontsize=14)
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # 速度
        axes[2].plot(times, metrics.history['velocity'], 'purple', linewidth=2)
        axes[2].set_xlabel('时间 (s)', fontsize=12)
        axes[2].set_ylabel('速度 (m/s)', fontsize=12)
        axes[2].set_title('线性速度', fontsize=14)
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
    @staticmethod
    def plot_joint_trajectories(joint_history: Dict[str, List[float]]):
        """绘制关节轨迹
        
        Args:
            joint_history: 关节历史数据
        """
        num_joints = len(joint_history)
        cols = 3
        rows = (num_joints + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(15, 4*rows))
        fig.suptitle('关节角度轨迹', fontsize=16, fontweight='bold')
        
        axes = axes.flatten() if num_joints > 1 else [axes]
        
        for idx, (joint_name, angles) in enumerate(joint_history.items()):
            axes[idx].plot(angles, linewidth=2)
            axes[idx].set_title(joint_name, fontsize=12)
            axes[idx].set_xlabel('步数', fontsize=10)
            axes[idx].set_ylabel('角度 (°)', fontsize=10)
            axes[idx].grid(True, alpha=0.3)
        
        # 隐藏多余的子图
        for idx in range(num_joints, len(axes)):
            axes[idx].axis('off')
        
        plt.tight_layout()
        plt.show()
        
    @staticmethod
    def plot_trajectory_2d(positions: np.ndarray):
        """绘制2D轨迹（俯视图）
        
        Args:
            positions: Nx3的位置数组
        """
        plt.figure(figsize=(10, 10))
        
        # 绘制轨迹
        plt.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, alpha=0.7)
        
        # 标记起点和终点
        plt.plot(positions[0, 0], positions[0, 1], 'go', 
                markersize=15, label='起点')
        plt.plot(positions[-1, 0], positions[-1, 1], 'ro', 
                markersize=15, label='终点')
        
        # 添加箭头指示方向
        for i in range(0, len(positions)-1, max(1, len(positions)//20)):
            dx = positions[i+1, 0] - positions[i, 0]
            dy = positions[i+1, 1] - positions[i, 1]
            plt.arrow(positions[i, 0], positions[i, 1], dx, dy,
                     head_width=0.02, head_length=0.01, 
                     fc='blue', ec='blue', alpha=0.3)
        
        plt.xlabel('X (m)', fontsize=12)
        plt.ylabel('Y (m)', fontsize=12)
        plt.title('机器人运动轨迹（俯视图）', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.legend(fontsize=12)
        plt.tight_layout()
        plt.show()
        
    @staticmethod
    def create_animation(positions: np.ndarray, 
                        interval: int = 50,
                        output_path: str = None):
        """创建动画
        
        Args:
            positions: Nx3的位置数组
            interval: 帧间隔（ms）
            output_path: 输出文件路径（可选）
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        
        line, = ax.plot([], [], 'b-', linewidth=2)
        point, = ax.plot([], [], 'ro', markersize=10)
        
        ax.set_xlim(positions[:, 0].min() - 0.1, positions[:, 0].max() + 0.1)
        ax.set_ylim(positions[:, 1].min() - 0.1, positions[:, 1].max() + 0.1)
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('机器人运动动画', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        def init():
            line.set_data([], [])
            point.set_data([], [])
            return line, point
            
        def update(frame):
            x = positions[:frame+1, 0]
            y = positions[:frame+1, 1]
            line.set_data(x, y)
            point.set_data([x[-1]], [y[-1]])
            return line, point
            
        anim = FuncAnimation(fig, update, init_func=init,
                           frames=len(positions), interval=interval,
                           blit=True)
        
        if output_path:
            anim.save(output_path, writer='pillow', fps=1000//interval)
            print(f"✅ 动画已保存: {output_path}")
        else:
            plt.show()


def demo_plotter():
    """演示可视化功能"""
    # 生成示例数据
    t = np.linspace(0, 10, 1000)
    
    # 模拟高度变化（轻微震荡）
    height = 0.25 + 0.02 * np.sin(2 * np.pi * 0.5 * t)
    
    # 模拟姿态角度
    roll = 5 * np.sin(2 * np.pi * 0.3 * t)
    pitch = 3 * np.sin(2 * np.pi * 0.4 * t)
    
    # 创建假的metrics对象
    class DemoMetrics:
        def __init__(self):
            self.history = {
                'time': t.tolist(),
                'height': height.tolist(),
                'roll': roll.tolist(),
                'pitch': pitch.tolist(),
                'velocity': (0.01 * np.abs(np.sin(2 * np.pi * 0.2 * t))).tolist()
            }
            self.thresholds = {
                'min_height': 0.15,
                'max_roll': 30
            }
    
    metrics = DemoMetrics()
    
    # 绘制
    print("📊 演示稳定性指标绘图...")
    Plotter.plot_stability_metrics(metrics)
    
    # 绘制2D轨迹
    print("📊 演示2D轨迹绘图...")
    positions = np.column_stack([
        0.5 * np.sin(2 * np.pi * 0.1 * t),
        0.5 * np.cos(2 * np.pi * 0.1 * t),
        height
    ])
    Plotter.plot_trajectory_2d(positions)


if __name__ == '__main__':
    demo_plotter()
