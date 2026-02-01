#!/usr/bin/env python3
"""
快速启动脚本

一键完成模型生成、环境搭建、基础测试
"""

import sys
import subprocess
from pathlib import Path


def print_header(text: str):
    """打印标题"""
    print("\n" + "=" * 60)
    print(text)
    print("=" * 60 + "\n")


def run_command(cmd: list, description: str) -> bool:
    """运行命令
    
    Args:
        cmd: 命令列表
        description: 描述
        
    Returns:
        是否成功
    """
    print(f"🔄 {description}...")
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print(f"✅ {description} 完成")
        if result.stdout:
            print(result.stdout)
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} 失败")
        print(f"错误信息: {e.stderr}")
        return False


def main():
    """主函数"""
    print_header("🤖 具身机器人仿真 - 快速启动")
    
    base_path = Path(__file__).parent
    
    # 步骤1: 检查依赖
    print_header("步骤1: 检查依赖")
    try:
        import pybullet
        import numpy
        import yaml
        import matplotlib
        print("✅ 所有依赖已安装")
        print(f"   - PyBullet: {pybullet.__version__}")
        print(f"   - NumPy: {numpy.__version__}")
    except ImportError as e:
        print(f"❌ 缺少依赖: {e}")
        print("\n请运行以下命令安装:")
        print("  pip install -r requirements.txt")
        sys.exit(1)
    
    # 步骤2: 生成URDF模型
    print_header("步骤2: 生成机器人URDF模型")
    if not run_command(
        [sys.executable, str(base_path / "src/robot/urdf_generator.py")],
        "生成URDF模型"
    ):
        sys.exit(1)
    
    # 检查URDF文件
    urdf_path = base_path / "models/humanoid_v1.urdf"
    if urdf_path.exists():
        print(f"✅ URDF文件已生成: {urdf_path}")
    else:
        print(f"❌ URDF文件未找到: {urdf_path}")
        sys.exit(1)
    
    # 步骤3: 询问用户要运行的测试
    print_header("步骤3: 选择测试模式")
    print("请选择要运行的测试:")
    print("  1. 交互式调试（手动调节关节）")
    print("  2. 基础站立测试")
    print("  3. 扰动测试")
    print("  4. 跳过测试")
    
    choice = input("\n请输入选择 (1-4): ").strip()
    
    if choice == "1":
        print_header("启动交互式调试")
        print("💡 提示:")
        print("   - 使用GUI滑块调整关节角度")
        print("   - 鼠标拖拽旋转视角")
        print("   - 按Ctrl+C退出")
        input("\n按Enter继续...")
        subprocess.run([
            sys.executable,
            str(base_path / "src/simulation/environment.py")
        ])
        
    elif choice == "2":
        print_header("启动基础站立测试")
        duration = input("测试时长（秒，默认5）: ").strip() or "5"
        subprocess.run([
            sys.executable,
            str(base_path / "tests/test_standing.py"),
            "--mode", "basic",
            "--duration", duration
        ])
        
    elif choice == "3":
        print_header("启动扰动测试")
        duration = input("测试时长（秒，默认10）: ").strip() or "10"
        subprocess.run([
            sys.executable,
            str(base_path / "tests/test_standing.py"),
            "--mode", "disturbance",
            "--duration", duration
        ])
        
    elif choice == "4":
        print("\n✅ 设置完成，跳过测试")
        
    else:
        print("\n❌ 无效选择")
        sys.exit(1)
    
    # 完成
    print_header("🎉 设置完成!")
    print("下一步:")
    print("  1. 查看文档: docs/phase1_guide.md")
    print("  2. 调整配置: config/robot_config.yaml")
    print("  3. 运行更多测试:")
    print("     python tests/test_standing.py --help")
    print("  4. 开始开发控制算法（第二阶段）")


if __name__ == '__main__':
    main()
