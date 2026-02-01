"""
基础验证脚本（不需要PyBullet）

验证项目核心功能：
1. URDF生成
2. 配置加载
3. 基础数据结构
"""

import sys
from pathlib import Path
import yaml

# 添加项目路径
project_root = Path(__file__).parent
sys.path.append(str(project_root))

def test_imports():
    """测试基础导入"""
    print("=" * 60)
    print("测试1: 检查依赖导入")
    print("=" * 60)
    
    try:
        import numpy as np
        print(f"✅ NumPy {np.__version__}")
    except ImportError as e:
        print(f"❌ NumPy 导入失败: {e}")
        return False
    
    try:
        import matplotlib
        print(f"✅ Matplotlib {matplotlib.__version__}")
    except ImportError as e:
        print(f"❌ Matplotlib 导入失败: {e}")
        return False
    
    try:
        import scipy
        print(f"✅ SciPy {scipy.__version__}")
    except ImportError as e:
        print(f"❌ SciPy 导入失败: {e}")
        return False
    
    try:
        import yaml
        print(f"✅ PyYAML 已安装")
    except ImportError as e:
        print(f"❌ PyYAML 导入失败: {e}")
        return False
    
    try:
        import pybullet as p
        # 尝试连接测试
        client = p.connect(p.DIRECT)
        version_info = p.getAPIVersion()
        p.disconnect()
        print(f"✅ PyBullet (API版本: {version_info})")
        return True
    except ImportError:
        print(f"⚠️  PyBullet 未安装（仿真功能不可用）")
        return False
    except Exception as e:
        print(f"⚠️  PyBullet 安装但无法连接: {e}")
        return False
    
    return True


def test_config():
    """测试配置加载"""
    print("\n" + "=" * 60)
    print("测试2: 配置文件加载")
    print("=" * 60)
    
    config_path = project_root / 'config' / 'robot_config.yaml'
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        print(f"✅ 配置文件加载成功: {config_path}")
        print(f"   - 机器人名称: {config['robot']['name']}")
        print(f"   - 高度: {config['robot']['dimensions']['height']}m")
        print(f"   - 重量: {config['robot']['dimensions']['weight']}kg")
        print(f"   - 自由度: {config['robot']['dof']['total']} DOF")
        return True
    except Exception as e:
        print(f"❌ 配置加载失败: {e}")
        return False


def test_urdf_generator():
    """测试URDF生成"""
    print("\n" + "=" * 60)
    print("测试3: URDF模型生成")
    print("=" * 60)
    
    try:
        from src.robot.urdf_generator import URDFGenerator
        
        config_path = project_root / 'config' / 'robot_config.yaml'
        urdf_path = project_root / 'models' / 'humanoid_v1.urdf'
        
        generator = URDFGenerator(str(config_path))
        generator.generate(str(urdf_path))
        
        # 验证文件存在
        if urdf_path.exists():
            file_size = urdf_path.stat().st_size
            print(f"✅ URDF文件生成成功")
            print(f"   - 路径: {urdf_path}")
            print(f"   - 大小: {file_size} 字节")
            
            # 读取并检查内容
            with open(urdf_path, 'r') as f:
                content = f.read()
                joint_count = content.count('<joint name=')
                link_count = content.count('<link name=')
                print(f"   - 链接数: {link_count}")
                print(f"   - 关节数: {joint_count}")
            
            return True
        else:
            print(f"❌ URDF文件未生成")
            return False
            
    except Exception as e:
        print(f"❌ URDF生成失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_utils():
    """测试工具模块"""
    print("\n" + "=" * 60)
    print("测试4: 工具模块")
    print("=" * 60)
    
    try:
        from src.utils.metrics import StabilityMetrics
        metrics = StabilityMetrics()
        print(f"✅ StabilityMetrics 导入成功")
        print(f"   - 稳定性阈值: {metrics.thresholds}")
        return True
    except Exception as e:
        print(f"❌ 工具模块导入失败: {e}")
        return False


def test_visualization():
    """测试可视化模块"""
    print("\n" + "=" * 60)
    print("测试5: 可视化模块")
    print("=" * 60)
    
    try:
        from src.utils.visualization import Plotter
        print(f"✅ Plotter 导入成功")
        print(f"   - 可用方法: plot_stability_metrics, plot_trajectory_2d")
        return True
    except Exception as e:
        print(f"❌ 可视化模块导入失败: {e}")
        return False


def main():
    """主函数"""
    print("\n" + "🤖" * 30)
    print("具身机器人项目 - 基础功能验证")
    print("🤖" * 30 + "\n")
    
    results = []
    
    # 运行所有测试
    results.append(("依赖导入", test_imports()))
    results.append(("配置加载", test_config()))
    results.append(("URDF生成", test_urdf_generator()))
    results.append(("工具模块", test_utils()))
    results.append(("可视化模块", test_visualization()))
    
    # 汇总结果
    print("\n" + "=" * 60)
    print("测试结果汇总")
    print("=" * 60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{name:.<40} {status}")
    
    print(f"\n通过率: {passed}/{total} ({passed/total*100:.1f}%)")
    
    if passed == total:
        print("\n🎉 所有测试通过！项目基础功能正常")
        print("\n📝 注意:")
        print("   - URDF模型生成功能正常")
        print("   - 如需运行仿真测试，请安装PyBullet")
        print("   - 基础数据分析功能可用")
        return 0
    else:
        print("\n⚠️  部分测试失败，请检查错误信息")
        return 1


if __name__ == '__main__':
    sys.exit(main())
