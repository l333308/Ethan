# Makefile for Humanoid Robot Simulation

.PHONY: help install setup test clean docs

help:
	@echo "具身机器人仿真项目 - 可用命令："
	@echo ""
	@echo "  make install    - 安装Python依赖"
	@echo "  make setup      - 生成机器人模型"
	@echo "  make test       - 运行基础站立测试"
	@echo "  make test-dist  - 运行扰动测试"
	@echo "  make debug      - 启动交互式调试"
	@echo "  make clean      - 清理生成文件"
	@echo "  make docs       - 打开文档"
	@echo ""

install:
	@echo "📦 安装依赖..."
	pip install -r requirements.txt
	@echo "✅ 依赖安装完成"

setup:
	@echo "🤖 生成机器人URDF模型..."
	python src/robot/urdf_generator.py
	@echo "✅ 模型生成完成"

test: setup
	@echo "🧪 运行基础站立测试..."
	python tests/test_standing.py --mode basic --duration 5

test-dist: setup
	@echo "💨 运行扰动测试..."
	python tests/test_standing.py --mode disturbance --duration 10

debug: setup
	@echo "🔧 启动交互式调试..."
	python src/simulation/environment.py

viz:
	@echo "📊 演示可视化工具..."
	python src/utils/visualization.py

quickstart:
	@echo "🚀 运行快速启动脚本..."
	python quickstart.py

clean:
	@echo "🧹 清理生成文件..."
	rm -rf models/*.urdf
	rm -rf __pycache__
	rm -rf src/__pycache__
	rm -rf src/*/__pycache__
	rm -rf tests/__pycache__
	rm -rf *.pyc
	rm -rf logs/
	rm -rf simulation_output/
	@echo "✅ 清理完成"

docs:
	@echo "📚 文档位置："
	@echo "  - 第一阶段指南: docs/phase1_guide.md"
	@echo "  - 开发路线图: docs/roadmap.md"
	@echo "  - 系统架构: docs/architecture.md"
	@echo "  - 总结文档: docs/phase1_summary.md"
