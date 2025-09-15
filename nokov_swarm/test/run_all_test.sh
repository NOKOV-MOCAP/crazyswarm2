#!/bin/bash

# 简化版单元测试运行脚本

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== NokovSwarm 单元测试 ==="
echo "项目根目录: $PROJECT_ROOT"
echo "检查目录结构:"
ls -la "$PROJECT_ROOT" | head -10
echo "检查是否存在src目录:"
if [ -d "$PROJECT_ROOT/src" ]; then
    echo "src目录存在"
    ls -la "$PROJECT_ROOT/src"
else
    echo "src目录不存在"
fi
echo "检查是否存在nokov_swarm目录:"
if [ -d "$PROJECT_ROOT/nokov_swarm" ]; then
    echo "nokov_swarm目录存在"
    ls -la "$PROJECT_ROOT/nokov_swarm"
else
    echo "nokov_swarm目录不存在"
fi
echo "检查测试脚本所在目录:"
echo "SCRIPT_DIR: $SCRIPT_DIR"
ls -la "$SCRIPT_DIR/.."

# 设置环境变量
# 检查是否存在nokov_swarm子目录（CI环境）
if [ -d "$PROJECT_ROOT/nokov_swarm" ]; then
    # CI环境：项目根目录包含nokov_swarm子目录
    export PYTHONPATH="$PROJECT_ROOT:$PROJECT_ROOT/nokov_swarm:$PROJECT_ROOT/nokov_swarm/src:$PYTHONPATH"
    echo "检测到CI环境，使用nokov_swarm子目录路径"
    echo "PYTHONPATH: $PYTHONPATH"
else
    # 本地环境：项目根目录就是nokov_swarm目录
    export PYTHONPATH="$PROJECT_ROOT:$PROJECT_ROOT/src:$PYTHONPATH"
    echo "检测到本地环境，使用直接路径"
    echo "PYTHONPATH: $PYTHONPATH"
fi
export QT_QPA_PLATFORM=offscreen

# 检查ROS2环境
if [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo "ROS2 Galactic环境已加载"
fi

# 运行所有单元测试
echo "运行单元测试..."
# 优先使用conda环境中的python
if command -v conda &> /dev/null && [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "使用conda环境: $CONDA_DEFAULT_ENV"
    python -m pytest "$SCRIPT_DIR/unit_test/" -v
else
    echo "使用系统python3"
    python3 -m pytest "$SCRIPT_DIR/unit_test/" -v
fi

echo "测试完成！"
