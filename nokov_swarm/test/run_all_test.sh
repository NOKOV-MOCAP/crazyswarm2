#!/bin/bash

# 简化版单元测试运行脚本

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== NokovSwarm 单元测试 ==="
echo "项目根目录: $PROJECT_ROOT"

# 设置环境变量
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"
export QT_QPA_PLATFORM=offscreen

# 检查ROS2环境
if [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo "ROS2 Galactic环境已加载"
fi

# 运行所有单元测试
echo "运行单元测试..."
python3 -m pytest "$SCRIPT_DIR/unit_test/" -v

echo "测试完成！"
