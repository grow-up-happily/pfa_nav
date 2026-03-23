#!/bin/bash

# 建图模式

# 切换到当前脚本所在的工作空间，避免误用其他同名工作区
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || { echo "❌ 未找到目录"; exit 1; }
WORKSPACE_DIR=$(pwd)

echo " 开始编译工作空间: $WORKSPACE_DIR"

# 编译，最多使用2个并行线程
# colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查编译是否成功
if [ $? -eq 0 ]; then
    echo "✅ 编译完成，准备 source 环境..."
    source install/setup.bash
    echo "✅ 环境已 source 完成。"

    # 执行建图 launch 文件
    echo "️  启动建图功能..."
    ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
      slam:=True \
      use_robot_state_pub:=True

else
    echo "❌ 编译失败，请检查错误日志。"
fi
