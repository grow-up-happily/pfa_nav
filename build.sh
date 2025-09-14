#!/bin/bash


# 进入工作空间目录
cd ~/sight/pfa-nav || { echo "❌ 未找到目录"; exit 1; }
# 脚本所在目录
WORKSPACE_DIR=$(pwd)

echo " 开始编译工作空间: $WORKSPACE_DIR"

# 编译，最多使用2个并行线程
colcon build --parallel-workers 2

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

