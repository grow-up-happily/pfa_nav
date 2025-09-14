#!/bin/bash

# 进入你的工作空间目录
cd ~/sight/pfa-nav || exit

# Source 环境变量
source install/setup.bash

# 拷贝 pcd 文件
cp src/point_lio/PCD/scans.pcd src/pb2025_nav_bringup/pcd/reality/game.pcd
echo "✅ 已拷贝 scans.pcd 为 game.pcd"

# 编译
colcon build --parallel-workers 2
if [ $? -ne 0 ]; then
  echo "❌ 编译失败！退出"
  exit 1
fi

# 再 source 一次编译好的环境
source install/setup.bash
echo "✅ 环境已加载"

# 启动导航
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=game slam:=False use_robot_state_pub:=True

