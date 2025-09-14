#!/bin/bash

# 工作空间目录
WORKSPACE_DIR=~/sight/pfa-nav

# 进入工作空间
cd $WORKSPACE_DIR || { echo "❌ 未找到目录"; exit 1; }

# Source ROS 2 和工作空间环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 地图保存目标目录
MAP_DIR=$WORKSPACE_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality
BACKUP_MAP_DIR=$WORKSPACE_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation

# 确保目录存在
mkdir -p $MAP_DIR

# 保存地图到当前目录
ros2 run nav2_map_server map_saver_cli -f game

# 移动保存好的地图文件到目标目录
cp game.yaml $BACKUP_MAP_DIR/
cp game.pgm $BACKUP_MAP_DIR/
mv game.yaml $MAP_DIR/
mv game.pgm $MAP_DIR/


echo "✅ 地图已保存到 $MAP_DIR and $BACKUP_MAP_DIR"

