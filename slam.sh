#!/bin/bash
# SLAM launch wrapper: auto-save grid map + 3D PCD + rosbag on exit
# Usage: ./slam.sh [extra launch args...]
# Example: ./slam.sh world:=rmuc_2025


# Source 环境变量
source install/setup.bash

NAMESPACE="red_standard_robot1"
MAP_SAVE_DIR="src/pb2025_sentry_nav/point_lio/PCD"
MAP_NAME="scans"
BAG_NAME="slam_bag_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${MAP_SAVE_DIR}/${BAG_NAME}"

# Wait for nodes to start, then record rosbag in a separate session
echo "[slam.sh] Starting rosbag recording to ${BAG_PATH} ..."
setsid ros2 bag record -a -o "${BAG_PATH}" &
BAG_PID=$!
echo "[slam.sh] Rosbag PID: ${BAG_PID}"

sleep 2

# Launch SLAM in a separate session so Ctrl+C won't reach it
setsid ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py slam:=True "$@" &
LAUNCH_PID=$!



cleanup() {
    echo ""

    # Stop rosbag - kill entire session group
    echo "[slam.sh] Stopping rosbag recording..."
    kill -INT -$BAG_PID 2>/dev/null
    sleep 5
    kill -0 $BAG_PID 2>/dev/null && kill -TERM -$BAG_PID 2>/dev/null
    wait $BAG_PID 2>/dev/null
    echo "[slam.sh] Rosbag saved to ${BAG_PATH}/"

    # Save grid map via map_saver service
    echo "[slam.sh] Saving grid map..."
    ros2 service call /"${NAMESPACE}"/map_saver/save_map nav2_msgs/srv/SaveMap \
        "{map_topic: '/${NAMESPACE}/map', map_url: '${MAP_SAVE_DIR}/${MAP_NAME}', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.25, occupied_thresh: 0.65}"
    echo "[slam.sh] Grid map saved to ${MAP_SAVE_DIR}/${MAP_NAME}.pgm/.yaml"

    # Shutdown launch - kill entire session group
    echo "[slam.sh] Shutting down launch..."
    kill -INT -$LAUNCH_PID 2>/dev/null
    wait $LAUNCH_PID 2>/dev/null
    echo "[slam.sh] 3D PCD saved to ${MAP_SAVE_DIR}/scans.pcd (by point_lio)"
    echo "[slam.sh] Done."
    exit 0
}

trap cleanup SIGINT

# Keep script alive
while kill -0 $LAUNCH_PID 2>/dev/null; do
    wait $LAUNCH_PID 2>/dev/null
done
