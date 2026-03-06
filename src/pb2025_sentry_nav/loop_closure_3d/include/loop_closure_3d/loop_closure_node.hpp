// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LOOP_CLOSURE_3D__LOOP_CLOSURE_NODE_HPP_
#define LOOP_CLOSURE_3D__LOOP_CLOSURE_NODE_HPP_

#include <memory>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace loop_closure_3d
{

struct KeyFrame
{
  int id;
  rclcpp::Time timestamp;
  Eigen::Isometry3d pose;  // odom frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

class LoopClosureNode : public rclcpp::Node
{
public:
  explicit LoopClosureNode(const rclcpp::NodeOptions & options);
  ~LoopClosureNode();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void detectAndCorrectLoop();
  void detectAndCorrectLoopAsync();  // Background thread version
  bool detectLoopClosure(
    const KeyFrame & current_kf,
    KeyFrame & matched_kf,
    Eigen::Isometry3d & relative_pose);

  void updateMapToOdomTransform(
    const KeyFrame & current_kf,
    const KeyFrame & matched_kf,
    const Eigen::Isometry3d & relative_pose);

  void publishTransform();
  void publishVisualization();

  bool shouldAddKeyFrame(const Eigen::Isometry3d & current_pose);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr tf_timer_;

  // Parameters
  double keyframe_distance_threshold_;
  double keyframe_rotation_threshold_;
  double loop_search_radius_;
  double loop_closure_score_threshold_;
  double downsample_resolution_;
  int num_threads_;
  int min_keyframes_for_loop_;
  int skip_recent_keyframes_;

  std::string map_frame_;
  std::string odom_frame_;

  // State
  std::deque<KeyFrame> keyframes_;
  int next_keyframe_id_;
  Eigen::Isometry3d last_keyframe_pose_;
  Eigen::Isometry3d map_to_odom_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  bool has_loop_correction_;
  std::vector<std::pair<int, int>> loop_pairs_;  // For visualization

  // Thread safety for async loop detection
  std::mutex map_to_odom_mutex_;
  std::atomic<bool> loop_detection_running_;
  std::thread loop_detection_thread_;
};

}  // namespace loop_closure_3d

#endif  // LOOP_CLOSURE_3D__LOOP_CLOSURE_NODE_HPP_
