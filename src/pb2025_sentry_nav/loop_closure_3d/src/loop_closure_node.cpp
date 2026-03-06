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

#include "loop_closure_3d/loop_closure_node.hpp"

#include <algorithm>
#include <chrono>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf2_eigen/tf2_eigen.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"

namespace loop_closure_3d
{

LoopClosureNode::LoopClosureNode(const rclcpp::NodeOptions & options)
: Node("loop_closure_3d", options),
  next_keyframe_id_(0),
  last_keyframe_pose_(Eigen::Isometry3d::Identity()),
  map_to_odom_(Eigen::Isometry3d::Identity()),
  has_loop_correction_(false),
  loop_detection_running_(false)
{
  // Declare parameters
  this->declare_parameter("keyframe_distance_threshold", 1.0);
  this->declare_parameter("keyframe_rotation_threshold", 0.3);
  this->declare_parameter("loop_search_radius", 10.0);
  this->declare_parameter("loop_closure_score_threshold", 0.5);
  this->declare_parameter("downsample_resolution", 0.3);
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("min_keyframes_for_loop", 10);
  this->declare_parameter("skip_recent_keyframes", 30);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");

  // Get parameters
  this->get_parameter("keyframe_distance_threshold", keyframe_distance_threshold_);
  this->get_parameter("keyframe_rotation_threshold", keyframe_rotation_threshold_);
  this->get_parameter("loop_search_radius", loop_search_radius_);
  this->get_parameter("loop_closure_score_threshold", loop_closure_score_threshold_);
  this->get_parameter("downsample_resolution", downsample_resolution_);
  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("min_keyframes_for_loop", min_keyframes_for_loop_);
  this->get_parameter("skip_recent_keyframes", skip_recent_keyframes_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);

  // Initialize
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Subscribers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_registered", 10,
    std::bind(&LoopClosureNode::cloudCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "aft_mapped_to_init", 10,
    std::bind(&LoopClosureNode::odometryCallback, this, std::placeholders::_1));

  // Publishers
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "loop_closure_markers", 10);

  // Timer for TF publishing (high frequency to ensure availability)
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&LoopClosureNode::publishTransform, this));

  // Immediately publish initial identity transform
  publishTransform();

  RCLCPP_INFO(this->get_logger(), "Loop closure node initialized");
  RCLCPP_INFO(this->get_logger(), "Publishing initial map->odom transform (identity)");
  RCLCPP_INFO(
    this->get_logger(), "  keyframe_distance: %.2f m, rotation: %.2f rad",
    keyframe_distance_threshold_, keyframe_rotation_threshold_);
  RCLCPP_INFO(
    this->get_logger(), "  loop_search_radius: %.2f m, score_threshold: %.2f",
    loop_search_radius_, loop_closure_score_threshold_);
}

LoopClosureNode::~LoopClosureNode()
{
  // Wait for loop detection thread to finish
  if (loop_detection_thread_.joinable()) {
    loop_detection_thread_.join();
  }
}

void LoopClosureNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

void LoopClosureNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!latest_odom_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for odometry messages...");
    return;
  }

  // Convert ROS message to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  // Accumulate cloud
  *accumulated_cloud_ += *cloud;

  // Get current pose from odometry
  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
  current_pose.translation() << latest_odom_->pose.pose.position.x,
    latest_odom_->pose.pose.position.y,
    latest_odom_->pose.pose.position.z;
  current_pose.linear() = Eigen::Quaterniond(
    latest_odom_->pose.pose.orientation.w,
    latest_odom_->pose.pose.orientation.x,
    latest_odom_->pose.pose.orientation.y,
    latest_odom_->pose.pose.orientation.z).toRotationMatrix();

  // Check if we should add a new keyframe
  if (shouldAddKeyFrame(current_pose)) {
    // Downsample accumulated cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(accumulated_cloud_);
    voxel_filter.setLeafSize(
      downsample_resolution_, downsample_resolution_, downsample_resolution_);
    voxel_filter.filter(*downsampled);

    // Create keyframe
    KeyFrame kf;
    kf.id = next_keyframe_id_++;
    kf.timestamp = msg->header.stamp;
    kf.pose = current_pose;
    kf.cloud = downsampled;

    keyframes_.push_back(kf);
    last_keyframe_pose_ = current_pose;

    // Clear accumulated cloud
    accumulated_cloud_->clear();

    RCLCPP_INFO(
      this->get_logger(), "Added keyframe %d at (%.2f, %.2f, %.2f), total: %zu",
      kf.id, current_pose.translation().x(), current_pose.translation().y(),
      current_pose.translation().z(), keyframes_.size());

    // Try to detect loop closure
    if (static_cast<int>(keyframes_.size()) >= min_keyframes_for_loop_) {
      detectAndCorrectLoopAsync();
    }
  }
}

bool LoopClosureNode::shouldAddKeyFrame(const Eigen::Isometry3d & current_pose)
{
  if (keyframes_.empty()) {
    return true;
  }

  // Check translation distance only (ignore rotation to avoid duplicates from spinning)
  double distance = (current_pose.translation() - last_keyframe_pose_.translation()).norm();

  return distance > keyframe_distance_threshold_;
}

void LoopClosureNode::detectAndCorrectLoopAsync()
{
  // If already running, skip
  if (loop_detection_running_.load()) {
    RCLCPP_DEBUG(this->get_logger(), "[LOOP] Detection already running, skipping");
    return;
  }

  // Join previous thread if exists
  if (loop_detection_thread_.joinable()) {
    loop_detection_thread_.join();
  }

  // Launch new thread
  loop_detection_running_.store(true);
  loop_detection_thread_ = std::thread([this]() {
    detectAndCorrectLoop();
    loop_detection_running_.store(false);
  });
}

void LoopClosureNode::detectAndCorrectLoop()
{
  const KeyFrame & current_kf = keyframes_.back();

  RCLCPP_INFO(
    this->get_logger(),
    "[LOOP] Checking loop for KF %d at (%.2f, %.2f, %.2f), total KFs: %zu",
    current_kf.id,
    current_kf.pose.translation().x(),
    current_kf.pose.translation().y(),
    current_kf.pose.translation().z(),
    keyframes_.size());

  auto start_time = std::chrono::steady_clock::now();

  KeyFrame matched_kf;
  Eigen::Isometry3d relative_pose;

  if (detectLoopClosure(current_kf, matched_kf, relative_pose)) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();

    RCLCPP_INFO(
      this->get_logger(), "🔄 Loop closure detected! Current KF: %d, Matched KF: %d (took %ld ms)",
      current_kf.id, matched_kf.id, elapsed);

    updateMapToOdomTransform(current_kf, matched_kf, relative_pose);

    // Record loop pair for visualization
    loop_pairs_.push_back(std::make_pair(current_kf.id, matched_kf.id));
    publishVisualization();

    has_loop_correction_ = true;
  } else {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_INFO(
      this->get_logger(), "[LOOP] No loop closure found for KF %d (took %ld ms)",
      current_kf.id, elapsed);
  }
}

bool LoopClosureNode::detectLoopClosure(
  const KeyFrame & current_kf,
  KeyFrame & matched_kf,
  Eigen::Isometry3d & relative_pose)
{
  // Search for candidate keyframes within radius
  std::vector<size_t> candidates;

  // Check if we have enough keyframes to skip recent ones
  if (keyframes_.size() <= static_cast<size_t>(skip_recent_keyframes_)) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "[LOOP] Not enough keyframes (%zu <= %d), waiting...",
      keyframes_.size(), skip_recent_keyframes_);
    return false;
  }

  size_t search_end = keyframes_.size() - skip_recent_keyframes_;
  RCLCPP_INFO(
    this->get_logger(),
    "[LOOP] Searching candidates in KF [0-%zu], skipping recent %d",
    search_end - 1, skip_recent_keyframes_);

  for (size_t i = 0; i < search_end; ++i) {
    const KeyFrame & kf = keyframes_[i];
    double distance = (kf.pose.translation() - current_kf.pose.translation()).norm();

    if (distance < loop_search_radius_) {
      candidates.push_back(i);
    }
  }

  if (candidates.empty()) {
    RCLCPP_INFO(
      this->get_logger(), "[LOOP] No candidates found within %.2fm radius",
      loop_search_radius_);
    return false;
  }

  // Limit number of candidates to avoid excessive computation
  const size_t MAX_CANDIDATES = 3;
  if (candidates.size() > MAX_CANDIDATES) {
    RCLCPP_WARN(
      this->get_logger(),
      "[LOOP] Too many candidates (%zu), limiting to %zu closest ones",
      candidates.size(), MAX_CANDIDATES);

    // Sort by distance and keep only closest MAX_CANDIDATES
    std::sort(candidates.begin(), candidates.end(),
      [&](size_t a, size_t b) {
        double dist_a = (keyframes_[a].pose.translation() - current_kf.pose.translation()).norm();
        double dist_b = (keyframes_[b].pose.translation() - current_kf.pose.translation()).norm();
        return dist_a < dist_b;
      });
    candidates.resize(MAX_CANDIDATES);
  }

  RCLCPP_INFO(
    this->get_logger(), "[LOOP] Found %zu loop candidates for KF %d",
    candidates.size(), current_kf.id);

  // Try GICP registration with each candidate
  double best_score = std::numeric_limits<double>::max();
  size_t best_candidate_idx = 0;
  Eigen::Isometry3d best_transform = Eigen::Isometry3d::Identity();

  for (size_t idx : candidates) {
    const KeyFrame & candidate_kf = keyframes_[idx];

    RCLCPP_INFO(
      this->get_logger(), "[LOOP]   Processing candidate KF %d (idx=%zu)...",
      candidate_kf.id, idx);

    try {
      // Safety check: ensure clouds are not empty
      if (!candidate_kf.cloud || candidate_kf.cloud->empty() ||
          !current_kf.cloud || current_kf.cloud->empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "[LOOP]   Skipping candidate KF %d due to empty cloud", candidate_kf.id);
        continue;
      }

      // Prepare clouds for small_gicp
      auto target = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *candidate_kf.cloud, downsample_resolution_);

      auto source = small_gicp::voxelgrid_sampling_omp<
        pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
        *current_kf.cloud, downsample_resolution_);

      // Check if downsampling resulted in valid clouds
      if (!target || target->empty() || !source || source->empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "[LOOP]   Skipping candidate KF %d due to empty downsampled cloud", candidate_kf.id);
        continue;
      }

      RCLCPP_INFO(
        this->get_logger(), "[LOOP]   Cloud sizes: target=%zu, source=%zu",
        target->size(), source->size());

      small_gicp::estimate_covariances_omp(*target, 20, num_threads_);
      small_gicp::estimate_covariances_omp(*source, 20, num_threads_);

      auto target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target, small_gicp::KdTreeBuilderOMP(num_threads_));

      // Initial guess: relative pose from odometry
      Eigen::Isometry3d initial_guess = candidate_kf.pose.inverse() * current_kf.pose;

      // Perform GICP registration
      small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
      registration.reduction.num_threads = num_threads_;
      registration.rejector.max_dist_sq = 1.0;
      registration.optimizer.max_iterations = 64;  // Increased for better convergence

      auto result = registration.align(*target, *source, *target_tree, initial_guess);

      // Evaluate fitness score (lower is better)
      // Use average error per inlier instead of total accumulated error

      double score = result.error;
      // double score = (result.num_inliers > 0) ? result.error / result.num_inliers : result.error;

      RCLCPP_INFO(
        this->get_logger(), "[LOOP]   Testing KF %d: score=%.4f, converged=%d",
        candidate_kf.id, score, result.converged);

      if (result.converged && score < best_score) {
        best_score = score;
        best_candidate_idx = idx;
        best_transform = result.T_target_source;
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[LOOP]   Exception while testing KF %d: %s",
        candidate_kf.id, e.what());
      continue;
    } catch (...) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[LOOP]   Unknown exception while testing KF %d",
        candidate_kf.id);
      continue;
    }
  }

  // Check if best score is good enough
  if (best_score < loop_closure_score_threshold_) {
    matched_kf = keyframes_[best_candidate_idx];
    relative_pose = best_transform;

    RCLCPP_INFO(
      this->get_logger(), "[LOOP] ✅ Best match: KF %d with score %.4f (threshold=%.2f)",
      matched_kf.id, best_score, loop_closure_score_threshold_);

    return true;
  }

  RCLCPP_INFO(
    this->get_logger(), "[LOOP] ❌ No good match: best score %.4f > threshold %.2f",
    best_score, loop_closure_score_threshold_);

  return false;
}

void LoopClosureNode::updateMapToOdomTransform(
  const KeyFrame & current_kf,
  const KeyFrame & matched_kf,
  const Eigen::Isometry3d & relative_pose)
{
  // relative_pose (T_target_source) is: current_kf -> matched_kf (source to target)
  // We need: matched_kf -> current_kf, so use relative_pose.inverse()

  // Current odometry says: odom -> current_kf is current_kf.pose
  // Loop closure says: matched_kf -> current_kf is relative_pose.inverse()
  // matched_kf pose in odom: matched_kf.pose

  // Corrected current pose in map frame
  Eigen::Isometry3d current_in_map_local;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
    current_in_map_local = map_to_odom_ * matched_kf.pose * relative_pose.inverse();
  }

  // Current pose in odom frame
  Eigen::Isometry3d current_in_odom = current_kf.pose;

  // Update map to odom transform
  Eigen::Isometry3d new_map_to_odom = current_in_map_local * current_in_odom.inverse();

  // Smooth update (average with previous)
  Eigen::Vector3d translation_update;
  Eigen::Quaterniond q_update;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);

    translation_update = 0.5 * (
      new_map_to_odom.translation() + map_to_odom_.translation());
    Eigen::Quaterniond q_new(new_map_to_odom.rotation());
    Eigen::Quaterniond q_old(map_to_odom_.rotation());
    q_update = q_old.slerp(0.5, q_new);

    map_to_odom_.translation() = translation_update;
    map_to_odom_.linear() = q_update.toRotationMatrix();
  }

  auto rpy = q_update.toRotationMatrix().eulerAngles(0, 1, 2);
  RCLCPP_INFO(
    this->get_logger(),
    "Updated map->odom: trans=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
    translation_update.x(),
    translation_update.y(),
    translation_update.z(),
    rpy.x(), rpy.y(), rpy.z());
}

void LoopClosureNode::publishTransform()
{
  // Use latest odometry timestamp if available, otherwise use current time
  rclcpp::Time stamp = this->now();
  if (latest_odom_) {
    stamp = latest_odom_->header.stamp;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  // Copy map_to_odom under lock
  Eigen::Vector3d translation;
  Eigen::Quaterniond q;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
    translation = map_to_odom_.translation();
    q = Eigen::Quaterniond(map_to_odom_.rotation());
  }

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();

  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform_stamped);

  // Periodic diagnostic output (every 5 seconds)
  static auto last_diag_time = this->now();
  if ((this->now() - last_diag_time).seconds() > 5.0) {
    RCLCPP_INFO(
      this->get_logger(),
      "[DIAG] TF pub: stamp=%.3f, trans=(%.3f,%.3f,%.3f), keyframes=%zu, loops=%zu",
      stamp.seconds(),
      translation.x(),
      translation.y(),
      translation.z(),
      keyframes_.size(),
      loop_pairs_.size());
    last_diag_time = this->now();
  }
}

void LoopClosureNode::publishVisualization()
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Publish keyframe poses
  visualization_msgs::msg::Marker keyframes_marker;
  keyframes_marker.header.frame_id = map_frame_;
  keyframes_marker.header.stamp = this->now();
  keyframes_marker.ns = "keyframes";
  keyframes_marker.id = 0;
  keyframes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  keyframes_marker.action = visualization_msgs::msg::Marker::ADD;
  keyframes_marker.scale.x = 0.3;
  keyframes_marker.scale.y = 0.3;
  keyframes_marker.scale.z = 0.3;
  keyframes_marker.color.r = 0.0;
  keyframes_marker.color.g = 1.0;
  keyframes_marker.color.b = 0.0;
  keyframes_marker.color.a = 0.8;

  for (const auto & kf : keyframes_) {
    geometry_msgs::msg::Point p;
    Eigen::Vector3d pos_in_map = map_to_odom_ * kf.pose.translation();
    p.x = pos_in_map.x();
    p.y = pos_in_map.y();
    p.z = pos_in_map.z();
    keyframes_marker.points.push_back(p);
  }

  marker_array.markers.push_back(keyframes_marker);

  // Publish loop closure edges
  visualization_msgs::msg::Marker loop_edges_marker;
  loop_edges_marker.header.frame_id = map_frame_;
  loop_edges_marker.header.stamp = this->now();
  loop_edges_marker.ns = "loop_edges";
  loop_edges_marker.id = 1;
  loop_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  loop_edges_marker.action = visualization_msgs::msg::Marker::ADD;
  loop_edges_marker.scale.x = 0.1;
  loop_edges_marker.color.r = 1.0;
  loop_edges_marker.color.g = 0.0;
  loop_edges_marker.color.b = 0.0;
  loop_edges_marker.color.a = 1.0;

  for (const auto & loop_pair : loop_pairs_) {
    int id1 = loop_pair.first;
    int id2 = loop_pair.second;

    if (id1 < static_cast<int>(keyframes_.size()) &&
      id2 < static_cast<int>(keyframes_.size()))
    {
      Eigen::Vector3d pos1 = map_to_odom_ * keyframes_[id1].pose.translation();
      Eigen::Vector3d pos2 = map_to_odom_ * keyframes_[id2].pose.translation();

      geometry_msgs::msg::Point p1, p2;
      p1.x = pos1.x();
      p1.y = pos1.y();
      p1.z = pos1.z();
      p2.x = pos2.x();
      p2.y = pos2.y();
      p2.z = pos2.z();

      loop_edges_marker.points.push_back(p1);
      loop_edges_marker.points.push_back(p2);
    }
  }

  marker_array.markers.push_back(loop_edges_marker);

  marker_pub_->publish(marker_array);
}

}  // namespace loop_closure_3d

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(loop_closure_3d::LoopClosureNode)
