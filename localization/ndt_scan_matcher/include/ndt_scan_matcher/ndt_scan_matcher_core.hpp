// Copyright 2015-2019 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_

#define FMT_HEADER_ONLY

#include "localization_util/smart_pose_buffer.hpp"
#include "ndt_scan_matcher/hyper_parameters.hpp"
#include "ndt_scan_matcher/map_update_module.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/logger_level_configure.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <multigrid_pclomp/multigrid_ndt_omp.h>
#include <pcl/point_types.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/transform_broadcaster.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

public:
  explicit NDTScanMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void service_ndt_align(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);
  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void callback_timer();
  void callback_sensor_points(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame);
  void callback_initial_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr);
  void callback_regularization_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  geometry_msgs::msg::PoseWithCovarianceStamped align_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

  void transform_sensor_measurement(
    const std::string & source_frame, const std::string & target_frame,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
    pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr);

  void publish_tf(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg);
  void publish_pose(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const std::array<double, 36> & ndt_covariance, const bool is_converged);
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr);
  void publish_marker(
    const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array);
  void publish_initial_to_result(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg);

  bool validate_num_iteration(const int iter_num, const int max_iter_num);
  bool validate_score(
    const double score, const double score_threshold, const std::string & score_name);
  bool validate_converged_param(
    const double & transform_probability, const double & nearest_voxel_transformation_likelihood);
  static int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array);

  std::array<double, 36> rotate_covariance(
    const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation) const;
  std::array<double, 36> estimate_covariance(
    const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
    const rclcpp::Time & sensor_ros_time);

  void add_regularization_pose(const rclcpp::Time & sensor_ros_time);

  void publish_diagnostic();

  rclcpp::TimerBase::SharedPtr map_update_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    regularization_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr no_ground_points_aligned_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr multi_ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr multi_initial_pose_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr transform_probability_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    no_ground_transform_probability_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    no_ground_nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Int32Stamped>::SharedPtr iteration_num_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    initial_to_result_relative_pose_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_old_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_new_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ndt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;
  std::shared_ptr<std::map<std::string, std::string>> state_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;

  std::mutex ndt_ptr_mtx_;
  std::unique_ptr<SmartPoseBuffer> initial_pose_buffer_;

  // Keep latest position for dynamic map loading
  std::mutex latest_ekf_position_mtx_;
  std::optional<geometry_msgs::msg::Point> latest_ekf_position_ = std::nullopt;

  std::unique_ptr<SmartPoseBuffer> regularization_pose_buffer_;

  std::atomic<bool> is_activated_;
  std::unique_ptr<MapUpdateModule> map_update_module_;
  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

  HyperParameters param_;
};

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
