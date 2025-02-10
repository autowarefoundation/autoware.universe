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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_

#define FMT_HEADER_ONLY

#include "autoware/localization_util/smart_pose_buffer.hpp"
#include "autoware/universe_utils/ros/diagnostics_interface.hpp"
#include "hyper_parameters.hpp"
#include "map_update_module.hpp"
#include "ndt_omp/multigrid_ndt_omp.h"

#include <autoware/universe_utils/ros/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
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
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace autoware::ndt_scan_matcher
{

class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

public:
  explicit NDTScanMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return map_update_timer_->time_until_trigger();
  }

private:
  void callback_timer();

  void callback_initial_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr);
  void callback_initial_pose_main(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr);

  void callback_regularization_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  void callback_sensor_points(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame);
  bool callback_sensor_points_main(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame);

  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void service_ndt_align(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);
  void service_ndt_align_main(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> align_pose(
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

  static int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array);

  Eigen::Matrix2d estimate_covariance(
    const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
    const rclcpp::Time & sensor_ros_time);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualize_point_score(
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
    const float & lower_nvs, const float & upper_nvs);

  void add_regularization_pose(const rclcpp::Time & sensor_ros_time);

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
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    transform_probability_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_score_points_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    no_ground_transform_probability_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    no_ground_nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr iteration_num_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    initial_to_result_relative_pose_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_old_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_new_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ndt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;

  rclcpp::Service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;

  std::mutex ndt_ptr_mtx_;
  std::unique_ptr<autoware::localization_util::SmartPoseBuffer> initial_pose_buffer_;

  // Keep latest position for dynamic map loading
  std::mutex latest_ekf_position_mtx_;
  std::optional<geometry_msgs::msg::Point> latest_ekf_position_ = std::nullopt;

  std::unique_ptr<autoware::localization_util::SmartPoseBuffer> regularization_pose_buffer_;

  std::atomic<bool> is_activated_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_scan_points_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_initial_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_regularization_pose_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_map_update_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_ndt_align_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_trigger_node_;
  std::unique_ptr<MapUpdateModule> map_update_module_;
  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  HyperParameters param_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
