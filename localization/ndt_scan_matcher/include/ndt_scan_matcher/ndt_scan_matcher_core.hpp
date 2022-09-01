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

#include "ndt_scan_matcher/particle.hpp"

#include <ndt/omp.hpp>
#include <ndt/pcl_generic.hpp>
#include <ndt/pcl_modified.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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

enum class NDTImplementType { PCL_GENERIC = 0, PCL_MODIFIED = 1, OMP = 2 };
enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

struct NdtResult
{
  geometry_msgs::msg::Pose pose;
  float transform_probability;
  float nearest_voxel_transformation_likelihood;
  int iteration_num;
  int iteration_num_max;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> transformation_array;
};

template <typename PointSource, typename PointTarget>
std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> get_ndt(
  const NDTImplementType & ndt_mode)
{
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr;
  if (ndt_mode == NDTImplementType::PCL_GENERIC) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::PCL_MODIFIED) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::OMP) {
    ndt_ptr.reset(new NormalDistributionsTransformOMP<PointSource, PointTarget>);
    return ndt_ptr;
  }

  const std::string s = fmt::format("Unknown NDT type {}", static_cast<int>(ndt_mode));
  throw std::runtime_error(s);
}

class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;

  // TODO(Tier IV): move file
  struct OMPParams
  {
    OMPParams() : search_method(pclomp::NeighborSearchMethod::KDTREE), num_threads(1) {}
    pclomp::NeighborSearchMethod search_method;
    int num_threads;
  };

public:
  NDTScanMatcher();

private:
  void service_ndt_align(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  void callback_map_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callback_sensor_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callback_initial_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  void callback_regularization_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  NdtResult align(const geometry_msgs::msg::Pose & initial_pose_cov_msg);
  geometry_msgs::msg::PoseWithCovarianceStamped align_using_monte_carlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

  void transform_sensor_measurement(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr & sensor_points_sensorTF_msg_ptr,
    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr);
  void update_transforms();

  bool calculate_initial_pose(
    const rclcpp::Time & sensor_ros_time,
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & initial_pose_cov_msg_ptr,
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & initial_pose_old_msg_ptr,
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & initial_pose_new_msg_ptr);

  void publish_tf(
    const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg);
  void publish_pose(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const bool is_converged);
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose,
    const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> & sensor_points_baselinkTF_ptr);
  void publish_marker(
    const rclcpp::Time & sensor_ros_time,
    const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &
      result_pose_matrix_array);
  void publish_scalars(
    const rclcpp::Time & sensor_ros_msg, const double & exe_time,
    const double & transform_probability, const double & nearest_voxel_transformation_likelihood,
    const double & iteration_num);
  void publish_initial_to_result_distances(
    const rclcpp::Time & sensor_ros_msg, const geometry_msgs::msg::Pose & result_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg);

  bool get_transform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr);

  bool validate_time_stamp_difference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec);
  bool validate_position_difference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_);
  bool validate_num_iteration(const int iter_num, const int max_iter_num);
  bool validate_score(
    const double score, const double score_threshold, const std::string score_name);
  bool validate_converged_param(
    const double & transform_probability, const double & nearest_voxel_transformation_likelihood);

  std::optional<Eigen::Matrix4f> interpolate_regularization_pose(
    const rclcpp::Time & sensor_ros_time);
  void add_regularization_pose(const rclcpp::Time & sensor_ros_time);

  void timer_diagnostic();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    regularization_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_with_covariance_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr transform_probability_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    nearest_voxel_transformation_likelihood_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Int32Stamped>::SharedPtr iteration_num_pub_;
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

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  NDTImplementType ndt_implement_type_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  std::string base_frame_;
  std::string ndt_base_frame_;
  std::string map_frame_;

  ConvergedParamType converged_param_type_;
  double converged_param_transform_probability_;
  double converged_param_nearest_voxel_transformation_likelihood_;

  int initial_estimate_particles_num_;
  double initial_pose_timeout_sec_;
  double initial_pose_distance_tolerance_m_;
  float inversion_vector_threshold_;
  float oscillation_threshold_;
  std::array<double, 36> output_pose_covariance_;

  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    initial_pose_msg_ptr_array_;
  std::mutex ndt_map_mtx_;
  std::mutex initial_pose_array_mtx_;

  OMPParams omp_params_;

  std::thread diagnostic_thread_;
  std::map<std::string, std::string> key_value_stdmap_;

  // variables for regularization
  const bool regularization_enabled_;
  const float regularization_scale_factor_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    regularization_pose_msg_ptr_array_;
};

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
