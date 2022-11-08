// Copyright 2022 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHING_MODULE_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHING_MODULE_HPP_

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/pose_array_interpolator.hpp"
#include "ndt_scan_matcher/tf2_listener_module.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

class NDTScanMatchingModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::NormalDistributionsTransform<PointSource, PointTarget>;

public:
  explicit NDTScanMatchingModule(
    rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
    std::shared_ptr<NormalDistributionsTransform> * ndt_ptr,
    std::shared_ptr<Tf2ListenerModule> tf2_listener_module, std::string map_frame,
    rclcpp::CallbackGroup::SharedPtr main_callback_group,
    rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group,
    std::map<std::string, std::string> * key_value_stdmap_ptr);

private:
  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  void callback_map_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callback_sensor_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callback_initial_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  void callback_regularization_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  void transform_sensor_measurement(
    const std::string source_frame, const std::string target_frame,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_input_ptr,
    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_output_ptr);
  void update_transforms();

  void publish_tf(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg);
  void publish_pose(
    const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const bool is_converged);
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const std::shared_ptr<const pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr);
  void publish_marker(
    const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array);
  void publish_initial_to_result_distances(
    const rclcpp::Time & sensor_ros_msg, const geometry_msgs::msg::Pose & result_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg);

  bool validate_num_iteration(const int iter_num, const int max_iter_num);
  bool validate_score(
    const double score, const double score_threshold, const std::string score_name);
  bool validate_converged_param(
    const double & transform_probability, const double & nearest_voxel_transformation_likelihood);

  std::optional<Eigen::Matrix4f> interpolate_regularization_pose(
    const rclcpp::Time & sensor_ros_time);
  void add_regularization_pose(const rclcpp::Time & sensor_ros_time);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
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
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  rclcpp::Node * node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<NormalDistributionsTransform> * ndt_ptr_ptr_;
  std::map<std::string, std::string> * key_value_stdmap_ptr_;
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  std::string base_frame_;
  std::string ndt_base_frame_;
  std::string map_frame_;
  double initial_pose_timeout_sec_;
  double initial_pose_distance_tolerance_m_;
  ConvergedParamType converged_param_type_;
  double converged_param_transform_probability_;
  double converged_param_nearest_voxel_transformation_likelihood_;
  float inversion_vector_threshold_;
  float oscillation_threshold_;
  std::mutex * ndt_ptr_mutex_;

  std::array<double, 36> output_pose_covariance_;

  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    initial_pose_msg_ptr_array_;
  std::mutex initial_pose_array_mtx_;

  const bool regularization_enabled_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    regularization_pose_msg_ptr_array_;

  bool is_activated_;
};

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHING_MODULE_HPP_
