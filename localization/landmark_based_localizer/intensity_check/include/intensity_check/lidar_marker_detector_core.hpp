// Copyright 2023 Autoware Foundation
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

#ifndef INTENSITY_CEHCK__LIDAR_MARKER_DETECTOR_CORE_HPP_
#define INTENSITY_CEHCK__LIDAR_MARKER_DETECTOR_CORE_HPP_


#include <memory>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef ROS_DISTRO_GALACTIC
  #include <tf2_eigen/tf2_eigen.h>
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
  #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
  #include <tf2_eigen/tf2_eigen.hpp>
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
  #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <pcl/point_types.h>

#include <landmark_manager/landmark_manager.hpp>


class LidarMarkerDetector : public rclcpp::Node
{
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  struct PointXYZIR
  {
    pcl::PointXYZ point;
    float intensity;
    unsigned short ring;  // ring number if available
  };
  typedef std::vector<PointXYZIR> PointCloudXYZIR;

  LidarMarkerDetector();

private:
  void self_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr);
  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_msg_ptr);
  void map_bin_callback(const HADMapBin::ConstSharedPtr & msg);
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void service_trigger_node(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_self_pose_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_bin_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_marker_points_on_base_link_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sensor_points_on_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_marker_pose_on_velodyne_top_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_marker_pose_on_map_from_self_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_marker_pose_on_base_link_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_initial_base_link_on_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_result_base_link_on_map_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_base_link_pose_with_covariance_on_map_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  diagnostic_updater::Updater diag_updater_;

  bool is_activated_;
  bool is_detected_marker_;
  bool is_exist_marker_within_self_pose_;
  std::mutex self_pose_array_mtx_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> self_pose_msg_ptr_array_;

  std::vector<geometry_msgs::msg::Pose> marker_pose_on_map_arrary_;
};

#endif  // INTENSITY_CEHCK__LIDAR_MARKER_DETECTOR_CORE_HPP_