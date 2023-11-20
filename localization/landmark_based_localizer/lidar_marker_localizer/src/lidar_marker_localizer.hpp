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

#ifndef LIDAR_MARKER_LOCALIZER_HPP_
#define LIDAR_MARKER_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <mutex>
#include <vector>

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
#include <landmark_manager/landmark_manager.hpp>

#include <pcl/point_types.h>

class LidarMarkerLocalizer : public rclcpp::Node
{
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using Vector3 = geometry_msgs::msg::Vector3;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using SetBool = std_srvs::srv::SetBool;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

  struct Param
  {
    double resolution;
    int filter_window_size;
    int intensity_difference_threshold;
    int positive_window_size;
    int negative_window_size;
    int positive_vote_threshold;
    int negative_vote_threshold;
    int vote_threshold_for_detect_marker;

    double self_pose_timeout_sec;
    double self_pose_distance_tolerance_m;

    double limit_distance_from_self_pose_to_marker_from_lanelet2;
    double limit_distance_from_self_pose_to_marker;
    std::vector<double> base_covariance_;
  };

public:
  LidarMarkerLocalizer();

private:
  void self_pose_callback(const PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr);
  void points_callback(const PointCloud2::ConstSharedPtr & points_msg_ptr);
  void map_bin_callback(const HADMapBin::ConstSharedPtr & msg);
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void service_trigger_node(
    const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_self_pose_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_bin_;

  rclcpp::Publisher<PoseStamped>::SharedPtr pub_marker_pose_on_map_from_self_pose_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr
    pub_base_link_pose_with_covariance_on_map_;
  rclcpp::Service<SetBool>::SharedPtr service_trigger_node_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  diagnostic_updater::Updater diag_updater_;

  Param param_;
  bool is_activated_;
  bool is_detected_marker_;
  bool is_exist_marker_within_self_pose_;
  std::mutex self_pose_array_mtx_;
  std::deque<PoseWithCovarianceStamped::ConstSharedPtr> self_pose_msg_ptr_array_;

  std::vector<Pose> marker_pose_on_map_array_;
};

#endif  // LIDAR_MARKER_LOCALIZER_HPP_
