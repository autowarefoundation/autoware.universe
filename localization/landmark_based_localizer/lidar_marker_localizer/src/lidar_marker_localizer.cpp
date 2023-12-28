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

#include "lidar_marker_localizer.hpp"

#include <autoware_point_types/types.hpp>
#include <rclcpp/qos.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <string>

LidarMarkerLocalizer::LidarMarkerLocalizer()
: Node("lidar_marker_localizer"),
  diag_updater_(this),
  is_activated_(false),
  is_detected_marker_(false),
  is_exist_marker_within_self_pose_(false)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  param_.resolution = static_cast<double>(this->declare_parameter<double>("resolution"));
  param_.intensity_pattern = this->declare_parameter<std::vector<int64_t>>("intensity_pattern");
  param_.match_intensity_difference_threshold =
    static_cast<int>(this->declare_parameter<int>("match_intensity_difference_threshold"));
  param_.positive_match_num_threshold =
    static_cast<int>(this->declare_parameter<int>("positive_match_num_threshold"));
  param_.negative_match_num_threshold =
    static_cast<int>(this->declare_parameter<int>("negative_match_num_threshold"));
  param_.vote_threshold_for_detect_marker =
    static_cast<int>(this->declare_parameter<int>("vote_threshold_for_detect_marker"));
  param_.self_pose_timeout_sec =
    static_cast<double>(this->declare_parameter<double>("self_pose_timeout_sec"));
  param_.self_pose_distance_tolerance_m =
    static_cast<double>(this->declare_parameter<double>("self_pose_distance_tolerance_m"));
  param_.limit_distance_from_self_pose_to_marker =
    static_cast<double>(this->declare_parameter<double>("limit_distance_from_self_pose_to_marker"));
  param_.base_covariance_ = this->declare_parameter<std::vector<double>>("base_covariance");
  ekf_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.self_pose_timeout_sec, param_.self_pose_distance_tolerance_m);

  rclcpp::CallbackGroup::SharedPtr points_callback_group;
  points_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto points_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = points_callback_group;
  sub_points_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud_ex", rclcpp::QoS(1).best_effort(),
    std::bind(&LidarMarkerLocalizer::points_callback, this, _1), points_sub_opt);

  rclcpp::CallbackGroup::SharedPtr self_pose_callback_group;
  self_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto self_pose_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = self_pose_callback_group;
  sub_self_pose_ = this->create_subscription<PoseWithCovarianceStamped>(
    "~/input/ekf_pose", rclcpp::QoS(1),
    std::bind(&LidarMarkerLocalizer::self_pose_callback, this, _1), points_sub_opt);
  sub_map_bin_ = this->create_subscription<HADMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&LidarMarkerLocalizer::map_bin_callback, this, _1));

  pub_base_link_pose_with_covariance_on_map_ =
    this->create_publisher<PoseWithCovarianceStamped>("~/output/pose_with_covariance", 10);
  rclcpp::QoS qos_marker = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_marker.transient_local();
  qos_marker.reliable();
  pub_marker_mapped_ = this->create_publisher<MarkerArray>("~/debug/marker_mapped", qos_marker);
  pub_marker_detected_ = this->create_publisher<PoseArray>("~/debug/marker_detected", 10);

  service_trigger_node_ = this->create_service<SetBool>(
    "~/service/trigger_node_srv",
    std::bind(&LidarMarkerLocalizer::service_trigger_node, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), points_callback_group);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  diag_updater_.setHardwareID(get_name());
  diag_updater_.add("lidar_marker_localizer", this, &LidarMarkerLocalizer::update_diagnostics);
}

void LidarMarkerLocalizer::map_bin_callback(const HADMapBin::ConstSharedPtr & msg)
{
  landmark_manager_.parse_landmarks(msg, "reflective_paint_marker");
  const MarkerArray marker_msg = landmark_manager_.get_landmarks_as_marker_array_msg();
  pub_marker_mapped_->publish(marker_msg);
}

void LidarMarkerLocalizer::self_pose_callback(
  const PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr)
{
  // TODO(YamatoAndo)
  // if (!is_activated_) return;

  if (self_pose_msg_ptr->header.frame_id == "map") {
    ekf_pose_buffer_->push_back(self_pose_msg_ptr);
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Received initial pose message with frame_id "
        << self_pose_msg_ptr->header.frame_id << ", but expected map. "
        << "Please check the frame_id in the input topic and ensure it is correct.");
  }
}

void LidarMarkerLocalizer::points_callback(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  const builtin_interfaces::msg::Time sensor_ros_time = points_msg_ptr->header.stamp;

  // (1) Get Self Pose
  const std::optional<SmartPoseBuffer::InterpolateResult> interpolate_result =
    ekf_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolate_result) {
    return;
  }
  ekf_pose_buffer_->pop_old(sensor_ros_time);
  const Pose self_pose = interpolate_result.value().interpolated_pose.pose.pose;

  // (2) detect marker
  const std::vector<landmark_manager::Landmark> detected_landmarks =
    detect_landmarks(points_msg_ptr);

  is_detected_marker_ = !detected_landmarks.empty();
  if (!is_detected_marker_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Could not detect marker");
    return;
  }

  // for debug
  if (pub_marker_detected_->get_subscription_count() > 0) {
    PoseArray pose_array_msg;
    pose_array_msg.header.stamp = sensor_ros_time;
    pose_array_msg.header.frame_id = "map";
    for (const landmark_manager::Landmark & landmark : detected_landmarks) {
      const Pose detected_marker_on_map =
        tier4_autoware_utils::transformPose(landmark.pose, self_pose);
      pose_array_msg.poses.push_back(detected_marker_on_map);
    }
    pub_marker_detected_->publish(pose_array_msg);
  }

  // (3) calculate diff pose
  const Pose new_self_pose =
    landmark_manager_.calculate_new_self_pose(detected_landmarks, self_pose, false);

  const double diff_x = new_self_pose.position.x - self_pose.position.x;
  const double diff_y = new_self_pose.position.y - self_pose.position.y;

  const double diff_norm = std::hypot(diff_x, diff_y);
  const bool is_exist_marker_within_lanelet2_map =
    diff_norm < param_.limit_distance_from_self_pose_to_marker;

  if (!is_exist_marker_within_lanelet2_map) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "The distance from lanelet2 to the detect marker is too far("
        << diff_norm << "). The limit is " << param_.limit_distance_from_self_pose_to_marker
        << ".");
    return;
  }

  // (4) Apply diff pose to self pose
  // only x and y is changed
  PoseWithCovarianceStamped result;
  result.header.stamp = sensor_ros_time;
  result.header.frame_id = "map";
  result.pose.pose.position.x = new_self_pose.position.x;
  result.pose.pose.position.y = new_self_pose.position.y;
  result.pose.pose.position.z = self_pose.position.z;
  result.pose.pose.orientation = self_pose.orientation;

  // TODO(YamatoAndo) transform covariance on base_link to map frame
  for (int i = 0; i < 36; i++) {
    result.pose.covariance[i] = param_.base_covariance_[i];
  }
  pub_base_link_pose_with_covariance_on_map_->publish(result);
}

void LidarMarkerLocalizer::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("exist_marker_within_self_pose", is_exist_marker_within_self_pose_ ? "Yes" : "No");
  stat.add("detected_marker", is_detected_marker_ ? "Yes" : "No");

  if (is_exist_marker_within_self_pose_ & is_detected_marker_) {
    stat.summary(DiagnosticStatus::OK, "OK. Detect a marker");
  } else if (is_exist_marker_within_self_pose_ & !is_detected_marker_) {
    stat.summary(DiagnosticStatus::ERROR, "NG. Could not detect a marker");
  } else if (!is_exist_marker_within_self_pose_ & is_detected_marker_) {
    stat.summary(DiagnosticStatus::OK, "OK. Detect a not marker-object");
  } else if (!is_exist_marker_within_self_pose_ & !is_detected_marker_) {
    stat.summary(DiagnosticStatus::OK, "OK. There are no markers within the range of self-pose");
  } else {
    stat.summary(DiagnosticStatus::ERROR, "NG. This message should not be displayed.");
  }
}

void LidarMarkerLocalizer::service_trigger_node(
  const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    ekf_pose_buffer_->clear();
  } else {
  }
  res->success = true;
}

std::vector<landmark_manager::Landmark> LidarMarkerLocalizer::detect_landmarks(
  const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  const std::string sensor_frame = points_msg_ptr->header.frame_id;
  // get sensor_frame pose to base_link
  // TransformStamped transform_sensor_to_base_link;
  // try
  // {
  //   transform_sensor_to_base_link = tf_buffer_->lookupTransform(
  //   "base_link", sensor_frame, tf2::TimePointZero);
  // }
  // catch (tf2::TransformException & ex)
  // {
  //   RCLCPP_WARN(get_logger(), "cannot get base_link to %s transform. %s", sensor_frame,
  //   ex.what());
  // }

  pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRADRT>);
  pcl::fromROSMsg(*points_msg_ptr, *points_ptr);

  if (points_ptr->empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No points!");
    is_detected_marker_ = false;
    return std::vector<landmark_manager::Landmark>{};
  }

  std::vector<pcl::PointCloud<autoware_point_types::PointXYZIRADRT>> ring_points(128);

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  for (const autoware_point_types::PointXYZIRADRT & point : points_ptr->points) {
    ring_points[point.ring].push_back(point);
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
  }

  // Check that the leaf size is not too small, given the size of the data
  const int bin_num = static_cast<int>((max_x - min_x) / param_.resolution + 1);

  // initialize variables
  std::vector<int> vote(bin_num, 0);
  std::vector<float> min_y(bin_num, std::numeric_limits<float>::max());

  // for each ring
  for (const pcl::PointCloud<autoware_point_types::PointXYZIRADRT> & one_ring : ring_points) {
    std::vector<double> intensity_sum(bin_num, 0.0);
    std::vector<int> intensity_num(bin_num, 0);

    for (const autoware_point_types::PointXYZIRADRT & point : one_ring.points) {
      const int bin_index = static_cast<int>((point.x - min_x) / param_.resolution);
      intensity_sum[bin_index] += point.intensity;
      intensity_num[bin_index]++;
      min_y[bin_index] = std::min(min_y[bin_index], point.y);
    }

    // filter
    for (size_t i = 0; i < bin_num - param_.intensity_pattern.size(); i++) {
      int64_t pos = 0;
      int64_t neg = 0;
      double min_intensity = std::numeric_limits<double>::max();
      double max_intensity = std::numeric_limits<double>::lowest();

      // find max_min
      for (size_t j = 0; j < param_.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }
        const double average = intensity_sum[i + j] / intensity_num[i + j];
        min_intensity = std::min(min_intensity, average);
        max_intensity = std::max(max_intensity, average);
      }

      if (max_intensity <= min_intensity) {
        continue;
      }

      const double center_intensity = (max_intensity - min_intensity) / 2.0 + min_intensity;

      for (size_t j = 0; j < param_.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        const double average = intensity_sum[i + j] / intensity_num[i + j];

        if (param_.intensity_pattern[j] == 1) {
          // check positive
          if (average > center_intensity + param_.match_intensity_difference_threshold) {
            pos++;
          }
        } else if (param_.intensity_pattern[j] == -1) {
          // check negative
          if (average < center_intensity - param_.match_intensity_difference_threshold) {
            neg++;
          }
        } else {
          // ignore param_.intensity_pattern[j] == 0
        }
      }

      if (
        pos >= param_.positive_match_num_threshold && neg >= param_.negative_match_num_threshold) {
        vote[i]++;
      }
    }
  }

  std::vector<landmark_manager::Landmark> detected_landmarks;

  for (size_t i = 0; i < bin_num - param_.intensity_pattern.size(); i++) {
    if (vote[i] > param_.vote_threshold_for_detect_marker) {
      const double bin_position =
        static_cast<double>(i) + static_cast<double>(param_.intensity_pattern.size()) / 2.0;
      Pose marker_pose_on_base_link;
      marker_pose_on_base_link.position.x = bin_position * param_.resolution + min_x;
      marker_pose_on_base_link.position.y = min_y[i];
      marker_pose_on_base_link.position.z = 0.2 + 1.75 / 2.0;  // TODO(YamatoAndo)
      marker_pose_on_base_link.orientation =
        tier4_autoware_utils::createQuaternionFromRPY(M_PI_2, 0.0, 0.0);  // TODO(YamatoAndo)
      detected_landmarks.push_back(landmark_manager::Landmark{"0", marker_pose_on_base_link});
    }
  }

  return detected_landmarks;
}
