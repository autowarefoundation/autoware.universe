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

#include "localization_util/pose_array_interpolator.hpp"

#include <autoware_point_types/types.hpp>
#include <rclcpp/qos.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

LidarMarkerLocalizer::LidarMarkerLocalizer()
: Node("lidar_marker_localizer"),
  diag_updater_(this),
  is_activated_(false),
  is_detected_marker_(false),
  is_exist_marker_within_self_pose_(false)
{
  using std::placeholders::_1;

  param_.resolution = static_cast<double>(this->declare_parameter<double>("resolution"));
  param_.filter_window_size = static_cast<int>(this->declare_parameter<int>("filter_window_size"));
  param_.intensity_difference_threshold =
    static_cast<int>(this->declare_parameter<int>("intensity_difference_threshold"));
  param_.positive_window_size =
    static_cast<int>(this->declare_parameter<int>("positive_window_size"));
  param_.negative_window_size =
    static_cast<int>(this->declare_parameter<int>("negative_window_size"));
  param_.positive_vote_threshold =
    static_cast<int>(this->declare_parameter<int>("positive_vote_threshold"));
  param_.negative_vote_threshold =
    static_cast<int>(this->declare_parameter<int>("negative_vote_threshold"));
  param_.vote_threshold_for_detect_marker =
    static_cast<int>(this->declare_parameter<int>("vote_threshold_for_detect_marker"));
  param_.self_pose_timeout_sec =
    static_cast<double>(this->declare_parameter<double>("self_pose_timeout_sec"));
  param_.self_pose_distance_tolerance_m =
    static_cast<double>(this->declare_parameter<double>("self_pose_distance_tolerance_m"));
  param_.limit_distance_from_self_pose_to_marker_from_lanelet2 = static_cast<double>(
    this->declare_parameter<double>("limit_distance_from_self_pose_to_marker_from_lanelet2"));
  param_.limit_distance_from_self_pose_to_marker =
    static_cast<double>(this->declare_parameter<double>("limit_distance_from_self_pose_to_marker"));
  param_.base_covariance_ = this->declare_parameter<std::vector<double>>("base_covariance");

  rclcpp::CallbackGroup::SharedPtr points_callback_group;
  points_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto points_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = points_callback_group;
  sub_points_ = this->create_subscription<PointCloud2>(
    "input/pointcloud_ex", rclcpp::QoS(1).best_effort(),
    std::bind(&LidarMarkerLocalizer::points_callback, this, _1), points_sub_opt);

  rclcpp::CallbackGroup::SharedPtr self_pose_callback_group;
  self_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto self_pose_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = self_pose_callback_group;
  sub_self_pose_ = this->create_subscription<PoseWithCovarianceStamped>(
    "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", rclcpp::QoS(1),
    std::bind(&LidarMarkerLocalizer::self_pose_callback, this, _1), points_sub_opt);
  sub_map_bin_ = this->create_subscription<HADMapBin>(
    "/map/vector_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&LidarMarkerLocalizer::map_bin_callback, this, std::placeholders::_1));

  pub_marker_pose_on_map_from_self_pose_ =
    this->create_publisher<PoseStamped>("marker_pose_on_map_from_self_pose", 10);
  pub_base_link_pose_with_covariance_on_map_ = this->create_publisher<PoseWithCovarianceStamped>(
    "/localization/pose_estimator/pose_with_covariance", 10);
  rclcpp::QoS qos_marker = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_marker.transient_local();
  qos_marker.reliable();
  pub_marker_ = this->create_publisher<MarkerArray>("~/debug/marker", qos_marker);

  service_trigger_node_ = this->create_service<SetBool>(
    "trigger_node_srv",
    std::bind(
      &LidarMarkerLocalizer::service_trigger_node, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(),
    points_callback_group);  // TODO refactor points_callback_group

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  diag_updater_.setHardwareID(get_name());
  diag_updater_.add("lidar_marker_localizer", this, &LidarMarkerLocalizer::update_diagnostics);
}

void LidarMarkerLocalizer::map_bin_callback(const HADMapBin::ConstSharedPtr & msg)
{
  const std::vector<landmark_manager::Landmark> landmarks =
    landmark_manager::parse_landmarks(msg, "reflective_paint_marker", this->get_logger());
  for (const landmark_manager::Landmark & landmark : landmarks) {
    marker_pose_on_map_array_.push_back(landmark.pose);
  }

  const MarkerArray marker_msg = landmark_manager::convert_landmarks_to_marker_array_msg(landmarks);
  pub_marker_->publish(marker_msg);
}

void LidarMarkerLocalizer::self_pose_callback(
  const PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr)
{
  // TODO
  // if (!is_activated_) return;

  // lock mutex for initial pose
  std::lock_guard<std::mutex> self_pose_array_lock(self_pose_array_mtx_);
  // if rosbag restart, clear buffer
  if (!self_pose_msg_ptr_array_.empty()) {
    const builtin_interfaces::msg::Time & t_front = self_pose_msg_ptr_array_.front()->header.stamp;
    const builtin_interfaces::msg::Time & t_msg = self_pose_msg_ptr->header.stamp;
    if (t_front.sec > t_msg.sec || (t_front.sec == t_msg.sec && t_front.nanosec > t_msg.nanosec)) {
      self_pose_msg_ptr_array_.clear();
    }
  }

  if (self_pose_msg_ptr->header.frame_id == "map") {
    self_pose_msg_ptr_array_.push_back(self_pose_msg_ptr);
  } else {
    TransformStamped transform_self_pose_frame_to_map;
    try {
      transform_self_pose_frame_to_map = tf_buffer_->lookupTransform(
        "map", self_pose_msg_ptr->header.frame_id, self_pose_msg_ptr->header.stamp,
        rclcpp::Duration::from_seconds(0.1));

      // transform self_pose_frame to map_frame
      auto self_pose_on_map_ptr = std::make_shared<PoseWithCovarianceStamped>();
      self_pose_on_map_ptr->pose.pose = tier4_autoware_utils::transformPose(
        self_pose_msg_ptr->pose.pose, transform_self_pose_frame_to_map);
      // self_pose_on_map_ptr->pose.covariance;  // TODO
      self_pose_on_map_ptr->header.stamp = self_pose_msg_ptr->header.stamp;
      self_pose_msg_ptr_array_.push_back(self_pose_on_map_ptr);

    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "cannot get map to %s transform. %s",
        self_pose_msg_ptr->header.frame_id.c_str(), ex.what());
    }
  }
}

void LidarMarkerLocalizer::points_callback(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  std::string sensor_frame = points_msg_ptr->header.frame_id;
  auto sensor_ros_time = points_msg_ptr->header.stamp;
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

  std::vector<pcl::PointCloud<autoware_point_types::PointXYZIRADRT>> ring_points;
  ring_points.resize(128);

  double min_x = 100.0;
  double max_x = -100.0;
  for (const auto & point : points_ptr->points) {
    ring_points[point.ring].push_back(point);

    if (min_x > point.x) {
      min_x = point.x;
    }

    if (max_x < point.x) {
      max_x = point.x;
    }
  }

  // Check that the leaf size is not too small, given the size of the data
  int dx = static_cast<int>((max_x - min_x) * (1 / param_.resolution) + 1);

  // initialize variables
  std::vector<int> vote(dx, 0);
  std::vector<double> feature_sum(dx, 0);
  std::vector<double> distance(dx, 100);

  // for target rings
  for (size_t target_ring = 0; target_ring < ring_points.size(); target_ring++) {
    // initialize intensity line image
    std::vector<double> intensity_line_image(dx, 0);
    std::vector<int> intensity_line_image_num(dx, 0);

    for (size_t i = 0; i < ring_points[target_ring].size(); i++) {
      autoware_point_types::PointXYZIRADRT point;
      point = ring_points[target_ring].points[i];
      int ix = (point.x - min_x) * (1 / param_.resolution);
      intensity_line_image[ix] += point.intensity;
      intensity_line_image_num[ix]++;
      if (distance[ix] > point.y) {
        distance[ix] = point.y;
      }
    }

    // average
    for (int i = 0; i < dx; i++) {
      if (intensity_line_image_num[i] > 0)
        intensity_line_image[i] /= (double)intensity_line_image_num[i];
    }

    // filter
    for (int i = param_.filter_window_size * 2; i < dx - param_.filter_window_size * 2; i++) {
      double pos = 0;
      double neg = 0;
      double max = -1;
      double min = 1000;

      // find max_min
      for (int j = -param_.filter_window_size; j <= param_.filter_window_size; j++) {
        if (max < intensity_line_image[i + j]) max = intensity_line_image[i + j];
        if (min > intensity_line_image[i + j]) min = intensity_line_image[i + j];
      }

      if (max > min) {
        double median = (max - min) / 2.0 + min;
        for (int j = -param_.positive_window_size; j <= param_.positive_window_size; j++) {
          if (median + param_.intensity_difference_threshold < intensity_line_image[i + j])
            pos += 1;
        }
        for (int j = -param_.filter_window_size; j <= -param_.negative_window_size; j++) {
          if (median - param_.intensity_difference_threshold > intensity_line_image[i + j])
            neg += 1;
        }
        for (int j = param_.negative_window_size; j <= param_.filter_window_size; j++) {
          if (median - param_.intensity_difference_threshold > intensity_line_image[i + j])
            neg += 1;
        }
        if (pos >= param_.positive_vote_threshold && neg >= param_.negative_vote_threshold) {
          vote[i]++;
          feature_sum[i] += (max - min);
        }
      }
    }
  }

  // extract feature points
  std::vector<PoseStamped> marker_pose_on_base_link_array;

  for (int i = param_.filter_window_size * 2; i < dx - param_.filter_window_size * 2; i++) {
    if (vote[i] > param_.vote_threshold_for_detect_marker) {
      PoseStamped marker_pose_on_base_link;
      marker_pose_on_base_link.header.stamp = sensor_ros_time;
      marker_pose_on_base_link.header.frame_id = "base_link";
      marker_pose_on_base_link.pose.position.x = i * param_.resolution + min_x;
      marker_pose_on_base_link.pose.position.y = distance[i];
      marker_pose_on_base_link.pose.position.z = 0.2 + 1.75 / 2.0;  // TODO
      marker_pose_on_base_link.pose.orientation =
        tier4_autoware_utils::createQuaternionFromRPY(M_PI_2, 0.0, 0.0);  // TODO
      marker_pose_on_base_link_array.push_back(marker_pose_on_base_link);
    }
  }

  // get self-position on map
  std::unique_lock<std::mutex> self_pose_array_lock(self_pose_array_mtx_);
  if (self_pose_msg_ptr_array_.size() <= 1) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No Pose!");
    return;
  }
  PoseArrayInterpolator interpolator(
    this, sensor_ros_time, self_pose_msg_ptr_array_, param_.self_pose_timeout_sec,
    param_.self_pose_distance_tolerance_m);
  if (!interpolator.is_success()) return;
  pop_old_pose(self_pose_msg_ptr_array_, sensor_ros_time);
  self_pose_array_mtx_.unlock();

  const auto self_pose_msg = interpolator.get_current_pose();

  // get nearest marker pose on map
  const auto marker_pose_on_map_from_lanelet2_map = *std::min_element(
    std::begin(marker_pose_on_map_array_), std::end(marker_pose_on_map_array_),
    [&self_pose_msg](const auto & lhs, const auto & rhs) {
      return tier4_autoware_utils::calcDistance3d(lhs.position, self_pose_msg.pose.pose.position) <
             tier4_autoware_utils::calcDistance3d(rhs.position, self_pose_msg.pose.pose.position);
    });

  const auto marker_pose_on_base_link_from_lanele2_map =
    tier4_autoware_utils::inverseTransformPoint(
      marker_pose_on_map_from_lanelet2_map.position, self_pose_msg.pose.pose);

  is_exist_marker_within_self_pose_ = std::fabs(marker_pose_on_base_link_from_lanele2_map.x) <
                                      param_.limit_distance_from_self_pose_to_marker_from_lanelet2;
  if (!is_exist_marker_within_self_pose_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "The distance from self-pose to the nearest marker of lanelet2 is too far("
        << marker_pose_on_base_link_from_lanele2_map.x << "). The limit is "
        << param_.limit_distance_from_self_pose_to_marker_from_lanelet2 << ".");
    // return;
  }

  // if(!is_detected_marker_ || !is_exist_marker_within_self_pose_) {
  //   return;
  // }

  is_detected_marker_ = !marker_pose_on_base_link_array.empty();
  if (!is_detected_marker_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Could not detect marker");
    return;
  }

  // get marker_pose on base_link
  PoseStamped marker_pose_on_base_link;
  marker_pose_on_base_link = marker_pose_on_base_link_array.at(0);  // TODO

  // get marker pose on map using self-pose
  const auto self_pose_rpy = tier4_autoware_utils::getRPY(self_pose_msg.pose.pose.orientation);
  const double self_pose_yaw = self_pose_rpy.z;

  PoseStamped marker_pose_on_map_from_self_pose;
  marker_pose_on_map_from_self_pose.header.stamp = sensor_ros_time;
  marker_pose_on_map_from_self_pose.header.frame_id = "map";
  marker_pose_on_map_from_self_pose.pose.position.x =
    marker_pose_on_base_link.pose.position.x * std::cos(self_pose_yaw) -
    marker_pose_on_base_link.pose.position.y * std::sin(self_pose_yaw) +
    self_pose_msg.pose.pose.position.x;
  marker_pose_on_map_from_self_pose.pose.position.y =
    marker_pose_on_base_link.pose.position.x * std::sin(self_pose_yaw) +
    marker_pose_on_base_link.pose.position.y * std::cos(self_pose_yaw) +
    self_pose_msg.pose.pose.position.y;
  marker_pose_on_map_from_self_pose.pose.position.z =
    marker_pose_on_base_link.pose.position.z + self_pose_msg.pose.pose.position.z;
  marker_pose_on_map_from_self_pose.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(
      self_pose_rpy.x + M_PI_2, self_pose_rpy.y, self_pose_rpy.z);
  pub_marker_pose_on_map_from_self_pose_->publish(marker_pose_on_map_from_self_pose);

  if (!is_detected_marker_ || !is_exist_marker_within_self_pose_) {
    return;
  }

  Vector3 diff_position_from_self_position_to_lanelet2_map;
  diff_position_from_self_position_to_lanelet2_map.x =
    marker_pose_on_map_from_lanelet2_map.position.x -
    marker_pose_on_map_from_self_pose.pose.position.x;
  diff_position_from_self_position_to_lanelet2_map.y =
    marker_pose_on_map_from_lanelet2_map.position.y -
    marker_pose_on_map_from_self_pose.pose.position.y;

  double diff_position_from_self_position_to_lanelet2_map_norm = std::hypot(
    diff_position_from_self_position_to_lanelet2_map.x,
    diff_position_from_self_position_to_lanelet2_map.y);
  bool is_exist_marker_within_lanelet2_map = diff_position_from_self_position_to_lanelet2_map_norm <
                                             param_.limit_distance_from_self_pose_to_marker;

  if (!is_exist_marker_within_lanelet2_map) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "The distance from lanelet2 to the detect marker is too far("
        << diff_position_from_self_position_to_lanelet2_map_norm << "). The limit is "
        << param_.limit_distance_from_self_pose_to_marker << ".");
    return;
  }

  Pose result_base_link_on_map;
  result_base_link_on_map.position.x =
    self_pose_msg.pose.pose.position.x + diff_position_from_self_position_to_lanelet2_map.x;
  result_base_link_on_map.position.y =
    self_pose_msg.pose.pose.position.y + diff_position_from_self_position_to_lanelet2_map.y;
  result_base_link_on_map.position.z = self_pose_msg.pose.pose.position.z;
  result_base_link_on_map.orientation = self_pose_msg.pose.pose.orientation;

  PoseWithCovarianceStamped base_link_pose_with_covariance_on_map;
  base_link_pose_with_covariance_on_map.header.stamp = sensor_ros_time;
  base_link_pose_with_covariance_on_map.header.frame_id = "map";
  base_link_pose_with_covariance_on_map.pose.pose = result_base_link_on_map;

  // TODO transform covariance on base_link to map frame
  for (int i = 0; i < 36; i++) {
    base_link_pose_with_covariance_on_map.pose.covariance[i] = param_.base_covariance_[i];
  }
  pub_base_link_pose_with_covariance_on_map_->publish(base_link_pose_with_covariance_on_map);
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
    std::lock_guard<std::mutex> self_pose_array_lock(self_pose_array_mtx_);
    self_pose_msg_ptr_array_.clear();
  } else {
  }
  res->success = true;
  return;
}
