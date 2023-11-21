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
    std::bind(&LidarMarkerLocalizer::map_bin_callback, this, _1));

  pub_marker_pose_on_map_from_self_pose_ =
    this->create_publisher<PoseStamped>("marker_pose_on_map_from_self_pose", 10);
  pub_base_link_pose_with_covariance_on_map_ = this->create_publisher<PoseWithCovarianceStamped>(
    "/localization/pose_estimator/pose_with_covariance", 10);
  rclcpp::QoS qos_marker = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_marker.transient_local();
  qos_marker.reliable();
  pub_marker_ = this->create_publisher<MarkerArray>("~/debug/marker", qos_marker);

  service_trigger_node_ = this->create_service<SetBool>(
    "trigger_node_srv", std::bind(&LidarMarkerLocalizer::service_trigger_node, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), points_callback_group);

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
  // TODO(YamatoAndo)
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
      // self_pose_on_map_ptr->pose.covariance;  // TODO(YamatoAndo)
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
  const builtin_interfaces::msg::Time sensor_ros_time = points_msg_ptr->header.stamp;
  std_msgs::msg::Header header;
  header.stamp = sensor_ros_time;
  header.frame_id = "map";

  // ----------------- //
  // (1) Get Self Pose //
  // ----------------- //
  Pose self_pose;
  {
    // get self-position on map
    std::unique_lock<std::mutex> self_pose_array_lock(self_pose_array_mtx_);
    if (self_pose_msg_ptr_array_.size() <= 1) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No Pose!");
      return;
    }
    PoseArrayInterpolator interpolator(
      this, sensor_ros_time, self_pose_msg_ptr_array_, param_.self_pose_timeout_sec,
      param_.self_pose_distance_tolerance_m);
    if (!interpolator.is_success()) {
      return;
    }
    pop_old_pose(self_pose_msg_ptr_array_, sensor_ros_time);
    self_pose = interpolator.get_current_pose().pose.pose;
  }

  // ---------------------------------- //
  // (2) Get marker_pose_from_detection //
  // ---------------------------------- //
  Pose marker_pose_from_detection;
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
      return;
    }

    std::vector<pcl::PointCloud<autoware_point_types::PointXYZIRADRT>> ring_points(128);

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    for (const auto & point : points_ptr->points) {
      ring_points[point.ring].push_back(point);
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
    }

    // Check that the leaf size is not too small, given the size of the data
    const int bin_num = static_cast<int>((max_x - min_x) / param_.resolution + 1);

    // initialize variables
    std::vector<int> vote(bin_num, 0);
    std::vector<float> min_y(bin_num, std::numeric_limits<float>::max());

    // for target rings
    for (size_t target_ring = 0; target_ring < ring_points.size(); target_ring++) {
      // initialize intensity line image
      std::vector<double> intensity_sum(bin_num, 0.0);
      std::vector<int> intensity_num(bin_num, 0);

      for (size_t i = 0; i < ring_points[target_ring].size(); i++) {
        autoware_point_types::PointXYZIRADRT point = ring_points[target_ring].points[i];
        const int bin_index = (point.x - min_x) / param_.resolution;
        intensity_sum[bin_index] += point.intensity;
        intensity_num[bin_index]++;
        min_y[bin_index] = std::min(min_y[bin_index], point.y);
      }

      // filter
      for (int i = param_.filter_window_size; i < bin_num - param_.filter_window_size; i++) {
        int64_t pos = 0;
        int64_t neg = 0;
        double min_intensity = std::numeric_limits<double>::max();
        double max_intensity = std::numeric_limits<double>::lowest();

        // find max_min
        for (int j = -param_.filter_window_size; j <= param_.filter_window_size; j++) {
          if (intensity_num[i + j] == 0) {
            continue;
          }
          const double & average = intensity_sum[i + j] / intensity_num[i + j];
          min_intensity = std::min(min_intensity, average);
          max_intensity = std::max(max_intensity, average);
        }

        if (max_intensity <= min_intensity) {
          continue;
        }

        const double center_intensity = (max_intensity - min_intensity) / 2.0 + min_intensity;

        for (int j = -param_.filter_window_size; j <= param_.filter_window_size; j++) {
          if (intensity_num[i + j] == 0) {
            continue;
          }
          const double & average = intensity_sum[i + j] / intensity_num[i + j];
          if (std::abs(j) >= param_.negative_window_size) {
            // check negative
            if (average < center_intensity - param_.intensity_difference_threshold) {
              neg++;
            }
          } else if (std::abs(j) <= param_.positive_window_size) {
            // check positive
            if (average > center_intensity + param_.intensity_difference_threshold) {
              pos++;
            }
          } else {
            // ignore
          }
        }

        if (pos >= param_.positive_vote_threshold && neg >= param_.negative_vote_threshold) {
          vote[i]++;
        }
      }
    }

    // extract feature points
    std::vector<Pose> marker_pose_on_base_link_array;

    for (int i = param_.filter_window_size; i < bin_num - param_.filter_window_size; i++) {
      if (vote[i] > param_.vote_threshold_for_detect_marker) {
        Pose marker_pose_on_base_link;
        marker_pose_on_base_link.position.x = i * param_.resolution + min_x;
        marker_pose_on_base_link.position.y = min_y[i];
        marker_pose_on_base_link.position.z = 0.2 + 1.75 / 2.0;  // TODO(YamatoAndo)
        marker_pose_on_base_link.orientation =
          tier4_autoware_utils::createQuaternionFromRPY(M_PI_2, 0.0, 0.0);  // TODO(YamatoAndo)
        marker_pose_on_base_link_array.push_back(marker_pose_on_base_link);
      }
    }

    is_detected_marker_ = !marker_pose_on_base_link_array.empty();
    if (!is_detected_marker_) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1, "Could not detect marker");
      return;
    }

    // get marker_pose on base_link
    const Pose marker_pose_on_base_link = marker_pose_on_base_link_array.at(0);  // TODO(YamatoAndo)

    // get marker pose on map using self-pose
    const Vector3 self_pose_rpy = tier4_autoware_utils::getRPY(self_pose.orientation);
    const double self_pose_yaw = self_pose_rpy.z;

    // calculate marker_pose_from_detection
    marker_pose_from_detection.position.x =
      marker_pose_on_base_link.position.x * std::cos(self_pose_yaw) -
      marker_pose_on_base_link.position.y * std::sin(self_pose_yaw) + self_pose.position.x;
    marker_pose_from_detection.position.y =
      marker_pose_on_base_link.position.x * std::sin(self_pose_yaw) +
      marker_pose_on_base_link.position.y * std::cos(self_pose_yaw) + self_pose.position.y;
    marker_pose_from_detection.position.z =
      marker_pose_on_base_link.position.z + self_pose.position.z;
    marker_pose_from_detection.orientation = tier4_autoware_utils::createQuaternionFromRPY(
      self_pose_rpy.x + M_PI_2, self_pose_rpy.y, self_pose_rpy.z);

    // publish
    PoseStamped marker_pose_from_detection_stamped;
    marker_pose_from_detection_stamped.header = header;
    marker_pose_from_detection_stamped.pose = marker_pose_from_detection;
    pub_marker_pose_on_map_from_self_pose_->publish(marker_pose_from_detection_stamped);
  }

  // ---------------------------- //
  // (3) Get marker_pose_from_map //
  // ---------------------------- //
  Pose marker_pose_from_map;
  {
    // get nearest marker pose on map
    marker_pose_from_map = *std::min_element(
      std::begin(marker_pose_on_map_array_), std::end(marker_pose_on_map_array_),
      [&self_pose](const Pose & lhs, const Pose & rhs) {
        return tier4_autoware_utils::calcDistance3d(lhs.position, self_pose.position) <
               tier4_autoware_utils::calcDistance3d(rhs.position, self_pose.position);
      });

    const auto marker_pose_on_base_link_from_map =
      tier4_autoware_utils::inverseTransformPoint(marker_pose_from_map.position, self_pose);

    is_exist_marker_within_self_pose_ =
      std::fabs(marker_pose_on_base_link_from_map.x) <
      param_.limit_distance_from_self_pose_to_marker_from_lanelet2;
    if (!is_exist_marker_within_self_pose_) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1,
        "The distance from self-pose to the nearest marker of lanelet2 is too far("
          << marker_pose_on_base_link_from_map.x << "). The limit is "
          << param_.limit_distance_from_self_pose_to_marker_from_lanelet2 << ".");
      return;
    }
  }

  // --------------------- //
  // (4) Compare two poses //
  // --------------------- //
  const double diff_x = marker_pose_from_map.position.x - marker_pose_from_detection.position.x;
  const double diff_y = marker_pose_from_map.position.y - marker_pose_from_detection.position.y;

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

  Pose result_base_link_on_map;
  result_base_link_on_map.position.x = self_pose.position.x + diff_x;
  result_base_link_on_map.position.y = self_pose.position.y + diff_y;
  result_base_link_on_map.position.z = self_pose.position.z;
  result_base_link_on_map.orientation = self_pose.orientation;

  PoseWithCovarianceStamped base_link_pose_with_covariance_on_map;
  base_link_pose_with_covariance_on_map.header = header;
  base_link_pose_with_covariance_on_map.pose.pose = result_base_link_on_map;

  // TODO(YamatoAndo) transform covariance on base_link to map frame
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
}
