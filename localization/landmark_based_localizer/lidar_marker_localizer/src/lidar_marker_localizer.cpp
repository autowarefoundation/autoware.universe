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

  rclcpp::CallbackGroup::SharedPtr points_callback_group;
  points_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto points_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = points_callback_group;
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud_ex", rclcpp::QoS(1).best_effort(),
    std::bind(&LidarMarkerLocalizer::points_callback, this, _1), points_sub_opt);

  rclcpp::CallbackGroup::SharedPtr self_pose_callback_group;
  self_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto self_pose_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = self_pose_callback_group;
  sub_self_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", rclcpp::QoS(1),
    std::bind(&LidarMarkerLocalizer::self_pose_callback, this, _1), points_sub_opt);
  sub_map_bin_ = this->create_subscription<HADMapBin>(
    "/map/vector_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&LidarMarkerLocalizer::map_bin_callback, this, std::placeholders::_1));

  pub_marker_points_on_base_link_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "marker_points_on_base_link", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  // pub_sensor_points_on_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  //   "sensor_points_on_map", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  pub_marker_pose_on_velodyne_top_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "marker_pose_on_velodyne_top", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  pub_marker_pose_on_map_from_self_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "marker_pose_on_map_from_self_pose", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  pub_marker_pose_on_base_link_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "marker_pose_on_base_link", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  pub_initial_base_link_on_map_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "initial_base_link_on_map", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  pub_result_base_link_on_map_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "result_base_link_on_map", 10);  // rclcpp::SensorDataQoS().keep_last(5));
  pub_base_link_pose_with_covariance_on_map_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_estimator/pose_with_covariance",
      10);  // rclcpp::SensorDataQoS().keep_last(5));
  rclcpp::QoS qos_marker = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_marker.transient_local();
  qos_marker.reliable();
  pub_marker_ = this->create_publisher<MarkerArray>("~/debug/marker", qos_marker);

  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
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
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr)
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
    geometry_msgs::msg::TransformStamped transform_self_pose_frame_to_map;
    try {
      transform_self_pose_frame_to_map = tf_buffer_->lookupTransform(
        "map", self_pose_msg_ptr->header.frame_id, self_pose_msg_ptr->header.stamp,
        rclcpp::Duration::from_seconds(0.1));

      // transform self_pose_frame to map_frame
      auto self_pose_on_map_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
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

void LidarMarkerLocalizer::points_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  std::string sensor_frame = points_msg_ptr->header.frame_id;
  auto sensor_ros_time = points_msg_ptr->header.stamp;
  // get sensor_frame pose to base_link
  // geometry_msgs::msg::TransformStamped transform_sensor_to_base_link;
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
  const double resolution = 0.05;
  int dx = static_cast<int>((max_x - min_x) * (1 / resolution) + 1);
  std::cerr << "dx " << dx << std::endl;

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
      int ix = (point.x - min_x) * (1 / resolution);
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
    const int filter_window_size = 5;
    // const double marker_width = 0.2;
    // const int positive_window_size = marker_width / resolution;
    const int positive_window_size = 2;
    const int negative_window_size = filter_window_size - positive_window_size;

    const int intensity_difference_threshold = 20;
    const int positive_vote_threshold = 3;
    const int negative_vote_threshold = 3;

    for (int i = filter_window_size * 2; i < dx - filter_window_size * 2; i++) {
      double pos = 0;
      double neg = 0;
      double max = -1;
      double min = 1000;

      // find max_min
      for (int j = -filter_window_size; j <= filter_window_size; j++) {
        if (max < intensity_line_image[i + j]) max = intensity_line_image[i + j];
        if (min > intensity_line_image[i + j]) min = intensity_line_image[i + j];
      }

      if (max > min) {
        double median = (max - min) / 2.0 + min;
        for (int j = -positive_window_size; j <= positive_window_size; j++) {
          if (median + intensity_difference_threshold < intensity_line_image[i + j]) pos += 1;
        }
        for (int j = -filter_window_size; j <= -negative_window_size; j++) {
          if (median - intensity_difference_threshold > intensity_line_image[i + j]) neg += 1;
        }
        for (int j = negative_window_size; j <= filter_window_size; j++) {
          if (median - intensity_difference_threshold > intensity_line_image[i + j]) neg += 1;
        }
        if (pos >= positive_vote_threshold && neg >= negative_vote_threshold) {
          vote[i]++;
          feature_sum[i] += (max - min);
        }
      }
    }
  }

  // extract feature points
  std::vector<geometry_msgs::msg::PoseStamped> marker_pose_on_base_link_array;
  const int vote_threshold_for_detect_marker = 20;
  const int filter_window_size = 5;  // TODO duplicated parameter

  for (int i = filter_window_size * 2; i < dx - filter_window_size * 2; i++) {
    if (vote[i] > vote_threshold_for_detect_marker) {
      geometry_msgs::msg::PoseStamped marker_pose_on_base_link;
      marker_pose_on_base_link.header.stamp = sensor_ros_time;
      marker_pose_on_base_link.header.frame_id = "base_link";
      marker_pose_on_base_link.pose.position.x = (i - dx / 2) * resolution;
      marker_pose_on_base_link.pose.position.y = distance[i];
      marker_pose_on_base_link.pose.position.z = 0.2 + 1.75 / 2.0;  // TODO
      marker_pose_on_base_link.pose.orientation =
        tier4_autoware_utils::createQuaternionFromRPY(M_PI_2, 0.0, 0.0);  // TODO
      marker_pose_on_base_link_array.push_back(marker_pose_on_base_link);
    }
  }

  // -------------------
  // for debug
  // get base_link pose on map frame
  // geometry_msgs::msg::TransformStamped transform_base_link_to_map;
  // try
  // {
  //   transform_base_link_to_map = tf_buffer_->lookupTransform(
  //     "map", "base_link", sensor_ros_time, rclcpp::Duration::from_seconds(0.1));
  // }
  // catch (tf2::TransformException & ex)
  // {
  //   RCLCPP_WARN(get_logger(), "cannot get map to base_link transform. %s", ex.what());
  // }

  // pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> sensor_points_on_base_link_ptr(
  //   new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::fromROSMsg(sensor_points_msg_on_base_link, *sensor_points_on_base_link_ptr);

  // geometry_msgs::msg::PoseStamped sensor_pose_on_map;
  // sensor_pose_on_map = tier4_autoware_utils::transform2pose(transform_base_link_to_map);
  // sensor_pose_on_map.header.stamp = sensor_ros_time;
  // sensor_pose_on_map.header.frame_id = "map";

  // Eigen::Affine3d sensor_pose_on_map_eigen_affine;
  // tf2::fromMsg(sensor_pose_on_map.pose, sensor_pose_on_map_eigen_affine);
  // Eigen::Matrix4f sensor_pose_on_map_eigen_matrix =
  // sensor_pose_on_map_eigen_affine.matrix().cast<float>();

  // pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> sensor_points_on_map_ptr(
  //   new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::transformPointCloud(
  //   *sensor_points_on_base_link_ptr, *sensor_points_on_map_ptr, sensor_pose_on_map_eigen_matrix);

  // sensor_msgs::msg::PointCloud2 sensor_points_on_map_msg;
  // pcl::toROSMsg(*sensor_points_on_map_ptr, sensor_points_on_map_msg);
  // sensor_points_on_map_msg.header = sensor_pose_on_map.header;
  // pub_sensor_points_on_map_->publish(sensor_points_on_map_msg);

  // --------------------------

  //-----
  // get marker pose array on map using lanelet2 map
  std::vector<std::string> marker_name_array = {"tag_0",  "tag_1",  "tag_2",  "tag_3",  "tag_4",
                                                "tag_5",  "tag_6",  "tag_7",  "tag_8",  "tag_9",
                                                "tag_10", "tag_11", "tag_12", "tag_13", "tag_14",
                                                "tag_15", "tag_16", "tag_17", "tag_18"};
  std::vector<geometry_msgs::msg::TransformStamped> transform_marker_on_map_array;

  // get self-position on map

  const double self_pose_timeout_sec_ = 1.0;           // TODO
  const double self_pose_distance_tolerance_m_ = 1.0;  // TODO

  std::unique_lock<std::mutex> self_pose_array_lock(self_pose_array_mtx_);
  if (self_pose_msg_ptr_array_.size() <= 1) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No Pose!");
    return;
  }
  PoseArrayInterpolator interpolator(
    this, sensor_ros_time, self_pose_msg_ptr_array_, self_pose_timeout_sec_,
    self_pose_distance_tolerance_m_);
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

  std::cerr << "marker_pose_on_base_link_from_lanele2_map "
            << marker_pose_on_base_link_from_lanele2_map.x << " "
            << marker_pose_on_base_link_from_lanele2_map.y << " "
            << marker_pose_on_base_link_from_lanele2_map.z << std::endl;

  const double limit_distance_from_self_pose_to_marker_from_lanelet2 = 5.0;
  // is_exist_marker_within_self_pose_ = distance_from_self_pose_to_marker <
  // limit_distance_from_self_pose_to_marker_from_lanelet2;
  is_exist_marker_within_self_pose_ = std::fabs(marker_pose_on_base_link_from_lanele2_map.x) <
                                      limit_distance_from_self_pose_to_marker_from_lanelet2;
  if (!is_exist_marker_within_self_pose_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "The distance from self-pose to the nearest marker of lanelet2 is too far("
        << marker_pose_on_base_link_from_lanele2_map.x << "). The limit is "
        << limit_distance_from_self_pose_to_marker_from_lanelet2 << ".");
    // return;
  }

  // if(!is_detected_marker_ || !is_exist_marker_within_self_pose_) {
  //   return;
  // }

  //-----
  is_detected_marker_ = !marker_pose_on_base_link_array.empty();
  if (!is_detected_marker_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Could not detect marker");
    return;
  }

  // get marker_pose on base_link
  geometry_msgs::msg::PoseStamped marker_pose_on_base_link;
  marker_pose_on_base_link = marker_pose_on_base_link_array.at(0);  // TODO
  std::cerr << "marker_pose_on_base_link " << marker_pose_on_base_link.pose.position.x << " "
            << marker_pose_on_base_link.pose.position.y << " "
            << marker_pose_on_base_link.pose.position.z << std::endl;

  pub_marker_pose_on_base_link_->publish(marker_pose_on_base_link);

  // get marker pose on map using self-pose
  const auto self_pose_rpy = tier4_autoware_utils::getRPY(self_pose_msg.pose.pose.orientation);
  const double self_pose_yaw = self_pose_rpy.z;

  geometry_msgs::msg::PoseStamped marker_pose_on_map_from_self_pose;
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
  std::cerr << "marker_pose_on_map_from_self_pose "
            << marker_pose_on_map_from_self_pose.pose.position.x << " "
            << marker_pose_on_map_from_self_pose.pose.position.y << " "
            << marker_pose_on_map_from_self_pose.pose.position.z << std::endl;
  pub_marker_pose_on_map_from_self_pose_->publish(marker_pose_on_map_from_self_pose);
  //-----

  // get base_link pose on map
  // geometry_msgs::msg::PoseStamped base_link_on_map;
  // base_link_on_map.header.stamp = sensor_ros_time;
  // base_link_on_map.header.frame_id = "map";
  // // base_link_on_map.pose =
  // tier4_autoware_utils::inverseTransformPose(marker_pose_on_map_from_lanelet2_map.pose,
  // marker_pose_on_base_link.pose);

  // const auto map_to_marker_pose = marker_pose_on_map_from_lanelet2_map;
  // Eigen::Affine3d eigen_map_to_marker_pose =
  // tf2::transformToEigen(tier4_autoware_utils::pose2transform(map_to_marker_pose.pose));

  // const auto base_link_to_marker_pose = marker_pose_on_base_link;
  // Eigen::Affine3d eigen_base_link_to_marker_pose =
  // tf2::transformToEigen(tier4_autoware_utils::pose2transform(base_link_to_marker_pose.pose));
  // Eigen::Affine3d eigen_marker_pose_to_base_link = eigen_base_link_to_marker_pose.inverse();

  // Eigen::Affine3d map_to_base_link = eigen_map_to_marker_pose * eigen_marker_pose_to_base_link;
  // base_link_on_map.pose = tf2::toMsg(map_to_base_link);
  // pub_base_link_on_map_->publish(base_link_on_map);

  if (!is_detected_marker_ || !is_exist_marker_within_self_pose_) {
    return;
  }

  //
  geometry_msgs::msg::Vector3 diff_position_from_self_position_to_lanelet2_map;
  diff_position_from_self_position_to_lanelet2_map.x =
    marker_pose_on_map_from_lanelet2_map.position.x -
    marker_pose_on_map_from_self_pose.pose.position.x;
  diff_position_from_self_position_to_lanelet2_map.y =
    marker_pose_on_map_from_lanelet2_map.position.y -
    marker_pose_on_map_from_self_pose.pose.position.y;
  std::cerr << "diff_position_from_self_position_to_lanelet2_map "
            << diff_position_from_self_position_to_lanelet2_map.x << " "
            << diff_position_from_self_position_to_lanelet2_map.y << std::endl;

  const double limit_distance_from_self_pose_to_marker = 2.0;
  double diff_position_from_self_position_to_lanelet2_map_norm = std::hypot(
    diff_position_from_self_position_to_lanelet2_map.x,
    diff_position_from_self_position_to_lanelet2_map.y);
  bool is_exist_marker_within_lanelet2_map =
    diff_position_from_self_position_to_lanelet2_map_norm < limit_distance_from_self_pose_to_marker;

  if (!is_exist_marker_within_lanelet2_map) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1,
      "The distance from lanelet2 to the detect marker is too far("
        << diff_position_from_self_position_to_lanelet2_map_norm << "). The limit is "
        << limit_distance_from_self_pose_to_marker << ".");
    return;
  }

  geometry_msgs::msg::PoseStamped initial_base_link_on_map;
  initial_base_link_on_map.header.stamp = sensor_ros_time;
  initial_base_link_on_map.header.frame_id = "map";
  initial_base_link_on_map.pose = self_pose_msg.pose.pose;
  pub_initial_base_link_on_map_->publish(initial_base_link_on_map);

  geometry_msgs::msg::PoseStamped result_base_link_on_map;
  result_base_link_on_map.header.stamp = sensor_ros_time;
  result_base_link_on_map.header.frame_id = "map";
  result_base_link_on_map.pose.position.x =
    self_pose_msg.pose.pose.position.x + diff_position_from_self_position_to_lanelet2_map.x;
  result_base_link_on_map.pose.position.y =
    self_pose_msg.pose.pose.position.y + diff_position_from_self_position_to_lanelet2_map.y;
  result_base_link_on_map.pose.position.z = self_pose_msg.pose.pose.position.z;
  result_base_link_on_map.pose.orientation = self_pose_msg.pose.pose.orientation;
  pub_result_base_link_on_map_->publish(result_base_link_on_map);

  geometry_msgs::msg::PoseWithCovarianceStamped base_link_pose_with_covariance_on_map;
  base_link_pose_with_covariance_on_map.header.stamp = sensor_ros_time;
  base_link_pose_with_covariance_on_map.header.frame_id = "map";
  base_link_pose_with_covariance_on_map.pose.pose = result_base_link_on_map.pose;

  // TODO transform covariance on base_link to map frame
  base_link_pose_with_covariance_on_map.pose.covariance[0 * 6 + 0] = 0.2 * 0.2;        // TODO
  base_link_pose_with_covariance_on_map.pose.covariance[1 * 6 + 1] = 0.2 * 0.2;        // TODO
  base_link_pose_with_covariance_on_map.pose.covariance[2 * 6 + 2] = 0.1 * 0.1;        // TODO
  base_link_pose_with_covariance_on_map.pose.covariance[3 * 6 + 3] = 0.0087 * 0.0087;  // TODO
  base_link_pose_with_covariance_on_map.pose.covariance[4 * 6 + 4] = 0.0087 * 0.0087;  // TODO
  base_link_pose_with_covariance_on_map.pose.covariance[5 * 6 + 5] = 0.0175 * 0.0175;  // TODO
  pub_base_link_pose_with_covariance_on_map_->publish(base_link_pose_with_covariance_on_map);
}

void LidarMarkerLocalizer::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("exist_marker_within_self_pose", is_exist_marker_within_self_pose_ ? "Yes" : "No");
  stat.add("detected_marker", is_detected_marker_ ? "Yes" : "No");

  if (is_exist_marker_within_self_pose_ & is_detected_marker_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK. Detect a marker");
  } else if (is_exist_marker_within_self_pose_ & !is_detected_marker_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "NG. Could not detect a marker");
  } else if (!is_exist_marker_within_self_pose_ & is_detected_marker_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK. Detect a not marker-object");
  } else if (!is_exist_marker_within_self_pose_ & !is_detected_marker_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "OK. There are no markers within the range of self-pose");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "NG. This message should not be displayed.");
  }
}

void LidarMarkerLocalizer::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
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
