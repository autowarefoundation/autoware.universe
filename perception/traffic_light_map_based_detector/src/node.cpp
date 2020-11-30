/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Yukihiro Saito
 *
 */

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <autoware_perception_msgs/msg/traffic_light_roi.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <traffic_light_map_based_detector/node.hpp>

#include <rclcpp/rclcpp.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace traffic_light
{
  MapBasedDetector::MapBasedDetector() : Node("traffic_light_map_based_detector"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{

  using std::placeholders::_1;

  // subscribers
  map_sub_ = create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/vector_map", 1, std::bind(&MapBasedDetector::mapCallback, this, _1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "input/camera_info", 1, std::bind(&MapBasedDetector::cameraInfoCallback, this, _1));
  route_sub_ = create_subscription<autoware_planning_msgs::msg::Route>(
    "input/route", 1, std::bind(&MapBasedDetector::routeCallback, this, _1));

  // publishers
  roi_pub_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightRoiArray>(
    "output/rois", 1);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "debug/markers", 1);

  // parameter declaration needs default values: are 0.0 goof defaults for this?
  config_.max_vibration_pitch = declare_parameter<double>("max_vibration_pitch", 0.0);
  config_.max_vibration_yaw = declare_parameter<double>("max_vibration_yaw", 0.0);
  config_.max_vibration_height = declare_parameter<double>("max_vibration_height", 0.0);
  config_.max_vibration_width = declare_parameter<double>("max_vibration_width", 0.0);
  config_.max_vibration_depth = declare_parameter<double>("max_vibration_depth", 0.0);
}

  void MapBasedDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg)
{
  camera_info_ptr_ = input_msg;
  if (lanelet_map_ptr_ == nullptr || camera_info_ptr_ == nullptr) return;

  autoware_perception_msgs::msg::TrafficLightRoiArray output_msg;
  output_msg.header = camera_info_ptr_->header;
  geometry_msgs::msg::PoseStamped camera_pose_stamped;
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer_.lookupTransform(
      "map",
      input_msg->header.frame_id,
      input_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.2));
    camera_pose_stamped.header = input_msg->header;
    camera_pose_stamped.pose.position.x = transform.transform.translation.x;
    camera_pose_stamped.pose.position.y = transform.transform.translation.y;
    camera_pose_stamped.pose.position.z = transform.transform.translation.z;
    camera_pose_stamped.pose.orientation.x = transform.transform.rotation.x;
    camera_pose_stamped.pose.orientation.y = transform.transform.rotation.y;
    camera_pose_stamped.pose.orientation.z = transform.transform.rotation.z;
    camera_pose_stamped.pose.orientation.w = transform.transform.rotation.w;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "cannot get transform frome map frame to camera frame");
    return;
  }

  const sensor_msgs::msg::CameraInfo & camera_info = *input_msg;
  std::vector<lanelet::ConstLineString3d> visible_traffic_lights;
  if (route_traffic_lights_ptr_ != nullptr)
    isInVisibility(
      *route_traffic_lights_ptr_, camera_pose_stamped.pose, camera_info, visible_traffic_lights);
  else if (all_traffic_lights_ptr_ != nullptr)
    isInVisibility(
      *all_traffic_lights_ptr_, camera_pose_stamped.pose, camera_info, visible_traffic_lights);
  else
    return;

  for (const auto & traffic_light : visible_traffic_lights) {
    autoware_perception_msgs::msg::TrafficLightRoi tl_roi;
    if (!getTrafficLightRoi(camera_pose_stamped.pose, camera_info, traffic_light, config_, tl_roi))
      continue;

    output_msg.rois.push_back(tl_roi);
  }
  roi_pub_->publish(output_msg);
  publishVisibleTrafficLights(camera_pose_stamped, visible_traffic_lights, viz_pub_);
}

bool MapBasedDetector::getTrafficLightRoi(
  const geometry_msgs::msg::Pose & camera_pose,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const lanelet::ConstLineString3d traffic_light,
  const Config & config,
  autoware_perception_msgs::msg::TrafficLightRoi & tl_roi)
{
  const double tl_height = traffic_light.attributeOr("height", 0.0);
  const double & fx = camera_info.k[(0 * 3) + 0];
  const double & fy = camera_info.k[(1 * 3) + 1];
  const double & cx = camera_info.k[(0 * 3) + 2];
  const double & cy = camera_info.k[(1 * 3) + 2];
  const auto & tl_left_down_point = traffic_light.front();
  const auto & tl_right_down_point = traffic_light.back();
  tf2::Transform tf_map2camera(
    tf2::Quaternion(
      camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z,
      camera_pose.orientation.w),
    tf2::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
  // id
  tl_roi.id = traffic_light.id();

  // for roi.x_offset and roi.y_offset
  {
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(
        tl_left_down_point.x(), tl_left_down_point.y(), tl_left_down_point.z() + tl_height));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    const double & camera_x =
      tf_camera2tl.getOrigin().x() -
      (std::sin(config.max_vibration_yaw / 2.0) * tf_camera2tl.getOrigin().z()) -
      config.max_vibration_width / 2.0;
    const double & camera_y =
      tf_camera2tl.getOrigin().y() -
      (std::sin(config.max_vibration_pitch / 2.0) * tf_camera2tl.getOrigin().z()) -
      config.max_vibration_height / 2.0;
    const double & camera_z = tf_camera2tl.getOrigin().z() - config.max_vibration_depth / 2.0;
    if (camera_z <= 0.0) return false;
    const double image_u = (fx * camera_x + cx * camera_z) / camera_z;
    const double image_v = (fy * camera_y + cy * camera_z) / camera_z;

    tl_roi.roi.x_offset = std::max(std::min(image_u, (double)camera_info.width), 0.0);
    tl_roi.roi.y_offset = std::max(std::min(image_v, (double)camera_info.height), 0.0);
  }

  // for roi.width and roi.height
  {
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(tl_right_down_point.x(), tl_right_down_point.y(), tl_right_down_point.z()));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    const double & camera_x =
      tf_camera2tl.getOrigin().x() +
      (std::sin(config.max_vibration_yaw / 2.0) * tf_camera2tl.getOrigin().z()) +
      config.max_vibration_width / 2.0;
    const double & camera_y =
      tf_camera2tl.getOrigin().y() +
      (std::sin(config.max_vibration_pitch / 2.0) * tf_camera2tl.getOrigin().z()) +
      config.max_vibration_height / 2.0;
    const double & camera_z = tf_camera2tl.getOrigin().z() - config.max_vibration_depth / 2.0;
    if (camera_z <= 0.0) return false;
    const double image_u = (fx * camera_x + cx * camera_z) / camera_z;
    const double image_v = (fy * camera_y + cy * camera_z) / camera_z;
    tl_roi.roi.width =
      std::max(std::min(image_u, (double)camera_info.width), 0.0) - tl_roi.roi.x_offset;
    tl_roi.roi.height =
      std::max(std::min(image_v, (double)camera_info.height), 0.0) - tl_roi.roi.y_offset;
    if (tl_roi.roi.width <= 0 || tl_roi.roi.height <= 0) return false;
  }
  return true;
}

void MapBasedDetector::mapCallback(
  const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr input_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  all_traffic_lights_ptr_ = std::make_shared<MapBasedDetector::TrafficLightSet>();
  // for each traffic light in map check if in range and in view angle of camera
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString())  // traffic ligths must be linestrings
        continue;
      all_traffic_lights_ptr_->insert(static_cast<lanelet::ConstLineString3d>(lsp));
    }
  }
}

void MapBasedDetector::routeCallback(
  const autoware_planning_msgs::msg::Route::ConstSharedPtr input_msg)
{
  if (lanelet_map_ptr_ == nullptr) {
    RCLCPP_WARN(get_logger(), "cannot set traffic light in route because don't recieve map");
    return;
  }
  lanelet::ConstLanelets route_lanelets;
  for (const auto & route_section : input_msg->route_sections) {
    for (const auto & lane_id : route_section.lane_ids) {
      route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(lane_id));
    }
  }
  std::vector<lanelet::AutowareTrafficLightConstPtr> route_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(route_lanelets);
  route_traffic_lights_ptr_ = std::make_shared<MapBasedDetector::TrafficLightSet>();
  for (auto tl_itr = route_lanelet_traffic_lights.begin();
       tl_itr != route_lanelet_traffic_lights.end(); ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights) {
      if (!lsp.isLineString())  // traffic ligths must be linestrings
        continue;
      route_traffic_lights_ptr_->insert(static_cast<lanelet::ConstLineString3d>(lsp));
    }
  }
}

void MapBasedDetector::isInVisibility(
  const MapBasedDetector::TrafficLightSet & all_traffic_lights,
  const geometry_msgs::msg::Pose & camera_pose,
  const sensor_msgs::msg::CameraInfo & camera_info,
  std::vector<lanelet::ConstLineString3d> & visible_traffic_lights)
{
  for (const auto & traffic_light : all_traffic_lights) {
    const auto & tl_left_down_point = traffic_light.front();
    const auto & tl_right_down_point = traffic_light.back();
    const double tl_height = traffic_light.attributeOr("height", 0.0);

    // check distance range
    geometry_msgs::msg::Point tl_central_point;
    tl_central_point.x = (tl_right_down_point.x() + tl_left_down_point.x()) / 2.0;
    tl_central_point.y = (tl_right_down_point.y() + tl_left_down_point.y()) / 2.0;
    tl_central_point.z = (tl_right_down_point.z() + tl_left_down_point.z() + tl_height) / 2.0;
    const double max_distance_range = 200.0;
    if (!isInDistanceRange(tl_central_point, camera_pose.position, max_distance_range)) continue;

    // check angle range
    const double tl_yaw = normalizeAngle(
      std::atan2(
        tl_right_down_point.y() - tl_left_down_point.y(),
        tl_right_down_point.x() - tl_left_down_point.x()) +
      M_PI_2);
    const double max_angele_range = 40.0 / 180.0 * M_PI;

    // get direction of z axis
    tf2::Vector3 camera_z_dir(0, 0, 1);
    tf2::Matrix3x3 camera_rotation_matrix(tf2::Quaternion(
      camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z,
      camera_pose.orientation.w));
    camera_z_dir = camera_rotation_matrix * camera_z_dir;
    double camera_yaw = std::atan2(camera_z_dir.y(), camera_z_dir.x());
    camera_yaw = normalizeAngle(camera_yaw);
    if (!isInAngleRange(tl_yaw, camera_yaw, max_angele_range)) continue;

    // check within image frame
    tf2::Transform tf_map2camera(
      tf2::Quaternion(
        camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z,
        camera_pose.orientation.w),
      tf2::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(tl_central_point.x, tl_central_point.y, tl_central_point.z));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    geometry_msgs::msg::Point camera2tl_point;
    camera2tl_point.x = tf_camera2tl.getOrigin().x();
    camera2tl_point.y = tf_camera2tl.getOrigin().y();
    camera2tl_point.z = tf_camera2tl.getOrigin().z();
    if (!isInImageFrame(camera_info, camera2tl_point)) continue;
    visible_traffic_lights.push_back(traffic_light);
  }
}

bool MapBasedDetector::isInDistanceRange(
  const geometry_msgs::msg::Point & tl_point,
  const geometry_msgs::msg::Point & camera_point,
  const double max_distance_range)
{
  const double sq_dist = (tl_point.x - camera_point.x) * (tl_point.x - camera_point.x) +
                         (tl_point.y - camera_point.y) * (tl_point.y - camera_point.y);
  return (sq_dist < (max_distance_range * max_distance_range));
}

bool MapBasedDetector::isInAngleRange(
  const double & tl_yaw,
  const double & camera_yaw,
  const double max_angele_range)
{
  Eigen::Vector2d vec1, vec2;
  vec1 << std::cos(tl_yaw), std::sin(tl_yaw);
  vec2 << std::cos(camera_yaw), std::sin(camera_yaw);
  const double diff_angle = std::acos(vec1.dot(vec2));
  return (std::fabs(diff_angle) < max_angele_range);
}

bool MapBasedDetector::isInImageFrame(
  const sensor_msgs::msg::CameraInfo & camera_info,
  const geometry_msgs::msg::Point & point)
{
  const double & camera_x = point.x;
  const double & camera_y = point.y;
  const double & camera_z = point.z;
  const double & fx = camera_info.k[(0 * 3) + 0];
  const double & fy = camera_info.k[(1 * 3) + 1];
  const double & cx = camera_info.k[(0 * 3) + 2];
  const double & cy = camera_info.k[(1 * 3) + 2];
  if (camera_z <= 0.0) return false;
  const double image_u = (fx * camera_x + cx * camera_z) / camera_z;
  const double image_v = (fy * camera_y + cy * camera_z) / camera_z;
  if (0 <= image_u && image_u < camera_info.width)
    if (0 <= image_v && image_v < camera_info.height) return true;
  return false;
}

void MapBasedDetector::publishVisibleTrafficLights(
  const geometry_msgs::msg::PoseStamped camera_pose_stamped,
  const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (const auto & traffic_light : visible_traffic_lights) {
    const auto & tl_left_down_point = traffic_light.front();
    const auto & tl_right_down_point = traffic_light.back();
    const double tl_height = traffic_light.attributeOr("height", 0.0);
    const int id = traffic_light.id();

    geometry_msgs::msg::Point tl_central_point;
    tl_central_point.x = (tl_right_down_point.x() + tl_left_down_point.x()) / 2.0;
    tl_central_point.y = (tl_right_down_point.y() + tl_left_down_point.y()) / 2.0;
    tl_central_point.z = (tl_right_down_point.z() + tl_left_down_point.z() + tl_height) / 2.0;

    visualization_msgs::msg::Marker marker;

    tf2::Transform tf_map2camera(
      tf2::Quaternion(
        camera_pose_stamped.pose.orientation.x, camera_pose_stamped.pose.orientation.y,
        camera_pose_stamped.pose.orientation.z, camera_pose_stamped.pose.orientation.w),
      tf2::Vector3(
        camera_pose_stamped.pose.position.x, camera_pose_stamped.pose.position.y,
        camera_pose_stamped.pose.position.z));
    tf2::Transform tf_map2tl(
      tf2::Quaternion(0, 0, 0, 1),
      tf2::Vector3(tl_central_point.x, tl_central_point.y, tl_central_point.z));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    marker.header = camera_pose_stamped.header;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.ns = std::string("beam");
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    marker.points.push_back(point);
    point.x = tf_camera2tl.getOrigin().x();
    point.y = tf_camera2tl.getOrigin().y();
    point.z = tf_camera2tl.getOrigin().z();
    marker.points.push_back(point);

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output_msg.markers.push_back(marker);
  }
  pub->publish(output_msg);

  return;
}

double MapBasedDetector::normalizeAngle(const double & angle)
{
  return std::atan2(std::cos(angle), std::sin(angle));
}

}  // namespace traffic_light
