// Copyright 2022-2023 TIER IV, Inc.
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

#include "radar_tracks_msgs_converter/radar_tracks_msgs_converter_node.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"
#include "tier4_autoware_utils/ros/msg_covariance.hpp"

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using std::chrono::nanoseconds;
using std::placeholders::_1;

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace radar_tracks_msgs_converter
{

enum class RadarTrackObjectID {
  UNKNOWN = 32000,
  CAR = 32001,
  TRUCK = 32002,
  BUS = 32003,
  TRAILER = 32004,
  MOTORCYCLE = 32005,
  BICYCLE = 32006,
  PEDESTRIAN = 32007
};

RadarTracksMsgsConverterNode::RadarTracksMsgsConverterNode(const rclcpp::NodeOptions & node_options)
: Node("radar_tracks_msgs_converter", node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadarTracksMsgsConverterNode::on_set_param, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 20.0);
  node_param_.new_frame_id = declare_parameter<std::string>("new_frame_id", "base_link");
  node_param_.use_twist_compensation = declare_parameter<bool>("use_twist_compensation", false);
  node_param_.use_twist_yaw_compensation =
    declare_parameter<bool>("use_twist_yaw_compensation", false);
  node_param_.static_object_speed_threshold =
    declare_parameter<double>("static_object_speed_threshold", 1.0);

  // Subscriber
  sub_radar_ = create_subscription<RadarTracks>(
    "~/input/radar_objects", rclcpp::QoS{1},
    std::bind(&RadarTracksMsgsConverterNode::on_radar_tracks, this, _1));
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    std::bind(&RadarTracksMsgsConverterNode::on_twist, this, _1));
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  // Publisher
  pub_tracked_objects_ = create_publisher<TrackedObjects>("~/output/radar_tracked_objects", 1);
  pub_detected_objects_ = create_publisher<DetectedObjects>("~/output/radar_detected_objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&RadarTracksMsgsConverterNode::on_timer, this));
}

void RadarTracksMsgsConverterNode::on_radar_tracks(const RadarTracks::ConstSharedPtr msg)
{
  radar_data_ = msg;
}

void RadarTracksMsgsConverterNode::on_twist(const Odometry::ConstSharedPtr msg)
{
  odometry_data_ = msg;
}

rcl_interfaces::msg::SetParametersResult RadarTracksMsgsConverterNode::on_set_param(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "update_rate_hz", p.update_rate_hz);
      update_param(params, "new_frame_id", p.new_frame_id);
      update_param(params, "use_twist_compensation", p.use_twist_compensation);
      update_param(params, "use_twist_yaw_compensation", p.use_twist_yaw_compensation);
      update_param(params, "static_object_speed_threshold", p.static_object_speed_threshold);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

bool RadarTracksMsgsConverterNode::is_data_ready()
{
  if (!radar_data_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for radar msg...");
    return false;
  }

  return true;
}

void RadarTracksMsgsConverterNode::on_timer()
{
  if (!is_data_ready()) {
    return;
  }
  const auto & header = radar_data_->header;
  transform_ = transform_listener_->getTransform(
    node_param_.new_frame_id, header.frame_id, header.stamp, rclcpp::Duration::from_seconds(0.01));

  TrackedObjects tracked_objects = convert_radar_track_to_tracked_objects();
  DetectedObjects detected_objects = convert_tracked_objects_to_detected_objects(tracked_objects);
  if (!tracked_objects.objects.empty()) {
    pub_tracked_objects_->publish(tracked_objects);
    pub_detected_objects_->publish(detected_objects);
  }
}

DetectedObjects RadarTracksMsgsConverterNode::convert_tracked_objects_to_detected_objects(
  TrackedObjects & objects)
{
  DetectedObjects detected_objects;
  detected_objects.header = objects.header;

  for (auto & object : objects.objects) {
    DetectedObject detected_object;
    detected_object.existence_probability = 1.0;
    detected_object.shape = object.shape;

    // kinematics setting
    DetectedObjectKinematics kinematics;
    kinematics.orientation_availability =
      autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
    kinematics.has_twist = true;
    kinematics.has_twist_covariance = true;
    kinematics.twist_with_covariance = object.kinematics.twist_with_covariance;
    kinematics.pose_with_covariance = object.kinematics.pose_with_covariance;
    detected_object.kinematics = kinematics;

    // classification
    detected_object.classification = object.classification;
    detected_objects.objects.emplace_back(detected_object);
  }
  return detected_objects;
}

bool RadarTracksMsgsConverterNode::is_static_object(
  const radar_msgs::msg::RadarTrack & radar_track,
  const geometry_msgs::msg::Vector3 & compensated_velocity)
{
  if (!(node_param_.use_twist_compensation && odometry_data_)) {
    return false;
  }

  // Calculate azimuth angle of the object in the vehicle coordinate
  const double sensor_yaw = tf2::getYaw(transform_->transform.rotation);
  const double radar_azimuth = std::atan2(radar_track.position.y, radar_track.position.x);
  const double azimuth = radar_azimuth + sensor_yaw;

  // Calculate longitudinal speed
  const double longitudinal_speed =
    compensated_velocity.x * std::cos(azimuth) + compensated_velocity.y * std::sin(azimuth);

  // Check if the object is static
  return std::abs(longitudinal_speed) < node_param_.static_object_speed_threshold;
}

TrackedObjects RadarTracksMsgsConverterNode::convert_radar_track_to_tracked_objects()
{
  TrackedObjects tracked_objects;
  tracked_objects.header = radar_data_->header;
  tracked_objects.header.frame_id = node_param_.new_frame_id;

  for (auto & radar_track : radar_data_->tracks) {
    TrackedObject tracked_object;

    tracked_object.object_id = radar_track.uuid;
    tracked_object.existence_probability = 1.0;

    tracked_object.shape.type = Shape::BOUNDING_BOX;
    tracked_object.shape.dimensions = radar_track.size;

    // Pose conversion
    geometry_msgs::msg::PoseStamped radar_pose_stamped{};
    radar_pose_stamped.pose.position = radar_track.position;

    geometry_msgs::msg::PoseStamped transformed_pose_stamped{};
    if (transform_ == nullptr) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "getTransform failed. radar output will be empty.");
      return tracked_objects;
    }
    tf2::doTransform(radar_pose_stamped, transformed_pose_stamped, *transform_);
    const auto & position_from_veh = transformed_pose_stamped.pose.position;

    // Velocity conversion
    // 1: Compensate radar coordinate
    // radar track velocity is defined in the radar coordinate
    // compensate radar coordinate to vehicle coordinate
    auto compensated_velocity = compensate_velocity_sensor_position(radar_track);
    // 2: Compensate ego motion
    if (node_param_.use_twist_compensation && odometry_data_) {
      compensated_velocity =
        compensate_velocity_ego_motion(compensated_velocity, position_from_veh);
    } else if (node_param_.use_twist_compensation && !odometry_data_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odometry data is not available. Radar track velocity will not be compensated.");
    }

    // yaw angle (vehicle heading) is obtained from ground velocity
    double yaw = tier4_autoware_utils::normalizeRadian(
      std::atan2(compensated_velocity.y, compensated_velocity.x));

    // kinematics setting
    TrackedObjectKinematics kinematics;
    kinematics.orientation_availability = TrackedObjectKinematics::AVAILABLE;
    kinematics.is_stationary = is_static_object(radar_track, compensated_velocity);
    kinematics.pose_with_covariance.pose = transformed_pose_stamped.pose;
    kinematics.pose_with_covariance.covariance = convert_pose_covariance_matrix(radar_track);
    // velocity of object is defined in the object coordinate
    // heading is obtained from ground velocity
    kinematics.pose_with_covariance.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw);
    // longitudinal velocity is the length of the velocity vector
    // lateral velocity is zero, use default value
    kinematics.twist_with_covariance.twist.linear.x = std::sqrt(
      compensated_velocity.x * compensated_velocity.x +
      compensated_velocity.y * compensated_velocity.y);
    kinematics.twist_with_covariance.covariance = convert_twist_covariance_matrix(radar_track);
    // acceleration is zero, use default value
    kinematics.acceleration_with_covariance.covariance =
      convert_acceleration_covariance_matrix(radar_track);

    // fill the kinematics to the tracked object
    tracked_object.kinematics = kinematics;

    // classification
    ObjectClassification classification;
    classification.probability = 1.0;
    classification.label = convert_classification(radar_track.classification);
    tracked_object.classification.emplace_back(classification);

    tracked_objects.objects.emplace_back(tracked_object);
  }
  return tracked_objects;
}

geometry_msgs::msg::Vector3 RadarTracksMsgsConverterNode::compensate_velocity_sensor_position(
  const radar_msgs::msg::RadarTrack & radar_track)
{
  // initialize compensated velocity
  geometry_msgs::msg::Vector3 compensated_velocity{};

  const double sensor_yaw = tf2::getYaw(transform_->transform.rotation);
  const geometry_msgs::msg::Vector3 & vel = radar_track.velocity;
  compensated_velocity.x = vel.x * std::cos(sensor_yaw) - vel.y * std::sin(sensor_yaw);
  compensated_velocity.y = vel.x * std::sin(sensor_yaw) + vel.y * std::cos(sensor_yaw);
  compensated_velocity.z = vel.z;

  return compensated_velocity;
}

geometry_msgs::msg::Vector3 RadarTracksMsgsConverterNode::compensate_velocity_ego_motion(
  const geometry_msgs::msg::Vector3 & velocity_in,
  const geometry_msgs::msg::Point & position_from_veh)
{
  geometry_msgs::msg::Vector3 velocity = velocity_in;
  // linear compensation
  velocity.x += odometry_data_->twist.twist.linear.x;
  velocity.y += odometry_data_->twist.twist.linear.y;
  velocity.z += odometry_data_->twist.twist.linear.z;
  if (node_param_.use_twist_yaw_compensation) {
    // angular compensation
    const double veh_yaw = odometry_data_->twist.twist.angular.z;
    velocity.x += -position_from_veh.y * veh_yaw;
    velocity.y += position_from_veh.x * veh_yaw;
  }
  return velocity;
}

std::array<double, 36> RadarTracksMsgsConverterNode::convert_pose_covariance_matrix(
  const radar_msgs::msg::RadarTrack & radar_track)
{
  using POSE_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_IDX = tier4_autoware_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;
  std::array<double, 36> pose_covariance{};
  pose_covariance[POSE_IDX::X_X] = radar_track.position_covariance[RADAR_IDX::X_X];
  pose_covariance[POSE_IDX::X_Y] = radar_track.position_covariance[RADAR_IDX::X_Y];
  pose_covariance[POSE_IDX::X_Z] = radar_track.position_covariance[RADAR_IDX::X_Z];
  pose_covariance[POSE_IDX::Y_X] = radar_track.position_covariance[RADAR_IDX::X_Y];
  pose_covariance[POSE_IDX::Y_Y] = radar_track.position_covariance[RADAR_IDX::Y_Y];
  pose_covariance[POSE_IDX::Y_Z] = radar_track.position_covariance[RADAR_IDX::Y_Z];
  pose_covariance[POSE_IDX::Z_X] = radar_track.position_covariance[RADAR_IDX::X_Z];
  pose_covariance[POSE_IDX::Z_Y] = radar_track.position_covariance[RADAR_IDX::Y_Z];
  pose_covariance[POSE_IDX::Z_Z] = radar_track.position_covariance[RADAR_IDX::Z_Z];
  return pose_covariance;
}
std::array<double, 36> RadarTracksMsgsConverterNode::convert_twist_covariance_matrix(
  const radar_msgs::msg::RadarTrack & radar_track)
{
  using POSE_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_IDX = tier4_autoware_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;
  std::array<double, 36> twist_covariance{};
  twist_covariance[POSE_IDX::X_X] = radar_track.velocity_covariance[RADAR_IDX::X_X];
  twist_covariance[POSE_IDX::X_Y] = radar_track.velocity_covariance[RADAR_IDX::X_Y];
  twist_covariance[POSE_IDX::X_Z] = radar_track.velocity_covariance[RADAR_IDX::X_Z];
  twist_covariance[POSE_IDX::Y_X] = radar_track.velocity_covariance[RADAR_IDX::X_Y];
  twist_covariance[POSE_IDX::Y_Y] = radar_track.velocity_covariance[RADAR_IDX::Y_Y];
  twist_covariance[POSE_IDX::Y_Z] = radar_track.velocity_covariance[RADAR_IDX::Y_Z];
  twist_covariance[POSE_IDX::Z_X] = radar_track.velocity_covariance[RADAR_IDX::X_Z];
  twist_covariance[POSE_IDX::Z_Y] = radar_track.velocity_covariance[RADAR_IDX::Y_Z];
  twist_covariance[POSE_IDX::Z_Z] = radar_track.velocity_covariance[RADAR_IDX::Z_Z];
  return twist_covariance;
}
std::array<double, 36> RadarTracksMsgsConverterNode::convert_acceleration_covariance_matrix(
  const radar_msgs::msg::RadarTrack & radar_track)
{
  using POSE_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  using RADAR_IDX = tier4_autoware_utils::xyz_upper_covariance_index::XYZ_UPPER_COV_IDX;
  std::array<double, 36> acceleration_covariance{};
  acceleration_covariance[POSE_IDX::X_X] = radar_track.acceleration_covariance[RADAR_IDX::X_X];
  acceleration_covariance[POSE_IDX::X_Y] = radar_track.acceleration_covariance[RADAR_IDX::X_Y];
  acceleration_covariance[POSE_IDX::X_Z] = radar_track.acceleration_covariance[RADAR_IDX::X_Z];
  acceleration_covariance[POSE_IDX::Y_X] = radar_track.acceleration_covariance[RADAR_IDX::X_Y];
  acceleration_covariance[POSE_IDX::Y_Y] = radar_track.acceleration_covariance[RADAR_IDX::Y_Y];
  acceleration_covariance[POSE_IDX::Y_Z] = radar_track.acceleration_covariance[RADAR_IDX::Y_Z];
  acceleration_covariance[POSE_IDX::Z_X] = radar_track.acceleration_covariance[RADAR_IDX::X_Z];
  acceleration_covariance[POSE_IDX::Z_Y] = radar_track.acceleration_covariance[RADAR_IDX::Y_Z];
  acceleration_covariance[POSE_IDX::Z_Z] = radar_track.acceleration_covariance[RADAR_IDX::Z_Z];
  return acceleration_covariance;
}

uint8_t RadarTracksMsgsConverterNode::convert_classification(const uint16_t classification)
{
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::UNKNOWN)) {
    return ObjectClassification::UNKNOWN;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::CAR)) {
    return ObjectClassification::CAR;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::TRUCK)) {
    return ObjectClassification::TRUCK;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::BUS)) {
    return ObjectClassification::BUS;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::TRAILER)) {
    return ObjectClassification::TRAILER;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::MOTORCYCLE)) {
    return ObjectClassification::MOTORCYCLE;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::BICYCLE)) {
    return ObjectClassification::BICYCLE;
  }
  if (classification == static_cast<uint16_t>(RadarTrackObjectID::PEDESTRIAN)) {
    return ObjectClassification::PEDESTRIAN;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Receive unknown label for RadarTracks");
  return ObjectClassification::UNKNOWN;
}

}  // namespace radar_tracks_msgs_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(radar_tracks_msgs_converter::RadarTracksMsgsConverterNode)
