// Copyright 2020 Tier IV, Inc.
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

#include "surround_obstacle_checker/node.hpp"

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace surround_obstacle_checker
{

namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d, false, false>;  // counter-clockwise, open

namespace
{
std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
  const std::string no_start_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus no_start_reason_diag;
  diagnostic_msgs::msg::KeyValue no_start_reason_diag_kv;
  no_start_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  no_start_reason_diag.name = "no_start_reason";
  no_start_reason_diag.message = no_start_reason;
  no_start_reason_diag_kv.key = "no_start_pose";
  no_start_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  no_start_reason_diag.values.push_back(no_start_reason_diag_kv);
  return no_start_reason_diag;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = size.x;
  const double w = size.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<Point2d>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  for (const auto point : footprint.points) {
    const Point2d point2d(point.x, point.y);
    obj_poly.outer().push_back(point2d);
  }

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

Polygon2d createSelfPolygon(const VehicleInfo & vehicle_info)
{
  const double front = vehicle_info.max_longitudinal_offset_m;
  const double rear = vehicle_info.min_longitudinal_offset_m;
  const double left = vehicle_info.max_lateral_offset_m;
  const double right = vehicle_info.min_lateral_offset_m;

  Polygon2d poly;
  boost::geometry::exterior_ring(poly) = boost::assign::list_of<Point2d>(front, left)(front, right)(
    rear, right)(rear, left)(front, left);
  return poly;
}
}  // namespace

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("surround_obstacle_checker_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  // Parameters
  use_pointcloud_ = this->declare_parameter("use_pointcloud", true);
  use_dynamic_object_ = this->declare_parameter("use_dynamic_object", true);
  surround_check_distance_ = this->declare_parameter("surround_check_distance", 2.0);
  surround_check_recover_distance_ =
    this->declare_parameter("surround_check_recover_distance", 2.5);
  state_clear_time_ = this->declare_parameter("state_clear_time", 2.0);
  stop_state_ego_speed_ = this->declare_parameter("stop_state_ego_speed", 0.1);
  stopped_state_entry_duration_time_ =
    this->declare_parameter("stopped_state_entry_duration_time_", 0.1);
  debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(
    vehicle_info_.max_longitudinal_offset_m, this->get_clock(), *this);

  // Publishers
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/no_start_reason", 1);
  pub_clear_velocity_limit_ =
    this->create_publisher<VelocityLimitClearCommand>("~/output/velocity_limit_clear_command", 1);
  pub_velocity_limit_ = this->create_publisher<VelocityLimit>("~/output/max_velocity", 1);

  // Subscriber
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&SurroundObstacleCheckerNode::pointCloudCallback, this, std::placeholders::_1));
  dynamic_object_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", 1,
      std::bind(&SurroundObstacleCheckerNode::dynamicObjectCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&SurroundObstacleCheckerNode::currentVelocityCallback, this, std::placeholders::_1));

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&SurroundObstacleCheckerNode::onTimer, this));

  last_running_time_ = std::make_shared<rclcpp::Time>(this->now());
}

void SurroundObstacleCheckerNode::onTimer()
{
  if (use_pointcloud_ && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (use_dynamic_object_ && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  if (!odometry_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000 /* ms */, "waiting for current velocity...");
    return;
  }

  // get nearest object
  double min_dist_to_obj = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_obj_point;
  getNearestObstacle(&min_dist_to_obj, &nearest_obj_point);

  // check current obstacle status (exist or not)
  const auto is_obstacle_found = isObstacleFound(min_dist_to_obj);
  // check current stop status (stop or not)
  const auto is_stopped = isVehicleStopped();

  const auto is_stop_required = isStopRequired(is_obstacle_found, is_stopped);

  switch (state_) {
    case State::PASS: {
      if (!is_stop_required) {
        break;
      }

      state_ = State::STOP;

      auto velocity_limit = std::make_shared<VelocityLimit>();
      velocity_limit->stamp = this->now();
      velocity_limit->max_velocity = 0.0;
      velocity_limit->use_constraints = false;
      velocity_limit->sender = "surround_obstacle_checker";

      pub_velocity_limit_->publish(*velocity_limit);

      // do not start when there is a obstacle near the ego vehicle.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 500 /* ms */,
        "do not start because there is obstacle near the ego vehicle.");

      break;
    }

    case State::STOP: {
      if (is_stop_required) {
        break;
      }

      state_ = State::PASS;

      auto velocity_limit_clear_command = std::make_shared<VelocityLimitClearCommand>();
      velocity_limit_clear_command->stamp = this->now();
      velocity_limit_clear_command->command = true;
      velocity_limit_clear_command->sender = "surround_obstacle_checker";

      pub_clear_velocity_limit_->publish(*velocity_limit_clear_command);

      break;
    }

    default:
      break;
  }

  diagnostic_msgs::msg::DiagnosticStatus no_start_reason_diag;
  if (state_ == State::STOP) {
    debug_ptr_->pushObstaclePoint(nearest_obj_point, PointType::NoStart);
    debug_ptr_->pushPose(odometry_ptr_->pose.pose, PoseType::NoStart);
    no_start_reason_diag = makeStopReasonDiag("obstacle", odometry_ptr_->pose.pose);
  }

  stop_reason_diag_pub_->publish(no_start_reason_diag);
  debug_ptr_->publish();
}

void SurroundObstacleCheckerNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  pointcloud_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::dynamicObjectCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr input_msg)
{
  object_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::currentVelocityCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input_msg)
{
  odometry_ptr_ = input_msg;
}

bool SurroundObstacleCheckerNode::convertPose(
  const geometry_msgs::msg::Pose & pose, const std::string & source, const std::string & target,
  const rclcpp::Time & time, geometry_msgs::msg::Pose & conv_pose)
{
  tf2::Transform src2tgt;
  try {
    // get transform from source to target
    geometry_msgs::msg::TransformStamped ros_src2tgt =
      tf_buffer_.lookupTransform(source, target, time, tf2::durationFromSec(0.1));
    tf2::fromMsg(ros_src2tgt.transform, src2tgt);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 500 /* ms */,
      "cannot get tf from " << source << " to " << target);
    return false;
  }

  tf2::Transform src2obj;
  tf2::fromMsg(pose, src2obj);
  tf2::Transform tgt2obj = src2tgt.inverse() * src2obj;
  tf2::toMsg(tgt2obj, conv_pose);
  return true;
}

void SurroundObstacleCheckerNode::getNearestObstacle(
  double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point)
{
  if (use_pointcloud_) {
    getNearestObstacleByPointCloud(min_dist_to_obj, nearest_obj_point);
  }

  if (use_dynamic_object_) {
    getNearestObstacleByDynamicObject(min_dist_to_obj, nearest_obj_point);
  }
}

void SurroundObstacleCheckerNode::getNearestObstacleByPointCloud(
  double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point)
{
  // wait to transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp,
      tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 500 /* ms */,
      "failed to get base_link to " << pointcloud_ptr_->header.frame_id << " transform.");
    return;
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> pcl;
  pcl::fromROSMsg(*pointcloud_ptr_, pcl);
  pcl::transformPointCloud(pcl, pcl, isometry);
  for (const auto & p : pcl) {
    // create boost point
    Point2d boost_p(p.x, p.y);

    // calc distance
    const auto self_poly = createSelfPolygon(vehicle_info_);
    const double dist_to_obj = boost::geometry::distance(self_poly, boost_p);

    // get minimum distance to obj
    if (dist_to_obj < *min_dist_to_obj) {
      *min_dist_to_obj = dist_to_obj;
      nearest_obj_point->x = p.x;
      nearest_obj_point->y = p.y;
      nearest_obj_point->z = p.z;
    }
  }
}

void SurroundObstacleCheckerNode::getNearestObstacleByDynamicObject(
  double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point)
{
  const auto obj_frame = object_ptr_->header.frame_id;
  const auto obj_time = object_ptr_->header.stamp;
  for (const auto & obj : object_ptr_->objects) {
    // change frame of obj_pose to base_link
    geometry_msgs::msg::Pose pose_baselink;
    if (!convertPose(
          obj.kinematics.initial_pose_with_covariance.pose, obj_frame, "base_link", obj_time,
          pose_baselink)) {
      return;
    }

    // create obj polygon
    Polygon2d obj_poly;
    if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
      obj_poly = createObjPolygon(pose_baselink, obj.shape.footprint);
    } else {
      obj_poly = createObjPolygon(pose_baselink, obj.shape.dimensions);
    }

    // calc distance
    const auto self_poly = createSelfPolygon(vehicle_info_);
    const double dist_to_obj = boost::geometry::distance(self_poly, obj_poly);

    // get minimum distance to obj
    if (dist_to_obj < *min_dist_to_obj) {
      *min_dist_to_obj = dist_to_obj;
      *nearest_obj_point = obj.kinematics.initial_pose_with_covariance.pose.position;
    }
  }
}

bool SurroundObstacleCheckerNode::isObstacleFound(const double min_dist_to_obj)
{
  const auto is_obstacle_inside_range = min_dist_to_obj < surround_check_distance_;
  const auto is_obstacle_outside_range = min_dist_to_obj > surround_check_recover_distance_;

  if (state_ == State::PASS) {
    return is_obstacle_inside_range;
  }

  if (state_ == State::STOP) {
    return !is_obstacle_outside_range;
  }

  throw std::runtime_error("invalid state");
}

bool SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_stopped)
{
  if (!is_stopped) {
    return false;
  }

  if (is_obstacle_found) {
    last_obstacle_found_time_ = std::make_shared<const rclcpp::Time>(this->now());
    return true;
  }

  if (state_ != State::STOP) {
    return false;
  }

  // Keep stop state
  if (last_obstacle_found_time_) {
    const auto elapsed_time = this->now() - *last_obstacle_found_time_;
    if (elapsed_time.seconds() <= state_clear_time_) {
      return true;
    }
  }

  last_obstacle_found_time_ = {};
  return false;
}

bool SurroundObstacleCheckerNode::isVehicleStopped()
{
  const auto current_velocity = std::abs(odometry_ptr_->twist.twist.linear.x);

  if (stop_state_ego_speed_ < current_velocity) {
    last_running_time_ = std::make_shared<rclcpp::Time>(this->now());
  }

  return stopped_state_entry_duration_time_ < (this->now() - *last_running_time_).seconds();
}

}  // namespace surround_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(surround_obstacle_checker::SurroundObstacleCheckerNode)
