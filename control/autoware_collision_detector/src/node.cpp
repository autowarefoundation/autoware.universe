// Copyright 2024 TIER IV, Inc.
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

#include "autoware/collision_detector/node.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::collision_detector
{
namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using autoware::universe_utils::createPoint;
using autoware::universe_utils::pose2transform;

namespace
{

geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint)
{
  geometry_msgs::msg::Polygon transformed_polygon{};
  geometry_msgs::msg::TransformStamped geometry_tf{};
  geometry_tf.transform = pose2transform(pose);
  tf2::doTransform(footprint, transformed_polygon, geometry_tf);

  Polygon2d object_polygon;
  for (const auto & p : transformed_polygon.points) {
    object_polygon.outer().push_back(Point2d(p.x, p.y));
  }

  bg::correct(object_polygon);

  return object_polygon;
}

Polygon2d createObjPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size)
{
  const double & length_m = size.x / 2.0;
  const double & width_m = size.y / 2.0;

  geometry_msgs::msg::Polygon polygon{};

  polygon.points.push_back(createPoint32(length_m, -width_m, 0.0));
  polygon.points.push_back(createPoint32(length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, width_m, 0.0));
  polygon.points.push_back(createPoint32(-length_m, -width_m, 0.0));

  return createObjPolygon(pose, polygon);
}

Polygon2d createObjPolygonForCylinder(const geometry_msgs::msg::Pose & pose, const double diameter)
{
  geometry_msgs::msg::Polygon polygon{};

  const double radius = diameter * 0.5;
  // add hexagon points
  for (int i = 0; i < 6; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / 6.0;
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    polygon.points.push_back(createPoint32(x, y, 0.0));
  }

  return createObjPolygon(pose, polygon);
}

Polygon2d createSelfPolygon(const VehicleInfo & vehicle_info)
{
  const double & front_m = vehicle_info.max_longitudinal_offset_m;
  const double & width_left_m = vehicle_info.max_lateral_offset_m;
  const double & width_right_m = vehicle_info.min_lateral_offset_m;
  const double & rear_m = vehicle_info.min_longitudinal_offset_m;

  Polygon2d ego_polygon;

  ego_polygon.outer().push_back(Point2d(front_m, width_left_m));
  ego_polygon.outer().push_back(Point2d(front_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_right_m));
  ego_polygon.outer().push_back(Point2d(rear_m, width_left_m));

  bg::correct(ego_polygon);

  return ego_polygon;
}
}  // namespace

CollisionDetectorNode::CollisionDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("collision_detector_node", node_options),
  node_param_(),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  updater_(this)
{
  // Parameters
  {
    auto & p = node_param_;
    p.use_pointcloud = this->declare_parameter<bool>("use_pointcloud");
    p.use_dynamic_object = this->declare_parameter<bool>("use_dynamic_object");
    p.collision_distance = this->declare_parameter<double>("collision_distance");
  }

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  // Subscribers
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&CollisionDetectorNode::onPointCloud, this, std::placeholders::_1));
  sub_dynamic_objects_ = this->create_subscription<PredictedObjects>(
    "~/input/objects", 1,
    std::bind(&CollisionDetectorNode::onDynamicObjects, this, std::placeholders::_1));

  sub_operation_mode_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&CollisionDetectorNode::onOperationMode, this, std::placeholders::_1));

  last_obstacle_found_stamp_ = this->now();

  // Diagnostics Updater
  updater_.setHardwareID("collision_detector");
  updater_.add("collision_check", this, &CollisionDetectorNode::checkCollision);
  updater_.setPeriod(0.1);
}

void CollisionDetectorNode::checkCollision(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (node_param_.use_pointcloud && !pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
    return;
  }

  if (node_param_.use_dynamic_object && !object_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
    return;
  }

  if (!operation_mode_ptr_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for operation mode info...");
    return;
  }

  const auto nearest_obstacle = getNearestObstacle();

  const auto is_obstacle_found =
    !nearest_obstacle ? false : nearest_obstacle.get().first < node_param_.collision_distance;

  if (is_obstacle_found) {
    last_obstacle_found_stamp_ = this->now();
  }

  const auto is_obstacle_found_recently =
    (this->now() - last_obstacle_found_stamp_).seconds() < 1.0;

  diagnostic_msgs::msg::DiagnosticStatus status;
  if (
    operation_mode_ptr_->mode == OperationModeState::AUTONOMOUS &&
    (is_obstacle_found || is_obstacle_found_recently)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "collision detected";
    if (is_obstacle_found) {
      stat.addf("Distance to nearest neighbor object", "%lf", nearest_obstacle.get().first);
    }
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

void CollisionDetectorNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pointcloud_ptr_ = msg;
}

void CollisionDetectorNode::onDynamicObjects(const PredictedObjects::ConstSharedPtr msg)
{
  object_ptr_ = msg;
}

void CollisionDetectorNode::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_ptr_ = msg;
}

boost::optional<Obstacle> CollisionDetectorNode::getNearestObstacle() const
{
  boost::optional<Obstacle> nearest_pointcloud{boost::none};
  boost::optional<Obstacle> nearest_object{boost::none};

  if (node_param_.use_pointcloud) {
    nearest_pointcloud = getNearestObstacleByPointCloud();
  }

  if (node_param_.use_dynamic_object) {
    nearest_object = getNearestObstacleByDynamicObject();
  }

  if (!nearest_pointcloud && !nearest_object) {
    return {};
  }

  if (!nearest_pointcloud) {
    return nearest_object;
  }

  if (!nearest_object) {
    return nearest_pointcloud;
  }

  return nearest_pointcloud.get().first < nearest_object.get().first ? nearest_pointcloud
                                                                     : nearest_object;
}

boost::optional<Obstacle> CollisionDetectorNode::getNearestObstacleByPointCloud() const
{
  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.get().transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*pointcloud_ptr_, transformed_pointcloud);
  pcl::transformPointCloud(transformed_pointcloud, transformed_pointcloud, isometry);

  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  for (const auto & p : transformed_pointcloud) {
    Point2d boost_point(p.x, p.y);

    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point = createPoint(p.x, p.y, p.z);
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

boost::optional<Obstacle> CollisionDetectorNode::getNearestObstacleByDynamicObject() const
{
  const auto transform_stamped =
    getTransform(object_ptr_->header.frame_id, "base_link", object_ptr_->header.stamp, 0.5);

  geometry_msgs::msg::Point nearest_point;
  auto minimum_distance = std::numeric_limits<double>::max();

  if (!transform_stamped) {
    return {};
  }

  tf2::Transform tf_src2target;
  tf2::fromMsg(transform_stamped.get().transform, tf_src2target);

  const auto ego_polygon = createSelfPolygon(vehicle_info_);

  for (const auto & object : object_ptr_->objects) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;

    tf2::Transform tf_src2object;
    tf2::fromMsg(object_pose, tf_src2object);

    geometry_msgs::msg::Pose transformed_object_pose;
    tf2::toMsg(tf_src2target.inverse() * tf_src2object, transformed_object_pose);

    const auto object_polygon = [&]() {
      switch (object.shape.type) {
        case Shape::POLYGON:
          return createObjPolygon(transformed_object_pose, object.shape.footprint);
        case Shape::CYLINDER:
          return createObjPolygonForCylinder(transformed_object_pose, object.shape.dimensions.x);
        case Shape::BOUNDING_BOX:
          return createObjPolygon(transformed_object_pose, object.shape.dimensions);
        default:
          // node return warning
          RCLCPP_WARN(this->get_logger(), "Unsupported shape type: %d", object.shape.type);
          return createObjPolygon(transformed_object_pose, object.shape.dimensions);
      }
    }();

    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_point = object_pose.position;
      minimum_distance = distance_to_object;
    }
  }

  return std::make_pair(minimum_distance, nearest_point);
}

boost::optional<geometry_msgs::msg::TransformStamped> CollisionDetectorNode::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

}  // namespace autoware::collision_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::collision_detector::CollisionDetectorNode)
