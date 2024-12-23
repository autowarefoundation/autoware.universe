// Copyright 2024 Tier IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee

#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace shapes
{
inline boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return boost::none;
  }
}

bool transformObjects(
  const types::DynamicObjectList & input_msg, const std::string & target_frame_id,
  const tf2_ros::Buffer & tf_buffer, types::DynamicObjectList & output_msg)
{
  output_msg = input_msg;

  // transform to world coordinate
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      auto & pose_with_cov = object.kinematics.pose_with_covariance;
      tf2::fromMsg(pose_with_cov.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      // transform pose, frame difference and object pose
      tf2::toMsg(tf_target2objects, pose_with_cov.pose);
      // transform covariance, only the frame difference
      pose_with_cov.covariance =
        tf2::transformCovariance(pose_with_cov.covariance, tf_target2objects_world);
    }
  }
  return true;
}

inline double getSumArea(const std::vector<autoware::universe_utils::Polygon2d> & polygons)
{
  return std::accumulate(
    polygons.begin(), polygons.end(), 0.0, [](double acc, autoware::universe_utils::Polygon2d p) {
      return acc + boost::geometry::area(p);
    });
}

inline double getIntersectionArea(
  const autoware::universe_utils::Polygon2d & source_polygon,
  const autoware::universe_utils::Polygon2d & target_polygon)
{
  std::vector<autoware::universe_utils::Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);
  return getSumArea(intersection_polygons);
}

inline double getUnionArea(
  const autoware::universe_utils::Polygon2d & source_polygon,
  const autoware::universe_utils::Polygon2d & target_polygon)
{
  std::vector<autoware::universe_utils::Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);
  return getSumArea(union_polygons);
}

double get2dIoU(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const double min_union_area)
{
  static const double MIN_AREA = 1e-6;

  const auto source_polygon = autoware::universe_utils::toPolygon2d(
    source_object.kinematics.pose_with_covariance.pose, source_object.shape);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return 0.0;
  const auto target_polygon = autoware::universe_utils::toPolygon2d(
    target_object.kinematics.pose_with_covariance.pose, target_object.shape);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  if (intersection_area < MIN_AREA) return 0.0;
  const double union_area = getUnionArea(source_polygon, target_polygon);

  const double iou =
    union_area < min_union_area ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
}

/**
 * @brief convert convex hull shape object to bounding box object
 * @param input_object: input convex hull objects
 * @param output_object: output bounding box objects
 */
bool convertConvexHullToBoundingBox(
  const types::DynamicObject & input_object, types::DynamicObject & output_object)
{
  // check footprint size
  if (input_object.shape.footprint.points.size() < 3) {
    return false;
  }

  // look for bounding box boundary
  float max_x = 0;
  float max_y = 0;
  float min_x = 0;
  float min_y = 0;
  float max_z = 0;
  for (const auto & point : input_object.shape.footprint.points) {
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  // calc new center
  const Eigen::Vector2d center{
    input_object.kinematics.pose_with_covariance.pose.position.x,
    input_object.kinematics.pose_with_covariance.pose.position.y};
  const auto yaw = tf2::getYaw(input_object.kinematics.pose_with_covariance.pose.orientation);
  const Eigen::Matrix2d R_inv = Eigen::Rotation2Dd(-yaw).toRotationMatrix();
  const Eigen::Vector2d new_local_center{(max_x + min_x) / 2.0, (max_y + min_y) / 2.0};
  const Eigen::Vector2d new_center = center + R_inv.transpose() * new_local_center;

  // set output parameters
  output_object = input_object;
  output_object.kinematics.pose_with_covariance.pose.position.x = new_center.x();
  output_object.kinematics.pose_with_covariance.pose.position.y = new_center.y();

  output_object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output_object.shape.dimensions.x = max_x - min_x;
  output_object.shape.dimensions.y = max_y - min_y;
  output_object.shape.dimensions.z = max_z;

  return true;
}

bool getMeasurementYaw(
  const types::DynamicObject & object, const double & predicted_yaw, double & measurement_yaw)
{
  measurement_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);

  // check orientation sign is known or not, and fix the limiting delta yaw
  double limiting_delta_yaw = M_PI_2;
  if (object.kinematics.orientation_availability == types::OrientationAvailability::AVAILABLE) {
    limiting_delta_yaw = M_PI;
  }
  // limiting delta yaw, even the availability is unknown
  while (std::fabs(predicted_yaw - measurement_yaw) > limiting_delta_yaw) {
    if (measurement_yaw < predicted_yaw) {
      measurement_yaw += 2 * limiting_delta_yaw;
    } else {
      measurement_yaw -= 2 * limiting_delta_yaw;
    }
  }
  // return false if the orientation is unknown
  return object.kinematics.orientation_availability != types::OrientationAvailability::UNAVAILABLE;
}

}  // namespace shapes

}  // namespace autoware::multi_object_tracker
