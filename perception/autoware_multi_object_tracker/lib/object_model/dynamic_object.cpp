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

#include "autoware/multi_object_tracker/object_model/dynamic_object.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>

#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{

namespace types
{

DynamicObject getDynamicObject(const autoware_perception_msgs::msg::DetectedObject & det_object)
{
  DynamicObject dynamic_object;
  dynamic_object.existence_probability = det_object.existence_probability;
  dynamic_object.classification = det_object.classification;

  dynamic_object.kinematics.pose_with_covariance = det_object.kinematics.pose_with_covariance;
  dynamic_object.kinematics.twist_with_covariance = det_object.kinematics.twist_with_covariance;
  dynamic_object.kinematics.has_position_covariance = det_object.kinematics.has_position_covariance;
  if (
    det_object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::UNAVAILABLE;
  } else if (
    det_object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::SIGN_UNKNOWN;
  } else if (
    det_object.kinematics.orientation_availability ==
    autoware_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE) {
    dynamic_object.kinematics.orientation_availability = OrientationAvailability::AVAILABLE;
  }
  dynamic_object.kinematics.has_twist = det_object.kinematics.has_twist;
  dynamic_object.kinematics.has_twist_covariance = det_object.kinematics.has_twist_covariance;

  dynamic_object.shape = det_object.shape;

  return dynamic_object;
}

DynamicObjects getDynamicObjects(const autoware_perception_msgs::msg::DetectedObjects & det_objects)
{
  DynamicObjects dynamic_objects;
  dynamic_objects.header = det_objects.header;
  dynamic_objects.objects.reserve(det_objects.objects.size());
  for (const auto & det_object : det_objects.objects) {
    dynamic_objects.objects.emplace_back(getDynamicObject(det_object));
  }
  return dynamic_objects;
}

autoware_perception_msgs::msg::TrackedObject toTrackedObjectMsg(const DynamicObject & dyn_object)
{
  autoware_perception_msgs::msg::TrackedObject tracked_object;
  tracked_object.object_id = dyn_object.object_id;
  tracked_object.existence_probability = dyn_object.existence_probability;
  tracked_object.classification = dyn_object.classification;

  tracked_object.kinematics.pose_with_covariance = dyn_object.kinematics.pose_with_covariance;
  tracked_object.kinematics.twist_with_covariance = dyn_object.kinematics.twist_with_covariance;
  if (dyn_object.kinematics.orientation_availability == OrientationAvailability::UNAVAILABLE) {
    tracked_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
  } else if (
    dyn_object.kinematics.orientation_availability == OrientationAvailability::SIGN_UNKNOWN) {
    tracked_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
  } else if (dyn_object.kinematics.orientation_availability == OrientationAvailability::AVAILABLE) {
    tracked_object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;
  }
  tracked_object.kinematics.is_stationary = false;

  tracked_object.shape = dyn_object.shape;

  return tracked_object;
}
}  // namespace types

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
  const types::DynamicObjects & input_msg, const std::string & target_frame_id,
  const tf2_ros::Buffer & tf_buffer, types::DynamicObjects & output_msg)
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
}  // namespace shapes
}  // namespace autoware::multi_object_tracker
