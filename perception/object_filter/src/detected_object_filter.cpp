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

#include "detected_object_filter/detected_object_filter.hpp"

#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <memory>

boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("detected_object_filter"), ex.what());
    return boost::none;
  }
}
namespace detected_object_filter
{
DetectedObjectFilterNode::DetectedObjectFilterNode(const rclcpp::NodeOptions & node_options)
: Node("detected_object_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  upper_bound_x_ = declare_parameter<float>("upper_bound_x", 100.0);
  upper_bound_y_ = declare_parameter<float>("upper_bound_y", 50.0);
  lower_bound_x_ = declare_parameter<float>("lower_bound_x", 0.0);
  lower_bound_y_ = declare_parameter<float>("lower_bound_y", -50.0);
  filter_by_xy_position_ = declare_parameter<bool>("filter_by_xy_position", false);

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DetectedObjectFilterNode::mapCallback, this, _1));
  object_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&DetectedObjectFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});
}

void DetectedObjectFilterNode::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

void DetectedObjectFilterNode::objectCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_auto_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  if (filter_by_xy_position_) {
    for (const auto & object : input_msg->objects) {
      const auto & position = object.kinematics.pose_with_covariance.pose.position;
      if (
        position.x > lower_bound_x_ && position.x < upper_bound_x_ && position.y > lower_bound_y_ &&
        position.y < upper_bound_y_) {
        output_object_msg.objects.emplace_back(object);
      }
    }
  } else {
    if (!lanelet_map_ptr_) {
      RCLCPP_ERROR(get_logger(), "No vector map received.");
      return;
    }
    autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
    if (!transformDetectedObjects(*input_msg, "map", tf_buffer_, transformed_objects)) {
      RCLCPP_ERROR(get_logger(), "Failed transform to map.");
      return;
    }

    // calculate convex hull
    const auto convex_hull = getConvexHull(transformed_objects);
    // get intersected lanelets
    lanelet::ConstLanelets intersected_lanelets =
      getIntersectedLanelets(convex_hull, road_lanelets_);

    int index = 0;
    for (const auto & object : transformed_objects.objects) {
      const auto position = object.kinematics.pose_with_covariance.pose.position;
      if (isPointWithinLanelets(Point2d(position.x, position.y), intersected_lanelets)) {
        output_object_msg.objects.emplace_back(input_msg->objects.at(index));
      }
      ++index;
    }
  }
  object_pub_->publish(output_object_msg);
}

bool DetectedObjectFilterNode::transformDetectedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::DetectedObjects & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
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
      tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
    }
  }
  return true;
}

LinearRing2d DetectedObjectFilterNode::getConvexHull(
  const autoware_auto_perception_msgs::msg::DetectedObjects & detected_objects)
{
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & p = object.kinematics.pose_with_covariance.pose.position;
    candidate_points.emplace_back(p.x, p.y);
  }

  LinearRing2d convex_hull;
  boost::geometry::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

lanelet::ConstLanelets DetectedObjectFilterNode::getIntersectedLanelets(
  const LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets)
{
  lanelet::ConstLanelets intersected_lanelets;
  for (const auto & road_lanelet : road_lanelets) {
    if (boost::geometry::intersects(convex_hull, road_lanelet.polygon2d().basicPolygon())) {
      intersected_lanelets.emplace_back(road_lanelet);
    }
  }
  return intersected_lanelets;
}

bool DetectedObjectFilterNode::isPointWithinLanelets(
  const Point2d & point, const lanelet::ConstLanelets & intersected_lanelets)
{
  for (const auto & lanelet : intersected_lanelets) {
    if (boost::geometry::within(point, lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }
  return false;
}

}  // namespace detected_object_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(detected_object_filter::DetectedObjectFilterNode)
