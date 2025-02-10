// Copyright 2022 TIER IV, Inc.
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

#include "lanelet_filter.hpp"

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"
#include "autoware_lanelet2_extension/utility/query.hpp"

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
ObjectLaneletFilterNode::ObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_lanelet_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN");
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR");
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK");
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS");
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER");
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE");
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE");
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN");
  // Set filter settings
  filter_settings_.polygon_overlap_filter =
    declare_parameter<bool>("filter_settings.polygon_overlap_filter.enabled");
  filter_settings_.lanelet_direction_filter =
    declare_parameter<bool>("filter_settings.lanelet_direction_filter.enabled");
  filter_settings_.lanelet_direction_filter_velocity_yaw_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.velocity_yaw_threshold");
  filter_settings_.lanelet_direction_filter_object_speed_threshold =
    declare_parameter<double>("filter_settings.lanelet_direction_filter.object_speed_threshold");
  filter_settings_.debug = declare_parameter<bool>("filter_settings.debug");
  filter_settings_.lanelet_extra_margin =
    declare_parameter<double>("filter_settings.lanelet_extra_margin");
  filter_settings_.use_height_threshold =
    declare_parameter<bool>("filter_settings.use_height_threshold");
  filter_settings_.max_height_threshold =
    declare_parameter<double>("filter_settings.max_height_threshold");
  filter_settings_.min_height_threshold =
    declare_parameter<double>("filter_settings.min_height_threshold");

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ObjectLaneletFilterNode::mapCallback, this, _1));
  object_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectLaneletFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  debug_publisher_ =
    std::make_unique<autoware::universe_utils::DebugPublisher>(this, "object_lanelet_filter");
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
  stop_watch_ptr_ =
    std::make_unique<autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
  if (filter_settings_.debug) {
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/marker", rclcpp::QoS{1});
  }
}

bool isInPolygon(
  const geometry_msgs::msg::Pose & current_pose, const lanelet::BasicPolygon2d & polygon,
  const double radius)
{
  constexpr double eps = 1.0e-9;
  const lanelet::BasicPoint2d p(current_pose.position.x, current_pose.position.y);
  return boost::geometry::distance(p, polygon) < radius + eps;
}

LinearRing2d expandPolygon(const LinearRing2d & polygon, double distance)
{
  universe_utils::MultiPolygon2d multi_polygon;
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_circle circle_strategy;
  bg::strategy::buffer::side_straight side_strategy;

  bg::buffer(
    polygon, multi_polygon, distance_strategy, side_strategy, join_strategy, end_strategy,
    circle_strategy);

  if (multi_polygon.empty()) {
    return polygon;
  }

  return multi_polygon.front().outer();
}

void ObjectLaneletFilterNode::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_msg)
{
  lanelet_frame_id_ = map_msg->header.frame_id;
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
}

void ObjectLaneletFilterNode::objectCallback(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->tic("processing_time");

  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  if (!lanelet_map_ptr_) {
    RCLCPP_ERROR(get_logger(), "No vector map received.");
    return;
  }
  autoware_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!autoware::object_recognition_utils::transformObjects(
        *input_msg, lanelet_frame_id_, tf_buffer_, transformed_objects)) {
    RCLCPP_ERROR(get_logger(), "Failed transform to %s.", lanelet_frame_id_.c_str());
    return;
  }
  // vehicle base pose :map -> base_link
  if (filter_settings_.use_height_threshold) {
    try {
      ego_base_height_ = tf_buffer_
                           .lookupTransform(
                             lanelet_frame_id_, "base_link", transformed_objects.header.stamp,
                             rclcpp::Duration::from_seconds(0.5))
                           .transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to get transform: " << ex.what());
      return;
    }
  }

  if (!transformed_objects.objects.empty()) {
    // calculate convex hull
    const auto convex_hull = getConvexHull(transformed_objects);

    // get intersected lanelets
    std::vector<BoxAndLanelet> intersected_lanelets_with_bbox = getIntersectedLanelets(convex_hull);

    // create R-Tree with intersected_lanelets for fast search
    bgi::rtree<BoxAndLanelet, RtreeAlgo> local_rtree;
    for (const auto & bbox_and_lanelet : intersected_lanelets_with_bbox) {
      local_rtree.insert(bbox_and_lanelet);
    }

    if (filter_settings_.debug) {
      publishDebugMarkers(input_msg->header.stamp, convex_hull, intersected_lanelets_with_bbox);
    }
    // filtering process
    for (size_t index = 0; index < transformed_objects.objects.size(); ++index) {
      const auto & transformed_object = transformed_objects.objects.at(index);
      const auto & input_object = input_msg->objects.at(index);
      filterObject(transformed_object, input_object, local_rtree, output_object_msg);
    }
  }

  object_pub_->publish(output_object_msg);
  published_time_publisher_->publish_if_subscribed(object_pub_, output_object_msg.header.stamp);

  // Publish debug info
  const double pipeline_latency =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (this->get_clock()->now() - output_object_msg.header.stamp).nanoseconds()))
      .count();
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/pipeline_latency_ms", pipeline_latency);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

bool ObjectLaneletFilterNode::filterObject(
  const autoware_perception_msgs::msg::DetectedObject & transformed_object,
  const autoware_perception_msgs::msg::DetectedObject & input_object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree,
  autoware_perception_msgs::msg::DetectedObjects & output_object_msg)
{
  const auto & label = transformed_object.classification.front().label;
  if (filter_target_.isTarget(label)) {
    // no tree, then no intersection
    if (local_rtree.empty()) {
      return false;
    }

    // 0. check height threshold
    if (filter_settings_.use_height_threshold) {
      const double object_height = transformed_object.shape.dimensions.z;
      if (
        object_height > ego_base_height_ + filter_settings_.max_height_threshold ||
        object_height < ego_base_height_ + filter_settings_.min_height_threshold) {
        return false;
      }
    }

    bool filter_pass = true;
    // 1. is polygon overlap with road lanelets or shoulder lanelets
    if (filter_settings_.polygon_overlap_filter) {
      const bool is_polygon_overlap = isObjectOverlapLanelets(transformed_object, local_rtree);
      filter_pass = filter_pass && is_polygon_overlap;
    }

    // 2. check if objects velocity is the same with the lanelet direction
    const bool orientation_not_available =
      transformed_object.kinematics.orientation_availability ==
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    if (filter_settings_.lanelet_direction_filter && !orientation_not_available) {
      const bool is_same_direction = isSameDirectionWithLanelets(transformed_object, local_rtree);
      filter_pass = filter_pass && is_same_direction;
    }

    // push back to output object
    if (filter_pass) {
      output_object_msg.objects.emplace_back(input_object);
      return true;
    }
  } else {
    output_object_msg.objects.emplace_back(input_object);
    return true;
  }
  return false;
}

geometry_msgs::msg::Polygon ObjectLaneletFilterNode::setFootprint(
  const autoware_perception_msgs::msg::DetectedObject & detected_object)
{
  geometry_msgs::msg::Polygon footprint;
  if (detected_object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const auto object_size = detected_object.shape.dimensions;
    const double x_front = object_size.x / 2.0;
    const double x_rear = -object_size.x / 2.0;
    const double y_left = object_size.y / 2.0;
    const double y_right = -object_size.y / 2.0;

    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_left).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_front).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_right).z(0.0));
    footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(x_rear).y(y_left).z(0.0));
  } else {
    footprint = detected_object.shape.footprint;
  }
  return footprint;
}

LinearRing2d ObjectLaneletFilterNode::getConvexHull(
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects)
{
  MultiPoint2d candidate_points;
  for (const auto & object : detected_objects.objects) {
    const auto & pos = object.kinematics.pose_with_covariance.pose.position;
    const auto footprint = setFootprint(object);
    for (const auto & p : footprint.points) {
      candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
    }
  }
  LinearRing2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  if (filter_settings_.lanelet_extra_margin > 0) {
    return expandPolygon(convex_hull, filter_settings_.lanelet_extra_margin);
  }
  return convex_hull;
}

LinearRing2d ObjectLaneletFilterNode::getConvexHullFromObjectFootprint(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  MultiPoint2d candidate_points;
  const auto & pos = object.kinematics.pose_with_covariance.pose.position;
  const auto footprint = setFootprint(object);

  for (const auto & p : footprint.points) {
    candidate_points.emplace_back(p.x + pos.x, p.y + pos.y);
  }

  LinearRing2d convex_hull;
  bg::convex_hull(candidate_points, convex_hull);

  return convex_hull;
}

// fetch the intersected candidate lanelets with bounding box and then
// check the intersections among the lanelets and the convex hull
std::vector<BoxAndLanelet> ObjectLaneletFilterNode::getIntersectedLanelets(
  const LinearRing2d & convex_hull)
{
  std::vector<BoxAndLanelet> intersected_lanelets_with_bbox;

  // convert convex_hull to a 2D bounding box for searching in the LaneletMap
  bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
  bg::envelope(convex_hull, bbox_of_convex_hull);
  const lanelet::BoundingBox2d bbox2d(
    lanelet::BasicPoint2d(
      bg::get<bg::min_corner, 0>(bbox_of_convex_hull),
      bg::get<bg::min_corner, 1>(bbox_of_convex_hull)),
    lanelet::BasicPoint2d(
      bg::get<bg::max_corner, 0>(bbox_of_convex_hull),
      bg::get<bg::max_corner, 1>(bbox_of_convex_hull)));

  const lanelet::Lanelets candidate_lanelets = lanelet_map_ptr_->laneletLayer.search(bbox2d);
  for (const auto & lanelet : candidate_lanelets) {
    // only check the road lanelets and road shoulder lanelets
    if (
      lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
      (lanelet.attribute(lanelet::AttributeName::Subtype).value() ==
         lanelet::AttributeValueString::Road ||
       lanelet.attribute(lanelet::AttributeName::Subtype).value() == "road_shoulder")) {
      if (bg::intersects(convex_hull, lanelet.polygon2d().basicPolygon())) {
        // create bbox using boost for making the R-tree in later phase
        auto polygon = getPolygon(lanelet);
        Box boost_bbox;
        bg::envelope(polygon, boost_bbox);

        intersected_lanelets_with_bbox.emplace_back(
          std::make_pair(boost_bbox, PolygonAndLanelet{polygon, lanelet}));
      }
    }
  }

  return intersected_lanelets_with_bbox;
}

lanelet::BasicPolygon2d ObjectLaneletFilterNode::getPolygon(const lanelet::ConstLanelet & lanelet)
{
  if (filter_settings_.lanelet_extra_margin <= 0) {
    return lanelet.polygon2d().basicPolygon();
  }

  auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
  Polygon2d polygon;
  bg::assign_points(polygon, lanelet_polygon);

  bg::correct(polygon);
  auto polygon_result = expandPolygon(polygon.outer(), filter_settings_.lanelet_extra_margin);
  lanelet::BasicPolygon2d result;

  bg::assign_points(result, polygon_result);

  return result;
}

bool ObjectLaneletFilterNode::isObjectOverlapLanelets(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  // if object has bounding box, use polygon overlap
  if (utils::hasBoundingBox(object)) {
    Polygon2d polygon;
    const auto footprint = setFootprint(object);
    for (const auto & point : footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware::universe_utils::transformPoint(
          point, object.kinematics.pose_with_covariance.pose);
      polygon.outer().emplace_back(point_transformed.x, point_transformed.y);
    }
    polygon.outer().push_back(polygon.outer().front());

    return isPolygonOverlapLanelets(polygon, local_rtree);
  } else {
    const LinearRing2d object_convex_hull = getConvexHullFromObjectFootprint(object);

    // create bounding box to search in the rtree
    std::vector<BoxAndLanelet> candidates;
    bg::model::box<bg::model::d2::point_xy<double>> bbox;
    bg::envelope(object_convex_hull, bbox);
    local_rtree.query(bgi::intersects(bbox), std::back_inserter(candidates));

    // if object do not have bounding box, check each footprint is inside polygon
    for (const auto & point : object.shape.footprint.points) {
      const geometry_msgs::msg::Point32 point_transformed =
        autoware::universe_utils::transformPoint(
          point, object.kinematics.pose_with_covariance.pose);
      geometry_msgs::msg::Pose point2d;
      point2d.position.x = point_transformed.x;
      point2d.position.y = point_transformed.y;

      for (const auto & candidate : candidates) {
        if (isInPolygon(point2d, candidate.second.polygon, 0.0)) {
          return true;
        }
      }
    }

    return false;
  }
}

bool ObjectLaneletFilterNode::isPolygonOverlapLanelets(
  const Polygon2d & polygon, const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  // create a bounding box from polygon for searching the local R-tree
  std::vector<BoxAndLanelet> candidates;
  bg::model::box<bg::model::d2::point_xy<double>> bbox_of_convex_hull;
  bg::envelope(polygon, bbox_of_convex_hull);
  local_rtree.query(bgi::intersects(bbox_of_convex_hull), std::back_inserter(candidates));

  for (const auto & box_and_lanelet : candidates) {
    if (!bg::disjoint(polygon, box_and_lanelet.second.polygon)) {
      return true;
    }
  }

  return false;
}

bool ObjectLaneletFilterNode::isSameDirectionWithLanelets(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree)
{
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double object_velocity_norm = std::hypot(
    object.kinematics.twist_with_covariance.twist.linear.x,
    object.kinematics.twist_with_covariance.twist.linear.y);
  const double object_velocity_yaw = std::atan2(
                                       object.kinematics.twist_with_covariance.twist.linear.y,
                                       object.kinematics.twist_with_covariance.twist.linear.x) +
                                     object_yaw;

  if (object_velocity_norm < filter_settings_.lanelet_direction_filter_object_speed_threshold) {
    return true;
  }

  // we can not query by points, so create a small bounding box
  // eps is not determined by any specific criteria; just a guess
  constexpr double eps = 1.0e-6;
  std::vector<BoxAndLanelet> candidates;
  const Point2d min_corner(
    object.kinematics.pose_with_covariance.pose.position.x - eps,
    object.kinematics.pose_with_covariance.pose.position.y - eps);
  const Point2d max_corner(
    object.kinematics.pose_with_covariance.pose.position.x + eps,
    object.kinematics.pose_with_covariance.pose.position.y + eps);
  const Box bbox(min_corner, max_corner);

  local_rtree.query(bgi::intersects(bbox), std::back_inserter(candidates));

  for (const auto & box_and_lanelet : candidates) {
    const bool is_in_lanelet =
      isInPolygon(object.kinematics.pose_with_covariance.pose, box_and_lanelet.second.polygon, 0.0);
    if (!is_in_lanelet) {
      continue;
    }

    const double lane_yaw = lanelet::utils::getLaneletAngle(
      box_and_lanelet.second.lanelet, object.kinematics.pose_with_covariance.pose.position);
    const double delta_yaw = object_velocity_yaw - lane_yaw;
    const double normalized_delta_yaw = autoware::universe_utils::normalizeRadian(delta_yaw);
    const double abs_norm_delta_yaw = std::fabs(normalized_delta_yaw);

    if (abs_norm_delta_yaw < filter_settings_.lanelet_direction_filter_velocity_yaw_threshold) {
      return true;
    }
  }

  return false;
}

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_validation::lanelet_filter::ObjectLaneletFilterNode)
