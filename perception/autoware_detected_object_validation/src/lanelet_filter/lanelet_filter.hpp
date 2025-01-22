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

#ifndef LANELET_FILTER__LANELET_FILTER_HPP_
#define LANELET_FILTER__LANELET_FILTER_HPP_

#include "autoware/detected_object_validation/utils/utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/ros/published_time_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware_lanelet2_extension/utility/utilities.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::detected_object_validation
{
namespace lanelet_filter
{
using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::MultiPoint2d;
using autoware::universe_utils::Polygon2d;

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using Point2d = bg::model::point<double, 2, bg::cs::cartesian>;
using Box = boost::geometry::model::box<Point2d>;

struct PolygonAndLanelet
{
  lanelet::BasicPolygon2d polygon;
  lanelet::ConstLanelet lanelet;
};
using BoxAndLanelet = std::pair<Box, PolygonAndLanelet>;
using RtreeAlgo = bgi::rstar<16>;

class ObjectLaneletFilterNode : public rclcpp::Node
{
public:
  explicit ObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr);
  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr);

  void publishDebugMarkers(
    rclcpp::Time stamp, const LinearRing2d & hull, const std::vector<BoxAndLanelet> & lanelets);

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;

  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_{nullptr};
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::string lanelet_frame_id_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  utils::FilterTargetLabel filter_target_;
  struct FilterSettings
  {
    bool polygon_overlap_filter;
    bool lanelet_direction_filter;
    double lanelet_direction_filter_velocity_yaw_threshold;
    double lanelet_direction_filter_object_speed_threshold;
    bool debug;
    double lanelet_extra_margin;
  } filter_settings_;

  bool filterObject(
    const autoware_perception_msgs::msg::DetectedObject & transformed_object,
    const autoware_perception_msgs::msg::DetectedObject & input_object,
    const bg::index::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree,
    autoware_perception_msgs::msg::DetectedObjects & output_object_msg);
  LinearRing2d getConvexHull(const autoware_perception_msgs::msg::DetectedObjects &);
  LinearRing2d getConvexHullFromObjectFootprint(
    const autoware_perception_msgs::msg::DetectedObject & object);
  std::vector<BoxAndLanelet> getIntersectedLanelets(const LinearRing2d &);
  bool isObjectOverlapLanelets(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const bg::index::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree);
  bool isPolygonOverlapLanelets(
    const Polygon2d & polygon, const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree);
  bool isSameDirectionWithLanelets(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const bgi::rtree<BoxAndLanelet, RtreeAlgo> & local_rtree);
  geometry_msgs::msg::Polygon setFootprint(const autoware_perception_msgs::msg::DetectedObject &);

  lanelet::BasicPolygon2d getPolygon(const lanelet::ConstLanelet & lanelet);
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;
};

}  // namespace lanelet_filter
}  // namespace autoware::detected_object_validation

#endif  // LANELET_FILTER__LANELET_FILTER_HPP_
