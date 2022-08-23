// Copyright 2022 Tier IV, Inc.
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

#include "pointcloud_preprocessor/no_detection_area_filter/no_detection_area_filter.hpp"

#include "pointcloud_preprocessor/filter.hpp"

#include <pcl_ros/transforms.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
bool pointWithinLanelets(const Point2d & point, const lanelet::ConstPolygons3d & lanelets)
{
  for (const auto & lanelet : lanelets) {
    if (boost::geometry::within(point, lanelet::utils::to2D(lanelet).basicPolygon())) {
      return true;
    }
  }

  return false;
}

tier4_autoware_utils::Box2d calcBoundingBox(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud)
{
  MultiPoint2d candidate_points;
  for (const auto & p : input_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  return boost::geometry::return_envelope<tier4_autoware_utils::Box2d>(candidate_points);
}

lanelet::ConstPolygons3d calcIntersectedPolygons(
  const tier4_autoware_utils::Box2d & bounding_box, const lanelet::ConstPolygons3d & polygons)
{
  lanelet::ConstPolygons3d intersected_polygons;
  for (const auto & polygon : polygons) {
    if (boost::geometry::intersects(bounding_box, lanelet::utils::to2D(polygon).basicPolygon())) {
      intersected_polygons.push_back(polygon);
    }
  }
  return intersected_polygons;
}

pcl::PointCloud<pcl::PointXYZ> removePointsWithinPolygons(
  const lanelet::ConstPolygons3d & polygons, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filtered_cloud.header = cloud->header;

  for (const auto & p : cloud->points) {
    if (pointWithinLanelets(Point2d(p.x, p.y), polygons)) {
      continue;
    }

    filtered_cloud.points.push_back(p);
  }

  return filtered_cloud;
}

}  // anonymous namespace

namespace pointcloud_preprocessor
{
NoDetectionAreaFilterComponent::NoDetectionAreaFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Filter("NoDetectionAreaFilter", node_options)
{
  polygon_type_ =
    static_cast<std::string>(declare_parameter("polygon_type", "no_obstacle_segmentation_area"));

  using std::placeholders::_1;
  // Set subscriber
  map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NoDetectionAreaFilterComponent::mapCallback, this, _1));
}

void NoDetectionAreaFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  if (no_detection_area_lanelets_.empty()) {
    output = *input;
    return;
  }

  // convert to PCL message
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*input, *pc_input);

  // calculate bounding box of points
  const auto bounding_box = calcBoundingBox(pc_input);

  // use only intersected lanelets to reduce calculation cost
  const auto intersected_lanelets =
    calcIntersectedPolygons(bounding_box, no_detection_area_lanelets_);

  // filter pointcloud by lanelet
  const auto filtered_pc = removePointsWithinPolygons(intersected_lanelets, pc_input);

  // convert to ROS message
  pcl::toROSMsg(filtered_pc, output);
  output.header = input->header;
}

void NoDetectionAreaFilterComponent::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  tf_input_frame_ = map_msg->header.frame_id;

  const auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr);
  no_detection_area_lanelets_ =
    lanelet::utils::query::getAllPolygonsByType(lanelet_map_ptr, polygon_type_);
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::NoDetectionAreaFilterComponent)
