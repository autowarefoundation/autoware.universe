// Copyright 2019 Autoware Foundation
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

#include "utility_functions.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <unordered_set>
#include <utility>

bool exists(const std::unordered_set<lanelet::Id> & set, const lanelet::Id & id)
{
  return set.find(id) != set.end();
}

tier4_autoware_utils::Polygon2d convert_linear_ring_to_polygon(
  tier4_autoware_utils::LinearRing2d footprint)
{
  tier4_autoware_utils::Polygon2d footprint_polygon;
  boost::geometry::append(footprint_polygon.outer(), footprint[0]);
  boost::geometry::append(footprint_polygon.outer(), footprint[1]);
  boost::geometry::append(footprint_polygon.outer(), footprint[2]);
  boost::geometry::append(footprint_polygon.outer(), footprint[3]);
  boost::geometry::append(footprint_polygon.outer(), footprint[4]);
  boost::geometry::append(footprint_polygon.outer(), footprint[5]);
  boost::geometry::correct(footprint_polygon);
  return footprint_polygon;
}

void set_color(std_msgs::msg::ColorRGBA * cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}

lanelet::ConstLanelet combine_lanelets(const lanelet::ConstLanelets & lanelets)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  lanelet::Points3d centers;
  std::vector<uint64_t> left_bound_ids;
  std::vector<uint64_t> right_bound_ids;

  for (const auto & llt : lanelets) {
    if (llt.id() != 0) {
      left_bound_ids.push_back(llt.leftBound().id());
      right_bound_ids.push_back(llt.rightBound().id());
    }
  }

  for (const auto & llt : lanelets) {
    if (std::count(right_bound_ids.begin(), right_bound_ids.end(), llt.leftBound().id()) < 1) {
      for (const auto & pt : llt.leftBound()) {
        lefts.push_back(lanelet::Point3d(pt));
      }
    }
    if (std::count(left_bound_ids.begin(), left_bound_ids.end(), llt.rightBound().id()) < 1) {
      for (const auto & pt : llt.rightBound()) {
        rights.push_back(lanelet::Point3d(pt));
      }
    }
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  auto combined_lanelet = lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
  return std::move(combined_lanelet);
}

boost::optional<size_t> findNearestIndex(
        const lanelet::ConstLineString3d & line, const geometry_msgs::msg::Point & point)
{
    if (line.empty()) {
        return {};
    }

    double min_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;

    for (size_t i = 0; i < line.size(); ++i) {
        geometry_msgs::msg::Point p1;
        p1.x = line[i].basicPoint().x();
        p1.y = line[i].basicPoint().y();

        const auto dx = p1.x - point.x;
        const auto dy = p1.y - point.y;
        const auto squared_distance = dx * dx + dy * dy;

        if (squared_distance < min_dist) {
            min_dist = squared_distance;
            min_idx = i;
        }
    }
    return min_idx;
}

geometry_msgs::msg::Pose convertBasicPoint3dToPose(
        const lanelet::BasicPoint3d & point, const geometry_msgs::msg::Quaternion & quaternion)
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();

    pose.orientation = quaternion;

    return pose;
}
