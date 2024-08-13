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

#include "out_of_lane_collisions.hpp"

#include "types.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <rclcpp/duration.hpp>

#include <boost/geometry/algorithms/detail/envelope/interface.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/index/predicates.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <unordered_set>

namespace autoware::motion_velocity_planner::out_of_lane
{
void calculate_object_time_collisions(
  OutOfLaneData & out_of_lane_data,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects)
{
  universe_utils::Polygon2d object_footprint;
  for (const auto & object : objects) {
    for (const auto & object_path : object.kinematics.predicted_paths) {
      const auto time_step = rclcpp::Duration(object_path.time_step).seconds();
      auto t = time_step;
      for (const auto & object_pose : object_path.path) {
        t += time_step;
        object_footprint = universe_utils::toPolygon2d(object_pose, object.shape);
        std::vector<OutAreaNode> query_results;
        out_of_lane_data.outside_areas_rtree.query(
          boost::geometry::index::intersects(object_footprint.outer()),
          std::back_inserter(query_results));
        std::unordered_set<size_t> out_of_lane_indexes;
        for (const auto & query_result : query_results) {
          out_of_lane_indexes.insert(query_result.second);
        }
        for (const auto index : out_of_lane_indexes) {
          auto & out_of_lane_point = out_of_lane_data.outside_points[index];
          if (out_of_lane_point.collision_times.count(t) == 0UL) {
            for (const auto & ring : out_of_lane_point.outside_rings) {
              if (!boost::geometry::disjoint(ring, object_footprint.outer())) {
                out_of_lane_point.collision_times.insert(t);
                break;
              }
            }
          }
        }
      }
    }
  }
}

void calculate_collisions_to_avoid(
  OutOfLaneData & out_of_lane_data, const EgoData & ego_data, const PlannerParam & params)
{
  for (auto & out_of_lane_point : out_of_lane_data.outside_points) {
    auto min_time = std::numeric_limits<double>::infinity();
    auto max_time = -std::numeric_limits<double>::infinity();
    for (const auto & t : out_of_lane_point.collision_times) {
      min_time = std::min(t, min_time);
      max_time = std::max(t, max_time);
    }
    if (min_time <= max_time) {
      out_of_lane_point.min_object_arrival_time = min_time;
      out_of_lane_point.max_object_arrival_time = max_time;
      const auto & ego_time =
        rclcpp::Duration(
          ego_data.trajectory_points[out_of_lane_point.trajectory_index].time_from_start)
          .seconds();
      if (ego_time >= min_time && ego_time <= max_time) {
        out_of_lane_point.ttc = 0.0;
      } else {
        out_of_lane_point.ttc =
          std::min(std::abs(ego_time - min_time), std::abs(ego_time - max_time));
      }
    }
  }
  for (auto & p : out_of_lane_data.outside_points) {
    p.to_avoid = params.mode == "ttc" ? (p.ttc && p.ttc <= params.ttc_threshold)
                                      : (p.min_object_arrival_time &&
                                         p.min_object_arrival_time <= params.time_threshold);
  }
}

OutOfLaneData calculate_out_of_lane_areas(const EgoData & ego_data)
{
  OutOfLaneData out_of_lane_data;
  out_of_lane::OutOfLanePoint p;
  for (auto i = 0UL; i < ego_data.trajectory_footprints.size(); ++i) {
    const auto & footprint = ego_data.trajectory_footprints[i];
    p.outside_rings.clear();
    out_of_lane::Polygons out_of_lane_polygons;
    boost::geometry::difference(footprint, ego_data.drivable_lane_polygons, out_of_lane_polygons);
    for (const auto & area : out_of_lane_polygons) {
      p.outside_rings.push_back(area.outer);
    }
    if (!p.outside_rings.empty()) {
      p.trajectory_index = i + ego_data.first_trajectory_idx;
      out_of_lane_data.outside_points.push_back(p);
    }
  }

  std::vector<OutAreaNode> rtree_nodes;
  for (auto i = 0UL; i < out_of_lane_data.outside_points.size(); ++i) {
    for (const auto & ring : out_of_lane_data.outside_points[i].outside_rings) {
      OutAreaNode n;
      n.first = boost::geometry::return_envelope<universe_utils::Box2d>(ring);
      n.second = i;
      rtree_nodes.push_back(n);
    }
  }
  out_of_lane_data.outside_areas_rtree = {rtree_nodes.begin(), rtree_nodes.end()};
  return out_of_lane_data;
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
