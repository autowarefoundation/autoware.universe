// Copyright 2024 TIER IV
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

#include "autoware_test_utils/mock_data_parser.hpp"

#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::test_utils
{
Pose parse_pose(const YAML::Node & node)
{
  Pose pose;
  pose.position.x = node["position"]["x"].as<double>();
  pose.position.y = node["position"]["y"].as<double>();
  pose.position.z = node["position"]["z"].as<double>();
  pose.orientation.x = node["orientation"]["x"].as<double>();
  pose.orientation.y = node["orientation"]["y"].as<double>();
  pose.orientation.z = node["orientation"]["z"].as<double>();
  pose.orientation.w = node["orientation"]["w"].as<double>();
  return pose;
}

LaneletPrimitive parse_lanelet_primitive(const YAML::Node & node)
{
  LaneletPrimitive primitive;
  primitive.id = node["id"].as<int64_t>();
  primitive.primitive_type = node["primitive_type"].as<std::string>();

  return primitive;
}

std::vector<LaneletPrimitive> parse_lanelet_primitives(const YAML::Node & node)
{
  std::vector<LaneletPrimitive> primitives;
  primitives.reserve(node.size());
  std::transform(node.begin(), node.end(), std::back_inserter(primitives), [&](const auto & p) {
    return parse_lanelet_primitive(p);
  });

  return primitives;
}

std::vector<LaneletSegment> parse_segments(const YAML::Node & node)
{
  std::vector<LaneletSegment> segments;
  std::transform(node.begin(), node.end(), std::back_inserter(segments), [&](const auto & input) {
    LaneletSegment segment;
    segment.preferred_primitive = parse_lanelet_primitive(input["preferred_primitive"]);
    segment.primitives = parse_lanelet_primitives(input["primitives"]);
    return segment;
  });

  return segments;
}

std::vector<Point> parse_geom_points(const YAML::Node & node)
{
  std::vector<Point> geom_points;

  std::transform(
    node.begin(), node.end(), std::back_inserter(geom_points), [&](const YAML::Node & input) {
      Point point;
      point.x = input["x"].as<double>();
      point.y = input["y"].as<double>();
      point.z = input["z"].as<double>();
      return point;
    });

  return geom_points;
}

Header parse_header(const YAML::Node & node)
{
  Header header;

  if (!node["header"]) {
    return header;
  }

  header.stamp.sec = node["header"]["stamp"]["sec"].as<int>();
  header.stamp.nanosec = node["header"]["stamp"]["nanosec"].as<uint32_t>();
  header.frame_id = node["header"]["frame_id"].as<std::string>();

  return header;
}

std::vector<PathPointWithLaneId> parse_path_points_with_lane_id(const YAML::Node & node)
{
  std::vector<PathPointWithLaneId> path_points;

  if (!node["points"]) {
    return path_points;
  }

  const auto & points = node["points"];
  path_points.reserve(points.size());
  std::transform(
    points.begin(), points.end(), std::back_inserter(path_points), [&](const YAML::Node & input) {
      PathPointWithLaneId point;
      if (!input["point"]) {
        return point;
      }
      const auto & point_node = input["point"];
      point.point.pose = parse_pose(point_node["pose"]);

      point.point.longitudinal_velocity_mps = point_node["longitudinal_velocity_mps"].as<float>();
      point.point.lateral_velocity_mps = point_node["lateral_velocity_mps"].as<float>();
      point.point.heading_rate_rps = point_node["heading_rate_rps"].as<float>();
      point.point.is_final = point_node["is_final"].as<bool>();
      // Fill the lane_ids
      for (const auto & lane_id_node : input["lane_ids"]) {
        point.lane_ids.push_back(lane_id_node.as<int64_t>());
      }

      return point;
    });

  return path_points;
}

// cppcheck-suppress unusedFunction
LaneletRoute parse_lanelet_route_file(const std::string & filename)
{
  LaneletRoute lanelet_route;
  try {
    YAML::Node config = YAML::LoadFile(filename);

    lanelet_route.start_pose = (config["start_pose"]) ? parse_pose(config["start_pose"]) : Pose();
    lanelet_route.goal_pose = (config["goal_pose"]) ? parse_pose(config["goal_pose"]) : Pose();
    lanelet_route.segments = parse_segments(config["segments"]);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("autoware_test_utils"), "Exception caught: %s", e.what());
  }
  return lanelet_route;
}

PathWithLaneId parse_path_with_lane_id_file(const std::string & filename)
{
  PathWithLaneId path;
  try {
    YAML::Node yaml_node = YAML::LoadFile(filename);

    path.header = parse_header(yaml_node);
    path.points = parse_path_points_with_lane_id(yaml_node);
    path.left_bound = parse_geom_points(yaml_node["left_bound"]);
    path.right_bound = parse_geom_points(yaml_node["right_bound"]);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("autoware_test_utils"), "Exception caught: %s", e.what());
  }

  return path;
}
}  // namespace autoware::test_utils
