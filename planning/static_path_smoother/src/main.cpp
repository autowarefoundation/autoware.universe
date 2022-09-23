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

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "map_loader/lanelet2_map_loader_node.hpp"
#include "mission_planner/mission_planner_lanelet2.hpp"
#include "motion_utils/motion_utils.hpp"
#include "rclcpp/time.hpp"
#include "route_handler/route_handler.hpp"
#include "static_path_smoother/node.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;

namespace
{
geometry_msgs::msg::Pose getCenterPose(
  const std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr, const size_t lanelet_id)
{
  const auto lanelet = lanelet_map_ptr->laneletLayer.get(lanelet_id);
  const auto center_line = lanelet.centerline();
  const size_t center_line_idx = std::floor(center_line.size() / 2.0);

  geometry_msgs::msg::Point center_pos;
  center_pos.x = center_line[center_line_idx].x();
  center_pos.y = center_line[center_line_idx].y();

  geometry_msgs::msg::Point next_center_pos;
  next_center_pos.x = center_line[center_line_idx + 1].x();
  next_center_pos.y = center_line[center_line_idx + 1].y();

  geometry_msgs::msg::Pose center_pose;
  center_pose.position = center_pos;
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(center_pos, next_center_pos);
  center_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

  return center_pose;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto main_node = rclcpp::Node::make_shared("static_path_smoother");

  // create publisher
  const auto pub_map_bin =
    main_node->create_publisher<HADMapBin>("lanelet2_map_topic", rclcpp::QoS{1}.transient_local());
  const auto pub_raw_path_with_lane_id = main_node->create_publisher<PathWithLaneId>(
    "raw_path_with_lane_id", rclcpp::QoS{1}.transient_local());
  const auto pub_raw_path =
    main_node->create_publisher<Path>("raw_path", rclcpp::QoS{1}.transient_local());

  const auto lanelet2_file_name = main_node->declare_parameter<std::string>("lanelet2_file_name");
  const size_t start_lanelet_id = 125;
  const size_t end_lanelet_id = 132;

  // rclcpp::NodeOptions options;
  // auto map_loader_node = Lanelet2MapLoaderNode(options);

  // load map
  const auto map = Lanelet2MapLoaderNode::load_map(lanelet2_file_name, "MGRS");
  if (!map) {
    std::cerr << "error" << std::endl;  // TODO(murooka)
    return 0;
  }

  // create map bin msg
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Time now = system_clock.now();
  const auto map_bin_msg = Lanelet2MapLoaderNode::create_map_bin_msg(map, lanelet2_file_name, now);

  std::cerr << "[INFO] Loaded map." << std::endl;

  // publish map bin msg
  pub_map_bin->publish(map_bin_msg);
  std::cerr << "[INFO] Published map." << std::endl;

  // calculate check points (= start and goal pose)
  // TODO(murooka) better to use route_handler once it is refactored.
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::routing::RoutingGraphPtr routing_graph_ptr;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr;
  lanelet::utils::conversion::fromBinMsg(
    map_bin_msg, lanelet_map_ptr, &traffic_rules_ptr, &routing_graph_ptr);
  std::cerr << "[INFO] Inintialized lanelet2 map instance." << std::endl;

  const auto start_pose = getCenterPose(lanelet_map_ptr, start_lanelet_id);
  const auto end_pose = getCenterPose(lanelet_map_ptr, end_lanelet_id);
  const std::vector<geometry_msgs::msg::Pose> check_points{start_pose, end_pose};
  std::cerr << end_pose.orientation.z << " " << end_pose.orientation.w << std::endl;
  std::cerr << "[INFO] Calculated check points." << std::endl;

  // plan route
  rclcpp::NodeOptions options;
  auto mission_planner_node = mission_planner::MissionPlannerLanelet2(options);
  mission_planner_node.map_callback(std::make_shared<HADMapBin>(map_bin_msg));
  const auto route = mission_planner_node.plan_route(check_points);
  std::cerr << "[INFO] Calculated route." << std::endl;

  // calculate center line
  route_handler::RouteHandler route_handler;
  route_handler.setMap(map_bin_msg);
  lanelet::ConstLanelets lanelet_sequence;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive_id;
    const auto target_lanelet = lanelet_map_ptr->laneletLayer.get(target_lanelet_id);
    lanelet_sequence.push_back(target_lanelet);
  }
  auto raw_path_with_lane_id =
    route_handler.getCenterLinePath(lanelet_sequence, 0, std::numeric_limits<double>::max());
  raw_path_with_lane_id.header.frame_id = "map";

  // generate drivable area
  auto start_pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>();
  start_pose_ptr->pose = start_pose;

  auto planner_data = std::make_shared<behavior_path_planner::PlannerData>();
  planner_data->route_handler = std::make_shared<route_handler::RouteHandler>(route_handler);
  planner_data->self_pose = start_pose_ptr;
  planner_data->parameters.drivable_lane_forward_length = 300.0;
  planner_data->parameters.drivable_lane_backward_length = -5.0;
  planner_data->parameters.drivable_lane_margin = 5.0;
  planner_data->parameters.ego_nearest_dist_threshold = 3.0;
  planner_data->parameters.ego_nearest_yaw_threshold = 1.57;
  raw_path_with_lane_id.drivable_area = behavior_path_planner::util::generateDrivableArea(
    raw_path_with_lane_id, lanelet_sequence, 0.1, 1.0, planner_data);

  // publish raw path with lane id
  pub_raw_path_with_lane_id->publish(raw_path_with_lane_id);
  std::cerr << "[INFO] Calculated center line." << std::endl;

  // convert path with lane id to path
  Path raw_path;
  raw_path.header = raw_path_with_lane_id.header;
  raw_path.drivable_area = raw_path_with_lane_id.drivable_area;
  for (const auto & point : raw_path_with_lane_id.points) {
    raw_path.points.push_back(point.point);
  }
  pub_raw_path->publish(raw_path);

  // optimize path
  std::cout << "PO1" << std::endl;
  rclcpp::NodeOptions node_options;
  StaticPathSmoother successive_path_optimizer(node_options);
  successive_path_optimizer.pathCallback(std::make_shared<Path>(raw_path));
  std::cout << "PO2" << std::endl;

  rclcpp::spin(main_node);

  rclcpp::shutdown();
  return 0;
}
