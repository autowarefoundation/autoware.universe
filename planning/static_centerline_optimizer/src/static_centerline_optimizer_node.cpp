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

#include "static_centerline_optimizer/static_centerline_optimizer_node.hpp"
#include "static_centerline_optimizer/msg/points_with_lane_id.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "static_centerline_optimizer/optimization_node.hpp"
#include "static_centerline_optimizer/type_alias.hpp"
#include "static_centerline_optimizer/utils.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
namespace
{
Path convert_to_path(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.drivable_area = path_with_lane_id.drivable_area;
  for (const auto & point : path_with_lane_id.points) {
    path.points.push_back(point.point);
  }

  return path;
}

lanelet::ConstLanelets get_lanelets_from_route(
  const RouteHandler & route_handler, const HADMapRoute & route)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive_id;
    const auto target_lanelet = route_handler.getLaneletsFromId(target_lanelet_id);
    lanelets.push_back(target_lanelet);
  }

  return lanelets;
}

rclcpp::NodeOptions create_node_options() { return rclcpp::NodeOptions{}; }

rclcpp::QoS create_transient_local_qos() { return rclcpp::QoS{1}.transient_local(); }

lanelet::BasicPoint2d convertToLaneletPoint(const geometry_msgs::msg::Point & geom_point)
{
  lanelet::BasicPoint2d point(geom_point.x, geom_point.y);
  return point;
}
}  // namespace

StaticCenterlineOptimizerNode::StaticCenterlineOptimizerNode(
  const rclcpp::NodeOptions & node_options)
: Node("static_centerlin_optimizer", node_options)
{
  // publishers
  pub_map_bin_ = create_publisher<HADMapBin>("lanelet2_map_topic", create_transient_local_qos());
  pub_raw_path_with_lane_id_ =
    create_publisher<PathWithLaneId>("raw_path_with_lane_id", create_transient_local_qos());
  pub_raw_path_ = create_publisher<Path>("debug/raw_centerline", create_transient_local_qos());

  // services
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_load_map_ = create_service<LoadMap>(
    "/planning/static_centerline_optimizer/load_map",
    std::bind(
      &StaticCenterlineOptimizerNode::on_load_map, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_route_ = create_service<PlanRoute>(
    "/planning/static_centerline_optimizer/plan_route",
    std::bind(
      &StaticCenterlineOptimizerNode::on_plan_route, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_path_ = create_service<PlanPath>(
    "/planning/static_centerline_optimizer/plan_path",
    std::bind(
      &StaticCenterlineOptimizerNode::on_plan_path, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
}

void StaticCenterlineOptimizerNode::run()
{
  // declare planning setting parameters
  const auto lanelet2_input_file_name = declare_parameter<std::string>("lanelet2_input_file_name");
  const auto lanelet2_output_file_name = "/tmp/lanelet2_map.osm";  // TODO(murooka)
  const int start_lanelet_id = declare_parameter<int>("start_lanelet_id");
  const int end_lanelet_id = declare_parameter<int>("end_lanelet_id");

  load_map(lanelet2_input_file_name);
  plan_route(start_lanelet_id, end_lanelet_id);
  plan_path(start_lanelet_id);
  std::cerr << "PO1" << std::endl;
  save_map(lanelet2_output_file_name);
}

void StaticCenterlineOptimizerNode::load_map(const std::string & lanelet2_input_file_name)
{
  // load map by the map_loader package
  map_bin_ptr_ = utils::create_map(*this, lanelet2_input_file_name, now());
  if (!map_bin_ptr_) {
    RCLCPP_ERROR(get_logger(), "Loading map failed");
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded map.");

  // publish map bin msg
  pub_map_bin_->publish(*map_bin_ptr_);
  RCLCPP_INFO(get_logger(), "Published map.");

  // create route_handler
  route_handler_ptr_ = std::make_shared<RouteHandler>();
  route_handler_ptr_->setMap(*map_bin_ptr_);
}

void StaticCenterlineOptimizerNode::on_load_map(
  const LoadMap::Request::SharedPtr request, const LoadMap::Response::SharedPtr response)
{
  const std::string tmp_lanelet2_input_file_name = "/tmp/input_lanelet2_map.osm";

  // save map
  std::ofstream map_writer;
  map_writer.open(tmp_lanelet2_input_file_name, std::ios::out);
  map_writer << request->map;
  map_writer.close();

  // load map
  load_map(tmp_lanelet2_input_file_name);

  response->success = map_bin_ptr_ ? true : false;
}

void StaticCenterlineOptimizerNode::on_plan_route(
  const PlanRoute::Request::SharedPtr request, const PlanRoute::Response::SharedPtr response)
{
  const int start_lanelet_id = request->start_lane_id;
  const int end_lanelet_id = request->end_lane_id;

  // plan route
  plan_route(start_lanelet_id, end_lanelet_id);

  // if (!lanelets_ptr_) {
  // }

  // extract lane ids
  std::vector<uint8_t> lane_ids;
  for (const auto & lanelet : *lanelets_ptr_) {
    lane_ids.push_back(lanelet.id());
  }

  // set response
  response->lane_ids = lane_ids;
}

void StaticCenterlineOptimizerNode::plan_route(const int start_lanelet_id, const int end_lanelet_id)
{
  // calculate check points (= start and goal pose)
  const auto check_points = [&]() {
    const auto start_center_pose = utils::get_center_pose(*route_handler_ptr_, start_lanelet_id);
    const auto end_center_pose = utils::get_center_pose(*route_handler_ptr_, end_lanelet_id);
    return std::vector<geometry_msgs::msg::Pose>{start_center_pose, end_center_pose};
  }();
  RCLCPP_INFO(get_logger(), "Calculated check points.");

  // plan route by the mission_planner package
  const auto route = utils::plan_route(map_bin_ptr_, check_points);
  RCLCPP_INFO(get_logger(), "Planned route.");

  // get lanelets
  const auto lanelets = get_lanelets_from_route(*route_handler_ptr_, route);
  lanelets_ptr_ = std::make_shared<lanelet::ConstLanelets>(lanelets);
}

void StaticCenterlineOptimizerNode::on_plan_path(
  const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response)
{
  const int start_lanelet_id = request->start_lane_id;

  const auto result = plan_path(start_lanelet_id);

  if (result == PlanPathResult::SUCCESS) {
  } else if (result == PlanPathResult::ROUTE_IS_NOT_READY) {
  }

  /*
  auto target_lanelet = lanelets_ptr_->cbegin();
  bool is_end_lanelet = false;
  current_lanelet_points;
  for (const auto & traj_point : optimized_traj_points_) {
    // convert optimized point to lanelet point
    const lanelet::BasicPoint2d point(traj_point.pose.position.x, traj_point.pose.position.y);

    // check if point is inside the target lanelet
    while(!lanelet::geometry::inside(*target_lanelet, point)) {
      current_lanelet_points.clear();
      // If not inside, next lanelet is target.
      target_lanelet++;

      if (target_lanelet == lanelets_ptr_->cend()) {
        is_end_lanelet = true;
        break;
      }
    }

    response->lane_ids.push_back(target_lanelet->id());
    std::unique(response->lane_ids);

    response->points_array.push_back(target_lanelet->id());

    if (is_end_lanelet) {
      break;
    }
  }
  */


  auto target_traj_point = optimized_traj_points_.cbegin();
  bool is_end_lanelet = false;
  for (const auto & lanelet : *lanelets_ptr_) {

    std::vector<geometry_msgs::msg::Point> current_lanelet_points;
    while(!lanelet::geometry::inside(lanelet, convertToLaneletPoint(target_traj_point->pose.position))) {
      current_lanelet_points.push_back(target_traj_point->pose.position);
      target_traj_point++;

      if (target_traj_point == optimized_traj_points_.cend()) {
        is_end_lanelet = true;
        break;
      }
    }

    if (!current_lanelet_points.empty()) {
      static_centerline_optimizer::msg::PointsWithLaneId points_with_lane_id;
      points_with_lane_id.lane_id = lanelet.id();
      points_with_lane_id.points = current_lanelet_points;
      response->points_with_lane_ids.push_back(points_with_lane_id);
    }

    if (is_end_lanelet) {
      break;
    }
  }
}

StaticCenterlineOptimizerNode::PlanPathResult StaticCenterlineOptimizerNode::plan_path(
  const int start_lanelet_id)
{
  if (!route_handler_ptr_ || !lanelets_ptr_) {
    return PlanPathResult::ROUTE_IS_NOT_READY;
  }

  // optimize centerline inside the lane
  const auto start_center_pose = utils::get_center_pose(*route_handler_ptr_, start_lanelet_id);

  // ego nearest search parameters
  const double ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = utils::get_path_with_lane_id(
    *route_handler_ptr_, *lanelets_ptr_, start_center_pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);

  pub_raw_path_with_lane_id_->publish(raw_path_with_lane_id);
  RCLCPP_INFO(get_logger(), "Calculated raw path with lane id and published.");

  // convert path with lane id to path
  const auto raw_path = convert_to_path(raw_path_with_lane_id);
  pub_raw_path_->publish(raw_path);
  RCLCPP_INFO(get_logger(), "Converted to path and published.");

  // optimize trajectory by the obstacle_avoidance_planner package
  auto a = create_node_options();
  StaticCenterlineOptmizer successive_path_optimizer(a);
  optimized_traj_points_ = successive_path_optimizer.pathCallback(std::make_shared<Path>(raw_path));
  RCLCPP_INFO(get_logger(), "Optimized trajectory and published.");
}

void StaticCenterlineOptimizerNode::save_map(const std::string & lanelet2_output_file_name)
{
  std::cerr << "PO2" << std::endl;
  if (!route_handler_ptr_) {
    return;
  }

  // update centerline in map
  utils::update_centerline(*route_handler_ptr_, *lanelets_ptr_, optimized_traj_points_);
  RCLCPP_INFO(get_logger(), "Updated centerline in map.");

  // save map with modified center line
  lanelet::write(lanelet2_output_file_name, *route_handler_ptr_->getLaneletMapPtr());
  RCLCPP_INFO(get_logger(), "Saved map.");
  std::cerr << "PO3" << std::endl;
}

}  // namespace static_centerline_optimizer
