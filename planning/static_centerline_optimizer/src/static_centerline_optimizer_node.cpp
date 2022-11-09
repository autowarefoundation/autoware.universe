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

LinearRing2d createVehicleFootprint(
  const geometry_msgs::msg::Pose & pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double margin=0.0)
{
  const auto & i = vehicle_info;

  const double x_front = i.front_overhang_m + i.wheel_base_m + margin;
  const double x_rear = -(i.rear_overhang_m + margin);
  const double y_left = i.wheel_tread_m / 2.0 + i.left_overhang_m + margin;
  const double y_right = -(i.wheel_tread_m / 2.0 + i.right_overhang_m + margin);

  std::vector<geometry_msgs::msg::Point> geom_points;
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_front, y_left, 0.0).position);
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_front, y_right, 0.0).position);
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_rear, y_right, 0.0).position);
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_rear, y_left, 0.0).position);

  LinearRing2d footprint;
  for (const auto & geom_point : geom_points) {
    footprint.push_back(Point2d{geom_point.x, geom_point.y});
  }
  footprint.push_back(footprint.back());

  return footprint;
}

std::array<double, 3> convertHexStrintToDecimal(const std::string & hex_str_color)
{
 unsigned int hex_int_color;
 std::istringstream iss(hex_str_color);
 iss >> std::hex >> hex_int_color;

 unsigned int unit = 16 * 16;
 unsigned int b = hex_int_color % unit;
 unsigned int g = (hex_int_color - b) / unit % unit;
 unsigned int r = (hex_int_color - g * unit - b) / unit / unit;

 return std::array<double, 3>{r / 255.0, g / 255.0, b / 255.0};
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

  // debug publishers
  pub_debug_unsafe_footprints_ =
    create_publisher<MarkerArray>("debug/unsafe_footprints", create_transient_local_qos());

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

  // vehicle info
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
}

void StaticCenterlineOptimizerNode::run()
{
  // declare planning setting parameters
  const auto lanelet2_input_file_name = declare_parameter<std::string>("lanelet2_input_file_name");
  const auto lanelet2_output_file_name = declare_parameter<std::string>("lanelet2_output_file_name");
  const int start_lanelet_id = declare_parameter<int>("start_lanelet_id");
  const int end_lanelet_id = declare_parameter<int>("end_lanelet_id");

  // process
  load_map(lanelet2_input_file_name);
  plan_route(start_lanelet_id, end_lanelet_id);
  plan_path(start_lanelet_id);
  evaluate();
  // save_map(lanelet2_output_file_name);
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

  // save map file
  std::ofstream map_writer;
  map_writer.open(tmp_lanelet2_input_file_name, std::ios::out);
  map_writer << request->map;
  map_writer.close();

  // load map from the saved map file
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

  if (!route_handler_ptr_ || !lanelets_ptr_) {
    response->message = "route_has_not_been_planned";
    return;
  }

  // extract lane ids
  std::vector<uint8_t> lane_ids;
  for (const auto & lanelet : *lanelets_ptr_) {
    lane_ids.push_back(lanelet.id());
  }

  // set response
  response->lane_ids = lane_ids;
  response->message = "";
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

  // plan path
  const auto result = plan_path(start_lanelet_id);
  if (result == PlanPathResult::ROUTE_IS_NOT_READY || !lanelets_ptr_) {
    response->message = "route_is_not_ready";
    return;
  }

  // create output data
  auto target_traj_point = optimized_traj_points_.cbegin();
  bool is_end_lanelet = false;
  for (const auto & lanelet : *lanelets_ptr_) {
    std::vector<geometry_msgs::msg::Point> current_lanelet_points;

    // check if target point is inside the lanelet
    while(!lanelet::geometry::inside(lanelet, convertToLaneletPoint(target_traj_point->pose.position))) {
      // memorize points inside the lanelet
      current_lanelet_points.push_back(target_traj_point->pose.position);
      target_traj_point++;

      if (target_traj_point == optimized_traj_points_.cend()) {
        is_end_lanelet = true;
        break;
      }
    }

    if (!current_lanelet_points.empty()) {
      // register points with lane_id
      static_centerline_optimizer::msg::PointsWithLaneId points_with_lane_id;
      points_with_lane_id.lane_id = lanelet.id();
      points_with_lane_id.points = current_lanelet_points;
      response->points_with_lane_ids.push_back(points_with_lane_id);
    }

    if (is_end_lanelet) {
      break;
    }
  }

  // empty string if error did not occurr
  response->message = "";
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

  // /*
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
  StaticCenterlineOptimizer successive_path_optimizer(a);
  optimized_traj_points_ = successive_path_optimizer.pathCallback(std::make_shared<Path>(raw_path));
  RCLCPP_INFO(get_logger(), "Optimized trajectory and published.");
  // */
}

void StaticCenterlineOptimizerNode::evaluate()
{
  const auto dist_thresh_vec = declare_parameter<std::vector<double>>("marker_color_dist_thresh");
  const auto marker_color_vec = declare_parameter<std::vector<std::string>>("marker_color");
  const auto get_marker_color =
    [&](const double dist) -> boost::optional<std::array<double, 3>> {
      for (size_t i = 0; i < dist_thresh_vec.size(); ++i) {
        const double dist_thresh = dist_thresh_vec.at(i);
        if (dist < dist_thresh) {
          return convertHexStrintToDecimal(marker_color_vec.at(i));
        }
      }
      return boost::none;
    };

  // create right/left bound
  LineString2d right_bound;
  LineString2d left_bound;
  for (const auto & lanelet : *lanelets_ptr_) {
    for (const auto & point : lanelet.rightBound()) {
      boost::geometry::append(right_bound, Point2d(point.x(), point.y()));
    }
    for (const auto & point : lanelet.leftBound()) {
      boost::geometry::append(left_bound, Point2d(point.x(), point.y()));
    }
  }

  // calculate the distance between footprint and right/left bounds
  MarkerArray marker_array;
  double min_dist = std::numeric_limits<double>::max();
  // for (const auto & traj_point : optimized_traj_points_) {
  for (size_t i = 0; i < optimized_traj_points_.size(); ++i) {
    const auto & traj_point = optimized_traj_points_.at(i);

    const auto footprint_poly = createVehicleFootprint(traj_point.pose, vehicle_info_);

    const double dist_to_right = boost::geometry::distance(footprint_poly, right_bound);
    const double dist_to_left = boost::geometry::distance(footprint_poly, left_bound);
    const double min_dist_to_bound = std::min(dist_to_right, dist_to_left);

    if (min_dist_to_bound < min_dist) {
      min_dist = min_dist_to_bound;
    }

    // create marker
    const auto marker_color_opt = get_marker_color(min_dist_to_bound);
    if (marker_color_opt) {
      const auto & marker_color = marker_color_opt.get();
      const auto footprint_marker = createFootprintMarker(footprint_poly, marker_color, i);
      tier4_autoware_utils::appendMarkerArray(footprint_marker, &marker_array);
    }

    std::cerr << dist_to_right << " " << dist_to_left << std::endl;
  }

  pub_debug_unsafe_footprints_->publish(marker_array);

  RCLCPP_INFO(get_logger(), "Minimum distance to road is ", min_dist);
}

MarkerArray StaticCenterlineOptimizerNode::createFootprintMarker(const LinearRing2d & footprint_poly, const std::array<double, 3> & marker_color, const size_t idx)
{
  const double r = marker_color.at(0);
  const double g = marker_color.at(1);
  const double b = marker_color.at(2);

  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock().now(), "unsafe_footprints", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.1, 0.0, 0.0), tier4_autoware_utils::createMarkerColor(r, g, b, 0.999));
  // marker.header.stamp = rclcpp::Time();
  // marker.lifetime = rclcpp::Duration(0, 0);

  for (const auto & point : footprint_poly) {
    geometry_msgs::msg::Point geom_point;
    geom_point.x = point.x();
    geom_point.y = point.y();
    // geom_point.z = point.z();

    marker.points.push_back(geom_point);
  }
  marker.points.push_back(marker.points.front());

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);

  return marker_array;
}

void StaticCenterlineOptimizerNode::save_map(const std::string & lanelet2_output_file_name)
{
  if (!route_handler_ptr_) {
    return;
  }

  // update centerline in map
  utils::update_centerline(*route_handler_ptr_, *lanelets_ptr_, optimized_traj_points_);
  RCLCPP_INFO(get_logger(), "Updated centerline in map.");

  // save map with modified center line
  lanelet::write(lanelet2_output_file_name, *route_handler_ptr_->getLaneletMapPtr());
  RCLCPP_INFO(get_logger(), "Saved map.");
}

}  // namespace static_centerline_optimizer
