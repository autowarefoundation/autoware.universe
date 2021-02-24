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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lane_departure_checker/lane_departure_checker_node.hpp"

#include "autoware_utils/math/unit_conversion.hpp"
#include "autoware_utils/ros/marker_helper.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/visualization/visualization.hpp"
#include "vehicle_info_util/vehicle_info.hpp"


using autoware_utils::rad2deg;

namespace
{
std::array<geometry_msgs::msg::Point, 3> triangle2points(
  const geometry_msgs::msg::Polygon & triangle)
{
  std::array<geometry_msgs::msg::Point, 3> points;
  for (size_t i = 0; i < 3; ++i) {
    const auto & p = triangle.points.at(i);

    geometry_msgs::msg::Point point;
    point.x = static_cast<double>(p.x);
    point.y = static_cast<double>(p.y);
    point.z = static_cast<double>(p.z);
    points.at(i) = point;
  }
  return points;
}

lanelet::ConstLanelets getRouteLanelets(
  const lanelet::LaneletMap & lanelet_map, const lanelet::routing::RoutingGraphPtr & routing_graph,
  const std::vector<autoware_planning_msgs::msg::RouteSection> & route_sections,
  const double vehicle_length)
{
  lanelet::ConstLanelets route_lanelets;

  // Add preceding lanes of front route_section to prevent detection errors
  {
    const auto extention_length = 2 * vehicle_length;

    for (const auto & lane_id : route_sections.front().lane_ids) {
      for (const auto & lanelet_sequence : lanelet::utils::query::getPrecedingLaneletSequences(
          routing_graph, lanelet_map.laneletLayer.get(lane_id), extention_length))
      {
        for (const auto & preceding_lanelet : lanelet_sequence) {
          route_lanelets.push_back(preceding_lanelet);
        }
      }
    }
  }

  for (const auto & route_section : route_sections) {
    for (const auto & lane_id : route_section.lane_ids) {
      route_lanelets.push_back(lanelet_map.laneletLayer.get(lane_id));
    }
  }

  // Add succeeding lanes of last route_section to prevent detection errors
  {
    const auto extention_length = 2 * vehicle_length;

    for (const auto & lane_id : route_sections.back().lane_ids) {
      for (const auto & lanelet_sequence : lanelet::utils::query::getSucceedingLaneletSequences(
          routing_graph, lanelet_map.laneletLayer.get(lane_id), extention_length))
      {
        for (const auto & succeeding_lanelet : lanelet_sequence) {
          route_lanelets.push_back(succeeding_lanelet);
        }
      }
    }
  }

  return route_lanelets;
}

template<typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}

}  // namespace

namespace lane_departure_checker
{
LaneDepartureCheckerNode::LaneDepartureCheckerNode(const rclcpp::NodeOptions & options)
: Node("lane_departure_checker_node", options),
  self_pose_listener_(this),
  debug_publisher_(this, "lane_departure_checker"),
  processing_time_publisher_(this),
  updater_(this)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);

  // Core Parameter

  // Vehicle Info
  auto i = vehicle_info_util::VehicleInfo::create(*this);
  param_.vehicle_info.wheel_radius = i.wheel_radius_m_;
  param_.vehicle_info.wheel_width = i.wheel_width_m_;
  param_.vehicle_info.wheel_base = i.wheel_base_m_;
  param_.vehicle_info.wheel_tread = i.wheel_tread_m_;
  param_.vehicle_info.front_overhang = i.front_overhang_m_;
  param_.vehicle_info.rear_overhang = i.rear_overhang_m_;
  param_.vehicle_info.left_overhang = i.left_overhang_m_;
  param_.vehicle_info.right_overhang = i.right_overhang_m_;
  param_.vehicle_info.vehicle_height = i.vehicle_height_m_;
  param_.vehicle_info.vehicle_length = i.vehicle_length_m_;
  param_.vehicle_info.vehicle_width = i.vehicle_width_m_;
  param_.vehicle_info.min_longitudinal_offset = i.min_longitudinal_offset_m_;
  param_.vehicle_info.max_longitudinal_offset = i.max_longitudinal_offset_m_;
  param_.vehicle_info.min_lateral_offset = i.min_lateral_offset_m_;
  param_.vehicle_info.max_lateral_offset = i.max_lateral_offset_m_;
  param_.vehicle_info.min_height_offset = i.min_height_offset_m_;
  param_.vehicle_info.max_height_offset = i.max_height_offset_m_;

  param_.footprint_margin = declare_parameter("footprint_margin", 0.0);
  param_.resample_interval = declare_parameter("resample_interval", 0.3);
  param_.max_deceleration = declare_parameter("max_deceleration", 3.0);
  param_.delay_time = declare_parameter("delay_time", 0.3);
  param_.max_lateral_deviation = declare_parameter("max_lateral_deviation", 1.0);
  param_.max_longitudinal_deviation = declare_parameter("max_longitudinal_deviation", 1.0);
  param_.max_yaw_deviation_deg = declare_parameter("max_yaw_deviation_deg", 30.0);

  // Parameter Callback
  set_param_res_ =
    add_on_set_parameters_callback(std::bind(&LaneDepartureCheckerNode::onParameter, this, _1));

  // Core
  lane_departure_checker_ = std::make_unique<LaneDepartureChecker>();
  lane_departure_checker_->setParam(param_);

  // Subscriber
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", 1, std::bind(&LaneDepartureCheckerNode::onTwist, this, _1));
  sub_lanelet_map_bin_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "input/lanelet_map_bin", rclcpp::QoS{1}.transient_local(),
    std::bind(&LaneDepartureCheckerNode::onLaneletMapBin, this, _1));
  sub_route_ = this->create_subscription<autoware_planning_msgs::msg::Route>(
    "input/route", 1, std::bind(&LaneDepartureCheckerNode::onRoute, this, _1));
  sub_reference_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1,
    std::bind(&LaneDepartureCheckerNode::onReferenceTrajectory, this, _1));
  sub_predicted_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/predicted_trajectory", 1,
    std::bind(&LaneDepartureCheckerNode::onPredictedTrajectory, this, _1));

  // Publisher
  // Nothing

  // Diagnostic Updater
  updater_.setHardwareID("lane_departure_checker");

  updater_.add(
    "lane_departure", this, &LaneDepartureCheckerNode::checkLaneDeparture);

  updater_.add(
    "trajectory_deviation", this, &LaneDepartureCheckerNode::checkTrajectoryDeviation);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  double delta_time = 1.0 / static_cast<double>(node_param_.update_rate);
  auto timer_callback_ = std::bind(&LaneDepartureCheckerNode::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback_)>>(
    this->get_clock(), period_ns, std::move(timer_callback_),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void LaneDepartureCheckerNode::onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_twist_ = msg;
}

void LaneDepartureCheckerNode::onLaneletMapBin(
  const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_, &traffif_rules_, &routing_graph_);
}

void LaneDepartureCheckerNode::onRoute(const autoware_planning_msgs::msg::Route::ConstSharedPtr msg)
{
  route_ = msg;
}

void LaneDepartureCheckerNode::onReferenceTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  reference_trajectory_ = msg;
}

void LaneDepartureCheckerNode::onPredictedTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  predicted_trajectory_ = msg;
}

bool LaneDepartureCheckerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for current_twist msg...");
    return false;
  }

  if (!lanelet_map_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for lanelet_map msg...");
    return false;
  }

  if (!route_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for route msg...");
    return false;
  }

  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for predicted_trajectory msg...");
    return false;
  }

  return true;
}

bool LaneDepartureCheckerNode::isDataTimeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp) - now;
  if (pose_time_diff.seconds() > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "pose is timeout...");
    return true;
  }

  return false;
}

void LaneDepartureCheckerNode::onTimer()
{
  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  // In order to wait for both of map and route will be ready, write this not in callback but here
  if (last_route_ != route_) {
    route_lanelets_ = getRouteLanelets(
      *lanelet_map_, routing_graph_, route_->route_sections, param_.vehicle_info.vehicle_length);
    last_route_ = route_;
  }

  input_.current_pose = current_pose_;
  input_.current_twist = current_twist_;
  input_.lanelet_map = lanelet_map_;
  input_.route = route_;
  input_.route_lanelets = route_lanelets_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;

  output_ = lane_departure_checker_->update(input_);

  updater_.force_update();

  {
    const auto & deviation = output_.trajectory_deviation;
    debug_publisher_.publish<autoware_debug_msgs::msg::Float64Stamped>(
      "deviation/lateral", deviation.lateral);
    debug_publisher_.publish<autoware_debug_msgs::msg::Float64Stamped>(
      "deviation/yaw", deviation.yaw);
    debug_publisher_.publish<autoware_debug_msgs::msg::Float64Stamped>(
      "deviation/yaw_deg", rad2deg(deviation.yaw));
  }

  debug_publisher_.publish<visualization_msgs::msg::MarkerArray>(
    std::string("marker_array"), createMarkerArray());

  processing_time_publisher_.publish(output_.processing_time_map);
}

rcl_interfaces::msg::SetParametersResult LaneDepartureCheckerNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    // Node
    update_param(parameters, "vizualize_lanelet", node_param_.visualize_lanelet);

    // Core
    update_param(parameters, "footprint_margin", param_.footprint_margin);
    update_param(parameters, "resample_interval", param_.resample_interval);
    update_param(parameters, "max_deceleration", param_.max_deceleration);
    update_param(parameters, "delay_time", param_.delay_time);

    if (lane_departure_checker_) {
      lane_departure_checker_->setParam(param_);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void LaneDepartureCheckerNode::checkLaneDeparture(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;
  std::string msg = "OK";

  if (output_.will_leave_lane) {
    level = DiagStatus::WARN;
    msg = "vehicle will leave lane";
  }

  if (output_.is_out_of_lane) {
    level = DiagStatus::ERROR;
    msg = "vehicle is out of lane";
  }

  stat.summary(level, msg);
}

void LaneDepartureCheckerNode::checkTrajectoryDeviation(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;

  if (std::abs(output_.trajectory_deviation.lateral) >= param_.max_lateral_deviation) {
    level = DiagStatus::ERROR;
  }

  if (std::abs(output_.trajectory_deviation.longitudinal) >= param_.max_longitudinal_deviation) {
    level = DiagStatus::ERROR;
  }

  if (std::abs(rad2deg(output_.trajectory_deviation.yaw)) >= param_.max_yaw_deviation_deg) {
    level = DiagStatus::ERROR;
  }

  std::string msg = "OK";
  if (level == DiagStatus::ERROR) {
    msg = "trajectory deviation is too large";
  }

  stat.addf("max lateral deviation", "%.3f", param_.max_lateral_deviation);
  stat.addf("lateral deviation", "%.3f", output_.trajectory_deviation.lateral);

  stat.addf("max longitudinal deviation", "%.3f", param_.max_longitudinal_deviation);
  stat.addf("longitudinal deviation", "%.3f", output_.trajectory_deviation.longitudinal);

  stat.addf("max yaw deviation", "%.3f", param_.max_yaw_deviation_deg);
  stat.addf("yaw deviation", "%.3f", rad2deg(output_.trajectory_deviation.yaw));

  stat.summary(level, msg);
}

visualization_msgs::msg::MarkerArray LaneDepartureCheckerNode::createMarkerArray() const
{
  using autoware_utils::createDefaultMarker;
  using autoware_utils::createMarkerColor;
  using autoware_utils::createMarkerScale;

  visualization_msgs::msg::MarkerArray marker_array;

  const auto base_link_z = current_pose_->pose.position.z;

  if (node_param_.visualize_lanelet) {
    // Route Lanelets
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "route_lanelets", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST,
        createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(0.0, 0.5, 0.5, 0.5));

      for (const auto & lanelet : input_.route_lanelets) {
        std::vector<geometry_msgs::msg::Polygon> triangles;
        lanelet::visualization::lanelet2Triangle(lanelet, &triangles);

        for (const auto & triangle : triangles) {
          for (const auto & point : triangle2points(triangle)) {
            marker.points.push_back(point);
            marker.colors.push_back(marker.color);
          }
        }
      }

      marker_array.markers.push_back(marker);
    }

    // Candidate Lanelets
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "candidate_lanelets", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST,
        createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(1.0, 1.0, 0.0, 0.1));

      for (const auto & lanelet : output_.candidate_lanelets) {
        std::vector<geometry_msgs::msg::Polygon> triangles;
        lanelet::visualization::lanelet2Triangle(lanelet, &triangles);

        for (const auto & triangle : triangles) {
          for (const auto & point : triangle2points(triangle)) {
            marker.points.push_back(point);
            marker.colors.push_back(marker.color);
          }
        }
      }

      marker_array.markers.push_back(marker);
    }
  }

  if (output_.resampled_trajectory.points.size() >= 2) {
    // Line of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "resampled_trajectory_line", 0,
        visualization_msgs::msg::Marker::LINE_STRIP,
        createMarkerScale(0.05, 0, 0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }

    // Points of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "resampled_trajectory_points", 0,
        visualization_msgs::msg::Marker::SPHERE_LIST,
        createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(0.0, 1.0, 0.0, 0.999));

      for (const auto & p : output_.resampled_trajectory.points) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Vehicle Footprints
  {
    const auto color_ok = createMarkerColor(0.0, 1.0, 0.0, 0.5);
    const auto color_will_leave_lane = createMarkerColor(0.5, 0.5, 0.0, 0.5);
    const auto color_is_out_of_lane = createMarkerColor(1.0, 0.0, 0.0, 0.5);

    auto color = color_ok;
    if (output_.will_leave_lane) {
      color = color_will_leave_lane;
    }
    if (output_.is_out_of_lane) {
      color = color_is_out_of_lane;
    }

    auto marker = createDefaultMarker(
      "map", this->now(), "vehicle_footprints", 0,
      visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0, 0), color);

    for (const auto & vehicle_footprint : output_.vehicle_footprints) {
      for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
        const auto p1 = vehicle_footprint.at(i);
        const auto p2 = vehicle_footprint.at(i + 1);

        marker.points.push_back(toMsg(p1.to_3d(base_link_z)));
        marker.points.push_back(toMsg(p2.to_3d(base_link_z)));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace lane_departure_checker
