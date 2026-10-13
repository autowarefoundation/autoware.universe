// Copyright 2026 TIER IV, Inc.
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

// Closed-loop scenarios that drive the real ExternalVelocityLimit and MapVelocityLimits plugins:
// the external limit is received through its topic and the map limit is resolved from a lanelet
// map, so parameter plumbing and the lanelet lookup are covered as well.

#include "autoware/trajectory_modifier/trajectory_modifier_context.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/external_velocity_limit.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/map_velocity_limits.hpp"
#include "velocity_limit_simulation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace sim = autoware::trajectory_modifier::test::velocity_limit_simulation;
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::ExternalVelocityLimit;
using autoware::trajectory_modifier::plugin::MapVelocityLimits;
using autoware::trajectory_modifier::plugin::ProcessingResult;
namespace detail = autoware::trajectory_modifier::plugin::detail;
using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using sim::kmph;

constexpr double lanelet_length = 50.0;
constexpr std::size_t lanelet_count = 30;
constexpr lanelet::Id first_lanelet_id = 5000;
// Lanelets 6 to 8 cover the 30 km/h zone at 300-450 m.
constexpr std::size_t first_zone_lanelet = 6;
constexpr std::size_t last_zone_lanelet = 8;

lanelet::Id lanelet_id(const std::size_t index)
{
  return first_lanelet_id + static_cast<lanelet::Id>(index);
}

/// @brief Straight one-lane road along +x. The speed_limit attribute is in km/h, as in Autoware
/// maps; the zone lanelets are 30 km/h and the rest 60 km/h.
lanelet::LaneletMapPtr make_straight_road()
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  std::vector<lanelet::Point3d> left;
  std::vector<lanelet::Point3d> right;
  for (std::size_t i = 0; i <= lanelet_count; ++i) {
    const double x = lanelet_length * static_cast<double>(i);
    left.emplace_back(lanelet::utils::getId(), x, 1.75, 0.0);
    right.emplace_back(lanelet::utils::getId(), x, -1.75, 0.0);
  }
  for (std::size_t i = 0; i < lanelet_count; ++i) {
    lanelet::LineString3d left_bound{lanelet::utils::getId(), {left[i], left[i + 1]}};
    lanelet::LineString3d right_bound{lanelet::utils::getId(), {right[i], right[i + 1]}};
    lanelet::Lanelet lanelet{lanelet_id(i), left_bound, right_bound};
    lanelet.setAttribute(lanelet::AttributeName::Type, lanelet::AttributeValueString::Lanelet);
    lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
    lanelet.setAttribute(lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
    lanelet.setAttribute(lanelet::AttributeName::OneWay, "yes");
    const bool in_zone = i >= first_zone_lanelet && i <= last_zone_lanelet;
    lanelet.setAttribute("speed_limit", in_zone ? "30" : "60");
    map->add(lanelet);
  }
  return map;
}

LaneletRoute make_route()
{
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position = sim::make_point(1.0, 0.0);
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.position = sim::make_point(lanelet_length * lanelet_count - 1.0, 0.0);
  route.goal_pose.orientation.w = 1.0;
  route.uuid.uuid.fill(7U);
  for (std::size_t i = 0; i < lanelet_count; ++i) {
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = lanelet_id(i);
    segment.preferred_primitive.primitive_type = "lane";
    segment.primitives.push_back(segment.preferred_primitive);
    route.segments.push_back(segment);
  }
  return route;
}

TrajectoryModifierParams make_params()
{
  TrajectoryModifierParams params;
  params.use_external_velocity_limit = true;
  params.use_map_velocity_limits = true;
  params.velocity_limits.max_acceleration = 1.0;
  params.velocity_limits.max_deceleration = 1.0;
  params.velocity_limits.max_jerk = 3.0;
  return params;
}

/// @brief Cruise at 60 km/h on the straight road with the map zone at 300-450 m.
sim::Scenario road_scenario(
  const std::string & name, const std::string & title, const std::string & description,
  const std::string & targets, const sim::StageKind stage)
{
  sim::Scenario scenario;
  scenario.name = name;
  scenario.title = title;
  scenario.description = description;
  scenario.targets = targets;
  scenario.level = "plugin";
  scenario.fixed_profile = detail::FixedProfileParameters{};  // Default plugin parameters.
  scenario.stages = {stage};
  scenario.upstream.time_stamps = sim::uniform_time_stamps(0.1, 80);
  scenario.config.initial_state = {0.0, 0.0, kmph(60.0), 0.0};
  return scenario;
}

class VelocityLimitPluginSimulation : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto node_options = rclcpp::NodeOptions{};
    const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
    node_ = std::make_shared<rclcpp::Node>("velocity_limit_plugin_simulation", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());

    const auto map = make_straight_road();
    map_bin_ = std::make_shared<LaneletMapBin>(
      autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map));
    map_bin_->header.frame_id = "map";
    route_ = std::make_shared<LaneletRoute>(make_route());
  }

  void TearDown() override
  {
    context_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  /// @brief Plugin input for a measured ego state; nullptr acceleration when not given.
  [[nodiscard]] TrajectoryModifierData make_data(
    const sim::Scenario & scenario, const sim::EgoState & ego,
    const bool with_acceleration = true) const
  {
    TrajectoryModifierData data;
    auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
    // Stamped so that the fixed profile (enabled by default) can measure elapsed times.
    odometry->header.stamp = rclcpp::Time(static_cast<int64_t>((100.0 + ego.time) * 1e9));
    odometry->pose.pose = scenario.path.pose_at(ego.s);
    odometry->twist.twist.linear.x = ego.velocity;
    data.current_odometry = odometry;
    if (with_acceleration) {
      auto acceleration = std::make_shared<geometry_msgs::msg::AccelWithCovarianceStamped>();
      acceleration->accel.accel.linear.x = ego.acceleration;
      data.current_acceleration = acceleration;
    }
    data.lanelet_map_bin = map_bin_;
    data.route = route_;
    return data;
  }

  template <class Plugin>
  void initialize(
    Plugin & plugin, const std::string & class_name, const TrajectoryModifierParams & params)
  {
    plugin.initialize(
      "autoware::trajectory_modifier::plugin::" + class_name, node_.get(), time_keeper_, context_,
      params);
  }

  /// @brief Wrap a plugin instance as a closed-loop stage.
  template <class Plugin>
  sim::Stage make_stage(
    const std::string & name, Plugin & plugin, const sim::Scenario & scenario,
    std::function<void(const sim::EgoState &)> before = nullptr)
  {
    return {
      name, [this, &plugin, &scenario, before](
              sim::TrajectoryPoints & points, const sim::EgoState & ego) {
        if (before) {
          before(ego);
        }
        auto data = make_data(scenario, ego);
        const auto status = plugin.process(points, data);
        return sim::StageOutcome{status, detail::to_string(plugin.last_profile_start())};
      }};
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<TrajectoryModifierContext> context_;
  std::shared_ptr<LaneletMapBin> map_bin_;
  std::shared_ptr<LaneletRoute> route_;
};

TEST_F(VelocityLimitPluginSimulation, P01_plugin_external_cruise_60_to_30)
{
  auto scenario = road_scenario(
    "P01_plugin_external_cruise_60_to_30", "Plugin: external limit 30 km/h received at t=2 s",
    "Real ExternalVelocityLimit; the limit is published on its topic at t=2 s (same profile as "
    "E01).",
    "baseline, wiring", sim::StageKind::External);
  const auto limit = sim::make_external_limit(kmph(30.0));
  scenario.external_events = {{2.0, limit}};
  scenario.config.duration = 30.0;
  scenario.expectations.steady_windows = {{sim::WindowDomain::Time, 20.0, 30.0, kmph(30.0)}};

  ExternalVelocityLimit plugin;
  initialize(plugin, "ExternalVelocityLimit", make_params());
  const auto publisher =
    node_->create_publisher<VelocityLimit>("~/input/external_velocity_limit_mps", rclcpp::QoS{1});

  // Publish when the limit becomes active and wait until the plugin has received it.
  bool published = false;
  const auto publish_when_due = [&](const sim::EgoState & ego) {
    if (published || ego.time < 2.0 - 1e-9) {
      return;
    }
    published = true;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    publisher->publish(limit);
    auto probe = scenario.upstream.generate(scenario.path, ego);
    for (auto & point : probe) {
      point.longitudinal_velocity_mps = 100.0F;
    }
    while (std::chrono::steady_clock::now() < deadline) {
      auto points = probe;
      auto data = make_data(scenario, ego);
      if (plugin.process(points, data) == ProcessingResult::Modified) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    FAIL() << "The external velocity limit was not received by the plugin";
  };

  sim::run_and_expect(scenario, {make_stage("external", plugin, scenario, publish_when_due)});
}

TEST_F(VelocityLimitPluginSimulation, P02_plugin_map_speed_limit_attribute_kmph)
{
  auto scenario = road_scenario(
    "P02_plugin_map_speed_limit_attribute_kmph",
    "Plugin: lanelet speed_limit attribute \"30\" (km/h)",
    "Real MapVelocityLimits on a map whose zone lanelets have speed_limit=30 and the others 60, "
    "in km/h as in Autoware maps. Expected: 8.33 m/s inside the zone.",
    "G (attribute units)", sim::StageKind::Map);
  scenario.zones = {
    {-100.0, 300.0, kmph(60.0)},
    {300.0, 450.0, kmph(30.0), kmph(60.0)},
    {450.0, 1500.0, kmph(60.0)}};
  scenario.config.duration = 65.0;
  scenario.expectations.steady_windows = {
    {sim::WindowDomain::Distance, 330.0, 445.0, kmph(30.0)},
    {sim::WindowDomain::Distance, 620.0, 700.0, kmph(60.0)}};

  MapVelocityLimits plugin;
  initialize(plugin, "MapVelocityLimits", make_params());
  sim::run_and_expect(scenario, {make_stage("map", plugin, scenario)});
}

TEST_F(VelocityLimitPluginSimulation, P03_plugin_map_debug_override_mps)
{
  auto scenario = road_scenario(
    "P03_plugin_map_debug_override_mps", "Plugin: zone lanelets overridden to 8.33 m/s",
    "Real MapVelocityLimits with limit_velocity_from_map_debug_* set to 8.33 m/s for the zone "
    "lanelets (same profile as M01).",
    "B, wiring", sim::StageKind::Map);
  scenario.zones = {
    {-100.0, 300.0, kmph(60.0)},
    {300.0, 450.0, kmph(30.0), kmph(60.0)},
    {450.0, 1500.0, kmph(60.0)}};
  scenario.config.duration = 65.0;
  scenario.expectations.steady_windows = {
    {sim::WindowDomain::Distance, 330.0, 445.0, kmph(30.0)},
    {sim::WindowDomain::Distance, 620.0, 700.0, kmph(60.0)}};

  auto params = make_params();
  for (std::size_t i = first_zone_lanelet; i <= last_zone_lanelet; ++i) {
    params.map_velocity_limits.limit_velocity_from_map_debug_lanelet_ids.push_back(lanelet_id(i));
    params.map_velocity_limits.limit_velocity_from_map_debug_max_velocities.push_back(kmph(30.0));
  }
  MapVelocityLimits plugin;
  initialize(plugin, "MapVelocityLimits", params);
  sim::run_and_expect(scenario, {make_stage("map", plugin, scenario)});
}

TEST_F(VelocityLimitPluginSimulation, P04_plugin_map_fixed_profile_two_candidates)
{
  auto scenario = road_scenario(
    "P04_plugin_map_fixed_profile_two_candidates", "Plugin: fixed profile with two candidates",
    "Real MapVelocityLimits with the default fixed profile, processing a second candidate that "
    "requests 40 km/h after the first one every cycle. Each candidate keeps its own previous "
    "plan, so the first one never restarts.",
    "fixed profile, candidates", sim::StageKind::Map);
  scenario.zones = {
    {-100.0, 300.0, kmph(60.0)},
    {300.0, 450.0, kmph(30.0), kmph(60.0)},
    {450.0, 1500.0, kmph(60.0)}};
  scenario.config.duration = 65.0;
  scenario.expectations.max_profile_resets = 0;
  scenario.expectations.steady_windows = {
    {sim::WindowDomain::Distance, 330.0, 445.0, kmph(30.0)},
    {sim::WindowDomain::Distance, 620.0, 700.0, kmph(60.0)}};

  auto params = make_params();
  for (std::size_t i = first_zone_lanelet; i <= last_zone_lanelet; ++i) {
    params.map_velocity_limits.limit_velocity_from_map_debug_lanelet_ids.push_back(lanelet_id(i));
    params.map_velocity_limits.limit_velocity_from_map_debug_max_velocities.push_back(kmph(30.0));
  }
  MapVelocityLimits plugin;
  initialize(plugin, "MapVelocityLimits", params);
  auto second = scenario.upstream;
  second.cruise_velocity = kmph(40.0);

  const sim::Stage stage{"map", [&](sim::TrajectoryPoints & points, const sim::EgoState & ego) {
                           auto data = make_data(scenario, ego);
                           data.candidate_count = 2;
                           const auto status = plugin.process(points, data);
                           const auto start = plugin.last_profile_start();
                           auto second_points = second.generate(scenario.path, ego);
                           auto second_data = make_data(scenario, ego);
                           second_data.candidate_index = 1;
                           second_data.candidate_count = 2;
                           plugin.process(second_points, second_data);
                           return sim::StageOutcome{status, detail::to_string(start)};
                         }};
  sim::run_and_expect(scenario, {stage});
}

TEST_F(VelocityLimitPluginSimulation, MapPluginToleratesMissingAcceleration)
{
  // The node checks the acceleration before calling the plugins, but the plugins dereference it
  // unconditionally. Run in a child process so that a crash fails only this test.
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  auto scenario = road_scenario("unused", "", "", "F", sim::StageKind::Map);
  MapVelocityLimits plugin;
  initialize(plugin, "MapVelocityLimits", make_params());
  auto points = scenario.upstream.generate(scenario.path, scenario.config.initial_state);
  for (auto & point : points) {
    point.longitudinal_velocity_mps = 100.0F;
  }
  auto data = make_data(scenario, scenario.config.initial_state, false);
  EXPECT_EXIT(
    {
      plugin.process(points, data);
      std::exit(0);
    },
    ::testing::ExitedWithCode(0), "");
}

}  // namespace
