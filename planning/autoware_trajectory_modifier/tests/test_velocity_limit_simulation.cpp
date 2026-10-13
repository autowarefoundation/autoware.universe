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

// Closed-loop scenarios for apply_velocity_limits() as called by ExternalVelocityLimit and
// MapVelocityLimits. Each scenario asserts the intended behavior and writes the data plotted by
// plot_velocity_limit_simulation.py, so a failing scenario also documents the defect.
//
// Expected behavior encoded here:
// - a limit stage is never faster than its input at the same arc length (it never accelerates on
//   its own and never removes an upstream deceleration or stop);
// - the acceleration, deceleration and jerk limits are never violated;
// - the ego converges to min(limit, upstream request) within the reference settle time;
// - a map zone is entered at its limit when feasible, and braking does not start much earlier than
//   the latest feasible point;
// - zero deceleration or jerk in an external limit message falls back to the parameters.

#include "velocity_limit_simulation.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

namespace
{
namespace sim = autoware::trajectory_modifier::test::velocity_limit_simulation;
using sim::kmph;
using sim::make_external_limit;
using sim::Scenario;
using sim::StageKind;
using sim::SteadyWindow;
using sim::WindowDomain;

constexpr double v20 = kmph(20.0);
constexpr double v30 = kmph(30.0);
constexpr double v40 = kmph(40.0);
constexpr double v50 = kmph(50.0);
constexpr double v60 = kmph(60.0);
constexpr double infinity = std::numeric_limits<double>::infinity();

SteadyWindow steady_time(const double begin, const double end, const double target)
{
  return {WindowDomain::Time, begin, end, target};
}

SteadyWindow steady_distance(const double begin, const double end, const double target)
{
  return {WindowDomain::Distance, begin, end, target};
}

/// @brief Cruise at 60 km/h on a straight road; the upstream converges to 60 km/h from the ego.
Scenario base_scenario(
  const std::string & title, const std::string & description, const std::string & targets,
  const std::vector<StageKind> & stages)
{
  Scenario scenario;
  scenario.title = title;
  scenario.description = description;
  scenario.targets = targets;
  scenario.stages = stages;
  scenario.upstream.time_stamps = sim::uniform_time_stamps(0.1, 80);
  scenario.config.initial_state = {0.0, 0.0, v60, 0.0};
  return scenario;
}

Scenario external_scenario(
  const std::string & title, const std::string & description, const std::string & targets)
{
  return base_scenario(title, description, targets, {StageKind::External});
}

Scenario map_scenario(
  const std::string & title, const std::string & description, const std::string & targets)
{
  return base_scenario(title, description, targets, {StageKind::Map});
}

/// @brief Recorded modifier input replayed every cycle: its geometry, its point timing and its
/// constant 60 km/h request with a final zero-velocity point.
Scenario recorded_scenario(
  const std::string & file, const std::string & title, const std::string & description,
  const std::vector<StageKind> & stages, const double velocity, const double acceleration)
{
  const auto recorded = sim::load_recorded_trajectory(file);
  std::vector<geometry_msgs::msg::Point> positions;
  std::vector<double> times;
  for (const auto & point : recorded) {
    positions.push_back(point.position);
    times.push_back(point.time - recorded.front().time);
  }
  auto scenario = base_scenario(title, description, "replay of a recorded input", stages);
  scenario.path = sim::ReferencePath::extended(positions, 3000.0);
  scenario.upstream.profile = sim::UpstreamProfile::ConstantSpeed;
  scenario.upstream.cruise_velocity = recorded.front().velocity;
  scenario.upstream.time_stamps = times;
  scenario.upstream.terminal_zero_velocity = true;
  scenario.config.initial_state = {0.0, 0.0, velocity, acceleration};
  return scenario;
}

/// @brief Arc length of a recorded point, used where the recorded limit changes.
double recorded_s(const std::string & file, const Scenario & scenario, const std::size_t index)
{
  return scenario.path.project(sim::load_recorded_trajectory(file).at(index).position);
}

// ---------------------------------------------------------------------------------------------
// External velocity limit
// ---------------------------------------------------------------------------------------------

Scenario e01_external_cruise_60_to_30()
{
  auto s = external_scenario(
    "External: cruise 60 km/h, limit 30 km/h at t=2 s",
    "Baseline jerk-limited deceleration with the nominal constraints (1.0 m/s^2, 3.0 m/s^3).",
    "baseline");
  s.external_events = {{2.0, make_external_limit(v30)}};
  s.config.duration = 30.0;
  s.expectations.steady_windows = {steady_time(20.0, 30.0, v30)};
  return s;
}

Scenario e02_external_message_constraints()
{
  auto s = external_scenario(
    "External: limit 30 km/h with message constraints",
    "use_constraints with min_acceleration -2.0 and min_jerk -0.6 replace the nominal values.",
    "message mapping");
  s.external_events = {{2.0, make_external_limit(v30, -2.0, -0.6)}};
  s.config.duration = 30.0;
  s.expectations.steady_windows = {steady_time(20.0, 30.0, v30)};
  return s;
}

Scenario e03_external_accelerating_below_limit()
{
  auto s = external_scenario(
    "External: ego accelerating below the limit",
    "Ego at 7.5 m/s accelerating at +1.0 m/s^2 toward 60 km/h with a 30 km/h limit. The ego "
    "should keep accelerating up to the limit and hold it.",
    "A1, D");
  s.external_events = {{0.0, make_external_limit(v30)}};
  s.config.initial_state = {0.0, 0.0, 7.5, 1.0};
  s.config.duration = 20.0;
  s.expectations.steady_windows = {steady_time(8.0, 20.0, v30)};
  return s;
}

Scenario e04_external_accelerating_above_limit()
{
  auto s = external_scenario(
    "External: limit arrives while accelerating above it",
    "Ego at 40 km/h accelerating at +1.0 m/s^2 when a 30 km/h limit arrives. The acceleration "
    "must first be ramped down, so a small unavoidable overshoot is expected.",
    "D");
  s.external_events = {{0.0, make_external_limit(v30)}};
  s.config.initial_state = {0.0, 0.0, v40, 1.0};
  s.config.duration = 20.0;
  s.expectations.steady_windows = {steady_time(12.0, 20.0, v30)};
  return s;
}

Scenario e05_external_slow_ego_below_limit()
{
  auto s = external_scenario(
    "External: slow ego below the limit",
    "Ego at 3 m/s, upstream accelerating toward 60 km/h, 30 km/h limit. The ego should "
    "accelerate up to the limit.",
    "A1");
  s.external_events = {{0.0, make_external_limit(v30)}};
  s.config.initial_state = {0.0, 0.0, 3.0, 0.0};
  s.config.duration = 25.0;
  s.expectations.steady_windows = {steady_time(12.0, 25.0, v30)};
  return s;
}

Scenario e06_external_departure_from_standstill()
{
  auto s = external_scenario(
    "External: departure from standstill under a limit",
    "Ego stopped, upstream departing toward 60 km/h, 30 km/h limit active.", "A1");
  s.external_events = {{0.0, make_external_limit(v30)}};
  s.config.initial_state = {0.0, 0.0, 0.0, 0.0};
  s.config.duration = 25.0;
  s.expectations.steady_windows = {steady_time(14.0, 25.0, v30)};
  return s;
}

Scenario e07_external_limit_raised()
{
  auto s = external_scenario(
    "External: limit raised from 30 to 50 km/h at t=10 s",
    "Ego cruising at the 30 km/h limit; the limit is raised to 50 km/h while the upstream still "
    "requests 60 km/h. The ego should accelerate to the new limit.",
    "A1");
  s.external_events = {{0.0, make_external_limit(v30)}, {10.0, make_external_limit(v50)}};
  s.config.initial_state = {0.0, 0.0, v30, 0.0};
  s.config.duration = 30.0;
  s.expectations.steady_windows = {steady_time(3.0, 9.0, v30), steady_time(22.0, 30.0, v50)};
  return s;
}

Scenario e08_external_limit_to_zero()
{
  auto s = external_scenario(
    "External: limit lowered to 0 km/h at t=2 s",
    "A zero external limit must bring the ego to a standstill without oscillation.",
    "stop by limit");
  s.external_events = {{2.0, make_external_limit(0.0)}};
  s.config.initial_state = {0.0, 0.0, v30, 0.0};
  s.config.duration = 20.0;
  s.expectations.steady_windows = {steady_time(15.0, 20.0, 0.0)};
  return s;
}

Scenario e09_external_upstream_stop_line()
{
  auto s = external_scenario(
    "External: upstream stop line under an active limit",
    "Ego at the 30 km/h limit; the upstream brakes at 2.0 m/s^2 to a stop line at s=150 m. The "
    "output must keep the upstream stop.",
    "A2, A3");
  s.external_events = {{0.0, make_external_limit(v30)}};
  s.upstream.stop_s = 150.0;
  s.upstream.stop_deceleration = 2.0;
  s.config.initial_state = {0.0, 0.0, v30, 0.0};
  s.config.duration = 30.0;
  s.expectations.check_jerk = false;  // The upstream braking curve starts with a jerk step.
  s.expectations.steady_windows = {steady_time(2.0, 8.0, v30)};
  return s;
}

Scenario e10_external_zero_jerk_constraint()
{
  auto s = external_scenario(
    "External: message constraints with zero jerk",
    "use_constraints with min_jerk 0. Expected: fall back to the nominal jerk instead of "
    "silently ignoring the limit.",
    "E");
  s.external_events = {{2.0, make_external_limit(v30, -2.0, 0.0)}};
  s.config.duration = 25.0;
  s.expectations.steady_windows = {steady_time(15.0, 25.0, v30)};
  return s;
}

Scenario e11_external_zero_deceleration_constraint()
{
  auto s = external_scenario(
    "External: message constraints with zero deceleration",
    "use_constraints with min_acceleration 0. Expected: fall back to the nominal deceleration.",
    "E");
  s.external_events = {{2.0, make_external_limit(v30, 0.0, -0.6)}};
  s.config.duration = 30.0;
  s.expectations.steady_windows = {steady_time(22.0, 30.0, v30)};
  return s;
}

Scenario e12_external_recorded_30kph()
{
  constexpr auto file = "external_30kph_positive_acceleration.csv";
  auto s = recorded_scenario(
    file, "External: recorded 30 km/h limit",
    "Recorded input: constant 60 km/h request while the ego is at 9.20 m/s, +0.20 m/s^2; limit "
    "30 km/h with constraints -2.0 m/s^2 and -0.6 m/s^3.",
    {StageKind::External}, 9.2015, 0.1975);
  s.external_events = {{0.0, make_external_limit(v30, -2.0, -0.6)}};
  s.config.duration = 20.0;
  s.expectations.steady_windows = {steady_time(12.0, 20.0, v30)};
  return s;
}

// ---------------------------------------------------------------------------------------------
// Map velocity limits
// ---------------------------------------------------------------------------------------------

Scenario m01_map_zone_far_ahead()
{
  auto s = map_scenario(
    "Map: 30 km/h zone at 300-450 m while cruising at 60 km/h",
    "The zone should be entered at 30 km/h with braking starting near the latest feasible point; "
    "the ego should resume 60 km/h after the zone.",
    "B");
  s.zones = {{300.0, 450.0, v30, v60}};
  s.config.duration = 65.0;
  s.expectations.steady_windows = {
    steady_distance(330.0, 445.0, v30), steady_distance(620.0, 700.0, v60)};
  return s;
}

Scenario m02_map_zone_appears_close()
{
  auto s = map_scenario(
    "Map: 30 km/h zone starting 40 m ahead",
    "The zone cannot be reached at 30 km/h with the nominal constraints. The entry speed should "
    "match the best reachable one and the ego should settle inside the zone.",
    "B");
  s.zones = {{40.0, 1000.0, v30}};
  s.config.duration = 40.0;
  s.expectations.steady_windows = {steady_distance(250.0, 380.0, v30)};
  return s;
}

Scenario m03_map_short_zone()
{
  auto s = map_scenario(
    "Map: 20 m long 30 km/h zone", "Short zone at 300-320 m inside a 60 km/h road.", "B");
  s.zones = {{300.0, 320.0, v30, v60}};
  s.config.duration = 50.0;
  s.expectations.steady_windows = {steady_distance(520.0, 620.0, v60)};
  return s;
}

Scenario m04_map_stepped_zones()
{
  auto s = map_scenario(
    "Map: 60 -> 40 -> 20 -> 50 km/h zones",
    "Consecutive zones: each zone should be respected at its own entry, not the lowest limit "
    "anywhere in the horizon.",
    "B");
  s.zones = {{200.0, 350.0, v40, v60}, {350.0, 450.0, v20, v40}, {450.0, infinity, v50}};
  s.config.duration = 75.0;
  s.expectations.steady_windows = {
    steady_distance(215.0, 290.0, v40), steady_distance(365.0, 445.0, v20),
    steady_distance(620.0, 720.0, v50)};
  return s;
}

Scenario m05_map_exit_zone()
{
  auto s = map_scenario(
    "Map: leaving a 30 km/h zone",
    "Ego cruising at 30 km/h inside a zone that ends at 100 m; the ego should resume 60 km/h.",
    "release");
  s.zones = {{-50.0, 100.0, v30}};
  s.config.initial_state = {0.0, 0.0, v30, 0.0};
  s.config.duration = 40.0;
  s.expectations.steady_windows = {
    steady_distance(10.0, 95.0, v30), steady_distance(300.0, 400.0, v60)};
  return s;
}

Scenario m06_map_stop_line_in_zone()
{
  auto s = map_scenario(
    "Map: upstream stop line inside a 30 km/h zone",
    "Zone at 200-450 m; the upstream brakes at 2.0 m/s^2 to a stop line at 400 m. The output must "
    "keep the upstream stop.",
    "A2, A3");
  s.zones = {{200.0, 450.0, v30, v60}};
  s.upstream.stop_s = 400.0;
  s.upstream.stop_deceleration = 2.0;
  s.config.duration = 50.0;
  s.expectations.check_jerk = false;  // The upstream braking curve starts with a jerk step.
  s.expectations.steady_windows = {steady_distance(240.0, 360.0, v30)};
  return s;
}

Scenario m07_map_departure_in_zone()
{
  auto s = map_scenario(
    "Map: departure from standstill inside a 30 km/h zone",
    "Ego stopped inside a zone, upstream departing toward 60 km/h.", "A1");
  s.zones = {{-50.0, 300.0, v30}};
  s.config.initial_state = {0.0, 0.0, 0.0, 0.0};
  s.config.duration = 50.0;
  s.expectations.steady_windows = {steady_distance(100.0, 290.0, v30)};
  return s;
}

Scenario m08_map_recorded_30kph_turn()
{
  constexpr auto file = "map_30kph_turn_positive_acceleration.csv";
  auto s = recorded_scenario(
    file, "Map: recorded 30 km/h turn",
    "Recorded input before a 30 km/h turn: constant 60 km/h request, ego at 12.47 m/s, "
    "+0.22 m/s^2.",
    {StageKind::Map}, 12.4737, 0.2214);
  const double boundary = recorded_s(file, s, 66);
  s.zones = {{-100.0, boundary, v60}, {boundary, infinity, v30}};
  s.config.duration = 25.0;
  s.expectations.steady_windows = {steady_distance(boundary + 60.0, boundary + 150.0, v30)};
  return s;
}

Scenario m09_map_recorded_50kph_turn()
{
  constexpr auto file = "map_50kph_turn_positive_acceleration.csv";
  auto s = recorded_scenario(
    file, "Map: recorded 50 km/h turn",
    "Recorded input before a 50 km/h turn: constant 60 km/h request, ego at 15.40 m/s, "
    "+0.22 m/s^2.",
    {StageKind::Map}, 15.4009, 0.2152);
  const double boundary = recorded_s(file, s, 37);
  s.zones = {{-100.0, boundary, v60}, {boundary, infinity, v50}};
  s.config.duration = 25.0;
  s.expectations.steady_windows = {steady_distance(boundary + 80.0, boundary + 200.0, v50)};
  return s;
}

// ---------------------------------------------------------------------------------------------
// External followed by map, as configured in trajectory_modifier.param.yaml
// ---------------------------------------------------------------------------------------------

Scenario c01_chain_external_stricter_than_map()
{
  auto s = base_scenario(
    "Chain: external 20 km/h with a 30 km/h map zone from 60 m",
    "The map stage runs after the external stage and must not undo the stricter external limit.",
    "C", {StageKind::External, StageKind::Map});
  s.external_events = {{0.0, make_external_limit(v20)}};
  s.zones = {{60.0, infinity, v30}};
  s.config.duration = 30.0;
  s.expectations.steady_windows = {steady_time(16.0, 30.0, v20)};
  return s;
}

Scenario c02_chain_map_stricter_than_external()
{
  auto s = base_scenario(
    "Chain: external 40 km/h with a 30 km/h map zone from 200 m",
    "The ego should hold 40 km/h, then brake for the zone near the latest feasible point.", "B, C",
    {StageKind::External, StageKind::Map});
  s.external_events = {{0.0, make_external_limit(v40)}};
  s.zones = {{200.0, infinity, v30, v40}};
  s.config.duration = 40.0;
  s.expectations.steady_windows = {
    steady_distance(100.0, 160.0, v40), steady_distance(230.0, 300.0, v30)};
  return s;
}

// ---------------------------------------------------------------------------------------------
// Robustness
// ---------------------------------------------------------------------------------------------

Scenario r01_coarse_trajectory_sampling()
{
  auto s = e01_external_cruise_60_to_30();
  s.title = "Robustness: 0.2 s point spacing with a 0.1 s planning cycle";
  s.description = "Same as E01 with 40 points spaced by 0.2 s.";
  s.targets = "sampling independence";
  s.upstream.time_stamps = sim::uniform_time_stamps(0.2, 40);
  return s;
}

Scenario r02_noisy_acceleration_feedback()
{
  auto s = e01_external_cruise_60_to_30();
  s.title = "Robustness: noisy acceleration feedback";
  s.description =
    "Same as E01; the acceleration given to the plugin has Gaussian noise (sigma 0.1 m/s^2) while "
    "the follower stays perfect.";
  s.targets = "D (sensitivity)";
  s.config.acceleration_noise_stddev = 0.1;
  return s;
}

// ---------------------------------------------------------------------------------------------
// Fixed profile (velocity_limits.fixed_profile)
// ---------------------------------------------------------------------------------------------

/// @brief A scenario with the default fixed profile parameters and no restart allowed.
Scenario with_fixed_profile(
  Scenario scenario, const std::string & title, const std::string & description)
{
  scenario.title = title;
  scenario.description = description;
  scenario.targets = "fixed profile";
  scenario.fixed_profile = autoware::trajectory_modifier::plugin::detail::FixedProfileParameters{};
  scenario.expectations.max_profile_resets = 0;
  return scenario;
}

Scenario f01_fixed_external_cruise_60_to_30()
{
  return with_fixed_profile(
    e01_external_cruise_60_to_30(), "Fixed profile: cruise 60 km/h, limit 30 km/h at t=2 s",
    "Same as E01 with the fixed profile: every cycle continues the previous plan, without "
    "restart.");
}

Scenario f02_fixed_noisy_acceleration()
{
  return with_fixed_profile(
    r02_noisy_acceleration_feedback(), "Fixed profile: noisy acceleration feedback",
    "Same as R02 with the fixed profile: the plans no longer start from the noisy measurement, so "
    "the acceleration does not chatter.");
}

Scenario f03_fixed_tracking_disturbance()
{
  auto s = with_fixed_profile(
    e01_external_cruise_60_to_30(), "Fixed profile: vehicle braking harder than planned",
    "Same as E01 with a follower that tracks the planned velocity (gain 0.5/s) and an extra "
    "-0.4 m/s^2 between t=4 and 8 s. The velocity falls behind the plan until it deviates by more "
    "than 0.5 m/s; the profile then restarts from the measured state and is fixed again.");
  s.config.tracking_gain = 0.5;
  s.config.disturbances = {{4.0, 8.0, -0.4}};
  s.expectations.check_executed_constraints = false;  // The disturbance is outside the plan.
  // The input starts at the measured velocity, which may lag the fixed plan up to the threshold.
  s.expectations.max_velocity_increase = 0.55;
  s.expectations.max_profile_resets = 3;
  s.expectations.expected_reset_windows = {{5.0, 8.5}};
  return s;
}

Scenario f04_fixed_map_stepped_zones()
{
  return with_fixed_profile(
    m04_map_stepped_zones(), "Fixed profile: 60 -> 40 -> 20 -> 50 km/h zones",
    "Same as M04 with the fixed profile.");
}

Scenario f05_fixed_limit_lowered_while_braking()
{
  auto s = with_fixed_profile(
    e01_external_cruise_60_to_30(), "Fixed profile: limit lowered to 20 km/h while braking",
    "Limit 30 km/h at t=2 s, then 20 km/h at t=6 s during the braking. The limits are evaluated "
    "every cycle, so the fixed profile follows the change without a restart.");
  s.external_events = {{2.0, make_external_limit(v30)}, {6.0, make_external_limit(v20)}};
  s.expectations.steady_windows = {steady_time(20.0, 30.0, v20)};
  return s;
}

Scenario f06_fixed_map_recorded_30kph_turn()
{
  return with_fixed_profile(
    m08_map_recorded_30kph_turn(), "Fixed profile: recorded 30 km/h turn",
    "Same as M08 with the fixed profile.");
}

Scenario f07_fixed_chain_external_stricter_than_map()
{
  return with_fixed_profile(
    c01_chain_external_stricter_than_map(), "Fixed profile: external 20 km/h and map 30 km/h",
    "Same as C01 with the fixed profile; each stage keeps its own previous plan.");
}

struct ScenarioCase
{
  std::string name;
  std::function<Scenario()> make;
};

void PrintTo(const ScenarioCase & scenario_case, std::ostream * stream)
{
  *stream << scenario_case.name;
}

std::vector<ScenarioCase> scenario_cases()
{
  return {
    {"E01_external_cruise_60_to_30", e01_external_cruise_60_to_30},
    {"E02_external_message_constraints", e02_external_message_constraints},
    {"E03_external_accelerating_below_limit", e03_external_accelerating_below_limit},
    {"E04_external_accelerating_above_limit", e04_external_accelerating_above_limit},
    {"E05_external_slow_ego_below_limit", e05_external_slow_ego_below_limit},
    {"E06_external_departure_from_standstill", e06_external_departure_from_standstill},
    {"E07_external_limit_raised", e07_external_limit_raised},
    {"E08_external_limit_to_zero", e08_external_limit_to_zero},
    {"E09_external_upstream_stop_line", e09_external_upstream_stop_line},
    {"E10_external_zero_jerk_constraint", e10_external_zero_jerk_constraint},
    {"E11_external_zero_deceleration_constraint", e11_external_zero_deceleration_constraint},
    {"E12_external_recorded_30kph", e12_external_recorded_30kph},
    {"M01_map_zone_far_ahead", m01_map_zone_far_ahead},
    {"M02_map_zone_appears_close", m02_map_zone_appears_close},
    {"M03_map_short_zone", m03_map_short_zone},
    {"M04_map_stepped_zones", m04_map_stepped_zones},
    {"M05_map_exit_zone", m05_map_exit_zone},
    {"M06_map_stop_line_in_zone", m06_map_stop_line_in_zone},
    {"M07_map_departure_in_zone", m07_map_departure_in_zone},
    {"M08_map_recorded_30kph_turn", m08_map_recorded_30kph_turn},
    {"M09_map_recorded_50kph_turn", m09_map_recorded_50kph_turn},
    {"C01_chain_external_stricter_than_map", c01_chain_external_stricter_than_map},
    {"C02_chain_map_stricter_than_external", c02_chain_map_stricter_than_external},
    {"R01_coarse_trajectory_sampling", r01_coarse_trajectory_sampling},
    {"R02_noisy_acceleration_feedback", r02_noisy_acceleration_feedback},
    {"F01_fixed_external_cruise_60_to_30", f01_fixed_external_cruise_60_to_30},
    {"F02_fixed_noisy_acceleration", f02_fixed_noisy_acceleration},
    {"F03_fixed_tracking_disturbance", f03_fixed_tracking_disturbance},
    {"F04_fixed_map_stepped_zones", f04_fixed_map_stepped_zones},
    {"F05_fixed_limit_lowered_while_braking", f05_fixed_limit_lowered_while_braking},
    {"F06_fixed_map_recorded_30kph_turn", f06_fixed_map_recorded_30kph_turn},
    {"F07_fixed_chain_external_stricter_than_map", f07_fixed_chain_external_stricter_than_map},
  };
}

class VelocityLimitSimulation : public ::testing::TestWithParam<ScenarioCase>
{
};

TEST_P(VelocityLimitSimulation, ClosedLoop)
{
  auto scenario = GetParam().make();
  scenario.name = GetParam().name;
  sim::run_and_expect(scenario, sim::make_helper_stages(scenario));
}

INSTANTIATE_TEST_SUITE_P(
  Scenarios, VelocityLimitSimulation, ::testing::ValuesIn(scenario_cases()),
  [](const ::testing::TestParamInfo<ScenarioCase> & info) { return info.param.name; });

}  // namespace
