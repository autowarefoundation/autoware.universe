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

#ifndef PLANNING__AUTOWARE_TRAJECTORY_MODIFIER__TESTS__VELOCITY_LIMIT_SIMULATION_HPP_
#define PLANNING__AUTOWARE_TRAJECTORY_MODIFIER__TESTS__VELOCITY_LIMIT_SIMULATION_HPP_

// Closed-loop harness for the external and map velocity limit plugins. Every planning cycle an
// upstream planner model emits a time-sampled trajectory from the current ego state, the plugin
// chain modifies it, and a perfect follower executes the result for one cycle. The executed
// velocity, acceleration and jerk are written to CSV files and rendered by
// plot_velocity_limit_simulation.py.

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/external_velocity_limit.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::test::velocity_limit_simulation
{
using autoware_internal_planning_msgs::msg::VelocityLimit;
using plugin::ProcessingResult;
using plugin::TrajectoryPoints;

constexpr double kmph(const double value)
{
  return value / 3.6;
}

inline double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).seconds();
}

inline geometry_msgs::msg::Point make_point(const double x, const double y, const double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

// ---------------------------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------------------------

/// @brief Polyline parameterized by arc length; queries before the start or past the end
/// extrapolate along the first or last segment.
class ReferencePath
{
public:
  explicit ReferencePath(const std::vector<geometry_msgs::msg::Point> & points)
  {
    for (const auto & point : points) {
      if (
        !points_.empty() &&
        std::hypot(point.x - points_.back().x, point.y - points_.back().y) < 1e-6) {
        continue;
      }
      points_.push_back(point);
    }
    if (points_.size() < 2) {
      throw std::invalid_argument("A reference path requires at least two distinct points");
    }
    arc_lengths_.assign(points_.size(), 0.0);
    for (std::size_t i = 1; i < points_.size(); ++i) {
      const auto & p0 = points_[i - 1];
      const auto & p1 = points_[i];
      arc_lengths_[i] = arc_lengths_[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    }
  }

  static ReferencePath straight()
  {
    return ReferencePath({make_point(0.0, 0.0), make_point(1.0, 0.0)});
  }

  /// @brief Recorded geometry followed by a straight extension along its final heading.
  static ReferencePath extended(
    std::vector<geometry_msgs::msg::Point> points, const double extension)
  {
    const ReferencePath recorded(points);
    const auto & last = recorded.points_.back();
    const auto & before = recorded.points_[recorded.points_.size() - 2];
    const double length = std::hypot(last.x - before.x, last.y - before.y);
    points.push_back(make_point(
      last.x + extension * (last.x - before.x) / length,
      last.y + extension * (last.y - before.y) / length, last.z));
    return ReferencePath(points);
  }

  [[nodiscard]] geometry_msgs::msg::Pose pose_at(const double s) const
  {
    std::size_t segment = 0;
    if (s >= arc_lengths_.back()) {
      segment = points_.size() - 2;
    } else if (s > 0.0) {
      segment = static_cast<std::size_t>(
        std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s) - arc_lengths_.begin() - 1);
    }
    const auto & p0 = points_[segment];
    const auto & p1 = points_[segment + 1];
    const double ratio =
      (s - arc_lengths_[segment]) / (arc_lengths_[segment + 1] - arc_lengths_[segment]);
    geometry_msgs::msg::Pose pose;
    pose.position = make_point(
      p0.x + ratio * (p1.x - p0.x), p0.y + ratio * (p1.y - p0.y), p0.z + ratio * (p1.z - p0.z));
    const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    pose.orientation.z = std::sin(0.5 * yaw);
    pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
  }

  /// @brief Arc length of the closest point on the path (2D).
  [[nodiscard]] double project(const geometry_msgs::msg::Point & point) const
  {
    double best_distance = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
      const auto & p0 = points_[i];
      const auto & p1 = points_[i + 1];
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      double ratio = ((point.x - p0.x) * dx + (point.y - p0.y) * dy) / (dx * dx + dy * dy);
      if (i > 0) {
        ratio = std::max(ratio, 0.0);
      }
      if (i + 2 < points_.size()) {
        ratio = std::min(ratio, 1.0);
      }
      const double distance =
        std::hypot(point.x - (p0.x + ratio * dx), point.y - (p0.y + ratio * dy));
      if (distance < best_distance) {
        best_distance = distance;
        best_s = arc_lengths_[i] + ratio * (arc_lengths_[i + 1] - arc_lengths_[i]);
      }
    }
    return best_s;
  }

  [[nodiscard]] double arc_length_of_vertex(const std::size_t index) const
  {
    return arc_lengths_.at(index);
  }

private:
  std::vector<geometry_msgs::msg::Point> points_;
  std::vector<double> arc_lengths_;
};

// ---------------------------------------------------------------------------------------------
// Longitudinal dynamics
// ---------------------------------------------------------------------------------------------

struct JerkLimits
{
  double max_acceleration{1.0};
  double max_deceleration{1.0};
  double jerk{1.0};
};

/// @brief Advance a jerk-limited velocity tracker by one step. The acceleration command uses the
/// velocity reached after releasing the current acceleration, so the approach does not overshoot.
inline void track_velocity(
  double & velocity, double & acceleration, const double target, const JerkLimits & limits,
  const double dt, const double gain)
{
  const double release_velocity =
    velocity + acceleration * std::abs(acceleration) / (2.0 * limits.jerk);
  const double desired = std::clamp(
    gain * (target - release_velocity), -limits.max_deceleration, limits.max_acceleration);
  const double next =
    std::clamp(desired, acceleration - limits.jerk * dt, acceleration + limits.jerk * dt);
  velocity += 0.5 * (acceleration + next) * dt;
  acceleration = next;
  if (velocity <= 0.0) {
    velocity = 0.0;
    acceleration = std::max(acceleration, 0.0);
  }
}

struct ProfilePoint
{
  double time{0.0};
  double distance{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

/// @brief Reference transition from (v0, a0) to the target speed. It is close to time-optimal
/// under the given acceleration, deceleration and jerk limits.
inline std::vector<ProfilePoint> simulate_transition(
  const double v0, const double a0, const double target, const JerkLimits & limits,
  const double max_time = 120.0)
{
  constexpr double dt = 0.01;
  constexpr double gain = 10.0;
  std::vector<ProfilePoint> profile{{0.0, 0.0, v0, a0}};
  double velocity = v0;
  double acceleration = a0;
  double distance = 0.0;
  for (double time = dt; time <= max_time; time += dt) {
    const double previous = velocity;
    track_velocity(velocity, acceleration, target, limits, dt, gain);
    distance += 0.5 * (previous + velocity) * dt;
    profile.push_back({time, distance, velocity, acceleration});
    if (std::abs(velocity - target) < 1e-3 && std::abs(acceleration) < 1e-3) {
      break;
    }
  }
  return profile;
}

/// @brief First profile point after which the velocity never exceeds target + tolerance.
inline const ProfilePoint & settled_point(
  const std::vector<ProfilePoint> & profile, const double target, const double tolerance)
{
  for (std::size_t i = profile.size(); i > 0; --i) {
    if (profile[i - 1].velocity > target + tolerance) {
      return profile[std::min(i, profile.size() - 1)];
    }
  }
  return profile.front();
}

inline double velocity_at_distance(const std::vector<ProfilePoint> & profile, const double distance)
{
  for (std::size_t i = 1; i < profile.size(); ++i) {
    if (profile[i].distance >= distance) {
      const auto & p0 = profile[i - 1];
      const auto & p1 = profile[i];
      const double span = p1.distance - p0.distance;
      const double ratio = span > 0.0 ? (distance - p0.distance) / span : 1.0;
      return p0.velocity + ratio * (p1.velocity - p0.velocity);
    }
  }
  return profile.back().velocity;
}

// ---------------------------------------------------------------------------------------------
// Scenario description
// ---------------------------------------------------------------------------------------------

struct EgoState
{
  double time{0.0};
  double s{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

enum class UpstreamProfile {
  ConstantSpeed,  ///< Every point at the cruise speed, whatever the ego state (recorded inputs).
  EgoAnchored,    ///< Starts at the ego state and converges to the cruise speed with limited jerk.
};

/// @brief Model of the planner that feeds the trajectory modifier.
struct UpstreamPlanner
{
  UpstreamProfile profile{UpstreamProfile::EgoAnchored};
  double cruise_velocity{kmph(60.0)};
  JerkLimits limits{1.0, 1.0, 3.0};
  std::optional<double> stop_s;     ///< Stop line arc length; capped by a braking curve.
  double stop_deceleration{1.0};    ///< Deceleration of the braking curve to the stop line.
  std::vector<double> time_stamps;  ///< Point times relative to the ego state, first one is 0.
  bool terminal_zero_velocity{false};

  [[nodiscard]] TrajectoryPoints generate(const ReferencePath & path, const EgoState & ego) const
  {
    constexpr double gain = 2.0;
    const bool anchored = profile == UpstreamProfile::EgoAnchored;
    // An anchored upstream requests the cruise speed; it keeps a positive ego acceleration but does
    // not inherit braking caused by the velocity limits.
    double velocity = anchored ? ego.velocity : cruise_velocity;
    double acceleration = anchored ? std::max(0.0, ego.acceleration) : 0.0;
    double s = ego.s;
    TrajectoryPoints points(time_stamps.size());
    for (std::size_t i = 0; i < time_stamps.size(); ++i) {
      const double dt = i == 0 ? 0.0 : time_stamps[i] - time_stamps[i - 1];
      const double previous_velocity = velocity;
      if (i > 0 && anchored) {
        track_velocity(velocity, acceleration, cruise_velocity, limits, dt, gain);
      }
      if (i + 1 == time_stamps.size() && terminal_zero_velocity) {
        velocity = 0.0;
        acceleration = 0.0;
      }
      double point_s = s + 0.5 * (previous_velocity + velocity) * dt;
      if (stop_s) {
        // Constant deceleration along the braking curve; once stopped, the planner stays stopped.
        const bool stopped = i > 0 && previous_velocity <= 0.0;
        const double cap = std::sqrt(2.0 * stop_deceleration * std::max(0.0, *stop_s - point_s));
        if (stopped || velocity > cap) {
          velocity = stopped ? 0.0 : std::max(cap, previous_velocity - stop_deceleration * dt);
          acceleration = velocity > 0.0 ? -stop_deceleration : 0.0;
          point_s = s + 0.5 * (previous_velocity + velocity) * dt;
        }
      }
      s = point_s;
      auto & point = points[i];
      point.time_from_start = rclcpp::Duration::from_seconds(time_stamps[i]);
      point.pose = path.pose_at(s);
      point.longitudinal_velocity_mps = static_cast<float>(velocity);
      point.acceleration_mps2 = static_cast<float>(acceleration);
    }
    return points;
  }
};

inline std::vector<double> uniform_time_stamps(const double step, const std::size_t count)
{
  std::vector<double> times(count);
  for (std::size_t i = 0; i < count; ++i) {
    times[i] = step * static_cast<double>(i);
  }
  return times;
}

/// @brief Map speed zone in arc length along the reference path.
struct SpeedZone
{
  SpeedZone(
    const double begin, const double end, const double max_velocity,
    const std::optional<double> approach = std::nullopt)
  : begin_s{begin}, end_s{end}, limit{max_velocity}, approach_velocity{approach}
  {
  }

  double begin_s{0.0};
  double end_s{std::numeric_limits<double>::infinity()};
  double limit{0.0};
  /// When set, the ego is expected to hold this speed until the latest feasible braking point.
  std::optional<double> approach_velocity;
};

/// @brief External limit becoming active at `time`; no message means that no limit is published.
struct ExternalLimitEvent
{
  double time{0.0};
  std::optional<VelocityLimit> message;
};

inline VelocityLimit make_external_limit(
  const double max_velocity, const std::optional<double> min_acceleration = std::nullopt,
  const std::optional<double> min_jerk = std::nullopt)
{
  VelocityLimit message;
  message.max_velocity = static_cast<float>(max_velocity);
  message.use_constraints = min_acceleration.has_value() || min_jerk.has_value();
  message.constraints.min_acceleration = static_cast<float>(min_acceleration.value_or(0.0));
  message.constraints.min_jerk = static_cast<float>(min_jerk.value_or(0.0));
  return message;
}

enum class StageKind { External, Map };

enum class WindowDomain { Time, Distance };

/// @brief Interval in which the ego must have settled at the target speed.
struct SteadyWindow
{
  WindowDomain domain{WindowDomain::Time};
  double begin{0.0};
  double end{0.0};
  double target{0.0};
};

struct Expectations
{
  double velocity_tolerance{0.1};  ///< Allowed overspeed once a limit is feasible [m/s].
  double settle_margin{1.0};       ///< Allowed delay beyond the reference settle time [s].
  double steady_velocity_tolerance{0.15};
  double steady_acceleration_tolerance{0.1};
  std::size_t max_jerk_sign_changes{4};   ///< In a steady window; more indicates chattering.
  double premature_braking_margin{10.0};  ///< Allowed braking ahead of the latest point [m].
  double max_velocity_increase{0.05};     ///< Output above input at the same arc length [m/s].
  double max_pose_error{0.5};             ///< Pose arc length vs. integrated velocity [m].
  bool check_jerk{true};                  ///< Disabled when the upstream itself has a jerk step.
  /// Bounds on the executed motion; disabled when the follower adds disturbances. The plans are
  /// always checked.
  bool check_executed_constraints{true};
  std::vector<SteadyWindow> steady_windows;
  std::optional<std::size_t> max_profile_resets;  ///< Fixed profile restarts allowed.
  /// [begin, end] time windows in each of which a fixed profile restart is expected.
  std::vector<std::pair<double, double>> expected_reset_windows;
};

/// @brief Acceleration added to the executed motion between `begin` and `end`, ramped in and out
/// over `ramp` (e.g. a slope or an actuator error).
struct AccelerationDisturbance
{
  double begin{0.0};
  double end{0.0};
  double value{0.0};
  double ramp{0.2};

  [[nodiscard]] double at(const double time) const
  {
    if (time <= begin || time >= end) {
      return 0.0;
    }
    return value * std::min({1.0, (time - begin) / ramp, (end - time) / ramp});
  }
};

struct SimulationConfig
{
  double duration{30.0};
  double cycle_period{0.1};
  double sample_period{0.01};
  std::size_t plan_log_stride{5};
  double acceleration_noise_stddev{0.0};  ///< Noise on the acceleration given to the plugins.
  unsigned int noise_seed{42U};
  EgoState initial_state;
  /// Without a gain, the follower drives each plan exactly. With one, it tracks the planned
  /// velocity with this feedback gain [1/s], so it can deviate from a plan that does not start at
  /// the ego state, and the disturbances apply.
  std::optional<double> tracking_gain;
  std::vector<AccelerationDisturbance> disturbances;
};

struct Scenario
{
  std::string name;
  std::string title;
  std::string description;
  std::string targets;  ///< Review findings the scenario exercises.
  std::string level{"helper"};
  ReferencePath path{ReferencePath::straight()};
  UpstreamPlanner upstream;
  std::vector<SpeedZone> zones;
  std::vector<ExternalLimitEvent> external_events;
  std::vector<StageKind> stages;
  /// Plugin parameters velocity_limits.max_acceleration, max_deceleration and max_jerk.
  plugin::detail::VelocityLimitConstraints constraints{1.0, 1.0, 3.0};
  /// Plugin parameters velocity_limits.fixed_profile; disabled when not set.
  std::optional<plugin::detail::FixedProfileParameters> fixed_profile;
  SimulationConfig config;
  Expectations expectations;

  [[nodiscard]] std::optional<VelocityLimit> external_limit_at(const double time) const
  {
    std::optional<VelocityLimit> active;
    for (const auto & event : external_events) {
      if (event.time <= time + 1e-9) {
        active = event.message;
      }
    }
    return active;
  }

  [[nodiscard]] std::optional<double> zone_limit_at(const double s) const
  {
    std::optional<double> limit;
    for (const auto & zone : zones) {
      if (s >= zone.begin_s && s < zone.end_s) {
        limit = limit ? std::min(*limit, zone.limit) : zone.limit;
      }
    }
    return limit;
  }

  /// @brief Ground-truth limit at the ego, independent of how the plugins resolve it.
  [[nodiscard]] std::optional<double> expected_limit(const double time, const double s) const
  {
    auto limit = zone_limit_at(s);
    if (const auto external = external_limit_at(time)) {
      const double value = external->max_velocity;
      limit = limit ? std::min(*limit, value) : value;
    }
    return limit;
  }

  /// @brief Deceleration and jerk that an external limit is expected to use. Zero message
  /// constraints fall back to the parameters, as they would otherwise disable braking.
  [[nodiscard]] JerkLimits expected_external_constraints(const VelocityLimit & message) const
  {
    JerkLimits limits = map_constraints();
    if (message.use_constraints) {
      if (std::abs(message.constraints.min_acceleration) > 0.0F) {
        limits.max_deceleration = std::abs(message.constraints.min_acceleration);
      }
      if (std::abs(message.constraints.min_jerk) > 0.0F) {
        limits.jerk = std::abs(message.constraints.min_jerk);
      }
    }
    return limits;
  }

  [[nodiscard]] JerkLimits map_constraints() const
  {
    return {constraints.max_acceleration, constraints.max_deceleration, constraints.max_jerk};
  }
};

// ---------------------------------------------------------------------------------------------
// Plugin stages
// ---------------------------------------------------------------------------------------------

struct StageOutcome
{
  ProcessingResult status{ProcessingResult::Unchanged};
  std::string start{"-"};  ///< How the profile was started (see plugin::detail::ProfileStart).
};

/// @brief One entry of the modifier chain; receives the (possibly noisy) measured ego state.
struct Stage
{
  std::string name;
  std::function<StageOutcome(TrajectoryPoints &, const EgoState &)> process;
};

inline plugin::detail::VelocityLimitOptions make_options(const EgoState & ego)
{
  plugin::detail::VelocityLimitOptions options;
  options.current_ego_velocity = ego.velocity;
  options.current_ego_acceleration = ego.acceleration;
  return options;
}

using Memory = std::shared_ptr<plugin::detail::FixedProfileMemory>;

inline Memory make_memory(const Scenario & scenario)
{
  if (!scenario.fixed_profile) {
    return nullptr;
  }
  auto memory = std::make_shared<plugin::detail::FixedProfileMemory>();
  memory->set_parameters(*scenario.fixed_profile);
  return memory;
}

/// @brief Same flow as the plugins: start from the previous plan when the fixed profile allows
/// it, limit, and remember the output.
inline StageOutcome limit_with_memory(
  const Scenario & scenario, const Memory & memory, TrajectoryPoints & points, const EgoState & ego,
  const plugin::detail::VelocityLimitConstraints & constraints,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & limit)
{
  plugin::detail::ProfileStartResult start{
    make_options(ego), plugin::detail::ProfileStart::Measured};
  if (memory) {
    start = memory->start(0, ego.time, scenario.path.pose_at(ego.s).position, make_options(ego));
  }
  const auto result =
    plugin::detail::apply_velocity_limits(points, constraints, limit, start.options);
  if (memory) {
    memory->store(0, 1, ego.time, points);
  }
  return {result.status, plugin::detail::to_string(start.start)};
}

/// @brief Same computation as ExternalVelocityLimit::process() for the message active at the ego
/// time.
inline Stage make_external_stage(const Scenario & scenario)
{
  return {
    "external",
    [&scenario, memory = make_memory(scenario)](TrajectoryPoints & points, const EgoState & ego) {
      const auto message = scenario.external_limit_at(ego.time);
      if (!message || !std::isfinite(message->max_velocity) || message->max_velocity < 0.0F) {
        if (memory) {
          memory->store(0, 1, ego.time, points);
        }
        return StageOutcome{};
      }
      auto constraints = scenario.constraints;
      constraints.max_deceleration = plugin::detail::get_external_velocity_limit_deceleration(
        *message, scenario.constraints.max_deceleration);
      constraints.max_jerk = plugin::detail::get_external_velocity_limit_min_jerk(
        *message, scenario.constraints.max_jerk);
      const double max_velocity = message->max_velocity;
      return limit_with_memory(
        scenario, memory, points, ego, constraints,
        [max_velocity](const geometry_msgs::msg::Point &) {
          return std::optional<double>{max_velocity};
        });
    }};
}

/// @brief Same computation as MapVelocityLimits::process(), resolving the lanelet limit from the
/// scenario zones by projecting each point onto the reference path.
inline Stage make_map_stage(const Scenario & scenario)
{
  return {
    "map",
    [&scenario, memory = make_memory(scenario)](TrajectoryPoints & points, const EgoState & ego) {
      return limit_with_memory(
        scenario, memory, points, ego, scenario.constraints,
        [&scenario](const geometry_msgs::msg::Point & position) {
          return scenario.zone_limit_at(scenario.path.project(position));
        });
    }};
}

inline std::vector<Stage> make_helper_stages(const Scenario & scenario)
{
  std::vector<Stage> stages;
  for (const auto kind : scenario.stages) {
    stages.push_back(
      kind == StageKind::External ? make_external_stage(scenario) : make_map_stage(scenario));
  }
  return stages;
}

// ---------------------------------------------------------------------------------------------
// Closed loop
// ---------------------------------------------------------------------------------------------

struct Sample
{
  double time{0.0};
  double s{0.0};
  double velocity{0.0};
  double acceleration{0.0};          ///< Slope of the executed velocity segment.
  double plan_acceleration{0.0};     ///< acceleration_mps2 field of the plan, for comparison.
  double implied_acceleration{0.0};  ///< Velocity difference over one cycle (shows steps).
  double jerk{0.0};                  ///< Acceleration change over one cycle.
  double upstream_velocity{0.0};     ///< Upstream request at the same time.
  std::optional<double> expected_limit;
  std::size_t cycle{0U};
};

struct StageRecord
{
  ProcessingResult status{ProcessingResult::Unchanged};
  std::string start;
  double max_velocity_increase{0.0};  ///< Output above input at the same arc length.
};

/// @brief Acceleration bounds and jerk of a plan, computed from its points.
struct PlanMotion
{
  double min_acceleration{0.0};
  double max_acceleration{0.0};
  double max_jerk{0.0};
};

struct CycleRecord
{
  std::size_t index{0U};
  EgoState ego;
  double input_end_s{0.0};
  double pose_error{0.0};
  std::vector<StageRecord> stages;
  std::optional<PlanMotion> plan;  ///< Of the output, when a stage modified it.
};

struct PlanRecord
{
  std::size_t cycle{0U};
  double time{0.0};
  TrajectoryPoints input;
  TrajectoryPoints output;
};

struct SimulationLog
{
  std::vector<std::string> stage_names;
  std::vector<Sample> samples;
  std::vector<CycleRecord> cycles;
  std::vector<PlanRecord> plans;
};

struct PlanState
{
  double velocity{0.0};
  double acceleration{0.0};  ///< Interpolated acceleration_mps2 field.
  double distance{0.0};
  double executed_acceleration{0.0};  ///< Acceleration of the motion driven by the follower.
};

/// @brief Whether the acceleration field holds the acceleration at each point (velocities follow
/// its trapezoidal integral) rather than over the segment starting there (Euler integral).
inline bool has_point_accelerations(const TrajectoryPoints & points)
{
  double trapezoid_error = 0.0;
  double euler_error = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double h = seconds(points[i + 1].time_from_start) - seconds(points[i].time_from_start);
    const double dv = points[i + 1].longitudinal_velocity_mps - points[i].longitudinal_velocity_mps;
    const double a0 = points[i].acceleration_mps2;
    const double a1 = points[i + 1].acceleration_mps2;
    trapezoid_error += std::abs(dv - 0.5 * (a0 + a1) * h);
    euler_error += std::abs(dv - a0 * h);
  }
  return trapezoid_error <= euler_error;
}

/// @brief Motion over one plan segment, a duration `tau` after its first point. With point
/// accelerations, the follower drives the linearly interpolated acceleration field (the jerk of
/// the plan); otherwise it drives the linearly interpolated velocity and its acceleration is the
/// slope. The follower thus executes what each plan describes, whatever its convention.
inline PlanState sample_segment(
  const TrajectoryPoints::value_type & first, const TrajectoryPoints::value_type & second,
  const double duration, const double tau, const bool point_accelerations)
{
  const double v0 = first.longitudinal_velocity_mps;
  const double v1 = second.longitudinal_velocity_mps;
  const double a0 = first.acceleration_mps2;
  const double a1 = second.acceleration_mps2;
  const double h = duration;
  const double field = h > 0.0 ? a0 + tau / h * (a1 - a0) : a1;
  if (!point_accelerations || h <= 0.0) {
    const double velocity = h > 0.0 ? v0 + tau / h * (v1 - v0) : v1;
    return {velocity, field, 0.5 * (v0 + velocity) * tau, h > 0.0 ? (v1 - v0) / h : 0.0};
  }
  const double jerk = (a1 - a0) / h;
  return {
    v0 + a0 * tau + 0.5 * jerk * tau * tau, field,
    v0 * tau + 0.5 * a0 * tau * tau + jerk * tau * tau * tau / 6.0, field};
}

/// @brief Plan state at a time after its first point.
inline PlanState sample_plan(
  const TrajectoryPoints & points, const double time, const bool point_accelerations)
{
  double distance = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double t0 = seconds(points[i].time_from_start);
    const double t1 = seconds(points[i + 1].time_from_start);
    if (time <= t1) {
      auto state = sample_segment(
        points[i], points[i + 1], t1 - t0, std::max(0.0, time - t0), point_accelerations);
      state.distance += distance;
      return state;
    }
    distance +=
      sample_segment(points[i], points[i + 1], t1 - t0, t1 - t0, point_accelerations).distance;
  }
  const auto & last = points.back();
  const double remaining = std::max(0.0, time - seconds(last.time_from_start));
  return {
    last.longitudinal_velocity_mps, last.acceleration_mps2,
    distance + last.longitudinal_velocity_mps * remaining, 0.0};
}

/// @brief Largest amount by which `after` is faster than `before` at the same arc length. Like the
/// velocity limit plugins, a velocity drop at the last point of `before` is ignored (horizon end,
/// not a stop).
inline double max_velocity_increase(
  const ReferencePath & path, const TrajectoryPoints & before, const TrajectoryPoints & after)
{
  std::vector<std::pair<double, double>> input;
  for (std::size_t i = 0; i < before.size(); ++i) {
    double velocity = before[i].longitudinal_velocity_mps;
    if (i > 0 && i + 1 == before.size()) {
      velocity = std::max(velocity, static_cast<double>(before[i - 1].longitudinal_velocity_mps));
    }
    input.emplace_back(path.project(before[i].pose.position), velocity);
  }
  const auto input_at = [&input](const double s) {
    if (s <= input.front().first) {
      return input.front().second;
    }
    for (std::size_t i = 1; i < input.size(); ++i) {
      if (s <= input[i].first) {
        const auto & [s0, v0] = input[i - 1];
        const auto & [s1, v1] = input[i];
        return s1 > s0 ? v0 + (s - s0) / (s1 - s0) * (v1 - v0) : v1;
      }
    }
    return input.back().second;
  };
  double increase = 0.0;
  for (const auto & point : after) {
    increase = std::max(
      increase, point.longitudinal_velocity_mps - input_at(path.project(point.pose.position)));
  }
  return increase;
}

inline PlanMotion plan_motion(const TrajectoryPoints & points)
{
  PlanMotion motion{
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 0.0};
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double acceleration = points[i].acceleration_mps2;
    motion.min_acceleration = std::min(motion.min_acceleration, acceleration);
    motion.max_acceleration = std::max(motion.max_acceleration, acceleration);
    if (i > 0) {
      const double dt = seconds(points[i].time_from_start) - seconds(points[i - 1].time_from_start);
      motion.max_jerk =
        std::max(motion.max_jerk, std::abs(acceleration - points[i - 1].acceleration_mps2) / dt);
    }
  }
  return motion;
}

/// @brief Largest gap between each point's arc length and the distance implied by its speed.
inline double pose_error(
  const ReferencePath & path, const TrajectoryPoints & points, const double s0)
{
  const bool point_accelerations = has_point_accelerations(points);
  double error = 0.0;
  for (const auto & point : points) {
    const double driven =
      sample_plan(points, seconds(point.time_from_start), point_accelerations).distance;
    error = std::max(error, std::abs(path.project(point.pose.position) - s0 - driven));
  }
  return error;
}

inline SimulationLog run_closed_loop(const Scenario & scenario, const std::vector<Stage> & stages)
{
  const auto & config = scenario.config;
  SimulationLog log;
  for (const auto & stage : stages) {
    log.stage_names.push_back(stage.name);
  }
  std::mt19937 generator(config.noise_seed);
  std::normal_distribution<double> noise(0.0, std::max(config.acceleration_noise_stddev, 1e-12));

  const auto cycle_count =
    static_cast<std::size_t>(std::llround(config.duration / config.cycle_period));
  const auto sub_steps =
    static_cast<std::size_t>(std::llround(config.cycle_period / config.sample_period));
  EgoState ego = config.initial_state;
  for (std::size_t cycle = 0; cycle < cycle_count; ++cycle) {
    const auto input = scenario.upstream.generate(scenario.path, ego);
    auto output = input;
    EgoState measured = ego;
    if (config.acceleration_noise_stddev > 0.0) {
      measured.acceleration += noise(generator);
    }

    CycleRecord record;
    record.index = cycle;
    record.ego = ego;
    record.input_end_s = scenario.path.project(input.back().pose.position);
    bool modified = false;
    for (const auto & stage : stages) {
      const auto before = output;
      StageRecord stage_record;
      const auto outcome = stage.process(output, measured);
      stage_record.status = outcome.status;
      stage_record.start = outcome.start;
      stage_record.max_velocity_increase = max_velocity_increase(scenario.path, before, output);
      modified |= outcome.status == ProcessingResult::Modified;
      record.stages.push_back(stage_record);
    }
    if (modified) {
      record.plan = plan_motion(output);
    }
    record.pose_error = pose_error(scenario.path, output, ego.s);
    log.cycles.push_back(record);
    if (cycle % config.plan_log_stride == 0) {
      log.plans.push_back({cycle, ego.time, input, output});
    }

    const bool output_points = has_point_accelerations(output);
    const bool input_points = has_point_accelerations(input);
    if (cycle == 0) {
      Sample sample;
      sample.time = ego.time;
      sample.s = ego.s;
      sample.velocity = ego.velocity;
      sample.acceleration = ego.acceleration;
      sample.plan_acceleration = sample_plan(output, 0.0, output_points).acceleration;
      sample.upstream_velocity = sample_plan(input, 0.0, input_points).velocity;
      log.samples.push_back(sample);
    }
    EgoState tracked = ego;
    for (std::size_t step = 1; step <= sub_steps; ++step) {
      const double tau = config.sample_period * static_cast<double>(step);
      const auto state = sample_plan(output, tau, output_points);
      Sample sample;
      sample.time = ego.time + tau;
      if (config.tracking_gain) {
        // Feedback on the planned velocity, plus disturbances, integrated over the sample period.
        double disturbance = 0.0;
        for (const auto & event : config.disturbances) {
          disturbance += event.at(sample.time);
        }
        const double acceleration = state.executed_acceleration +
                                    *config.tracking_gain * (state.velocity - tracked.velocity) +
                                    disturbance;
        const double velocity =
          std::max(0.0, tracked.velocity + acceleration * config.sample_period);
        tracked = {
          sample.time, tracked.s + 0.5 * (tracked.velocity + velocity) * config.sample_period,
          velocity, acceleration};
        sample.s = tracked.s;
        sample.velocity = tracked.velocity;
        sample.acceleration = tracked.acceleration;
      } else {
        sample.s = ego.s + state.distance;
        sample.velocity = state.velocity;
        sample.acceleration = state.executed_acceleration;
      }
      sample.plan_acceleration = state.acceleration;
      sample.upstream_velocity = sample_plan(input, tau, input_points).velocity;
      sample.cycle = cycle;
      log.samples.push_back(sample);
    }
    const auto end = sample_plan(output, config.cycle_period, output_points);
    const auto ego_state_with_tracking_gain =
      EgoState{ego.time + config.cycle_period, tracked.s, tracked.velocity, tracked.acceleration};
    ego = config.tracking_gain ? ego_state_with_tracking_gain
                               : EgoState{
                                   ego.time + config.cycle_period, ego.s + end.distance,
                                   end.velocity, end.executed_acceleration};
  }

  // Velocity and acceleration changes over one planning cycle: the average acceleration and jerk
  // over that window, so a step appears as step / cycle_period. Before the start, the initial
  // state is extrapolated with its constant acceleration.
  const auto & initial = config.initial_state;
  for (std::size_t i = 0; i < log.samples.size(); ++i) {
    auto & sample = log.samples[i];
    sample.expected_limit = scenario.expected_limit(sample.time, sample.s);
    double previous_velocity = initial.velocity - initial.acceleration * config.cycle_period;
    double previous_acceleration = initial.acceleration;
    if (i >= sub_steps) {
      previous_velocity = log.samples[i - sub_steps].velocity;
      previous_acceleration = log.samples[i - sub_steps].acceleration;
    } else {
      previous_velocity += initial.acceleration * (sample.time - initial.time);
    }
    sample.implied_acceleration = (sample.velocity - previous_velocity) / config.cycle_period;
    sample.jerk = (sample.acceleration - previous_acceleration) / config.cycle_period;
  }
  return log;
}

// ---------------------------------------------------------------------------------------------
// Evaluation
// ---------------------------------------------------------------------------------------------

struct Check
{
  std::string name;
  bool passed{true};
  double value{0.0};
  double bound{0.0};
  std::string detail;
};

/// @brief Reference curve drawn on the plots (time or distance domain).
struct ReferenceCurve
{
  std::string domain;
  std::string label;
  std::vector<std::pair<double, double>> points;
};

struct Evaluation
{
  std::vector<Check> checks;
  std::vector<ReferenceCurve> references;

  [[nodiscard]] bool passed() const
  {
    return std::all_of(checks.begin(), checks.end(), [](const auto & c) { return c.passed; });
  }
};

inline std::string format_number(const double value, const int precision = 3)
{
  if (!std::isfinite(value)) {
    return value > 0.0 ? "inf" : (value < 0.0 ? "-inf" : "nan");
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

inline Check make_upper_check(
  std::string name, const double value, const double bound, std::string detail)
{
  return {std::move(name), value <= bound, value, bound, std::move(detail)};
}

/// @brief Ego state at the first cycle at or after the given time.
inline const CycleRecord & cycle_at(const SimulationLog & log, const double time)
{
  for (const auto & cycle : log.cycles) {
    if (cycle.ego.time >= time - 1e-9) {
      return cycle;
    }
  }
  return log.cycles.back();
}

inline std::optional<double> velocity_at_s(const SimulationLog & log, const double s)
{
  for (std::size_t i = 1; i < log.samples.size(); ++i) {
    const auto & p0 = log.samples[i - 1];
    const auto & p1 = log.samples[i];
    if (p1.s >= s && p0.s < s) {
      const double ratio = (s - p0.s) / (p1.s - p0.s);
      return p0.velocity + ratio * (p1.velocity - p0.velocity);
    }
  }
  return std::nullopt;
}

inline ReferenceCurve time_reference(
  const std::string & label, const std::vector<ProfilePoint> & profile, const double t0)
{
  ReferenceCurve curve{"time", label, {}};
  for (std::size_t i = 0; i < profile.size(); i += 5) {
    curve.points.emplace_back(t0 + profile[i].time, profile[i].velocity);
  }
  curve.points.emplace_back(t0 + profile.back().time, profile.back().velocity);
  return curve;
}

inline ReferenceCurve distance_reference(
  const std::string & label, const std::vector<ProfilePoint> & profile, const double s0)
{
  ReferenceCurve curve{"distance", label, {}};
  for (std::size_t i = 0; i < profile.size(); i += 5) {
    curve.points.emplace_back(s0 + profile[i].distance, profile[i].velocity);
  }
  curve.points.emplace_back(s0 + profile.back().distance, profile.back().velocity);
  return curve;
}

inline void evaluate_constraints(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation);

inline void evaluate_global_bounds(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  const auto & expectations = scenario.expectations;
  double max_increase = 0.0;
  std::string increase_detail = "no stage output was faster than its input at the same arc length";
  for (const auto & cycle : log.cycles) {
    for (std::size_t i = 0; i < cycle.stages.size(); ++i) {
      if (cycle.stages[i].max_velocity_increase > max_increase) {
        max_increase = cycle.stages[i].max_velocity_increase;
        increase_detail = "stage '" + log.stage_names[i] +
                          "' at t=" + format_number(cycle.ego.time, 1) + " s was " +
                          format_number(max_increase) +
                          " m/s faster than its input at the same arc length";
      }
    }
  }
  evaluation.checks.push_back(make_upper_check(
    "output_never_faster_than_input", max_increase, expectations.max_velocity_increase,
    increase_detail));

  double max_pose_error = 0.0;
  double pose_error_time = 0.0;
  for (const auto & cycle : log.cycles) {
    if (cycle.pose_error > max_pose_error) {
      max_pose_error = cycle.pose_error;
      pose_error_time = cycle.ego.time;
    }
  }
  evaluation.checks.push_back(make_upper_check(
    "pose_matches_velocity", max_pose_error, expectations.max_pose_error,
    "largest gap between point arc length and integrated speed at t=" +
      format_number(pose_error_time, 1) + " s"));

  evaluate_constraints(scenario, log, evaluation);
}

/// @brief Acceleration, deceleration and jerk bounds of a scenario's profiles.
struct MotionBounds
{
  double acceleration{0.0};
  double deceleration{0.0};
  double jerk{0.0};
};

inline MotionBounds motion_bounds(const Scenario & scenario)
{
  // Bounds that any profile built from the configured constraints should satisfy.
  double deceleration_bound = scenario.upstream.limits.max_deceleration;
  double jerk_bound = scenario.upstream.limits.jerk;
  if (scenario.upstream.stop_s) {
    deceleration_bound = std::max(deceleration_bound, scenario.upstream.stop_deceleration);
  }
  for (const auto kind : scenario.stages) {
    if (kind == StageKind::Map) {
      deceleration_bound =
        std::max(deceleration_bound, scenario.map_constraints().max_deceleration);
      jerk_bound = std::max(jerk_bound, scenario.map_constraints().jerk);
    }
  }
  for (const auto & event : scenario.external_events) {
    if (event.message) {
      const auto limits = scenario.expected_external_constraints(*event.message);
      deceleration_bound = std::max(deceleration_bound, limits.max_deceleration);
      jerk_bound = std::max(jerk_bound, limits.jerk);
    }
  }
  const double acceleration_bound = std::max(
    {scenario.upstream.limits.max_acceleration, scenario.constraints.max_acceleration,
     scenario.config.initial_state.acceleration});
  return {acceleration_bound, deceleration_bound, jerk_bound};
}

/// @brief The plans of the limit stages must respect the bounds; so must the executed motion when
/// the follower adds no disturbance.
inline void evaluate_constraints(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  const auto & expectations = scenario.expectations;
  const auto bounds = motion_bounds(scenario);
  PlanMotion plan{0.0, 0.0, 0.0};
  for (const auto & cycle : log.cycles) {
    if (cycle.plan) {
      plan.min_acceleration = std::min(plan.min_acceleration, cycle.plan->min_acceleration);
      plan.max_acceleration = std::max(plan.max_acceleration, cycle.plan->max_acceleration);
      plan.max_jerk = std::max(plan.max_jerk, cycle.plan->max_jerk);
    }
  }
  evaluation.checks.push_back(make_upper_check(
    "plan_deceleration_within_bound", -plan.min_acceleration, bounds.deceleration + 0.01,
    "strongest deceleration in the plans modified by a limit stage"));
  evaluation.checks.push_back(make_upper_check(
    "plan_acceleration_within_bound", plan.max_acceleration, bounds.acceleration + 0.01,
    "strongest acceleration in the plans modified by a limit stage"));
  evaluation.checks.push_back(make_upper_check(
    "plan_jerk_within_bound", plan.max_jerk, bounds.jerk + 0.01,
    "largest jerk between points of the plans modified by a limit stage"));
  if (!expectations.check_executed_constraints) {
    return;
  }
  const double deceleration_bound = bounds.deceleration;
  const double acceleration_bound = bounds.acceleration;
  const double jerk_bound = bounds.jerk;

  double min_acceleration = std::numeric_limits<double>::infinity();
  double max_acceleration = -std::numeric_limits<double>::infinity();
  double max_jerk = 0.0;
  double min_time = 0.0;
  double max_time = 0.0;
  double jerk_time = 0.0;
  for (std::size_t i = 1; i < log.samples.size(); ++i) {
    const auto & sample = log.samples[i];
    const double low = std::min(sample.acceleration, sample.implied_acceleration);
    const double high = std::max(sample.acceleration, sample.implied_acceleration);
    if (low < min_acceleration) {
      min_acceleration = low;
      min_time = sample.time;
    }
    if (high > max_acceleration) {
      max_acceleration = high;
      max_time = sample.time;
    }
    if (std::abs(sample.jerk) > max_jerk) {
      max_jerk = std::abs(sample.jerk);
      jerk_time = sample.time;
    }
  }
  evaluation.checks.push_back(make_upper_check(
    "deceleration_within_bound", -min_acceleration, deceleration_bound + 0.05,
    "strongest deceleration at t=" + format_number(min_time, 2) + " s"));
  evaluation.checks.push_back(make_upper_check(
    "acceleration_within_bound", max_acceleration, acceleration_bound + 0.05,
    "strongest acceleration at t=" + format_number(max_time, 2) + " s"));
  if (expectations.check_jerk) {
    evaluation.checks.push_back(make_upper_check(
      "jerk_within_bound", max_jerk, jerk_bound + 0.1,
      "largest |jerk| at t=" + format_number(jerk_time, 2) + " s"));
  }
}

inline void evaluate_external_events(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  const auto & expectations = scenario.expectations;
  const double end_time = log.samples.back().time;
  for (std::size_t e = 0; e < scenario.external_events.size(); ++e) {
    const auto & event = scenario.external_events[e];
    if (!event.message) {
      continue;
    }
    const double limit = event.message->max_velocity;
    const double window_end =
      e + 1 < scenario.external_events.size() ? scenario.external_events[e + 1].time : end_time;
    const auto & start = cycle_at(log, event.time);
    const auto limits = scenario.expected_external_constraints(*event.message);
    const auto ideal =
      simulate_transition(start.ego.velocity, start.ego.acceleration, limit, limits);
    const double ideal_settle = settled_point(ideal, limit, expectations.velocity_tolerance).time;
    const auto label = "external limit " + format_number(limit, 2) + " m/s";
    evaluation.references.push_back(time_reference("reference: " + label, ideal, start.ego.time));
    if (start.ego.time + ideal_settle + expectations.settle_margin > window_end) {
      continue;
    }

    double last_violation = -1.0;
    for (const auto & sample : log.samples) {
      if (
        sample.time >= start.ego.time && sample.time < window_end &&
        sample.velocity > limit + expectations.velocity_tolerance) {
        last_violation = sample.time;
      }
    }
    double settle = 0.0;
    if (last_violation >= window_end - scenario.config.sample_period - 1e-9) {
      settle = std::numeric_limits<double>::infinity();
    } else if (last_violation >= 0.0) {
      settle = last_violation + scenario.config.sample_period - start.ego.time;
    }
    evaluation.checks.push_back(make_upper_check(
      "settle_below_" + label, settle, ideal_settle + expectations.settle_margin,
      "time from t=" + format_number(start.ego.time, 1) + " s until the speed stays below " +
        format_number(limit + expectations.velocity_tolerance, 2) + " m/s (reference " +
        format_number(ideal_settle, 2) + " s)"));
  }
}

/// @brief Ego state when the upstream horizon first reaches the given arc length.
inline const CycleRecord * find_visibility(const SimulationLog & log, const double s)
{
  for (const auto & cycle : log.cycles) {
    if (cycle.input_end_s >= s) {
      return &cycle;
    }
  }
  return nullptr;
}

/// @brief Entry speed versus the best entry speed reachable from where the zone became visible.
inline void check_zone_entry(
  const Scenario & scenario, const SimulationLog & log, const SpeedZone & zone,
  const std::string & label, const CycleRecord & visible, const std::vector<ProfilePoint> & ideal,
  Evaluation & evaluation)
{
  const double tolerance = scenario.expectations.velocity_tolerance;
  const double available = zone.begin_s - visible.ego.s;
  const double bound = std::max(zone.limit, velocity_at_distance(ideal, available)) + tolerance;
  if (const auto entry = velocity_at_s(log, zone.begin_s)) {
    evaluation.checks.push_back(make_upper_check(
      "entry_speed_" + label, *entry, bound,
      "zone visible at s=" + format_number(visible.ego.s, 1) +
        " m with v=" + format_number(visible.ego.velocity, 2) + " m/s, " +
        format_number(available, 1) + " m before its start"));
  }
  evaluation.references.push_back(
    distance_reference("reference from visibility: " + label, ideal, visible.ego.s));
}

/// @brief Overspeed inside the zone once the reference profile has settled.
inline void check_zone_inside(
  const Scenario & scenario, const SimulationLog & log, const SpeedZone & zone,
  const std::string & label, const double inside_from, Evaluation & evaluation)
{
  double max_overspeed = -std::numeric_limits<double>::infinity();
  double overspeed_s = 0.0;
  for (const auto & sample : log.samples) {
    const bool inside = sample.s >= inside_from && sample.s < zone.end_s;
    if (inside && sample.velocity - zone.limit > max_overspeed) {
      max_overspeed = sample.velocity - zone.limit;
      overspeed_s = sample.s;
    }
  }
  if (std::isfinite(max_overspeed)) {
    evaluation.checks.push_back(make_upper_check(
      "inside_speed_" + label, max_overspeed, scenario.expectations.velocity_tolerance,
      "largest overspeed from s=" + format_number(inside_from, 1) +
        " m, at s=" + format_number(overspeed_s, 1) + " m"));
  }
}

/// @brief Braking for the zone should not start much earlier than the latest feasible point.
inline void check_braking_onset(
  const Scenario & scenario, const SimulationLog & log, const SpeedZone & zone,
  const std::string & label, Evaluation & evaluation)
{
  constexpr double drop = 0.3;
  const double approach = *zone.approach_velocity;
  const auto latest = simulate_transition(approach, 0.0, zone.limit, scenario.map_constraints());
  const double latest_start = zone.begin_s - settled_point(latest, zone.limit, 0.05).distance;
  const auto first_drop = std::find_if(latest.begin(), latest.end(), [&](const auto & point) {
    return point.velocity < approach - drop;
  });
  const double latest_drop =
    latest_start + (first_drop != latest.end() ? first_drop->distance : latest.back().distance);
  const double search_from = latest_start - 50.0;
  double actual_drop = zone.begin_s;
  for (const auto & sample : log.samples) {
    if (sample.s >= search_from && sample.s < zone.begin_s && sample.velocity < approach - drop) {
      actual_drop = sample.s;
      break;
    }
  }
  evaluation.checks.push_back(make_upper_check(
    "braking_onset_" + label, latest_drop - actual_drop,
    scenario.expectations.premature_braking_margin,
    "speed first falls " + format_number(drop, 1) + " m/s below the approach speed at s=" +
      format_number(actual_drop, 1) + " m; latest feasible s=" + format_number(latest_drop, 1) +
      " m (positive = metres too early)"));
  evaluation.references.push_back(
    distance_reference("latest braking: " + label, latest, latest_start));
}

inline void evaluate_zones(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  const double tolerance = scenario.expectations.velocity_tolerance;
  const double max_s = log.samples.back().s;
  for (std::size_t z = 0; z < scenario.zones.size(); ++z) {
    const auto & zone = scenario.zones[z];
    const auto * visible = find_visibility(log, zone.begin_s);
    if (max_s < zone.begin_s || visible == nullptr) {
      continue;
    }
    const auto label = "zone " + std::to_string(z) + " (" + format_number(zone.limit, 2) + " m/s)";
    const auto ideal = simulate_transition(
      visible->ego.velocity, visible->ego.acceleration, zone.limit, scenario.map_constraints());
    const bool starts_inside = visible->ego.s >= zone.begin_s;
    const bool needs_braking = std::any_of(ideal.begin(), ideal.end(), [&](const auto & point) {
      return point.velocity > zone.limit + tolerance;
    });
    if (!starts_inside && needs_braking) {
      check_zone_entry(scenario, log, zone, label, *visible, ideal, evaluation);
    }
    const double settle_distance = settled_point(ideal, zone.limit, tolerance).distance;
    check_zone_inside(
      scenario, log, zone, label, std::max(zone.begin_s, visible->ego.s + settle_distance + 5.0),
      evaluation);
    if (zone.approach_velocity && !starts_inside) {
      check_braking_onset(scenario, log, zone, label, evaluation);
    }
  }
}

inline void evaluate_steady_windows(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  const auto & expectations = scenario.expectations;
  for (std::size_t w = 0; w < expectations.steady_windows.size(); ++w) {
    const auto & window = expectations.steady_windows[w];
    const bool by_time = window.domain == WindowDomain::Time;
    const auto label = std::string("steady_") + (by_time ? "t" : "s") + "[" +
                       format_number(window.begin, 0) + "," + format_number(window.end, 0) +
                       "]_at_" + format_number(window.target, 2);
    double velocity_error = 0.0;
    double acceleration_error = 0.0;
    std::size_t sign_changes = 0;
    int previous_sign = 0;
    std::size_t count = 0;
    for (const auto & sample : log.samples) {
      const double x = by_time ? sample.time : sample.s;
      if (x < window.begin || x > window.end) {
        continue;
      }
      ++count;
      velocity_error = std::max(velocity_error, std::abs(sample.velocity - window.target));
      acceleration_error = std::max(acceleration_error, std::abs(sample.acceleration));
      const int sign = sample.jerk > 0.1 ? 1 : (sample.jerk < -0.1 ? -1 : 0);
      if (sign != 0) {
        sign_changes += (previous_sign != 0 && sign != previous_sign) ? 1U : 0U;
        previous_sign = sign;
      }
    }
    if (count == 0) {
      evaluation.checks.push_back(
        {label + "_reached", false, 0.0, 1.0,
         "the ego never reached the window (final s=" + format_number(log.samples.back().s, 1) +
           " m, final v=" + format_number(log.samples.back().velocity, 2) + " m/s)"});
      continue;
    }
    evaluation.checks.push_back(make_upper_check(
      label + "_velocity", velocity_error, expectations.steady_velocity_tolerance,
      "largest |v - target| in the window"));
    evaluation.checks.push_back(make_upper_check(
      label + "_acceleration", acceleration_error, expectations.steady_acceleration_tolerance,
      "largest |a| in the window"));
    evaluation.checks.push_back(make_upper_check(
      label + "_jerk_sign_changes", static_cast<double>(sign_changes),
      static_cast<double>(expectations.max_jerk_sign_changes),
      "sign changes of the jerk (|j| > 0.1) in the window; many indicate chattering"));
  }
}

inline void evaluate_stop(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  if (!scenario.upstream.stop_s) {
    return;
  }
  double max_s = -std::numeric_limits<double>::infinity();
  for (const auto & sample : log.samples) {
    max_s = std::max(max_s, sample.s);
  }
  evaluation.checks.push_back(make_upper_check(
    "stops_before_stop_line", max_s - *scenario.upstream.stop_s, 0.5,
    "furthest ego position relative to the stop line at s=" +
      format_number(*scenario.upstream.stop_s, 1) + " m"));
  evaluation.checks.push_back(
    make_upper_check("stopped_at_end", log.samples.back().velocity, 0.05, "final velocity"));
}

/// @brief Fixed profile restarts: at most the allowed count, and one in each expected window.
inline void evaluate_profile_resets(
  const Scenario & scenario, const SimulationLog & log, Evaluation & evaluation)
{
  const auto & expectations = scenario.expectations;
  std::vector<double> times;
  for (const auto & cycle : log.cycles) {
    for (const auto & stage : cycle.stages) {
      if (stage.start.rfind("reset", 0) == 0) {
        times.push_back(cycle.ego.time);
      }
    }
  }
  if (expectations.max_profile_resets) {
    evaluation.checks.push_back(make_upper_check(
      "profile_resets", static_cast<double>(times.size()),
      static_cast<double>(*expectations.max_profile_resets),
      "fixed profile restarts from the measured ego state"));
  }
  for (const auto & [begin, end] : expectations.expected_reset_windows) {
    const bool found = std::any_of(
      times.begin(), times.end(), [&](const double time) { return time >= begin && time <= end; });
    evaluation.checks.push_back(
      {"profile_reset_in_t[" + format_number(begin, 1) + "," + format_number(end, 1) + "]", found,
       found ? 1.0 : 0.0, 1.0, "a fixed profile restart is expected in this window"});
  }
}

inline Evaluation evaluate(const Scenario & scenario, const SimulationLog & log)
{
  Evaluation evaluation;
  evaluate_global_bounds(scenario, log, evaluation);
  evaluate_external_events(scenario, log, evaluation);
  evaluate_zones(scenario, log, evaluation);
  evaluate_steady_windows(scenario, log, evaluation);
  evaluate_stop(scenario, log, evaluation);
  evaluate_profile_resets(scenario, log, evaluation);
  return evaluation;
}

// ---------------------------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------------------------

/// @brief Output directory: $VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR, else the build-tree default.
inline std::filesystem::path output_directory()
{
  if (const char * value = std::getenv("VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR");
      value != nullptr && *value != '\0') {
    return value;
  }
#ifdef VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR
  return VELOCITY_LIMIT_SIMULATION_OUTPUT_DIR;
#else
  return std::filesystem::temp_directory_path() / "velocity_limit_simulation";
#endif
}

inline std::string json_string(const std::string & text)
{
  std::string escaped = "\"";
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      escaped += '\\';
    }
    escaped += c;
  }
  return escaped + "\"";
}

inline std::string json_number(const double value)
{
  return std::isfinite(value) ? format_number(value, 6) : "null";
}

inline std::string csv_number(const std::optional<double> value, const int precision = 4)
{
  return value && std::isfinite(*value) ? format_number(*value, precision) : "";
}

inline std::string status_name(const ProcessingResult status)
{
  return status == ProcessingResult::Modified ? "modified" : "unchanged";
}

inline void write_plans(
  const std::filesystem::path & file, const Scenario & scenario, const SimulationLog & log)
{
  std::ofstream stream(file);
  stream << "cycle,cycle_time,kind,index,time,s,velocity,acceleration\n";
  for (const auto & plan : log.plans) {
    for (const auto & [kind, points] :
         {std::make_pair("input", &plan.input), std::make_pair("output", &plan.output)}) {
      for (std::size_t i = 0; i < points->size(); ++i) {
        const auto & point = (*points)[i];
        stream << plan.cycle << ',' << format_number(plan.time, 2) << ',' << kind << ',' << i << ','
               << format_number(plan.time + seconds(point.time_from_start), 3) << ','
               << format_number(scenario.path.project(point.pose.position), 3) << ','
               << format_number(point.longitudinal_velocity_mps, 4) << ','
               << format_number(point.acceleration_mps2, 4) << '\n';
      }
    }
  }
}

inline void write_meta(
  const std::filesystem::path & file, const Scenario & scenario, const SimulationLog & log,
  const Evaluation & evaluation)
{
  const auto & upstream = scenario.upstream;
  std::ofstream stream(file);
  stream << "{\n";
  stream << "  \"name\": " << json_string(scenario.name) << ",\n";
  stream << "  \"title\": " << json_string(scenario.title) << ",\n";
  stream << "  \"description\": " << json_string(scenario.description) << ",\n";
  stream << "  \"targets\": " << json_string(scenario.targets) << ",\n";
  stream << "  \"level\": " << json_string(scenario.level) << ",\n";
  stream << "  \"passed\": " << (evaluation.passed() ? "true" : "false") << ",\n";
  stream << "  \"cycle_period\": " << json_number(scenario.config.cycle_period) << ",\n";
  stream << "  \"max_acceleration\": " << json_number(scenario.constraints.max_acceleration)
         << ",\n";
  stream << "  \"max_deceleration\": " << json_number(scenario.constraints.max_deceleration)
         << ",\n";
  stream << "  \"max_jerk\": " << json_number(scenario.constraints.max_jerk) << ",\n";
  stream << "  \"upstream\": {\"profile\": "
         << json_string(
              upstream.profile == UpstreamProfile::EgoAnchored ? "ego-anchored" : "constant-speed")
         << ", \"cruise_velocity\": " << json_number(upstream.cruise_velocity)
         << ", \"max_acceleration\": " << json_number(upstream.limits.max_acceleration)
         << ", \"jerk\": " << json_number(upstream.limits.jerk)
         << ", \"points\": " << upstream.time_stamps.size()
         << ", \"stop_s\": " << (upstream.stop_s ? json_number(*upstream.stop_s) : "null")
         << "},\n";
  stream << "  \"stages\": [";
  for (std::size_t i = 0; i < scenario.stages.size(); ++i) {
    stream << (i > 0 ? ", " : "")
           << json_string(scenario.stages[i] == StageKind::External ? "external" : "map");
  }
  stream << "],\n  \"zones\": [";
  for (std::size_t i = 0; i < scenario.zones.size(); ++i) {
    const auto & zone = scenario.zones[i];
    stream << (i > 0 ? ", " : "") << "{\"begin_s\": " << json_number(zone.begin_s)
           << ", \"end_s\": " << json_number(zone.end_s)
           << ", \"limit\": " << json_number(zone.limit) << "}";
  }
  stream << "],\n  \"external_events\": [";
  for (std::size_t i = 0; i < scenario.external_events.size(); ++i) {
    const auto & event = scenario.external_events[i];
    stream << (i > 0 ? ", " : "") << "{\"time\": " << json_number(event.time)
           << ", \"max_velocity\": "
           << (event.message ? json_number(event.message->max_velocity) : "null");
    if (event.message) {
      const auto limits = scenario.expected_external_constraints(*event.message);
      stream << ", \"deceleration\": " << json_number(limits.max_deceleration)
             << ", \"jerk\": " << json_number(limits.jerk);
    }
    stream << "}";
  }
  stream << "],\n  \"steady_windows\": [";
  for (std::size_t i = 0; i < scenario.expectations.steady_windows.size(); ++i) {
    const auto & window = scenario.expectations.steady_windows[i];
    stream << (i > 0 ? ", " : "") << "{\"domain\": "
           << json_string(window.domain == WindowDomain::Time ? "time" : "distance")
           << ", \"begin\": " << json_number(window.begin)
           << ", \"end\": " << json_number(window.end)
           << ", \"target\": " << json_number(window.target) << "}";
  }
  stream << "],\n  \"fixed_profile\": " << (scenario.fixed_profile ? "true" : "false")
         << ",\n  \"profile_resets\": [";
  bool first_reset = true;
  for (const auto & cycle : log.cycles) {
    for (std::size_t i = 0; i < cycle.stages.size(); ++i) {
      if (cycle.stages[i].start.rfind("reset", 0) == 0) {
        stream << (first_reset ? "" : ", ") << "{\"time\": " << json_number(cycle.ego.time)
               << ", \"stage\": " << json_string(log.stage_names[i])
               << ", \"reason\": " << json_string(cycle.stages[i].start) << "}";
        first_reset = false;
      }
    }
  }
  stream << "],\n  \"checks\": [\n";
  for (std::size_t i = 0; i < evaluation.checks.size(); ++i) {
    const auto & check = evaluation.checks[i];
    stream << "    {\"name\": " << json_string(check.name)
           << ", \"passed\": " << (check.passed ? "true" : "false")
           << ", \"value\": " << json_number(check.value)
           << ", \"bound\": " << json_number(check.bound)
           << ", \"detail\": " << json_string(check.detail) << "}"
           << (i + 1 < evaluation.checks.size() ? ",\n" : "\n");
  }
  stream << "  ]\n}\n";
}

/// @brief Write every artifact the plotting script needs; returns the metadata file path.
inline std::filesystem::path write_outputs(
  const Scenario & scenario, const SimulationLog & log, const Evaluation & evaluation)
{
  const auto directory = output_directory();
  std::filesystem::create_directories(directory);
  const auto prefix = directory / scenario.name;

  {
    std::ofstream stream(prefix.string() + ".samples.csv");
    stream << "time,s,velocity,acceleration,implied_acceleration,jerk,expected_limit,"
              "upstream_velocity,cycle,plan_acceleration\n";
    for (const auto & sample : log.samples) {
      stream << format_number(sample.time, 3) << ',' << format_number(sample.s, 3) << ','
             << format_number(sample.velocity, 4) << ',' << format_number(sample.acceleration, 4)
             << ',' << format_number(sample.implied_acceleration, 4) << ','
             << format_number(sample.jerk, 4) << ',' << csv_number(sample.expected_limit) << ','
             << format_number(sample.upstream_velocity, 4) << ',' << sample.cycle << ','
             << format_number(sample.plan_acceleration, 4) << '\n';
    }
  }
  {
    std::ofstream stream(prefix.string() + ".cycles.csv");
    stream << "cycle,time,s,velocity,acceleration,input_end_s,pose_error";
    for (const auto & name : log.stage_names) {
      stream << ',' << name << "_status," << name << "_start," << name << "_velocity_increase";
    }
    stream << '\n';
    for (const auto & cycle : log.cycles) {
      stream << cycle.index << ',' << format_number(cycle.ego.time, 2) << ','
             << format_number(cycle.ego.s, 3) << ',' << format_number(cycle.ego.velocity, 4) << ','
             << format_number(cycle.ego.acceleration, 4) << ','
             << format_number(cycle.input_end_s, 2) << ',' << format_number(cycle.pose_error, 3);
      for (const auto & stage : cycle.stages) {
        stream << ',' << status_name(stage.status) << ',' << stage.start << ','
               << csv_number(stage.max_velocity_increase);
      }
      stream << '\n';
    }
  }
  {
    std::ofstream stream(prefix.string() + ".reference.csv");
    stream << "domain,label,x,velocity\n";
    for (const auto & curve : evaluation.references) {
      for (const auto & [x, velocity] : curve.points) {
        stream << curve.domain << ',' << json_string(curve.label) << ',' << format_number(x, 3)
               << ',' << format_number(velocity, 4) << '\n';
      }
    }
  }
  write_plans(prefix.string() + ".plans.csv", scenario, log);
  auto meta = prefix.string() + ".meta.json";
  write_meta(meta, scenario, log, evaluation);
  return meta;
}

/// @brief Run, evaluate and write a scenario; every failed check becomes a test failure.
inline void run_and_expect(const Scenario & scenario, const std::vector<Stage> & stages)
{
  const auto log = run_closed_loop(scenario, stages);
  const auto evaluation = evaluate(scenario, log);
  const auto meta = write_outputs(scenario, log, evaluation);
  for (const auto & check : evaluation.checks) {
    EXPECT_TRUE(check.passed) << check.name << ": value " << format_number(check.value)
                              << " exceeds bound " << format_number(check.bound) << " ("
                              << check.detail << ")\n  data: " << meta.string();
  }
}

// ---------------------------------------------------------------------------------------------
// Recorded inputs
// ---------------------------------------------------------------------------------------------

struct RecordedPoint
{
  double time{0.0};
  geometry_msgs::msg::Point position;
  double velocity{0.0};
  double acceleration{0.0};
};

/// @brief Load a recorded modifier input (time, x, y, z, velocity, acceleration per row).
inline std::vector<RecordedPoint> load_recorded_trajectory(const std::string & filename)
{
  const auto path = std::filesystem::path(__FILE__).parent_path() / "data" / filename;
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot open recorded trajectory fixture: " + path.string());
  }
  std::vector<RecordedPoint> points;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::stringstream values(line);
    std::string cell;
    std::vector<double> columns;
    while (std::getline(values, cell, ',')) {
      columns.push_back(std::stod(cell));
    }
    if (columns.size() != 6) {
      throw std::runtime_error("Invalid trajectory fixture row: " + line);
    }
    points.push_back(
      {columns[0], make_point(columns[1], columns[2], columns[3]), columns[4], columns[5]});
  }
  return points;
}

}  // namespace autoware::trajectory_modifier::test::velocity_limit_simulation

#endif  // PLANNING__AUTOWARE_TRAJECTORY_MODIFIER__TESTS__VELOCITY_LIMIT_SIMULATION_HPP_
