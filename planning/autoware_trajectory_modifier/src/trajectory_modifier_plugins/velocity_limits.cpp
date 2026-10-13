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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spherical_linear_interpolation.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::trajectory_modifier::plugin::detail
{
namespace
{
constexpr double k_time_step = 0.01;      // [s] integration step of the forward simulation
constexpr double k_tracking_gain = 10.0;  // [1/s] gain of the release-aware velocity tracking
constexpr double k_braking_margin = 0.5;  // [m] added to braking distances before triggering
// Braking for a bound starts when this ratio of the deceleration limit is needed, which leaves a
// reserve to correct deviations without exceeding the limit. Since every cycle re-plans without
// memory, braking continues while the ego decelerates and the lower ratio is needed; the wide band
// keeps a braking manoeuvre from toggling on and off between cycles.
constexpr double k_start_ratio = 0.95;
constexpr double k_continue_ratio = 0.5;
constexpr double k_decelerating = -0.05;    // [m/s^2] acceleration below which the ego decelerates
constexpr double k_limit_tolerance = 1e-3;  // [m/s] input velocity tolerated above a limit
constexpr double k_limit_change_epsilon = 1e-6;  // [m/s] negligible map limit change

struct LongitudinalState
{
  double s{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

/// @brief Velocity reached after releasing the acceleration to zero with the given jerk.
double release_velocity(const double velocity, const double acceleration, const double jerk)
{
  return velocity + acceleration * std::abs(acceleration) / (2.0 * jerk);
}

/// @brief Constant-jerk motion over a duration.
LongitudinalState integrate(
  const LongitudinalState & state, const double jerk, const double duration)
{
  const double t = duration;
  return {
    state.s + state.velocity * t + 0.5 * state.acceleration * t * t + jerk * t * t * t / 6.0,
    state.velocity + state.acceleration * t + 0.5 * jerk * t * t, state.acceleration + jerk * t};
}

using LimitFunction = std::function<std::optional<double>(const geometry_msgs::msg::Point &)>;

double resolve_limit(
  const LimitFunction & velocity_limit, const geometry_msgs::msg::Point & position)
{
  const auto limit = velocity_limit(position);
  return limit && std::isfinite(*limit) && *limit >= 0.0 ? *limit
                                                         : std::numeric_limits<double>::infinity();
}

bool limits_differ(const double lhs, const double rhs)
{
  // Resolved limits are finite and non-negative, or positive infinity for an absent limit.
  if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
    return std::isfinite(lhs) != std::isfinite(rhs);
  }
  return std::abs(lhs - rhs) > k_limit_change_epsilon;
}

/// @brief Ratio along a segment where the limit changes from its value at the segment start.
double find_limit_change(
  const LimitFunction & velocity_limit, const geometry_msgs::msg::Point & from,
  const geometry_msgs::msg::Point & to, const double start_limit)
{
  double low = 0.0;
  double high = 1.0;
  // Twelve bisections locate the change within 1/4096 of the segment length.
  for (int iteration = 0; iteration < 12; ++iteration) {
    const double middle = 0.5 * (low + high);
    geometry_msgs::msg::Point position;
    position.x = autoware::interpolation::lerp(from.x, to.x, middle);
    position.y = autoware::interpolation::lerp(from.y, to.y, middle);
    position.z = autoware::interpolation::lerp(from.z, to.z, middle);
    (limits_differ(resolve_limit(velocity_limit, position), start_limit) ? high : low) = middle;
  }
  return high;
}

/// @brief Bounds along the input polyline, sampled at nodes: the input points plus the positions
/// where the limit changes between two points, located by bisection. The limit of a node holds
/// until the next node. A velocity drop at the last input point is ignored (horizon end, not a
/// stop).
struct PathBounds
{
  std::vector<double> s;
  std::vector<double> input;
  std::vector<double> input_acceleration;  ///< Constant acceleration of the input after a node.
  std::vector<double> limit;
  std::vector<double> lowest_limit_after;
  /// Nodes where the cap stops decreasing (zone entries, stops, bottoms of dips). Braking ends
  /// there, while a decreasing stretch before one is satisfied by braking for it or followed.
  std::vector<std::uint8_t> local_minimum;

  [[nodiscard]] std::size_t size() const { return s.size(); }

  /// @brief Lower of the input velocity and the limit at a node; must hold from that node on.
  [[nodiscard]] double cap(const std::size_t index) const
  {
    return std::min(input[index], limit[index]);
  }

  [[nodiscard]] double input_at(const double arc_length, const std::size_t node) const
  {
    if (node + 1 >= size()) {
      return input.back();
    }
    const double length = s[node + 1] - s[node];
    const double ratio = length > 0.0 ? std::clamp((arc_length - s[node]) / length, 0.0, 1.0) : 1.0;
    return autoware::interpolation::lerp(input[node], input[node + 1], ratio);
  }

  void add_node(
    const double arc_length, const double velocity, const double acceleration, const double max)
  {
    s.push_back(arc_length);
    input.push_back(velocity);
    input_acceleration.push_back(acceleration);
    limit.push_back(max);
  }
};

PathBounds make_path_bounds(
  const TrajectoryPoints & points, const std::vector<double> & limits,
  const LimitFunction & velocity_limit)
{
  const auto count = points.size();
  std::vector<double> arc_lengths(count, 0.0);
  std::vector<double> velocities(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      const auto & p0 = points[i - 1].pose.position;
      const auto & p1 = points[i].pose.position;
      arc_lengths[i] = arc_lengths[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    }
    velocities[i] = std::max(0.0, static_cast<double>(points[i].longitudinal_velocity_mps));
  }
  if (count > 1) {
    velocities[count - 1] = std::max(velocities[count - 1], velocities[count - 2]);
  }

  PathBounds bounds;
  bounds.s.reserve(count * 2);
  bounds.input.reserve(count * 2);
  bounds.input_acceleration.reserve(count * 2);
  bounds.limit.reserve(count * 2);
  for (std::size_t i = 0; i < count; ++i) {
    const double length = i + 1 < count ? arc_lengths[i + 1] - arc_lengths[i] : 0.0;
    const double acceleration =
      length > 1e-6
        ? (velocities[i + 1] * velocities[i + 1] - velocities[i] * velocities[i]) / (2.0 * length)
        : 0.0;
    bounds.add_node(arc_lengths[i], velocities[i], acceleration, limits[i]);
    if (i + 1 < count && limits_differ(limits[i], limits[i + 1])) {
      const double ratio = find_limit_change(
        velocity_limit, points[i].pose.position, points[i + 1].pose.position, limits[i]);
      bounds.add_node(
        arc_lengths[i] + ratio * length,
        autoware::interpolation::lerp(velocities[i], velocities[i + 1], ratio), acceleration,
        limits[i + 1]);
    }
  }

  const auto nodes = bounds.size();
  bounds.lowest_limit_after.assign(nodes, std::numeric_limits<double>::infinity());
  for (std::size_t i = nodes - 1; i > 0; --i) {
    bounds.lowest_limit_after[i - 1] = std::min(bounds.limit[i], bounds.lowest_limit_after[i]);
  }
  constexpr double epsilon = 1e-6;
  bounds.local_minimum.assign(nodes, 0);
  for (std::size_t i = 1; i < nodes; ++i) {
    const bool decreasing_into = bounds.cap(i) < bounds.cap(i - 1) - epsilon;
    const bool not_decreasing_after = i + 1 == nodes || bounds.cap(i + 1) > bounds.cap(i) - epsilon;
    bounds.local_minimum[i] = decreasing_into && not_decreasing_after;
  }
  return bounds;
}

/// @brief Forward simulation of the limited profile. Each step follows the input and the limit at
/// the current position and, for each local minimum of the bounds ahead, starts braking at the
/// latest step that still reaches it with the deceleration and jerk limits. A triggered minimum
/// stays latched until it is passed.
class LimitedProfileSimulation
{
public:
  LimitedProfileSimulation(
    const PathBounds & bounds, const VelocityLimitConstraints & constraints,
    const LongitudinalState & initial)
  : bounds_{bounds}, constraints_{constraints}, state_{initial}, latched_(bounds.size(), 0)
  {
  }

  [[nodiscard]] const LongitudinalState & state() const { return state_; }

  void advance(const double dt)
  {
    while (segment_ + 1 < bounds_.size() && bounds_.s[segment_ + 1] <= state_.s) {
      ++segment_;
    }
    double command = local_command();
    const auto predicted = step(command, dt);
    command = std::min(command, lookahead_command(predicted));
    state_ = step(command, dt);
  }

private:
  /// @brief Brake above the bounds at the current position, hold between them and the lowest
  /// limit ahead, and accelerate below it.
  [[nodiscard]] double local_command() const
  {
    return std::clamp(
      std::min(input_command(), limit_command()), -constraints_.max_deceleration,
      constraints_.max_acceleration);
  }

  /// @brief Follow the input velocity at the current position, with its acceleration as
  /// feedforward. The release term is relative to that acceleration so catching up does not
  /// overshoot.
  [[nodiscard]] double input_command() const
  {
    const double acceleration = bounds_.input_acceleration[segment_];
    const double relative = state_.acceleration - acceleration;
    const double velocity =
      state_.velocity + relative * std::abs(relative) / (2.0 * constraints_.max_jerk);
    return acceleration + k_tracking_gain * (bounds_.input_at(state_.s, segment_) - velocity);
  }

  /// @brief Brake above the limit at the current position, do not accelerate above the lowest
  /// limit ahead, and leave the acceleration free below it.
  [[nodiscard]] double limit_command() const
  {
    const double limit = bounds_.limit[segment_];
    const double ceiling = std::min(limit, bounds_.lowest_limit_after[segment_]);
    const double velocity =
      release_velocity(state_.velocity, state_.acceleration, constraints_.max_jerk);
    if (velocity > limit) {
      return k_tracking_gain * (limit - velocity);
    }
    return velocity >= ceiling ? 0.0 : k_tracking_gain * (ceiling - velocity);
  }

  /// @brief Lowest braking command of the bounds ahead that require braking now.
  [[nodiscard]] double lookahead_command(const LongitudinalState & predicted)
  {
    const auto braking_distance = [this, &predicted](
                                    const double target, const double deceleration) {
      return jerk_limited_braking_distance(
        predicted.velocity, predicted.acceleration, target, deceleration, constraints_.max_jerk);
    };
    const double trigger_deceleration =
      (state_.acceleration < k_decelerating ? k_continue_ratio : k_start_ratio) *
      constraints_.max_deceleration;
    const double reach = braking_distance(0.0, constraints_.max_deceleration) + k_braking_margin;
    double command = std::numeric_limits<double>::infinity();
    double lowest = std::numeric_limits<double>::infinity();
    for (std::size_t j = segment_ + 1; j < bounds_.size(); ++j) {
      if (!bounds_.local_minimum[j] && !latched_[j]) {
        continue;
      }
      const double cap = bounds_.cap(j);
      if (!latched_[j]) {
        // A bound beyond the stopping distance, or above a lower bound before it, cannot require
        // braking yet. Latched bounds are always nearer than the stopping distance when latched.
        if (bounds_.s[j] - predicted.s > reach && !any_latched_after(j)) {
          break;
        }
        if (cap >= lowest) {
          continue;
        }
        lowest = cap;
        const double distance = braking_distance(cap, trigger_deceleration);
        if (distance <= 0.0 || distance + k_braking_margin < bounds_.s[j] - predicted.s) {
          continue;
        }
        latched_[j] = 1;
      }
      lowest = std::min(lowest, cap);
      command = std::min(command, braking_command(j));
    }
    return command;
  }

  /// @brief Decelerate as much as needed to reach a bound where it begins, then release onto it.
  [[nodiscard]] double braking_command(const std::size_t index) const
  {
    const double target = bounds_.cap(index);
    const double available = bounds_.s[index] - state_.s - k_braking_margin;
    return std::max(-required_deceleration(target, available), landing_command(target));
  }

  /// @brief Smallest deceleration, up to the limit, whose jerk-limited braking profile from the
  /// current state reaches the target within the available distance.
  [[nodiscard]] double required_deceleration(const double target, const double available) const
  {
    const auto distance = [this, target](const double deceleration) {
      return jerk_limited_braking_distance(
        state_.velocity, state_.acceleration, target, deceleration, constraints_.max_jerk);
    };
    double high = constraints_.max_deceleration;
    if (available <= 0.0 || distance(high) >= available) {
      return high;
    }
    double low = 0.0;
    // Twenty bisections leave at most max_deceleration / 2^20 of uncertainty.
    for (int iteration = 0; iteration < 20; ++iteration) {
      const double middle = 0.5 * (low + high);
      (distance(middle) <= available ? high : low) = middle;
    }
    return high;
  }

  [[nodiscard]] bool any_latched_after(const std::size_t index) const
  {
    return std::any_of(
      latched_.begin() + static_cast<std::ptrdiff_t>(index), latched_.end(),
      [](const std::uint8_t latched) { return latched != 0; });
  }

  /// @brief Brake toward a target and release so that the acceleration reaches zero there.
  [[nodiscard]] double landing_command(const double target) const
  {
    const double velocity =
      release_velocity(state_.velocity, state_.acceleration, constraints_.max_jerk);
    return std::clamp(k_tracking_gain * (target - velocity), -constraints_.max_deceleration, 0.0);
  }

  /// @brief State after one step toward the commanded acceleration under the jerk limit.
  [[nodiscard]] LongitudinalState step(const double command, const double dt) const
  {
    const double jerk_step = constraints_.max_jerk * dt;
    const double acceleration =
      std::clamp(command, state_.acceleration - jerk_step, state_.acceleration + jerk_step);
    auto next = integrate(state_, (acceleration - state_.acceleration) / dt, dt);
    if (next.velocity < 0.0) {
      next.velocity = 0.0;
      next.acceleration = std::max(next.acceleration, 0.0);
    }
    return next;
  }

  const PathBounds & bounds_;
  const VelocityLimitConstraints & constraints_;
  LongitudinalState state_;
  std::vector<std::uint8_t> latched_;
  std::size_t segment_{0};
};

/// @brief Move every pose but the first along the input polyline to match the new velocities.
void resample_poses(
  TrajectoryPoints & points, const std::vector<geometry_msgs::msg::Pose> & original,
  const std::vector<double> & times)
{
  const auto count = points.size();
  std::vector<double> original_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    const auto & p0 = original[i - 1].position;
    const auto & p1 = original[i].position;
    original_s[i] = original_s[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
  }

  std::vector<double> new_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    const double dt = times[i] - times[i - 1];
    new_s[i] =
      new_s[i - 1] +
      0.5 * (points[i - 1].longitudinal_velocity_mps + points[i].longitudinal_velocity_mps) * dt;
  }

  std::size_t segment = 0;
  for (std::size_t i = 1; i < count; ++i) {
    while (segment + 1 < count && original_s[segment + 1] <= new_s[i]) {
      ++segment;
    }

    if (segment + 1 == count) {
      points[i].pose = original.back();
    } else {
      const double segment_length = original_s[segment + 1] - original_s[segment];
      const double ratio = (new_s[i] - original_s[segment]) / std::max(segment_length, 1e-6);

      const auto & p0 = original[segment];
      const auto & p1 = original[segment + 1];
      auto & pose = points[i].pose;

      pose.position.x = autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      pose.position.y = autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      pose.position.z = autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);
      pose.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
    }
  }
}
}  // namespace

VelocityLimitOptions make_velocity_limit_options(const TrajectoryModifierData & data)
{
  VelocityLimitOptions options;
  if (data.current_odometry) {
    options.current_ego_velocity = data.current_odometry->twist.twist.linear.x;
  }
  if (data.current_acceleration) {
    options.current_ego_acceleration = data.current_acceleration->accel.accel.linear.x;
  }
  return options;
}

double jerk_limited_braking_distance(
  const double velocity, const double acceleration, const double target_velocity,
  const double max_deceleration, const double max_jerk)
{
  if (release_velocity(velocity, acceleration, max_jerk) <= target_velocity) {
    return 0.0;
  }
  // Triangular profile: jerk toward the peak deceleration, then release to zero at the target.
  // (a0^2 - 2 peak^2) / (2 jerk) = target - velocity, saturated at the deceleration limit.
  const double peak = std::max(
    -max_deceleration,
    -std::sqrt(
      0.5 * (acceleration * acceleration + 2.0 * max_jerk * (velocity - target_velocity))));
  LongitudinalState state{0.0, velocity, acceleration};
  state = integrate(
    state, peak < acceleration ? -max_jerk : max_jerk, std::abs(peak - acceleration) / max_jerk);
  const double release_loss = peak * peak / (2.0 * max_jerk);
  const double hold = std::max(0.0, (state.velocity - release_loss - target_velocity) / -peak);
  state = integrate(state, 0.0, hold);
  state = integrate(state, max_jerk, -peak / max_jerk);
  return state.s;
}

VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const VelocityLimitConstraints & constraints,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options)
{
  if (points.empty()) {
    return {};
  }
  const bool valid_constraints =
    std::isfinite(constraints.max_acceleration) && constraints.max_acceleration >= 0.0 &&
    std::isfinite(constraints.max_deceleration) && constraints.max_deceleration > 0.0 &&
    std::isfinite(constraints.max_jerk) && constraints.max_jerk > 0.0;
  if (!valid_constraints) {
    return VelocityLimitResult{
      ProcessingResult::Unchanged,
      "Velocity limiting requires a finite non-negative acceleration and positive deceleration "
      "and jerk"};
  }

  const auto count = points.size();
  std::vector<double> times(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    times[i] = rclcpp::Duration(points[i].time_from_start).seconds();
    if (!std::isfinite(times[i]) || times[i] < 0.0 || (i > 0 && times[i] <= times[i - 1])) {
      return VelocityLimitResult{
        ProcessingResult::Unchanged,
        "Velocity limiting requires finite, non-negative, strictly increasing timestamps"};
    }
  }

  // Resolve the limits on the input path before any pose changes.
  std::vector<double> limits(count, std::numeric_limits<double>::infinity());
  bool needs_limiting = false;
  for (std::size_t i = 0; i < count; ++i) {
    limits[i] = resolve_limit(velocity_limit, points[i].pose.position);
    needs_limiting |= points[i].longitudinal_velocity_mps > limits[i] + k_limit_tolerance;
  }
  if (!needs_limiting) {
    return {ProcessingResult::Unchanged, {}};
  }

  LongitudinalState initial;
  initial.velocity = std::max(
    0.0, options.current_ego_velocity && std::isfinite(*options.current_ego_velocity)
           ? *options.current_ego_velocity
           : static_cast<double>(points.front().longitudinal_velocity_mps));
  // A measured acceleration outside the limits (e.g. noise) is clamped so that the profile never
  // violates them.
  initial.acceleration =
    options.current_ego_acceleration.value_or(points.front().acceleration_mps2);
  initial.acceleration =
    std::isfinite(initial.acceleration)
      ? std::clamp(
          initial.acceleration, -constraints.max_deceleration, constraints.max_acceleration)
      : 0.0;

  const auto bounds = make_path_bounds(points, limits, velocity_limit);
  std::vector<geometry_msgs::msg::Pose> original_poses;
  original_poses.reserve(count);
  for (const auto & point : points) {
    original_poses.push_back(point.pose);
  }
  LimitedProfileSimulation simulation(bounds, constraints, initial);
  double time = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    while (time < times[i] - 1e-9) {
      const double dt = std::min(k_time_step, times[i] - time);
      simulation.advance(dt);
      time += dt;
    }
    points[i].longitudinal_velocity_mps = static_cast<float>(simulation.state().velocity);
    points[i].acceleration_mps2 = static_cast<float>(simulation.state().acceleration);
  }

  resample_poses(points, original_poses, times);
  return {ProcessingResult::Modified, {}};
}

namespace
{
struct PlanSample
{
  double velocity{0.0};
  double acceleration{0.0};
  geometry_msgs::msg::Point position;
};

/// @brief State of a plan `elapsed` seconds after the ego state it was planned from; none beyond
/// its last point. Where the velocities follow the trapezoidal integral of the accelerations, the
/// motion between points has a constant jerk; otherwise the velocity is linear.
std::optional<PlanSample> sample_plan(const TrajectoryPoints & plan, const double elapsed)
{
  const auto time_of = [](const auto & point) {
    return rclcpp::Duration(point.time_from_start).seconds();
  };
  if (plan.empty() || elapsed > time_of(plan.back())) {
    return std::nullopt;
  }
  if (elapsed <= time_of(plan.front())) {
    return PlanSample{
      plan.front().longitudinal_velocity_mps, plan.front().acceleration_mps2,
      plan.front().pose.position};
  }
  for (std::size_t i = 0; i + 1 < plan.size(); ++i) {
    const double t0 = time_of(plan[i]);
    const double t1 = time_of(plan[i + 1]);
    if (elapsed > t1) {
      continue;
    }
    const double h = t1 - t0;
    if (h <= 1e-6) {
      return PlanSample{
        plan[i + 1].longitudinal_velocity_mps, plan[i + 1].acceleration_mps2,
        plan[i + 1].pose.position};
    }
    const double u = elapsed - t0;
    const double v0 = plan[i].longitudinal_velocity_mps;
    const double v1 = plan[i + 1].longitudinal_velocity_mps;
    const double a0 = plan[i].acceleration_mps2;
    const double a1 = plan[i + 1].acceleration_mps2;
    const double jerk = (a1 - a0) / h;
    const bool constant_jerk = std::abs(v1 - v0 - 0.5 * (a0 + a1) * h) <= 5e-3;
    const auto travelled = [&](const double time) {
      return constant_jerk ? v0 * time + 0.5 * a0 * time * time + jerk * time * time * time / 6.0
                           : v0 * time + 0.5 * (v1 - v0) / h * time * time;
    };
    PlanSample sample;
    sample.velocity = constant_jerk ? v0 + a0 * u + 0.5 * jerk * u * u : v0 + u / h * (v1 - v0);
    sample.acceleration = a0 + u / h * (a1 - a0);
    const double length = travelled(h);
    const double ratio = length > 1e-6 ? std::clamp(travelled(u) / length, 0.0, 1.0) : u / h;
    const auto & p0 = plan[i].pose.position;
    const auto & p1 = plan[i + 1].pose.position;
    sample.position.x = autoware::interpolation::lerp(p0.x, p1.x, ratio);
    sample.position.y = autoware::interpolation::lerp(p0.y, p1.y, ratio);
    sample.position.z = autoware::interpolation::lerp(p0.z, p1.z, ratio);
    return sample;
  }
  return std::nullopt;
}

/// @brief Measurement time of the ego state of a plugin input; none when unset.
std::optional<double> measurement_time(const TrajectoryModifierData & data)
{
  if (!data.current_odometry) {
    return std::nullopt;
  }
  const auto & stamp = data.current_odometry->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    return std::nullopt;
  }
  return rclcpp::Time(stamp).seconds();
}
}  // namespace

bool operator==(const FixedProfileParameters & lhs, const FixedProfileParameters & rhs)
{
  return lhs.enable == rhs.enable && lhs.reset_velocity_deviation == rhs.reset_velocity_deviation &&
         lhs.reset_acceleration_deviation == rhs.reset_acceleration_deviation &&
         lhs.reset_distance_deviation == rhs.reset_distance_deviation &&
         lhs.reset_time_gap == rhs.reset_time_gap;
}

VelocityLimitConstraints make_velocity_limit_constraints(const TrajectoryModifierParams & params)
{
  return {
    params.velocity_limits.max_acceleration, params.velocity_limits.max_deceleration,
    params.velocity_limits.max_jerk};
}

FixedProfileParameters make_fixed_profile_parameters(const TrajectoryModifierParams & params)
{
  const auto & fixed = params.velocity_limits.fixed_profile;
  return {
    fixed.enable, fixed.reset_velocity_deviation, fixed.reset_acceleration_deviation,
    fixed.reset_distance_deviation, fixed.reset_time_gap};
}

const char * to_string(const ProfileStart start)
{
  switch (start) {
    case ProfileStart::Measured:
      return "measured";
    case ProfileStart::NoPreviousPlan:
      return "no_previous_plan";
    case ProfileStart::PreviousPlan:
      return "previous_plan";
    case ProfileStart::ResetTimeGap:
      return "reset_time_gap";
    case ProfileStart::ResetVelocity:
      return "reset_velocity";
    case ProfileStart::ResetAcceleration:
      return "reset_acceleration";
    case ProfileStart::ResetDistance:
      return "reset_distance";
  }
  return "unknown";
}

bool is_reset(const ProfileStart start)
{
  return start == ProfileStart::ResetTimeGap || start == ProfileStart::ResetVelocity ||
         start == ProfileStart::ResetAcceleration || start == ProfileStart::ResetDistance;
}

void FixedProfileMemory::set_parameters(const FixedProfileParameters & parameters)
{
  if (!(parameters == parameters_)) {
    parameters_ = parameters;
    clear();
  }
}

ProfileStartResult FixedProfileMemory::start(
  const std::size_t candidate, const std::optional<double> & time,
  const geometry_msgs::msg::Point & position, const VelocityLimitOptions & measured) const
{
  if (!parameters_.enable) {
    return {measured, ProfileStart::Measured};
  }
  if (!time || candidate >= plans_.size() || !plans_[candidate]) {
    return {measured, ProfileStart::NoPreviousPlan};
  }
  const auto & previous = *plans_[candidate];
  const double elapsed = *time - previous.time;
  const auto sample = elapsed >= 0.0 && elapsed <= parameters_.reset_time_gap
                        ? sample_plan(previous.points, elapsed)
                        : std::nullopt;
  if (!sample) {
    return {measured, ProfileStart::ResetTimeGap};
  }
  if (
    measured.current_ego_velocity && std::abs(*measured.current_ego_velocity - sample->velocity) >
                                       parameters_.reset_velocity_deviation) {
    return {measured, ProfileStart::ResetVelocity};
  }
  if (
    measured.current_ego_acceleration &&
    std::abs(*measured.current_ego_acceleration - sample->acceleration) >
      parameters_.reset_acceleration_deviation) {
    return {measured, ProfileStart::ResetAcceleration};
  }
  const double distance = std::hypot(
    position.x - sample->position.x, position.y - sample->position.y,
    position.z - sample->position.z);
  if (distance > parameters_.reset_distance_deviation) {
    return {measured, ProfileStart::ResetDistance};
  }
  VelocityLimitOptions options;
  options.current_ego_velocity = sample->velocity;
  options.current_ego_acceleration = sample->acceleration;
  return {options, ProfileStart::PreviousPlan};
}

ProfileStartResult FixedProfileMemory::start(const TrajectoryModifierData & data) const
{
  const auto measured = make_velocity_limit_options(data);
  if (!data.current_odometry) {
    return {measured, parameters_.enable ? ProfileStart::NoPreviousPlan : ProfileStart::Measured};
  }
  return start(
    data.candidate_index, measurement_time(data), data.current_odometry->pose.pose.position,
    measured);
}

void FixedProfileMemory::store(
  const std::size_t candidate, const std::size_t candidate_count,
  const std::optional<double> & time, const TrajectoryPoints & plan)
{
  // Candidates beyond the current count belong to an earlier batch.
  plans_.resize(std::max(candidate_count, candidate + 1));
  if (time && !plan.empty() && parameters_.enable) {
    plans_[candidate] = Plan{*time, plan};
  } else {
    plans_[candidate].reset();
  }
}

void FixedProfileMemory::store(const TrajectoryModifierData & data, const TrajectoryPoints & plan)
{
  store(data.candidate_index, data.candidate_count, measurement_time(data), plan);
}

void FixedProfileMemory::clear()
{
  plans_.clear();
}

}  // namespace autoware::trajectory_modifier::plugin::detail
