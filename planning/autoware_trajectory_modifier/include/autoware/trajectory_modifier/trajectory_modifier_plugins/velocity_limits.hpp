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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin::detail
{

struct VelocityLimitResult
{
  ProcessingResult status{ProcessingResult::Unchanged};
  std::string error;
};

/// @brief Bounds that the limited velocity profile never violates (all positive magnitudes).
struct VelocityLimitConstraints
{
  double max_acceleration{1.0};  ///< [m/s^2]
  double max_deceleration{1.0};  ///< [m/s^2]
  double max_jerk{1.0};          ///< [m/s^3]
};

struct VelocityLimitOptions
{
  std::optional<double> current_ego_velocity;
  std::optional<double> current_ego_acceleration;
};

/// @brief Ego velocity and acceleration of the plugin input, when available.
VelocityLimitOptions make_velocity_limit_options(const TrajectoryModifierData & data);

/// @brief Distance needed to reach target_velocity from (velocity, acceleration) and end with zero
/// acceleration, braking with at most max_deceleration and max_jerk. Zero when releasing the
/// current acceleration with max_jerk already ends at or below target_velocity.
double jerk_limited_braking_distance(
  double velocity, double acceleration, double target_velocity, double max_deceleration,
  double max_jerk);

/// @brief Limit the velocity of a time-sampled trajectory.
///
/// The callback resolves the limit at a position: one external limit everywhere, or spatially
/// varying map limits. When any point exceeds its limit, the profile is regenerated from the ego
/// state (the first point when no ego state is given) by a forward simulation that never violates
/// the constraints. At every arc length it stays at or below the input velocity and the limit,
/// except where reaching them in time would violate the constraints; such a limit is exceeded
/// temporarily. Braking for a lower limit ahead starts when 95% of the deceleration limit is needed
/// to reach it where it begins, and the profile does not accelerate above the lowest limit ahead.
/// Limit changes between input points are located by querying the callback. A velocity
/// drop at the last input point is ignored, since planners commonly end their horizon with a zero
/// velocity that is not a stop. A measured acceleration outside the constraints is clamped into
/// them. The first pose, the point count and every timestamp are kept, and the other poses are
/// re-sampled along the input polyline to match the new profile.
VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const VelocityLimitConstraints & constraints,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options = {});

/// @brief Parameters of velocity_limits.fixed_profile.
struct FixedProfileParameters
{
  bool enable{true};
  double reset_velocity_deviation{0.5};      ///< [m/s]
  double reset_acceleration_deviation{0.5};  ///< [m/s^2]
  double reset_distance_deviation{1.0};      ///< [m]
  double reset_time_gap{0.5};                ///< [s]
};

bool operator==(const FixedProfileParameters & lhs, const FixedProfileParameters & rhs);

/// @brief Constraints of the velocity_limits parameter group.
VelocityLimitConstraints make_velocity_limit_constraints(const TrajectoryModifierParams & params);

/// @brief Parameters of the velocity_limits.fixed_profile parameter group.
FixedProfileParameters make_fixed_profile_parameters(const TrajectoryModifierParams & params);

/// @brief How the profile of a cycle was started.
enum class ProfileStart {
  Measured,           ///< Fixed profiles are disabled: measured ego state.
  NoPreviousPlan,     ///< No previous plan or no measurement time: measured ego state.
  PreviousPlan,       ///< Previous plan at the measurement time (fixed profile).
  ResetTimeGap,       ///< Previous plan too old, or newer than the measurement.
  ResetVelocity,      ///< Measured velocity too far from the previous plan.
  ResetAcceleration,  ///< Measured acceleration too far from the previous plan.
  ResetDistance,      ///< Ego too far from its planned position.
};

const char * to_string(ProfileStart start);

/// @brief Whether a previous plan existed but the profile restarted from the measured ego state.
bool is_reset(ProfileStart start);

struct ProfileStartResult
{
  VelocityLimitOptions options;
  ProfileStart start{ProfileStart::Measured};
};

/// @brief Previous plan of each candidate, so that a profile can start from the previous plan at
/// the current time instead of the measured ego state ("fixed profile"). The profile then no longer
/// follows measurement noise or tracking errors; it restarts from the measured ego state when that
/// state deviates from the previous plan by more than the reset thresholds. The limits and the
/// input are still evaluated every cycle, so constraint changes are followed without a reset.
class FixedProfileMemory
{
public:
  /// @brief Update the parameters; a change forgets every previous plan.
  void set_parameters(const FixedProfileParameters & parameters);

  /// @brief Starting state for a candidate, from the measured ego state at `time` (unknown when
  /// not given) and `position`.
  [[nodiscard]] ProfileStartResult start(
    std::size_t candidate, const std::optional<double> & time,
    const geometry_msgs::msg::Point & position, const VelocityLimitOptions & measured) const;

  /// @brief Starting state for the candidate and the ego state of a plugin input.
  [[nodiscard]] ProfileStartResult start(const TrajectoryModifierData & data) const;

  /// @brief Remember the trajectory sent for a candidate, planned from the ego state at `time`.
  /// Without a time the candidate's plan is forgotten, as are candidates beyond `candidate_count`.
  void store(
    std::size_t candidate, std::size_t candidate_count, const std::optional<double> & time,
    const TrajectoryPoints & plan);

  /// @brief Remember the trajectory sent for the candidate of a plugin input.
  void store(const TrajectoryModifierData & data, const TrajectoryPoints & plan);

  void clear();

private:
  struct Plan
  {
    double time{0.0};
    TrajectoryPoints points;
  };

  FixedProfileParameters parameters_;
  std::vector<std::optional<Plan>> plans_;
};

}  // namespace autoware::trajectory_modifier::plugin::detail

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
