// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY__PATH_POINT_HPP_
#define AUTOWARE__TRAJECTORY__PATH_POINT_HPP_

#include "autoware/trajectory/detail/interpolated_array.hpp"
#include "autoware/trajectory/pose.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::trajectory
{
template <>
class Trajectory<autoware_planning_msgs::msg::PathPoint>
: public Trajectory<geometry_msgs::msg::Pose>
{
  using BaseClass = Trajectory<geometry_msgs::msg::Pose>;
  using PointType = autoware_planning_msgs::msg::PathPoint;

  std::shared_ptr<detail::InterpolatedArray<double>>
    longitudinal_velocity_mps_;  //!< Longitudinal velocity in m/s
  std::shared_ptr<detail::InterpolatedArray<double>>
    lateral_velocity_mps_;  //!< Lateral velocity in m/s
  std::shared_ptr<detail::InterpolatedArray<double>>
    heading_rate_rps_;  //!< Heading rate in rad/s};

public:
  Trajectory();
  ~Trajectory() override = default;
  Trajectory(const Trajectory & rhs);
  Trajectory(Trajectory && rhs) = default;
  Trajectory & operator=(const Trajectory & rhs);
  Trajectory & operator=(Trajectory && rhs) = default;

  [[nodiscard]] std::vector<double> get_internal_bases() const override;

  detail::InterpolatedArray<double> & longitudinal_velocity_mps()
  {
    return *longitudinal_velocity_mps_;
  }

  detail::InterpolatedArray<double> & lateral_velocity_mps() { return *lateral_velocity_mps_; }

  detail::InterpolatedArray<double> & heading_rate_rps() { return *heading_rate_rps_; }

  [[nodiscard]] const detail::InterpolatedArray<double> & longitudinal_velocity_mps() const
  {
    return *longitudinal_velocity_mps_;
  }

  [[nodiscard]] const detail::InterpolatedArray<double> & lateral_velocity_mps() const
  {
    return *lateral_velocity_mps_;
  }

  [[nodiscard]] const detail::InterpolatedArray<double> & heading_rate_rps() const
  {
    return *heading_rate_rps_;
  }

  /**
   * @brief Build the trajectory from the points
   * @param points Vector of points
   * @return True if the build is successful
   */
  bool build(const std::vector<PointType> & points);

  /**
   * @brief Compute the point on the trajectory at a given s value
   * @param s Arc length
   * @return Point on the trajectory
   */
  [[nodiscard]] PointType compute(double s) const;

  /**
   * @brief Restore the trajectory points
   * @param min_points Minimum number of points
   * @return Vector of points
   */
  [[nodiscard]] std::vector<PointType> restore(const size_t & min_points = 4) const;

  class Builder
  {
  private:
    std::unique_ptr<Trajectory> trajectory_;

  public:
    Builder() : trajectory_(std::make_unique<Trajectory>()) {}

    template <class InterpolatorType, class... Args>
    Builder & set_xy_interpolator(Args &&... args)
    {
      trajectory_->x_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      trajectory_->y_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_z_interpolator(Args &&... args)
    {
      trajectory_->z_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_orientation_interpolator(Args &&... args)
    {
      trajectory_->orientation_interpolator_ =
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_longitudinal_velocity_interpolator(Args &&... args)
    {
      trajectory_->longitudinal_velocity_mps_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_lateral_velocity_interpolator(Args &&... args)
    {
      trajectory_->lateral_velocity_mps_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    template <class InterpolatorType, class... Args>
    Builder & set_heading_rate_interpolator(Args &&... args)
    {
      trajectory_->heading_rate_rps_ = std::make_shared<detail::InterpolatedArray<double>>(
        std::make_shared<InterpolatorType>(std::forward<Args>(args)...));
      return *this;
    }

    std::optional<Trajectory> build(const std::vector<PointType> & points)
    {
      if (trajectory_->build(points)) {
        auto result = std::make_optional<Trajectory>(std::move(*trajectory_));
        trajectory_.reset();
        return result;
      }
      return std::nullopt;
    }
  };
};

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__PATH_POINT_HPP_
