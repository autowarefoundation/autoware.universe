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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_CREATOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_CREATOR_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/cubic_spline.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/linear.hpp"
#include "autoware/motion_utils/trajectory_container/interpolator/stairstep.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_path_point_with_lane_id.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_point.hpp"
#include "autoware/motion_utils/trajectory_container/trajectory/trajectory_pose.hpp"

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion_utils::trajectory_container::trajectory
{

/**
 * @brief Template class for creating trajectories of different point types.
 * @tparam PointType The type of points used in the trajectory.
 */
template <class PointType>
class TrajectoryCreator;

/**
 * @brief Specialization of TrajectoryCreator for geometry_msgs::msg::Point.
 */
template <>
class TrajectoryCreator<geometry_msgs::msg::Point>
{
private:
  std::shared_ptr<interpolator::Interpolator<double>> x_interpolator_;  //!< Interpolator for x
  std::shared_ptr<interpolator::Interpolator<double>> y_interpolator_;  //!< Interpolator for y
  std::shared_ptr<interpolator::Interpolator<double>> z_interpolator_;  //!< Interpolator for z

public:
  TrajectoryCreator()
  {
    set_xy_interpolator<interpolator::CubicSpline>();
    set_z_interpolator<interpolator::Linear>();
  }

  /**
   * @brief Sets the interpolator for x and y coordinates.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_xy_interpolator(Args &&... args)
  {
    x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for z coordinate.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_z_interpolator(Args &&... args)
  {
    z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Creates a TrajectoryContainer from a vector of points.
   * @param points Vector of geometry_msgs::msg::Point to create the trajectory from.
   * @return Optional TrajectoryContainer, or nullopt if creation failed.
   */
  std::optional<TrajectoryContainer<geometry_msgs::msg::Point>> create(
    const std::vector<geometry_msgs::msg::Point> & points)
  {
    TrajectoryContainer<geometry_msgs::msg::Point> trajectory(
      x_interpolator_, y_interpolator_, z_interpolator_);
    bool is_valid = trajectory.build(points);
    return is_valid ? std::make_optional(trajectory) : std::nullopt;
  }
};

template <>
class TrajectoryCreator<geometry_msgs::msg::Pose>
{
private:
  std::shared_ptr<interpolator::Interpolator<double>> x_interpolator_;  //!< Interpolator for x
  std::shared_ptr<interpolator::Interpolator<double>> y_interpolator_;  //!< Interpolator for y
  std::shared_ptr<interpolator::Interpolator<double>> z_interpolator_;  //!< Interpolator for z
  std::shared_ptr<interpolator::Interpolator<double>> orientation_x_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_y_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_z_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_w_interpolator_;

public:
  TrajectoryCreator()
  {
    set_xy_interpolator<interpolator::CubicSpline>();
    set_z_interpolator<interpolator::Linear>();
    set_orientation_interpolator<interpolator::CubicSpline>();
  }

  /**
   * @brief Sets the interpolator for x and y coordinates.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_xy_interpolator(Args &&... args)
  {
    x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for z coordinate.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_z_interpolator(Args &&... args)
  {
    z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for orientation.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_orientation_interpolator(Args &&... args)
  {
    orientation_x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_w_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Creates a TrajectoryContainer from a vector of points.
   * @param points Vector of geometry_msgs::msg::Pose to create the trajectory from.
   * @return Optional TrajectoryContainer, or nullopt if creation failed.
   */
  std::optional<TrajectoryContainer<geometry_msgs::msg::Pose>> create(
    const std::vector<geometry_msgs::msg::Pose> & points)
  {
    TrajectoryContainer<geometry_msgs::msg::Pose> trajectory(
      x_interpolator_, y_interpolator_, z_interpolator_, orientation_x_interpolator_,
      orientation_y_interpolator_, orientation_z_interpolator_, orientation_w_interpolator_);
    bool is_valid = trajectory.build(points);
    return is_valid ? std::make_optional(trajectory) : std::nullopt;
  }
};

template <>
class TrajectoryCreator<autoware_planning_msgs::msg::PathPoint>
{
private:
  std::shared_ptr<interpolator::Interpolator<double>> x_interpolator_;  //!< Interpolator for x
  std::shared_ptr<interpolator::Interpolator<double>> y_interpolator_;  //!< Interpolator for y
  std::shared_ptr<interpolator::Interpolator<double>> z_interpolator_;  //!< Interpolator for z
  std::shared_ptr<interpolator::Interpolator<double>> orientation_x_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_y_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_z_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_w_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> longitudinal_velocity_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> lateral_velocity_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> heading_rate_interpolator_;

public:
  TrajectoryCreator()
  {
    set_xy_interpolator<interpolator::CubicSpline>();
    set_z_interpolator<interpolator::Linear>();
    set_orientation_interpolator<interpolator::CubicSpline>();
    set_longitudinal_velocity_interpolator<interpolator::Stairstep<double>>();
    set_lateral_velocity_interpolator<interpolator::Stairstep<double>>();
    set_heading_rate_interpolator<interpolator::Stairstep<double>>();
  }

  /**
   * @brief Sets the interpolator for x and y coordinates.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_xy_interpolator(Args &&... args)
  {
    x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for z coordinate.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_z_interpolator(Args &&... args)
  {
    z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for orientation.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_orientation_interpolator(Args &&... args)
  {
    orientation_x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_w_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for longitudinal velocity.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_longitudinal_velocity_interpolator(Args &&... args)
  {
    longitudinal_velocity_interpolator_ =
      std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for lateral velocity.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_lateral_velocity_interpolator(Args &&... args)
  {
    lateral_velocity_interpolator_ =
      std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for heading rate.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_heading_rate_interpolator(Args &&... args)
  {
    heading_rate_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Creates a TrajectoryContainer from a vector of points.
   * @param points Vector of autoware_planning_msgs::msg::PathPoint to create the trajectory from.
   * @return Optional TrajectoryContainer, or nullopt if creation failed.
   */
  std::optional<TrajectoryContainer<autoware_planning_msgs::msg::PathPoint>> create(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & points)
  {
    TrajectoryContainer<autoware_planning_msgs::msg::PathPoint> trajectory(
      x_interpolator_, y_interpolator_, z_interpolator_, orientation_x_interpolator_,
      orientation_y_interpolator_, orientation_z_interpolator_, orientation_w_interpolator_,
      longitudinal_velocity_interpolator_, lateral_velocity_interpolator_,
      heading_rate_interpolator_);
    bool is_valid = trajectory.build(points);
    return is_valid ? std::make_optional(trajectory) : std::nullopt;
  }
};

template <>
class TrajectoryCreator<tier4_planning_msgs::msg::PathPointWithLaneId>
{
private:
  using LaneIdType = std::vector<int64_t>;

  std::shared_ptr<interpolator::Interpolator<double>> x_interpolator_;  //!< Interpolator for x
  std::shared_ptr<interpolator::Interpolator<double>> y_interpolator_;  //!< Interpolator for y
  std::shared_ptr<interpolator::Interpolator<double>> z_interpolator_;  //!< Interpolator for z
  std::shared_ptr<interpolator::Interpolator<double>> orientation_x_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_y_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_z_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> orientation_w_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> longitudinal_velocity_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> lateral_velocity_interpolator_;
  std::shared_ptr<interpolator::Interpolator<double>> heading_rate_interpolator_;
  std::shared_ptr<interpolator::Interpolator<LaneIdType>> lane_id_interpolator_;

public:
  TrajectoryCreator()
  {
    set_xy_interpolator<interpolator::CubicSpline>();
    set_z_interpolator<interpolator::Linear>();
    set_orientation_interpolator<interpolator::CubicSpline>();
    set_longitudinal_velocity_interpolator<interpolator::Stairstep<double>>();
    set_lateral_velocity_interpolator<interpolator::Stairstep<double>>();
    set_heading_rate_interpolator<interpolator::Stairstep<double>>();
    set_lane_id_interpolator<interpolator::Stairstep<LaneIdType>>();
  }

  /**
   * @brief Sets the interpolator for x and y coordinates.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_xy_interpolator(Args &&... args)
  {
    x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for z coordinate.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_z_interpolator(Args &&... args)
  {
    z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for orientation.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_orientation_interpolator(Args &&... args)
  {
    orientation_x_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_y_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_z_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    orientation_w_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for longitudinal velocity.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_longitudinal_velocity_interpolator(Args &&... args)
  {
    longitudinal_velocity_interpolator_ =
      std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for lateral velocity.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_lateral_velocity_interpolator(Args &&... args)
  {
    lateral_velocity_interpolator_ =
      std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for heading rate.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_heading_rate_interpolator(Args &&... args)
  {
    heading_rate_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Sets the interpolator for lane ID.
   * @tparam InterpolatorType The type of interpolator to use.
   * @param args Arguments to forward to the interpolator constructor.
   * @return Reference to this TrajectoryCreator.
   */
  template <class InterpolatorType, class... Args>
  TrajectoryCreator & set_lane_id_interpolator(Args &&... args)
  {
    lane_id_interpolator_ = std::make_shared<InterpolatorType>(std::forward<Args>(args)...);
    return *this;
  }

  /**
   * @brief Creates a TrajectoryContainer from a vector of points.
   * @param points Vector of tier4_planning_msgs::msg::PathPointWithLaneId to create the trajectory
   * from.
   * @return Optional TrajectoryContainer, or nullopt if creation failed.
   */
  std::optional<TrajectoryContainer<tier4_planning_msgs::msg::PathPointWithLaneId>> create(
    const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points)
  {
    TrajectoryContainer<tier4_planning_msgs::msg::PathPointWithLaneId> trajectory(
      x_interpolator_, y_interpolator_, z_interpolator_, orientation_x_interpolator_,
      orientation_y_interpolator_, orientation_z_interpolator_, orientation_w_interpolator_,
      longitudinal_velocity_interpolator_, lateral_velocity_interpolator_,
      heading_rate_interpolator_, lane_id_interpolator_);
    bool is_valid = trajectory.build(points);
    return is_valid ? std::make_optional(trajectory) : std::nullopt;
  }
};

}  // namespace autoware::motion_utils::trajectory_container::trajectory

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__TRAJECTORY__TRAJECTORY_CREATOR_HPP_
