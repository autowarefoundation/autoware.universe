// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__JERK_FILTERED_SMOOTHER_HPP_
#define AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__JERK_FILTERED_SMOOTHER_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/qp_interface/qp_interface.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "autoware/velocity_smoother/smoother/smoother_base.hpp"

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include "boost/optional.hpp"

#include <memory>
#include <vector>

namespace autoware::velocity_smoother
{
class JerkFilteredSmoother : public SmootherBase
{
public:
  struct Param
  {
    double jerk_weight;
    double over_v_weight;
    double over_a_weight;
    double over_j_weight;
    double jerk_filter_ds;
  };

  explicit JerkFilteredSmoother(
    rclcpp::Node & node, const std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper);

  bool apply(
    const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
    TrajectoryPoints & output, std::vector<TrajectoryPoints> & debug_trajectories,
    const bool publish_debug_trajs) override;

  TrajectoryPoints resampleTrajectory(
    const TrajectoryPoints & input, [[maybe_unused]] const double v0,
    const geometry_msgs::msg::Pose & current_pose, const double nearest_dist_threshold,
    const double nearest_yaw_threshold) const override;

  void setParam(const Param & param);
  Param getParam() const;

private:
  Param smoother_param_;
  std::shared_ptr<autoware::qp_interface::QPInterface> qp_interface_;
  rclcpp::Logger logger_{rclcpp::get_logger("smoother").get_child("jerk_filtered_smoother")};

  TrajectoryPoints forwardJerkFilter(
    const double v0, const double a0, const double a_max, const double a_stop, const double j_max,
    const TrajectoryPoints & input) const;
  TrajectoryPoints backwardJerkFilter(
    const double v0, const double a0, const double a_min, const double a_stop, const double j_min,
    const TrajectoryPoints & input) const;
  TrajectoryPoints mergeFilteredTrajectory(
    const double v0, const double a0, const double a_min, const double j_min,
    const TrajectoryPoints & forward_filtered, const TrajectoryPoints & backward_filtered) const;
};
}  // namespace autoware::velocity_smoother

#endif  // AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__JERK_FILTERED_SMOOTHER_HPP_
