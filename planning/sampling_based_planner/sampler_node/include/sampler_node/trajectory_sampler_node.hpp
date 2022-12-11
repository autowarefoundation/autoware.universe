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

#ifndef SAMPLER_NODE__TRAJECTORY_SAMPLER_NODE_HPP_
#define SAMPLER_NODE__TRAJECTORY_SAMPLER_NODE_HPP_

#include "frenet_planner/frenet_planner.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/gui/gui.hpp"
#include "sampler_node/parameters.hpp"
#include "sampler_node/trajectory_generation.hpp"
#include "vehicle_info_util/vehicle_info.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>

#include <lanelet2_core/Forward.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace sampler_node
{

class TrajectorySamplerNode : public rclcpp::Node
{
private:
  gui::GUI gui_;
  // Parameters
  double fallback_timeout_{};
  Parameters params_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<rclcpp::Time> prev_replanned_time_ptr_;

  // Cached data
  vehicle_info_util::VehicleInfo vehicle_info_;
  autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steer_ptr_{};
  std::unique_ptr<geometry_msgs::msg::Pose> prev_ego_pose_ptr_;
  std::unique_ptr<autoware_auto_perception_msgs::msg::PredictedObjects> in_objects_ptr_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr fallback_traj_ptr_{};
  sampler_common::Trajectory prev_traj_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::Ids drivable_ids_;
  lanelet::Ids prefered_ids_;
  boost::circular_buffer<double> velocities_{5};
  boost::circular_buffer<double> accelerations_{5};
  boost::circular_buffer<sampler_common::Point> points_{50};
  boost::circular_buffer<double> yaws_{50};

  // ROS pub / sub
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    objects_sub_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr fallback_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acc_sub_;

  rclcpp::TimerBase::SharedPtr gui_process_timer_;

  // callback functions
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  void pathCallback(const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr);
  void steerCallback(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr);
  void objectsCallback(const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr);
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr);
  void routeCallback(const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr);
  void fallbackCallback(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr);

  // other functions
  void publishTrajectory(
    const sampler_common::Trajectory & trajectory, const std::string & frame_id);
  std::optional<sampler_common::Configuration> getCurrentEgoConfiguration();
  static std::optional<size_t> selectBestTrajectory(
    const std::vector<sampler_common::Trajectory> & trajectories);
  sampler_common::Configuration getPlanningConfiguration(
    sampler_common::Configuration configuration,
    const sampler_common::transform::Spline2D & path_spline) const;
  sampler_common::Trajectory prependTrajectory(
    const sampler_common::Trajectory & trajectory,
    const sampler_common::transform::Spline2D & reference,
    const sampler_common::Configuration & current_state) const;

  /// @brief update a trajectory such that the given configuration corresponds to t=0 and length=0
  /// @param [in] trajectory trajectory to update
  /// @param [in] current_configuration the configuration to use as t=0 and length=0
  /// @param [in] max_deviation [m] reset the trajectory if the given configuration is further that
  /// this distance
  /// @param [in] max_behind [m] maximum distance behind the given configuration to leave in the
  /// trajectory
  inline sampler_common::Trajectory updatePreviousTrajectory(
    sampler_common::Trajectory trajectory,
    const sampler_common::Configuration & current_configuration, const double max_deviation,
    const double max_behind)
  {
    // reset trajectory if velocity is 0.0 // TODO(Maxime): make this optional ?
    if (std::abs(current_configuration.velocity) < 0.01) return {};
    const auto closest_iter = std::min_element(
      trajectory.points.begin(), trajectory.points.end(), [&](const auto & p1, const auto & p2) {
        return boost::geometry::distance(p1, current_configuration.pose) <=
               boost::geometry::distance(p2, current_configuration.pose);
      });
    if (
      closest_iter == trajectory.points.end() ||
      boost::geometry::distance(*closest_iter, current_configuration.pose) > max_deviation)
      return {};

    const auto current_idx = std::distance(trajectory.points.begin(), closest_iter);
    auto zero_vel_idx = current_idx;
    for (; zero_vel_idx < trajectory.longitudinal_velocities.size() &&
           trajectory.longitudinal_velocities[zero_vel_idx] > 0.1;
         ++zero_vel_idx)
      ;
    const auto time_offset = trajectory.times[current_idx];
    const auto length_offset = trajectory.lengths[current_idx];
    for (auto & t : trajectory.times) t -= time_offset;
    for (auto & l : trajectory.lengths) l -= length_offset;
    auto max_behind_idx = current_idx;
    for (; max_behind_idx > 0 && trajectory.lengths[max_behind_idx] <= -max_behind;
         --max_behind_idx)
      ;
    if (zero_vel_idx <= max_behind_idx) return {};
    return *trajectory.subset(max_behind_idx, zero_vel_idx);
  }

public:
  explicit TrajectorySamplerNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace sampler_node

#endif  // SAMPLER_NODE__TRAJECTORY_SAMPLER_NODE_HPP_
