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

#include <lanelet2_core/Forward.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>

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
    const sampler_common::transform::Spline2D & reference) const;

public:
  explicit TrajectorySamplerNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace sampler_node

#endif  // SAMPLER_NODE__TRAJECTORY_SAMPLER_NODE_HPP_
