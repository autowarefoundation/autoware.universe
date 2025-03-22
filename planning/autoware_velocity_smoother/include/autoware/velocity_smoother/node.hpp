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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__NODE_HPP_
#define AUTOWARE__VELOCITY_SMOOTHER__NODE_HPP_

#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/osqp_interface/osqp_interface.hpp"
#include "autoware/velocity_smoother/resample.hpp"
#include "autoware/velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp"
#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "autoware/velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "autoware/velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"
#include "autoware/velocity_smoother/smoother/smoother_base.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/math/unit_conversion.hpp"
#include "autoware_utils/ros/diagnostics_interface.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "autoware_utils/ros/self_pose_listener.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_utils/system/time_keeper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"

#include <autoware_utils/ros/published_time_publisher.hpp>

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_internal_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float64_stamped.hpp"
#include "autoware_internal_planning_msgs/msg/velocity_limit.hpp"  // temporary
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::velocity_smoother
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_internal_planning_msgs::msg::VelocityLimit;  // temporary
using autoware_utils::DiagnosticsInterface;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::MarkerArray;

struct Motion
{
  double vel = 0.0;
  double acc = 0.0;

  Motion() {}
  Motion(const double v, const double a) : vel(v), acc(a) {}
};

class VelocitySmootherNode : public rclcpp::Node
{
public:
  explicit VelocitySmootherNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_virtual_wall_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_current_trajectory_;
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "/localization/kinematic_state"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<
    VelocityLimit, autoware_utils::polling_policy::Newest>
    sub_external_velocity_limit_{this, "~/input/external_velocity_limit_mps"};
  autoware_utils::InterProcessPollingSubscriber<OperationModeState> sub_operation_mode_{
    this, "~/input/operation_mode_state"};

  Odometry::ConstSharedPtr current_odometry_ptr_;  // current odometry
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_ptr_;
  VelocityLimit::ConstSharedPtr external_velocity_limit_ptr_{
    nullptr};                                     // external velocity limit message
  Trajectory::ConstSharedPtr base_traj_raw_ptr_;  // current base_waypoints
  double max_velocity_with_deceleration_;         // maximum velocity with deceleration
                                                  // for external velocity limit
  double wheelbase_;                              // wheelbase
  double base_link2front_;                        // base_link to front

  TrajectoryPoints prev_output_;  // previously published trajectory

  // previous trajectory point closest to ego vehicle
  boost::optional<TrajectoryPoint> prev_closest_point_{};
  boost::optional<TrajectoryPoint> current_closest_point_from_prev_output_{};

  bool is_reverse_;

  // check if the vehicle is under control of the planning module
  OperationModeState operation_mode_;

  enum class AlgorithmType {
    INVALID = 0,
    JERK_FILTERED = 1,
    L2 = 2,
    LINF = 3,
    ANALYTICAL = 4,
  };

  enum class InitializeType {
    EGO_VELOCITY = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };

  struct Param
  {
    bool enable_lateral_acc_limit;
    bool enable_steering_rate_limit;

    double max_velocity;                              // max velocity [m/s]
    double margin_to_insert_external_velocity_limit;  // for external velocity limit [m]
    double replan_vel_deviation;                      // if speed error exceeds this [m/s],
                                                      // replan from current velocity
    double engage_velocity;                           // use this speed when start moving [m/s]
    double engage_acceleration;           // use this acceleration when start moving [m/ss]
    double engage_exit_ratio;             // exit engage sequence
                                          // when the speed exceeds ratio x engage_vel.
    double stopping_velocity;             // change target velocity to this value before v=0 point.
    double stopping_distance;             // distance for the stopping_velocity
    double extract_ahead_dist;            // forward waypoints distance from current position [m]
    double extract_behind_dist;           // backward waypoints distance from current position [m]
    double stop_dist_to_prohibit_engage;  // prevent to move toward close stop point
    double ego_nearest_dist_threshold;    // for ego's closest index calculation
    double ego_nearest_yaw_threshold;     // for ego's closest index calculation

    resampling::ResampleParam post_resample_param;
    AlgorithmType algorithm_type;  // Option : JerkFiltered, Linf, L2

    bool plan_from_ego_speed_on_manual_mode = true;
  } node_param_{};

  struct AccelerationRequest
  {
    bool request{false};
    double max_acceleration{0.0};
    double max_jerk{0.0};
  };
  struct ExternalVelocityLimit
  {
    double velocity{0.0};  // current external_velocity_limit
    double dist{0.0};      // distance to set external velocity limit
    AccelerationRequest acceleration_request;
    std::string sender{""};
  };
  ExternalVelocityLimit
    external_velocity_limit_;  // velocity and distance constraint  of external velocity limit

  std::shared_ptr<SmootherBase> smoother_;

  bool publish_debug_trajs_;  // publish planned trajectories

  double over_stop_velocity_warn_thr_;  // threshold to publish over velocity warn

  mutable rclcpp::Clock::SharedPtr clock_;

  void setup_smoother(const double wheelbase);
  // parameter update
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // topic callback
  void on_current_trajectory(const Trajectory::ConstSharedPtr msg);

  void calc_external_velocity_limit();

  // publish methods
  void publish_trajectory(const TrajectoryPoints & traj) const;

  void publish_stop_distance(const TrajectoryPoints & trajectory) const;

  // non-const methods
  void publish_closest_state(const TrajectoryPoints & trajectory);

  void update_prev_values(const TrajectoryPoints & final_result);

  // const methods
  bool check_data() const;

  void update_data_for_external_velocity_limit();

  AlgorithmType get_algorithm_type(const std::string & algorithm_name) const;

  TrajectoryPoints calc_trajectory_velocity(const TrajectoryPoints & traj_input) const;

  bool smooth_velocity(
    const TrajectoryPoints & input, const size_t input_closest,
    TrajectoryPoints & traj_smoothed) const;

  std::pair<Motion, InitializeType> calc_initial_motion(
    const TrajectoryPoints & input_traj, const size_t input_closest) const;

  void apply_external_velocity_limit(TrajectoryPoints & traj) const;

  void insert_behind_velocity(
    const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const;

  void apply_stop_approaching_velocity(TrajectoryPoints & traj) const;

  void overwrite_stop_point(const TrajectoryPoints & input, TrajectoryPoints & output) const;

  double calc_travel_distance() const;

  bool is_engage_status(const double target_velocity) const;

  void publish_debug_trajectories(const std::vector<TrajectoryPoints> & debug_trajectories) const;

  void publish_closest_velocity(
    const TrajectoryPoints & trajectory, const Pose & current_pose,
    const rclcpp::Publisher<Float32Stamped>::SharedPtr pub) const;

  Trajectory to_trajectory_msg(
    const TrajectoryPoints & points, const std_msgs::msg::Header * header = nullptr) const;

  TrajectoryPoint calc_projected_trajectory_point(
    const TrajectoryPoints & trajectory, const Pose & pose) const;
  TrajectoryPoint calc_projected_trajectory_point_from_ego(const TrajectoryPoints & trajectory) const;

  // parameter handling
  void init_common_param();

  // debug
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  std::shared_ptr<rclcpp::Time> prev_time_;
  double prev_acc_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_dist_to_stopline_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_raw_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_vel_lim_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_latacc_filtered_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_steering_rate_limited_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_resampled_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_velocity_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_acc_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_jerk_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr debug_calculation_time_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_closest_max_velocity_;
  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr debug_processing_time_detail_;

  // For Jerk Filtered Algorithm Debug
  rclcpp::Publisher<Trajectory>::SharedPtr pub_forward_filtered_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_backward_filtered_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_merged_filtered_trajectory_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_closest_merged_velocity_;

  // helper functions
  size_t find_nearest_index_from_ego(const TrajectoryPoints & points) const;
  bool is_reverse(const TrajectoryPoints & points) const;
  void flip_velocity(TrajectoryPoints & points) const;
  void publish_stop_watch_time();

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};

  std::unique_ptr<DiagnosticsInterface> diagnostics_interface_{nullptr};
};
}  // namespace autoware::velocity_smoother

#endif  // AUTOWARE__VELOCITY_SMOOTHER__NODE_HPP_
