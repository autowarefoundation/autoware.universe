// Copyright 2021 The Autoware Foundation
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware/mpc_lateral_controller/mpc.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

nav_msgs::msg::Odometry makeOdometry(const geometry_msgs::msg::Pose & pose, const double velocity)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = pose;
  odometry.twist.twist.linear.x = velocity;
  return odometry;
}

struct VehicleModelParameters
{
  double wheelbase;
  double steer_limit;
  double steer_tau;
  double mass_fl;
  double mass_fr;
  double mass_rl;
  double mass_rr;
  double cf;
  double cr;
};

class MPCTest : public ::testing::Test
{
protected:
  MPCParam param;
  VehicleModelParameters vehicle_model_param;

  // Test inputs
  Trajectory dummy_straight_trajectory;
  Trajectory dummy_right_turn_trajectory;
  SteeringReport neutral_steer;
  Pose pose_zero;
  double default_velocity = 1.0;
  rclcpp::Logger logger = rclcpp::get_logger("mpc_test_logger");

  // Filters parameter
  double steering_lpf_cutoff_hz;
  double error_deriv_lpf_cutoff_hz;

  // Test Parameters
  double admissible_position_error = 5.0;
  double admissible_yaw_error_rad = M_PI_2;
  double steer_lim = 0.610865;      // 35 degrees
  double steer_rate_lim = 2.61799;  // 150 degrees
  double ctrl_period = 0.03;

  bool use_steer_prediction = true;

  TrajectoryFilteringParam trajectory_param;

  void initParam()
  {
    rclcpp::NodeOptions node_options;

    const auto share_dir = ament_index_cpp::get_package_share_directory("mpc_lateral_controller");

    test_utils::updateNodeOptions(
      node_options, {share_dir + "/param/lateral_controller_defaults.param.yaml",
                     share_dir + "/test/test_vehicle_info.param.yaml"});

    auto temp_node = std::make_shared<rclcpp::Node>("temp_node", node_options);
    const auto dp_int = [&](const std::string & s) { return temp_node->declare_parameter<int>(s); };
    const auto dp_bool = [&](const std::string & s) {
      return temp_node->declare_parameter<bool>(s);
    };
    const auto dp_double = [&](const std::string & s) {
      return temp_node->declare_parameter<double>(s);
    };

    vehicle_model_param.wheelbase = dp_double("wheel_base");
    vehicle_model_param.steer_limit = dp_double("max_steer_angle");
    vehicle_model_param.steer_tau = dp_double("vehicle_model_steer_tau");
    vehicle_model_param.mass_fl = dp_double("mass_fl");
    vehicle_model_param.mass_fr = dp_double("mass_fr");
    vehicle_model_param.mass_rl = dp_double("mass_rl");
    vehicle_model_param.mass_rr = dp_double("mass_rr");
    vehicle_model_param.cf = dp_double("cf");
    vehicle_model_param.cr = dp_double("cr");

    steering_lpf_cutoff_hz = dp_double("steering_lpf_cutoff_hz");
    error_deriv_lpf_cutoff_hz = dp_double("error_deriv_lpf_cutoff_hz");

    param.prediction_horizon = dp_int("mpc_prediction_horizon");
    param.prediction_dt = dp_double("mpc_prediction_dt");
    param.zero_ff_steer_deg = dp_double("mpc_zero_ff_steer_deg");
    param.input_delay = dp_double("input_delay");
    param.acceleration_limit = dp_double("mpc_acceleration_limit");
    param.velocity_time_constant = dp_double("mpc_velocity_time_constant");
    param.min_prediction_length = dp_double("mpc_min_prediction_length");
    param.steer_tau = vehicle_model_param.steer_tau;
    param.nominal_weight.lat_error = dp_double("mpc_weight_lat_error");
    param.nominal_weight.heading_error = dp_double("mpc_weight_heading_error");
    param.nominal_weight.heading_error_squared_vel =
      dp_double("mpc_weight_heading_error_squared_vel");
    param.nominal_weight.terminal_lat_error = dp_double("mpc_weight_terminal_lat_error");
    param.nominal_weight.terminal_heading_error = dp_double("mpc_weight_terminal_heading_error");
    param.low_curvature_weight.lat_error = dp_double("mpc_low_curvature_weight_lat_error");
    param.low_curvature_weight.heading_error = dp_double("mpc_low_curvature_weight_heading_error");
    param.low_curvature_weight.heading_error_squared_vel =
      dp_double("mpc_low_curvature_weight_heading_error_squared_vel");
    param.nominal_weight.steering_input = dp_double("mpc_weight_steering_input");
    param.nominal_weight.steering_input_squared_vel =
      dp_double("mpc_weight_steering_input_squared_vel");
    param.nominal_weight.lat_jerk = dp_double("mpc_weight_lat_jerk");
    param.nominal_weight.steer_rate = dp_double("mpc_weight_steer_rate");
    param.nominal_weight.steer_acc = dp_double("mpc_weight_steer_acc");
    param.low_curvature_weight.steering_input =
      dp_double("mpc_low_curvature_weight_steering_input");
    param.low_curvature_weight.steering_input_squared_vel =
      dp_double("mpc_low_curvature_weight_steering_input_squared_vel");
    param.low_curvature_weight.lat_jerk = dp_double("mpc_low_curvature_weight_lat_jerk");
    param.low_curvature_weight.steer_rate = dp_double("mpc_low_curvature_weight_steer_rate");
    param.low_curvature_weight.steer_acc = dp_double("mpc_low_curvature_weight_steer_acc");
    param.low_curvature_thresh_curvature = dp_double("mpc_low_curvature_thresh_curvature");

    trajectory_param.traj_resample_dist = dp_double("traj_resample_dist");
    trajectory_param.path_filter_moving_ave_num = dp_int("path_filter_moving_ave_num");
    trajectory_param.curvature_smoothing_num_traj = dp_int("curvature_smoothing_num_traj");
    trajectory_param.curvature_smoothing_num_ref_steer =
      dp_int("curvature_smoothing_num_ref_steer");
    trajectory_param.enable_path_smoothing = dp_bool("enable_path_smoothing");
    trajectory_param.extend_trajectory_for_end_yaw_control =
      dp_bool("extend_trajectory_for_end_yaw_control");

    dummy_straight_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(1.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(2.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(3.0, 0.0, 1.0f));
    dummy_straight_trajectory.points.push_back(makePoint(4.0, 0.0, 1.0f));

    dummy_right_turn_trajectory.points.push_back(makePoint(-1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(0.0, 0.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(1.0, -1.0, 1.0f));
    dummy_right_turn_trajectory.points.push_back(makePoint(2.0, -2.0, 1.0f));

    neutral_steer.steering_tire_angle = 0.0;
    pose_zero.position.x = 0.0;
    pose_zero.position.y = 0.0;
  }

  void initializeMPC(mpc_lateral_controller::MPC & mpc)
  {
    mpc.m_param = param;
    mpc.m_admissible_position_error = admissible_position_error;
    mpc.m_admissible_yaw_error_rad = admissible_yaw_error_rad;
    mpc.m_steer_lim = steer_lim;
    mpc.m_steer_rate_lim_map_by_curvature.emplace_back(0.0, steer_rate_lim);
    mpc.m_steer_rate_lim_map_by_velocity.emplace_back(0.0, steer_rate_lim);
    mpc.m_ctrl_period = ctrl_period;
    mpc.m_use_steer_prediction = use_steer_prediction;

    mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
    mpc.initializeSteeringPredictor();

    // Init trajectory
    const auto current_kinematics =
      makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
    mpc.setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    initParam();
  }

  void TearDown() override { rclcpp::shutdown(); }
};  // class MPCTest

/* cppcheck-suppress syntaxError */
TEST_F(MPCTest, InitializeAndCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  EXPECT_FALSE(mpc->hasVehicleModel());
  EXPECT_FALSE(mpc->hasQPSolver());

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit,
      vehicle_model_param.steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, InitializeAndCalculateRightTurn)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  EXPECT_FALSE(mpc->hasVehicleModel());
  EXPECT_FALSE(mpc->hasQPSolver());

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit,
      vehicle_model_param.steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init parameters and reference trajectory
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, OsqpCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit,
      vehicle_model_param.steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  EXPECT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, OsqpCalculateRightTurn)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit,
      vehicle_model_param.steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverOSQP>(logger);
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, KinematicsNoDelayCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModelNoDelay>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init filters
  mpc->initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  // Init trajectory
  const auto current_kinematics = makeOdometry(dummy_straight_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_straight_trajectory, trajectory_param, current_kinematics);
  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, KinematicsNoDelayCalculateRightTurn)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);
  const auto current_kinematics =
    makeOdometry(dummy_right_turn_trajectory.points.front().pose, 0.0);
  mpc->setReferenceTrajectory(dummy_right_turn_trajectory, trajectory_param, current_kinematics);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModelNoDelay>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Init filters
  mpc->initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);

  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_LT(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_LT(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, DynamicCalculate)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  initializeMPC(*mpc);

  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr = std::make_shared<DynamicsBicycleModel>(
    vehicle_model_param.wheelbase, vehicle_model_param.mass_fl, vehicle_model_param.mass_fr,
    vehicle_model_param.mass_rl, vehicle_model_param.mass_rr, vehicle_model_param.cf,
    vehicle_model_param.cr);
  mpc->setVehicleModel(vehicle_model_ptr);
  ASSERT_TRUE(mpc->hasVehicleModel());

  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);
  ASSERT_TRUE(mpc->hasQPSolver());

  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
}

TEST_F(MPCTest, MultiSolveWithBuffer)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit,
      vehicle_model_param.steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  mpc->m_input_buffer = {0.0, 0.0, 0.0};
  // Calculate MPC
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_zero, default_velocity);

  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
  ASSERT_TRUE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));
  EXPECT_EQ(ctrl_cmd.steering_tire_angle, 0.0f);
  EXPECT_EQ(ctrl_cmd.steering_tire_rotation_rate, 0.0f);
  EXPECT_EQ(mpc->m_input_buffer.size(), size_t(3));
}

TEST_F(MPCTest, FailureCases)
{
  auto node = rclcpp::Node("mpc_test_node", rclcpp::NodeOptions{});
  auto mpc = std::make_unique<MPC>(node);
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr =
    std::make_shared<KinematicsBicycleModel>(
      vehicle_model_param.wheelbase, vehicle_model_param.steer_limit, param.steer_tau);
  mpc->setVehicleModel(vehicle_model_ptr);
  std::shared_ptr<QPSolverInterface> qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
  mpc->setQPSolver(qpsolver_ptr);

  // Init parameters and reference trajectory
  initializeMPC(*mpc);

  // Calculate MPC with a pose too far from the trajectory
  Pose pose_far;
  pose_far.position.x = pose_zero.position.x - admissible_position_error - 1.0;
  pose_far.position.y = pose_zero.position.y - admissible_position_error - 1.0;
  Lateral ctrl_cmd;
  Trajectory pred_traj;
  Float32MultiArrayStamped diag;
  const auto odom = makeOdometry(pose_far, default_velocity);
  EXPECT_FALSE(mpc->calculateMPC(neutral_steer, odom, ctrl_cmd, pred_traj, diag));

  // Calculate MPC with a fast velocity to make the prediction go further than the reference path
  EXPECT_FALSE(mpc->calculateMPC(
    neutral_steer, makeOdometry(pose_far, default_velocity + 10.0), ctrl_cmd, pred_traj, diag));
}
}  // namespace autoware::motion::control::mpc_lateral_controller
