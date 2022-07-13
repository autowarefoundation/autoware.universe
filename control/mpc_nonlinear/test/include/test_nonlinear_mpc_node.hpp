/*
* Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef TEST_NONLINEAR_MPC_NODE_HPP_
#define TEST_NONLINEAR_MPC_NODE_HPP_

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include "Eigen/Core"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "nonlinear_mpc_node/nonlinear_mpc_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief  Logistic map function, generates a curve using the logistic function.
 * @param [in] max_y_value  value of the maximum y coordinate (asymtote )
 * @param [in] center_x_value value at which the curve starts to bend
 * @param [in] slope value defines how fast curve climb.
 * @param [in] x_coord
 * @param [out] ycurve_output = std::vector
 */

std::vector<double> logisticMap(
  double const & max_y_value, double const & center_x_value, double const & slope,
  std::vector<double> const & x_coord);

/**
 * @brief Returns the derivative of the logistic function df/dx = f(x)*(1-f(x))
 * where f(x) is the logistic function defined.
 */

std::vector<double> logisticMapDerivative(
  double const & max_y_value, double const & center_x_value, double const & slope,
  std::vector<double> const & x_coord);

// Test helper functions.
/**
 * @brief Create a circular trajectory with a large curvature value.
 * x = R cos(yaw)
 * y = R sin(yaw)
 *
 */

void createTrajectoryMessage(autoware_planning_msgs::msg::Trajectory & traj_msg);

// Test fixture for members.
class NMPCTestSuiteMembers : public ::testing::Test
{
protected:
  void SetUp() { rclcpp::init(0, nullptr); }

  void TearDown() { (void)rclcpp::shutdown(); }
};

// For node testing.
class FakeNode : public rclcpp::Node
{
public:
  //using nmpc_msg_t = autoware_control_msgs::msg::NonlinearMPCPerformanceStamped;

  // using NonlinearMPCNode::NonlinearMPCNode;
  explicit FakeNode(const std::string & node_name, rclcpp::NodeOptions const & node_options)
  : rclcpp::Node(node_name, node_options)
  {
    // Publishers and subscriber methods.
    using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

    // Trajectory publisher
    pub_dummy_traj_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "/mpc_nonlinear/input/reference_trajectory", rclcpp::QoS{1}, PubAllocT{});

    pub_dummy_velocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mpc_nonlinear/input/current_velocity", rclcpp::QoS{1}, PubAllocT{});

    pub_dummy_steering_ = this->create_publisher<autoware_vehicle_msgs::msg::Steering>(
      "/mpc_nonlinear/input/current_steering", rclcpp::QoS{1}, PubAllocT{});

    sub_nmpc_msgs_ =
      this->create_subscription<autoware_control_msgs::msg::NonlinearMPCPerformanceStamped>(
        "/mpc_nonlinear/debug/nmpc_predicted_performance_vars", rclcpp::QoS{1},
        std::bind(&FakeNode::onNMPCPerformance, this, _1));

    sub_ctrl_cmd_msgs_ = create_subscription<autoware_control_msgs::msg::ControlCommandStamped>(
      "/mpc_nonlinear/output/control_steering_raw", rclcpp::QoS{1},
      std::bind(&FakeNode::onNMPCControl, this, _1));

    // Initialize the timer.
    initTimer(0.05);

    autoware_planning_msgs::msg::Trajectory traj_msg;
    createTrajectoryMessage(traj_msg);
    trajectory_ = std::make_shared<autoware_planning_msgs::msg::Trajectory>(traj_msg);
  };

  ~FakeNode() {}
  void transformBroadcast();
  void publishTrajectory();
  void publishVehicleMeasuredSpeed();
  void publishVehicleMeasuredSteering();

  void onTimer();
  void initTimer(double control_period);
  void onNMPCPerformance(autoware_control_msgs::msg::NonlinearMPCPerformanceStamped::SharedPtr msg);
  void onNMPCControl(autoware_control_msgs::msg::ControlCommandStamped::SharedPtr msg);

  void setInitialPoseSpeed(double const & v0_initial_speed) { initial_speed = v0_initial_speed; }
  void setPoseInd(size_t const & pose_ind) { ind_pose_ = pose_ind; }

  // Publishers and subscribers.
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_dummy_traj_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_dummy_velocity_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr pub_dummy_steering_;

  // Subscribers
  rclcpp::Subscription<autoware_control_msgs::msg::NonlinearMPCPerformanceStamped>::SharedPtr
    sub_nmpc_msgs_;

  rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
    sub_ctrl_cmd_msgs_;

  boost::optional<autoware_control_msgs::msg::NonlinearMPCPerformanceStamped> nmpc_msgs_;
  boost::optional<autoware_control_msgs::msg::ControlCommandStamped> ctrl_cmd_msgs_;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  // Data members
  double initial_speed{};
  size_t ind_pose_{};
  std::shared_ptr<autoware_planning_msgs::msg::Trajectory> trajectory_;
};

class NonlinearMPCNodeTestSuit : public ::testing::Test
{
public:
  //using nmpc_msg_t = autoware_control_msgs::msg::NonlinearMPCPerformanceStamped;

  void SetUp() override;
  void TearDown() override { (void)rclcpp::shutdown(); }

  // Node being tested.
  std::shared_ptr<NonlinearMPCNode> nmpc_follower_;

  // Node publishing and subscriptions.
  std::shared_ptr<FakeNode> fake_node_{nullptr};

  // Subscription.
  // rclcpp::Subscription<autoware_control_msgs::msg::NonlinearMPCPerformanceStamped>::SharedPtr
  //   sub_nmpc_msgs_;

  // rclcpp::Subscription<autoware_control_msgs::msg::ControlCommandStamped>::SharedPtr
  //   sub_ctrl_cmd_msgs_;

  // boost::optional<autoware_control_msgs::msg::NonlinearMPCPerformanceStamped> nmpc_msgs_;
  // boost::optional<autoware_control_msgs::msg::ControlCommandStamped> ctrl_cmd_msgs_;
};

#endif  // TEST_NONLINEAR_MPC_NODE_HPP_
