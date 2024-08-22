#include <gtest/gtest.h>
#include <memory>
#include "pid_longitudinal_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

using namespace autoware::motion::control::pid_longitudinal_controller;

class TestPidLongitudinalController : public ::testing::Test
{
protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<PidLongitudinalController> controller_;
    std::shared_ptr<diagnostic_updater::Updater> diag_updater_;

    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_node");
        diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
        controller_ = std::make_shared<PidLongitudinalController>(*node_, diag_updater_);
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

TEST_F(TestPidLongitudinalController, setupDiagnosticUpdater)
{
    controller_->setupDiagnosticUpdater();
    EXPECT_EQ(diag_updater_->getHardwareID(), "autoware_pid_longitudinal_controller");

    bool control_state_added = false;
    for (const auto & task : diag_updater_->getDiagnosticTasks())
    {
        if (task.name == "control_state")
        {
            control_state_added = true;
            break;
        }
    }
    EXPECT_TRUE(control_state_added);
}

TEST_F(TestPidLongitudinalController, setKinematicState)
{
    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = 10.0;
    controller_->setKinematicState(odom);
    EXPECT_EQ(controller_->m_current_kinematic_state.twist.twist.linear.x, 10.0);
}

TEST_F(TestPidLongitudinalController, setCurrentAcceleration)
{
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.accel.accel.linear.x = 2.0;
    controller_->setCurrentAcceleration(accel);
    EXPECT_EQ(controller_->m_current_accel.accel.accel.linear.x, 2.0);
}

TEST_F(TestPidLongitudinalController, setCurrentOperationMode)
{
    autoware_adapi_v1_msgs::msg::OperationModeState mode;
    mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
    controller_->setCurrentOperationMode(mode);
    EXPECT_EQ(controller_->m_current_operation_mode.mode, autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS);
}

TEST_F(TestPidLongitudinalController, setTrajectory)
{
    autoware_planning_msgs::msg::Trajectory traj;
    traj.points.resize(3);
    controller_->setTrajectory(traj);
    EXPECT_EQ(controller_->m_trajectory.points.size(), 3);
}

TEST_F(TestPidLongitudinalController, isReady)
{
    trajectory_follower::InputData input_data;
    EXPECT_TRUE(controller_->isReady(input_data));
}

TEST_F(TestPidLongitudinalController, run)
{
    trajectory_follower::InputData input_data;
    autoware_planning_msgs::msg::Trajectory traj;
    traj.points.resize(2);
    input_data.current_trajectory = traj;

    nav_msgs::msg::Odometry odom;
    odom.twist.twist.linear.x = 1.0;
    input_data.current_odometry = odom;

    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.accel.accel.linear.x = 0.5;
    input_data.current_accel = accel;

    autoware_adapi_v1_msgs::msg::OperationModeState mode;
    mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
    input_data.current_operation_mode = mode;
    controller_->m_control_state = PidLongitudinalController::ControlState::DRIVE;
    auto output = controller_->run(input_data);
    EXPECT_EQ(output.control_cmd.velocity, 0.0);
}

TEST_F(TestPidLongitudinalController, getControlData) // more tests can be added
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 1.0;
    controller_->m_current_kinematic_state.twist.twist.linear.x = 1.0;

    auto control_data = controller_->getControlData(pose);
    EXPECT_EQ(control_data.current_motion.vel, 1.0);
}

TEST_F(TestPidLongitudinalController, calcEmergencyCtrlCmd)
{
    double dt = 0.1;
    controller_->m_prev_raw_ctrl_cmd.vel = 0.0;
    auto cmd = controller_->calcEmergencyCtrlCmd(dt);
    EXPECT_EQ(cmd.vel, 0.0);
}

TEST_F(TestPidLongitudinalController, updateControlState)
{
    PidLongitudinalController::ControlData control_data;
    controller_->m_control_state = ControlState::DRIVE;
    control_data.current_motion.vel = 5.0;
    control_data.stop_dist = -5.0;
    control_data.nearest_idx = 0;
    control_data.interpolated_traj.points.at(control_data.nearest_idx).longitudinal_velocity_mps = 0.0;

    controller_->updateControlState(control_data);
    EXPECT_EQ(controller_->m_control_state, PidLongitudinalController::ControlState::EMERGENCY);
}

TEST_F(TestPidLongitudinalController, calcCtrlCmd)
{
    PidLongitudinalController::ControlData control_data;
    controller_->m_control_state == ControlState::STOPPED;
    controller_->m_prev_ctrl_cmd.acc = 0.0;
    control_data.target_idx = 0;
    control_data.interpolated_traj.points.at(control_data.target_idx).longitudinal_velocity_mps = 0.0;
    control_data.interpolated_traj.points.at(control_data.target_idx).acceleration_mps2 = 0.0;

    auto cmd = controller_->calcCtrlCmd(control_data);
    EXPECT_EQ(cmd.acc, -3.4);
}

TEST_F(TestPidLongitudinalController, createCtrlCmdMsg)
{
    PidLongitudinalController::Motion cmd;
    cmd.vel = 10.0;
    cmd.acc = 1.0;

    auto ctrl_cmd_msg = controller_->createCtrlCmdMsg(cmd, 5.0);
    EXPECT_EQ(ctrl_cmd_msg.velocity, 10.0);
    EXPECT_EQ(ctrl_cmd_msg.acceleration, 1.0);
}

TEST_F(TestPidLongitudinalController, getDt)
{
    auto dt = controller_->getDt();
    EXPECT_GT(dt, 0.03);
}

TEST_F(TestPidLongitudinalController, getCurrentShift)
{
    PidLongitudinalController::ControlData control_data;
    control_data.target_idx = 0;
    control_data.interpolated_traj.points[control_data.target_idx].longitudinal_velocity_mps = 1.0;

    auto shift = controller_->getCurrentShift(control_data);
    EXPECT_EQ(shift, PidLongitudinalController::Shift::Forward);
}

TEST_F(TestPidLongitudinalController, storeAccelCmd)
{
    controller_->m_ctrl_cmd_vec.clear();
    controller_->storeAccelCmd(1.0);
    EXPECT_FALSE(controller_->m_ctrl_cmd_vec.empty());
}

TEST_F(TestPidLongitudinalController, applySlopeCompensation)
{
    auto acc = controller_->applySlopeCompensation(1.0, 0.05, PidLongitudinalController::Shift::Forward);
    EXPECT_EQ(acc, 1.0 + 9.81 * std::sin(0.05));
}

TEST_F(TestPidLongitudinalController, keepBrakeBeforeStop)
{
    PidLongitudinalController::ControlData control_data;
    PidLongitudinalController::Motion target_motion;
    target_motion.acc = 1.0;
    auto motion = controller_->keepBrakeBeforeStop(control_data, target_motion, 0);
    EXPECT_EQ(motion.acc, 1.0);
}

TEST_F(TestPidLongitudinalController, predictedStateAfterDelay)
{
    PidLongitudinalController::Motion current_motion;
    current_motion.vel = 10.0;
    current_motion.acc = 1.0;
    controller_->m_ctrl_cmd_vec.clear();
    auto state = controller_->predictedStateAfterDelay(current_motion, 0.5);
    EXPECT_GT(state.vel, 10.0 + 1.0 * 0.5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
