// Copyright 2018-2019 Autoware Foundation
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

#include "ekf_localizer/ekf_localizer.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;

class EKFLocalizerTestSuite : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { (void)rclcpp::shutdown(); }
};  // sanity_check

class TestEKFLocalizerNode : public EKFLocalizer
{
public:
  TestEKFLocalizerNode(const std::string & node_name, const rclcpp::NodeOptions & node_options)
  : EKFLocalizer(node_name, node_options)
  {
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ekf_odom", 1, std::bind(&TestEKFLocalizerNode::testCallbackOdom, this, _1));

    using std::chrono_literals::operator""ms;
    test_timer_ = rclcpp::create_timer(
      this, get_clock(), 100ms, std::bind(&TestEKFLocalizerNode::testTimerCallback, this));
  }
  ~TestEKFLocalizerNode() {}

  std::string frame_id_a_ = "map";
  std::string frame_id_b_ = "base_link";

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

  rclcpp::TimerBase::SharedPtr test_timer_;

  nav_msgs::msg::Odometry::SharedPtr test_current_odom_ptr_;

  void testTimerCallback()
  {
    /* !!! this should be defined before sendTransform() !!! */
    static std::shared_ptr<tf2_ros::TransformBroadcaster> br =
      std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    geometry_msgs::msg::TransformStamped sent;

    rclcpp::Time current_time = this->now();

    sent.header.stamp = current_time;
    sent.header.frame_id = frame_id_a_;
    sent.child_frame_id = frame_id_b_;
    sent.transform.translation.x = -7.11;
    sent.transform.translation.y = 0.0;
    sent.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.5);
    sent.transform.rotation.x = q.x();
    sent.transform.rotation.y = q.y();
    sent.transform.rotation.z = q.z();
    sent.transform.rotation.w = q.w();

    br->sendTransform(sent);
  }

  void testCallbackOdom(nav_msgs::msg::Odometry::SharedPtr pose)
  {
    test_current_odom_ptr_ = std::make_shared<nav_msgs::msg::Odometry>(*pose);
  }

  void resetCurrentPoseAndTwist() { test_current_odom_ptr_ = nullptr; }
};

TEST_F(EKFLocalizerTestSuite, measurementUpdateTest)
{
  rclcpp::NodeOptions node_options;
  auto ekf = std::make_shared<TestEKFLocalizerNode>("EKFLocalizerTestSuite", node_options);

  auto pub_pose = ekf->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/in_pose_with_covariance", 1);

  /* test for valid value in pose */
  geometry_msgs::msg::PoseWithCovarianceStamped in_pose;
  in_pose.header.frame_id = "map";
  in_pose.pose.pose.position.x = 1.0;
  in_pose.pose.pose.position.y = 2.0;
  in_pose.pose.pose.position.z = 3.0;
  in_pose.pose.pose.orientation.x = 0.0;
  in_pose.pose.pose.orientation.y = 0.0;
  in_pose.pose.pose.orientation.z = 0.0;
  in_pose.pose.pose.orientation.w = 1.0;
  for (int i = 0; i < 36; ++i) {
    in_pose.pose.covariance[i] = 0.1;
  }

  const double pos_x = 12.3;
  in_pose.pose.pose.position.x = pos_x;  // for valid value

  for (int i = 0; i < 20; ++i) {
    in_pose.header.stamp = ekf->now();
    pub_pose->publish(in_pose);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_NE(ekf->test_current_odom_ptr_, nullptr);

  double ekf_x = ekf->test_current_odom_ptr_->pose.pose.position.x;
  bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1)
    << "ekf pos x: " << ekf_x << " should be close to " << pos_x;

  /* test for invalid value in pose */
  in_pose.pose.pose.position.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_pose.header.stamp = ekf->now();
    pub_pose->publish(in_pose);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ekf->resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdateTwist)
{
  rclcpp::NodeOptions node_options;
  auto ekf = std::make_shared<TestEKFLocalizerNode>("EKFLocalizerTestSuite", node_options);

  auto pub_twist = ekf->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/in_twist_with_covariance", 1);

  /* test for valid value in twist */
  geometry_msgs::msg::TwistWithCovarianceStamped in_twist;
  in_twist.header.frame_id = "base_link";
  const double vx = 12.3;
  in_twist.twist.twist.linear.x = vx;  // for valid value
  for (int i = 0; i < 36; ++i) {
    in_twist.twist.covariance[i] = 0.1;
  }
  for (int i = 0; i < 20; ++i) {
    in_twist.header.stamp = ekf->now();
    pub_twist->publish(in_twist);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_FALSE(ekf->test_current_odom_ptr_ == nullptr);

  double ekf_vx = ekf->test_current_odom_ptr_->twist.twist.linear.x;
  bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE(std::fabs(ekf_vx - vx) < 0.1)
    << "ekf vel x: " << ekf_vx << ", should be close to " << vx;

  /* test for invalid value in twist */
  in_twist.twist.twist.linear.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ekf->now();
    pub_twist->publish(in_twist);
    rclcpp::spin_some(ekf);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  ekf_vx = ekf->test_current_odom_ptr_->twist.twist.linear.x;
  is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  ekf->resetCurrentPoseAndTwist();
}
