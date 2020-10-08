/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <iostream>

#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include "ekf_localizer/ekf_localizer.h"

class EKFLocalizerTestSuite : public ::testing::Test
{
public:
  EKFLocalizerTestSuite() : nh_(""), pnh_("~")
  {
    sub_twist = nh_.subscribe("/ekf_twist", 1, &EKFLocalizerTestSuite::callbackTwist, this);
    sub_pose = nh_.subscribe("/ekf_pose", 1, &EKFLocalizerTestSuite::callbackPose, this);
    tiemr_ = nh_.createTimer(ros::Duration(0.1), &EKFLocalizerTestSuite::timerCallback, this);
  }
  ~EKFLocalizerTestSuite() {}

  ros::NodeHandle nh_, pnh_;

  std::string frame_id_a_ = "world";
  std::string frame_id_b_ = "base_link";

  ros::Subscriber sub_twist;
  ros::Subscriber sub_pose;

  ros::Timer tiemr_;

  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;

  void timerCallback(const ros::TimerEvent & e)
  {
    /* !!! this should be defined before sendTransform() !!! */
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped sended;

    ros::Time current_time = ros::Time::now();

    sended.header.stamp = current_time;
    sended.header.frame_id = frame_id_a_;
    sended.child_frame_id = frame_id_b_;
    sended.transform.translation.x = -7.11;
    sended.transform.translation.y = 0.0;
    sended.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0.5);
    sended.transform.rotation.x = q.x();
    sended.transform.rotation.y = q.y();
    sended.transform.rotation.z = q.z();
    sended.transform.rotation.w = q.w();

    br.sendTransform(sended);
  };

  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr & pose)
  {
    current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(*pose);
  }

  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr & twist)
  {
    current_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*twist);
  }

  void resetCurrentPoseAndTwist()
  {
    current_pose_ptr_ = nullptr;
    current_twist_ptr_ = nullptr;
  }
};

TEST_F(EKFLocalizerTestSuite, measurementUpdatePose)
{
  EKFLocalizer ekf_;

  ros::Publisher pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("/in_pose", 1);
  geometry_msgs::PoseStamped in_pose;
  in_pose.header.frame_id = "world";
  in_pose.pose.position.x = 1.0;
  in_pose.pose.position.y = 2.0;
  in_pose.pose.position.z = 3.0;
  in_pose.pose.orientation.x = 0.0;
  in_pose.pose.orientation.y = 0.0;
  in_pose.pose.orientation.z = 0.0;
  in_pose.pose.orientation.w = 1.0;

  /* test for valid value */
  const double pos_x = 12.3;
  in_pose.pose.position.x = pos_x;  // for vaild value

  for (int i = 0; i < 20; ++i) {
    in_pose.header.stamp = ros::Time::now();
    pub_pose.publish(in_pose);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ASSERT_FALSE(current_pose_ptr_ == nullptr);
  ASSERT_FALSE(current_twist_ptr_ == nullptr);

  double ekf_x = current_pose_ptr_->pose.position.x;
  bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1)
    << "ekf pos x: " << ekf_x << " should be close to " << pos_x;

  /* test for invalid value */
  in_pose.pose.position.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_pose.header.stamp = ros::Time::now();
    pub_pose.publish(in_pose);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdateTwist)
{
  EKFLocalizer ekf_;

  ros::Publisher pub_twist = nh_.advertise<geometry_msgs::TwistStamped>("/in_twist", 1);
  geometry_msgs::TwistStamped in_twist;
  in_twist.header.frame_id = "base_link";

  /* test for valid value */
  const double vx = 12.3;
  in_twist.twist.linear.x = vx;  // for vaild value
  for (int i = 0; i < 20; ++i) {
    in_twist.header.stamp = ros::Time::now();
    pub_twist.publish(in_twist);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ASSERT_FALSE(current_pose_ptr_ == nullptr);
  ASSERT_FALSE(current_twist_ptr_ == nullptr);

  double ekf_vx = current_twist_ptr_->twist.linear.x;
  bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE(std::fabs(ekf_vx - vx) < 0.1)
    << "ekf vel x: " << ekf_vx << ", should be close to " << vx;

  /* test for invalid value */
  in_twist.twist.linear.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ros::Time::now();
    pub_twist.publish(in_twist);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ekf_vx = current_twist_ptr_->twist.linear.x;
  is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdatePoseWithCovariance)
{
  pnh_.setParam("use_pose_with_covariance", true);
  ros::Duration(0.2).sleep();
  EKFLocalizer ekf_;

  ros::Publisher pub_pose =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/in_pose_with_covariance", 1);
  geometry_msgs::PoseWithCovarianceStamped in_pose;
  in_pose.header.frame_id = "world";
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

  /* test for valid value */
  const double pos_x = 99.3;
  in_pose.pose.pose.position.x = pos_x;  // for vaild value

  for (int i = 0; i < 20; ++i) {
    in_pose.header.stamp = ros::Time::now();
    pub_pose.publish(in_pose);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ASSERT_FALSE(current_pose_ptr_ == nullptr);
  ASSERT_FALSE(current_twist_ptr_ == nullptr);

  double ekf_x = current_pose_ptr_->pose.position.x;
  bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1)
    << "ekf pos x: " << ekf_x << " should be close to " << pos_x;

  /* test for invalid value */
  in_pose.pose.pose.position.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_pose.header.stamp = ros::Time::now();
    pub_pose.publish(in_pose);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

  resetCurrentPoseAndTwist();
}

TEST_F(EKFLocalizerTestSuite, measurementUpdateTwistWithCovariance)
{
  EKFLocalizer ekf_;

  ros::Publisher pub_twist =
    nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/in_twist_with_covariance", 1);
  geometry_msgs::TwistWithCovarianceStamped in_twist;
  in_twist.header.frame_id = "base_link";

  /* test for valid value */
  const double vx = 12.3;
  in_twist.twist.twist.linear.x = vx;  // for vaild value
  for (int i = 0; i < 36; ++i) {
    in_twist.twist.covariance[i] = 0.1;
  }
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ros::Time::now();
    pub_twist.publish(in_twist);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ASSERT_FALSE(current_pose_ptr_ == nullptr);
  ASSERT_FALSE(current_twist_ptr_ == nullptr);

  double ekf_vx = current_twist_ptr_->twist.linear.x;
  bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
  ASSERT_TRUE((ekf_vx - vx) < 0.1) << "vel x should be close to " << vx;

  /* test for invalid value */
  in_twist.twist.twist.linear.x = NAN;  // check for invalid values
  for (int i = 0; i < 10; ++i) {
    in_twist.header.stamp = ros::Time::now();
    pub_twist.publish(in_twist);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ekf_vx = current_twist_ptr_->twist.linear.x;
  is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
  ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "EKFLocalizerTestSuite");

  return RUN_ALL_TESTS();
}