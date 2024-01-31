// Copyright 2023 Tier IV, Inc.
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

#include "radar_crossing_objects_noise_filter/radar_crossing_objects_noise_filter_node.hpp"

#include <gtest/gtest.h>

std::shared_ptr<radar_crossing_objects_noise_filter::RadarCrossingObjectsNoiseFilterNode> get_node(
  float angle_threshold, float velocity_threshold)
{
  rclcpp::NodeOptions node_options;

  node_options.parameter_overrides(
    {{"angle_threshold", angle_threshold}, {"velocity_threshold", velocity_threshold}});
  auto node =
    std::make_shared<radar_crossing_objects_noise_filter::RadarCrossingObjectsNoiseFilterNode>(
      node_options);
  return node;
}

std::shared_ptr<autoware_auto_perception_msgs::msg::DetectedObject> get_object(
  float twist_x, float twist_y, float twist_z, float orientation_x, float orientation_y,
  float orientation_z, float orientation_w, float pose_x, float pose_y)
{
  auto ret = std::make_shared<autoware_auto_perception_msgs::msg::DetectedObject>();
  ret->kinematics.twist_with_covariance.twist.linear.x = static_cast<double>(twist_x);
  ret->kinematics.twist_with_covariance.twist.linear.y = static_cast<double>(twist_y);
  ret->kinematics.twist_with_covariance.twist.linear.z = static_cast<double>(twist_z);
  ret->kinematics.pose_with_covariance.pose.position.x = static_cast<double>(pose_x);
  ret->kinematics.pose_with_covariance.pose.position.y = static_cast<double>(pose_y);
  ret->kinematics.pose_with_covariance.pose.orientation.x = static_cast<double>(orientation_x);
  ret->kinematics.pose_with_covariance.pose.orientation.y = static_cast<double>(orientation_y);
  ret->kinematics.pose_with_covariance.pose.orientation.z = static_cast<double>(orientation_z);
  ret->kinematics.pose_with_covariance.pose.orientation.w = static_cast<double>(orientation_w);
  return ret;
}

TEST(RadarCrossingObjectsFilter, IsNoise)
{
  rclcpp::init(0, nullptr);

  {
    float twist_x = 40.0;
    float twist_y = 30.0;
    float twist_z = 0.0;
    float orientation_x = 1.0;
    float orientation_y = 1.0;
    float orientation_z = 1.0;
    float orientation_w = 0.0;
    float pose_x = 1.0;
    float pose_y = 0.0;
    auto object = get_object(
      twist_x, twist_y, twist_z, orientation_x, orientation_y, orientation_z, orientation_w, pose_x,
      pose_y);
    {
      float velocity_threshold = 40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(*object));
    }
    {
      float velocity_threshold = 40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(*object));
    }
  }

  {
    float twist_x = 40.0;
    float twist_y = 30.0;
    float twist_z = 0.0;
    float orientation_x = 1.0;
    float orientation_y = 1.0;
    float orientation_z = 1.0;
    float orientation_w = 0.0;
    float pose_x = 1.0;
    float pose_y = 2.0;
    auto object = get_object(
      twist_x, twist_y, twist_z, orientation_x, orientation_y, orientation_z, orientation_w, pose_x,
      pose_y);
    {
      float velocity_threshold = 40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = 40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
  }

  {
    float twist_x = 24.0;
    float twist_y = 18.0;
    float twist_z = 0.0;
    float orientation_x = 1.0;
    float orientation_y = 1.0;
    float orientation_z = 1.0;
    float orientation_w = 0.0;
    float pose_x = 1.0;
    float pose_y = 0.0;
    auto object = get_object(
      twist_x, twist_y, twist_z, orientation_x, orientation_y, orientation_z, orientation_w, pose_x,
      pose_y);
    {
      float velocity_threshold = 40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = 40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(*object));
    }
  }

  {
    float twist_x = 24.0;
    float twist_y = 18.0;
    float twist_z = 0.0;
    float orientation_x = 1.0;
    float orientation_y = 1.0;
    float orientation_z = 1.0;
    float orientation_w = 0.0;
    float pose_x = 1.0;
    float pose_y = 2.0;
    auto object = get_object(
      twist_x, twist_y, twist_z, orientation_x, orientation_y, orientation_z, orientation_w, pose_x,
      pose_y);
    {
      float velocity_threshold = 40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = 40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
    {
      float velocity_threshold = -40.0;
      float angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(*object));
    }
  }
}
