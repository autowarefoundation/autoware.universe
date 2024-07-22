// Copyright 2024 TIER IV
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

#include "test.hpp"

#include "autoware/autonomous_emergency_braking/node.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/shape__struct.hpp>

#include <gtest/gtest.h>

namespace autoware::motion::control::autonomous_emergency_braking::test
{
using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Header;

Header get_header(const char * const frame_id, rclcpp::Time t)
{
  std_msgs::msg::Header header;
  header.stamp = t;
  header.frame_id = frame_id;
  return header;
};

Imu make_imu_message(
  const Header & header, const double ax, const double ay, const double yaw,
  const double angular_velocity_z)
{
  Imu imu_msg;
  imu_msg.header = header;
  imu_msg.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  imu_msg.angular_velocity.z = angular_velocity_z;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  return imu_msg;
};

VelocityReport make_velocity_report_msg(
  const Header & header, const double lat_velocity, const double long_velocity,
  const double heading_rate)
{
  VelocityReport velocity_msg;
  velocity_msg.header = header;
  velocity_msg.lateral_velocity = lat_velocity;
  velocity_msg.longitudinal_velocity = long_velocity;
  velocity_msg.heading_rate = heading_rate;
  return velocity_msg;
}

std::shared_ptr<AEB> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto aeb_dir =
    ament_index_cpp::get_package_share_directory("autoware_autonomous_emergency_braking");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("autoware_vehicle_info_utils");

  node_options.arguments(
    {"--ros-args", "--params-file", aeb_dir + "/config/autonomous_emergency_braking.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});
  return std::make_shared<AEB>(node_options);
};

TEST_F(TestAEB, checkCollision)
{
  constexpr double longitudinal_velocity = 3.0;
  ObjectData object_collision;
  object_collision.distance_to_object = 0.5;
  object_collision.velocity = 0.1;
  ASSERT_TRUE(aeb_node_->hasCollision(longitudinal_velocity, object_collision));

  ObjectData object_no_collision;
  object_no_collision.distance_to_object = 10.0;
  object_no_collision.velocity = 0.1;
  ASSERT_FALSE(aeb_node_->hasCollision(longitudinal_velocity, object_no_collision));
}

TEST_F(TestAEB, checkImuPathGeneration)
{
  constexpr double longitudinal_velocity = 3.0;
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(longitudinal_velocity, yaw_rate);
  ASSERT_FALSE(imu_path.empty());

  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));
}

TEST_F(TestAEB, checkIncompleteImuPathGeneration)
{
  const double dt = aeb_node_->imu_prediction_time_interval_;
  const double horizon = aeb_node_->imu_prediction_time_horizon_;
  const double min_generated_path_length = aeb_node_->min_generated_path_length_;
  const double slow_velocity = min_generated_path_length / (2.0 * horizon);
  constexpr double yaw_rate = 0.05;
  const auto imu_path = aeb_node_->generateEgoPath(slow_velocity, yaw_rate);

  ASSERT_FALSE(imu_path.empty());
  ASSERT_TRUE(imu_path.size() >= static_cast<size_t>(horizon / dt));
  ASSERT_TRUE(autoware::motion_utils::calcArcLength(imu_path) >= min_generated_path_length);

  const auto footprint = aeb_node_->generatePathFootprint(imu_path, 0.0);
  ASSERT_FALSE(footprint.empty());
  ASSERT_TRUE(footprint.size() == imu_path.size() - 1);
}

TEST_F(TestAEB, checkEmptyPathAtZeroSpeed)
{
  const double velocity = 0.0;
  constexpr double yaw_rate = 0.0;
  const auto imu_path = aeb_node_->generateEgoPath(velocity, yaw_rate);
  ASSERT_EQ(imu_path.size(), 1);
}

TEST_F(TestAEB, checkParamUpdate)
{
  std::vector<rclcpp::Parameter> parameters{rclcpp::Parameter("param")};
  const auto result = aeb_node_->onParameter(parameters);
  ASSERT_TRUE(result.successful);
}

TEST_F(TestAEB, checkEmptyFetchData)
{
  ASSERT_FALSE(aeb_node_->fetchLatestData());
}

TEST_F(TestAEB, checkConvertObjectToPolygon)
{
  using autoware_perception_msgs::msg::Shape;
  PredictedObject obj_cylinder;
  obj_cylinder.shape.type = Shape::CYLINDER;
  obj_cylinder.shape.dimensions.x = 1.0;
  Pose obj_cylinder_pose;
  obj_cylinder_pose.position.x = 1.0;
  obj_cylinder_pose.position.y = 1.0;
  obj_cylinder.kinematics.initial_pose_with_covariance.pose = obj_cylinder_pose;
  const auto cylinder_polygon = utils::convertObjToPolygon(obj_cylinder);
  ASSERT_FALSE(cylinder_polygon.outer().empty());

  PredictedObject obj_box;
  obj_box.shape.type = Shape::BOUNDING_BOX;
  obj_box.shape.dimensions.x = 1.0;
  obj_box.shape.dimensions.y = 2.0;
  Pose obj_box_pose;
  obj_box_pose.position.x = 1.0;
  obj_box_pose.position.y = 1.0;
  obj_box.kinematics.initial_pose_with_covariance.pose = obj_box_pose;
  const auto box_polygon = utils::convertObjToPolygon(obj_box);
  ASSERT_FALSE(box_polygon.outer().empty());
}

TEST_F(TestAEB, checkDataFetch)
{
  pub_sub_node_->publishDefaultTopicsNoSpin();
  for (int j = 0; j < 20; ++j) {
    rclcpp::spin_some(pub_sub_node_->get_node_base_interface());
    rclcpp::spin_some(aeb_node_->get_node_base_interface());
    aeb_node_->fetchLatestData();
  }

  bool g{false};
  if (aeb_node_->current_velocity_ptr_) g = true;

  ASSERT_TRUE(g);
}
}  // namespace autoware::motion::control::autonomous_emergency_braking::test
