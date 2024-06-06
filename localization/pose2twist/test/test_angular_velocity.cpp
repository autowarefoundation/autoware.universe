// Copyright 2022 The Autoware Contributors
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

#include "pose2twist/pose2twist_core.hpp"

#include <gtest/gtest.h>

TEST(AngularVelocityFromQuaternion, CheckNumericalValidity)
{
  tf2::Quaternion initial_q;
  initial_q.setRPY(0.2, 0.3, 0.4);

  const tf2::Vector3 expected_axis = tf2::Vector3(0.1, 0.2, 0.3).normalized();
  const double expected_angle = 0.1;  // 0.1 radian = 5.7 degrees

  tf2::Quaternion expected_q;
  expected_q.setRotation(expected_axis, expected_angle);

  const tf2::Quaternion final_q = initial_q * expected_q;

  const geometry_msgs::msg::Vector3 rotation_vector =
    rotation_vector_from_quaternion(initial_q, final_q);

  // 1e-3 radian = 0.057 degrees
  constexpr double ACCEPTABLE_ERROR = 1e-3;

  EXPECT_NEAR(rotation_vector.x, expected_axis.x() * expected_angle, ACCEPTABLE_ERROR);
  EXPECT_NEAR(rotation_vector.y, expected_axis.y() * expected_angle, ACCEPTABLE_ERROR);
  EXPECT_NEAR(rotation_vector.z, expected_axis.z() * expected_angle, ACCEPTABLE_ERROR);
}
