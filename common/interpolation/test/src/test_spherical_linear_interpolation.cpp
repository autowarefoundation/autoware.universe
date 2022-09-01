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

#include "interpolation/spherical_linear_interpolation.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

TEST(spherical_linear_interpolation, slerp_scalar)
{
  using interpolation::spherical_linear_interpolation;

  // Same value
  {
    const double src_yaw = 0.0;
    const double dst_yaw = 0.0;
    tf2::Quaternion src_quat;
    tf2::Quaternion dst_quat;
    src_quat.setRPY(0, 0, src_yaw);
    dst_quat.setRPY(0, 0, dst_yaw);

    const double ans_yaw = 0.0;
    tf2::Quaternion ans;
    ans.setRPY(0, 0, ans_yaw);
    const geometry_msgs::msg::Quaternion ans_quat = tf2::toMsg(ans);

    for (double ratio = -2.0; ratio < 2.0 + epsilon; ratio += 0.1) {
      const auto interpolated_quat =
        spherical_linear_interpolation(tf2::toMsg(src_quat), tf2::toMsg(dst_quat), ratio);
      EXPECT_NEAR(ans_quat.x, interpolated_quat.x, epsilon);
      EXPECT_NEAR(ans_quat.y, interpolated_quat.y, epsilon);
      EXPECT_NEAR(ans_quat.z, interpolated_quat.z, epsilon);
      EXPECT_NEAR(ans_quat.w, interpolated_quat.w, epsilon);
    }
  }

  // Random Value
  {
    const double src_yaw = 0.0;
    const double dst_yaw = M_PI;
    tf2::Quaternion src_quat;
    tf2::Quaternion dst_quat;
    src_quat.setRPY(0, 0, src_yaw);
    dst_quat.setRPY(0, 0, dst_yaw);

    for (double ratio = -2.0; ratio < 2.0 + epsilon; ratio += 0.1) {
      const auto interpolated_quat =
        spherical_linear_interpolation(tf2::toMsg(src_quat), tf2::toMsg(dst_quat), ratio);

      const double ans_yaw = M_PI * ratio;
      tf2::Quaternion ans;
      ans.setRPY(0, 0, ans_yaw);
      const geometry_msgs::msg::Quaternion ans_quat = tf2::toMsg(ans);

      EXPECT_NEAR(ans_quat.x, interpolated_quat.x, epsilon);
      EXPECT_NEAR(ans_quat.y, interpolated_quat.y, epsilon);
      EXPECT_NEAR(ans_quat.z, interpolated_quat.z, epsilon);
      EXPECT_NEAR(ans_quat.w, interpolated_quat.w, epsilon);
    }
  }
}
