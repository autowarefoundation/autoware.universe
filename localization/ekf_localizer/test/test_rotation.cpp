// Copyright 2023  Autoware Foundation
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

#include "ekf_localizer/rotation.hpp"

#include <gtest/gtest.h>


TEST(QuaternionToEulerXYZ, SmokeTest)
{
  // >>> from scipy.spatial.transform import Rotation
  // >>> xyzw = np.array([1., 0., 1., -1.])
  // >>> Rotation.from_quat(xyzw / np.linalg.norm(xyzw)).as_euler('xyz')
  // array([-1.10714872, -0.72972766, -1.10714872])

  const double x = 1.;
  const double y = 0.;
  const double z = 1.;
  const double w = -1.;

  const double scale = Eigen::Vector4d(x, y, z, w).norm();
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = x / scale;
  quaternion.y = y / scale;
  quaternion.z = z / scale;
  quaternion.w = w / scale;

  const Eigen::Vector3d euler = quaternionToEulerXYZ(quaternion);
  Eigen::Vector3d expected(-1.10714872, -0.72972766, -1.10714872);
  EXPECT_LT((euler - expected).norm(), 1e-6);
}

TEST(QuaternionToEulerXYZ, NonScaledXYZW)
{
  // >>> from scipy.spatial.transform import Rotation
  // >>> xyzw = np.array([1., 0., 1., -1.])
  // >>> Rotation.from_quat(xyzw / np.linalg.norm(xyzw)).as_euler('xyz')
  // array([-1.10714872, -0.72972766, -1.10714872])

  const double x = 1.;
  const double y = 0.;
  const double z = 1.;
  const double w = -1.;

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;

  const Eigen::Vector3d euler = quaternionToEulerXYZ(quaternion);
  Eigen::Vector3d expected(-1.10714872, -0.72972766, -1.10714872);
  EXPECT_LT((euler - expected).norm(), 1e-6);
}

TEST(QuaternionToEulerXYZ, Identity)
{
  const double x = 0.;
  const double y = 0.;
  const double z = 0.;
  const double w = 1.;

  const tf2::Quaternion q(x, y, z, w);
  const Eigen::Vector3d euler = quaternionToEulerXYZ(q);
  EXPECT_LT(euler.norm(), 1e-8);  // expect that all elements of euler are zero
}
