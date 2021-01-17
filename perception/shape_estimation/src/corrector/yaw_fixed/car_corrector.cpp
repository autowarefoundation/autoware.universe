// Copyright 2020 TierIV
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

#include "car_corrector.hpp"
#include "tf2_eigen/tf2_eigen.h"

namespace yaw_fixed
{
bool CarCorrector::correct(
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  const double min_width = 1.2;
  const double max_width = 2.2;
  const double min_length = 3.0;
  const double max_length = 5.0;

  /*
    c1 is nearest point and other points are arranged like below
    c is center of bounding box
    width
    4---2
    |   |
    | c |length
    |   |
    3---1
   */

  Eigen::Vector3d c1, c2, c3, c4;

  Eigen::Affine3d base2obj_transform;
  tf2::fromMsg(pose_output, base2obj_transform);

  std::vector<Eigen::Vector3d> v_point;
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(shape_output.dimensions.x * 0.5, shape_output.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(-shape_output.dimensions.x * 0.5, shape_output.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(shape_output.dimensions.x * 0.5, -shape_output.dimensions.y * 0.5, 0.0));
  v_point.push_back(
    base2obj_transform *
    Eigen::Vector3d(-shape_output.dimensions.x * 0.5, -shape_output.dimensions.y * 0.5, 0.0));

  double distance = std::pow(24, 24);
  size_t nearest_idx = 0;
  for (size_t i = 0; i < v_point.size(); i++) {
    if (v_point.at(i).norm() < distance) {
      distance = v_point.at(i).norm();
      nearest_idx = i;
    }
  }

  c1 = v_point.at(nearest_idx);

  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_c1 = base2obj_transform.inverse() * c1;
  Eigen::Vector3d radiation_vec = c - local_c1;

  double ex = radiation_vec.x();
  double ey = radiation_vec.y();
  Eigen::Vector3d e1 = (Eigen::Vector3d(0, -ey, 0) - local_c1).normalized();
  Eigen::Vector3d e2 = (Eigen::Vector3d(-ex, 0, 0) - local_c1).normalized();

  double length = 0;
  if (min_length < shape_output.dimensions.x && shape_output.dimensions.x < max_length) {
    length = shape_output.dimensions.x;
  } else {
    length = (max_length + min_length) * 0.5;
  }
  double width = 0;
  if (min_width < shape_output.dimensions.y && shape_output.dimensions.y < max_width) {
    width = shape_output.dimensions.y;
  } else {
    width = (max_width + min_width) * 0.5;
  }

  c2 = c1 + base2obj_transform.rotation() * (e1 * length);
  c3 = c1 + base2obj_transform.rotation() * (e2 * width);
  c4 = c1 + (c2 - c1) + (c3 - c1);

  shape_output.dimensions.x = (c2 - c1).norm();
  shape_output.dimensions.y = (c3 - c1).norm();
  Eigen::Vector3d new_centroid = c1 + ((c4 - c1) * 0.5);
  pose_output.position.x = new_centroid.x();
  pose_output.position.y = new_centroid.y();
  pose_output.position.z = new_centroid.z();

  return true;
}
}  // namespace yaw_fixed
