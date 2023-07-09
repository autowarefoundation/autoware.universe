// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef RADAR_OBJECT_TRACKER__UTILS__UTILS_HPP_
#define RADAR_OBJECT_TRACKER__UTILS__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

namespace utils
{
enum MSG_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};

// concatenate matrices vertically
Eigen::MatrixXd stackMatricesVertically(const std::vector<Eigen::MatrixXd> & matrices)
{
  int totalRows = 0;
  int cols = -1;

  // calculate total number of rows and check that all matrices have the same number of columns
  for (const auto & matrix : matrices) {
    totalRows += matrix.rows();
    if (cols == -1) {
      cols = matrix.cols();
    } else if (cols != matrix.cols()) {
      throw std::invalid_argument("All matrices must have the same number of columns.");
    }
  }

  Eigen::MatrixXd result(totalRows, cols);

  int currentRow = 0;
  for (const auto & matrix : matrices) {
    // copy each matrix into result
    result.block(currentRow, 0, matrix.rows(), cols) = matrix;
    currentRow += matrix.rows();
  }
  return result;
}

// concatenate matrices diagonally
Eigen::MatrixXd stackMatricesDiagonally(const std::vector<Eigen::MatrixXd> & matrices)
{
  int dimension = 0;

  // calc dimension of result matrix
  for (const auto & matrix : matrices) {
    if (matrix.rows() != matrix.cols()) {
      throw std::invalid_argument("All matrices must be square.");
    }
    dimension += matrix.rows();
  }

  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(dimension, dimension);

  int currentDimension = 0;
  for (const auto & matrix : matrices) {
    // copy each matrix into result
    result.block(currentDimension, currentDimension, matrix.rows(), matrix.cols()) = matrix;
    currentDimension += matrix.rows();
  }

  return result;
}

}  // namespace utils

#endif  // RADAR_OBJECT_TRACKER__UTILS__UTILS_HPP_
