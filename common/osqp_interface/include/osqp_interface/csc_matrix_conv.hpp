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
#ifndef OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_
#define OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_

#include <eigen3/Eigen/Core>

#include <osqp/types.h>  // for 'c_int' type ('long' or 'long long')

#include <vector>

namespace osqp
{
// Struct for containing a 'Compressed-Column-Sparse Matrix' object.
//
// Elements:
//   vals: Vector of non-zero values. Ex: [4,1,1,2]
//   row_idxs:  Vector of row index corresponding to values. Ex: [0, 1, 0, 1] (Eigen: 'inner')
//   col_idxs:  Vector of 'val' indices where each column starts. Ex: [0, 2, 4] (Eigen: 'outer')
struct CSC_Matrix
{
  std::vector<c_float> vals;
  std::vector<c_int> row_idxs;
  std::vector<c_int> col_idxs;
};

CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat);
CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat);

void printCSCMatrix(CSC_Matrix & csc_mat);

}  // namespace osqp

#endif  // OSQP_INTERFACE__CSC_MATRIX_CONV_HPP_
