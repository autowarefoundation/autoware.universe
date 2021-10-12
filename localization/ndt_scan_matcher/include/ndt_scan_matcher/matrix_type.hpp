#ifndef NDT_SCAN_MATCHER__MATRIX_TYPE_HPP__
#define NDT_SCAN_MATCHER__MATRIX_TYPE_HPP__

#include "eigen3/Eigen/Core"

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

#endif  // NDT_SCAN_MATCHER__MATRIX_TYPE_HPP__
