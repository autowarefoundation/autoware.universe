// Copyright 2023 TIER IV, Inc.
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

#include "refine_optimizer/optimizer.hpp"

#include "refine_optimizer/cost.hpp"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include <iomanip>

namespace refine_optimizer
{

Optimizer::Optimizer(const RefineConfig & config) : config_(config)
{
  param_t_ = Eigen::Vector3d::Zero();
  param_euler_ = Eigen::Vector3d::Zero();
}

void Optimizer::setStaticParams(const Eigen::Matrix3f & intrinsic, const Sophus::SE3f & extrinsic)
{
  extrinsic_ = extrinsic.cast<double>();
  intrinsic_ = intrinsic.cast<double>();
}

Sophus::SE3f Optimizer::execute(
  const cv::Mat & cost_image, const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointXYZ> & samples,
  std::string * summary_text)
{
  // Convert types from something float to double*
  const Sophus::SE3d pose_d = pose.cast<double>();

  // Declare optimization problem and variables
  ceres::Problem problem;

  const Grid grid(cost_image.data, 0, cost_image.rows, 0, cost_image.cols);
  const Interpolator interpolator(grid);

  // Add parameter blocks
  problem.AddParameterBlock(param_t_.data(), 3);
  problem.AddParameterBlock(param_euler_.data(), 3);

  // Add boundary conditions
  {
    double lon = config_.long_bound_;
    double lat = config_.late_bound_;
    double hei = config_.height_bound_;
    problem.SetParameterLowerBound(param_t_.data(), 0, -lon);  // longitudinal
    problem.SetParameterUpperBound(param_t_.data(), 0, lon);   // longitudinal
    problem.SetParameterLowerBound(param_t_.data(), 1, -lat);  // lateral
    problem.SetParameterUpperBound(param_t_.data(), 1, lat);   // lateral
    problem.SetParameterLowerBound(param_t_.data(), 2, -hei);  // height
    problem.SetParameterUpperBound(param_t_.data(), 2, hei);   // height
    if (config_.euler_bound_ > 0) {
      for (int axis = 0; axis < 3; ++axis) {
        problem.SetParameterLowerBound(param_euler_.data(), axis, -config_.euler_bound_);
        problem.SetParameterUpperBound(param_euler_.data(), axis, config_.euler_bound_);
      }
    } else {
      problem.SetParameterBlockConstant(param_euler_.data());
    }
  }

  // Add residual blocks
  for (const pcl::PointXYZ & p : samples) {
    Eigen::Vector3d p_d = p.getVector3fMap().cast<double>();
    problem.AddResidualBlock(
      ProjectionCost::Create(interpolator, p_d, intrinsic_.value(), extrinsic_.value(), pose_d),
      nullptr, param_t_.data(), param_euler_.data());
  }

  // Solve the optimization problem
  ceres::Solver::Options options;
  options.max_num_iterations = config_.max_iteration_;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (config_.verbose_) std::cout << summary.BriefReport() << std::endl;

  // Assemble status string
  *summary_text = makeSummaryText(param_t_, param_euler_, summary);

  // Assemble optimized parameters
  {
    Eigen::Vector3f t = param_t_.cast<float>();
    double R_data[9];
    ceres::EulerAnglesToRotationMatrix(param_euler_.data(), 3, R_data);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(R_data);
    return pose * Sophus::SE3f{R.cast<float>(), t};
  }
}

std::string Optimizer::makeSummaryText(
  const Eigen::Vector3d & param_t, const Eigen::Vector3d & param_euler,
  const ceres::Solver::Summary & summary)
{
  std::stringstream ss;
  ss << std::showpos << std::fixed << std::setprecision(2);
  ss << "x: " << param_t(0) << std::endl;
  ss << "y: " << param_t(1) << std::endl;
  ss << "z: " << param_t(2) << std::endl;
  ss << "p: " << param_euler(0) << std::endl;
  ss << "r: " << param_euler(1) << std::endl;
  ss << "y: " << param_euler(2) << std::endl;
  ss << "time: " << (summary.total_time_in_seconds * 1000) << std::endl;
  ss << ceres::TerminationTypeToString(summary.termination_type) << std::endl;
  return ss.str();
}

}  // namespace refine_optimizer