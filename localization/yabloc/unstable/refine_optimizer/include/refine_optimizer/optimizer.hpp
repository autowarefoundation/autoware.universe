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

#pragma once
#include "refine_optimizer/config.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sophus/geometry.hpp>

#include <ceres/solver.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace refine_optimizer
{
class Optimizer
{
public:
  Optimizer(const RefineConfig & config);

  Sophus::SE3f execute(
    const cv::Mat & cost_image, const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointXYZ> & samples,
    std::string * summary_text);

  void setStaticParams(const Eigen::Matrix3f & intrinsic, const Sophus::SE3f & extrinsic);

private:
  const RefineConfig config_;

  std::string makeSummaryText(
    const Eigen::Vector3d & param_t, const Eigen::Vector3d & param_euler,
    const ceres::Solver::Summary & summary);

  std::optional<Sophus::SE3d> extrinsic_{std::nullopt};
  std::optional<Eigen::Matrix3d> intrinsic_{std::nullopt};

  Eigen::Vector3d param_t_;
  Eigen::Vector3d param_euler_;
};

}  // namespace refine_optimizer