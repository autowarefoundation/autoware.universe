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