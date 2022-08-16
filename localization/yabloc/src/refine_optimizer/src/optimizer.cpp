#include "optimizer.hpp"

#include "cost.hpp"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include <iomanip>

namespace validation
{
std::string makeSummaryText(
  const Eigen::Vector3d & param_t, const Eigen::Vector3d & param_euler,
  const ceres::Solver::Summary & summary);

Sophus::SE3f refinePose(
  const Sophus::SE3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointXYZ> & samples, const RefineConfig & config,
  std::string * summary_text)
{
  // Convert types from something float to double*
  const Sophus::SE3d extrinsic_d = extrinsic.cast<double>();
  const Sophus::SE3d pose_d = pose.cast<double>();
  const Eigen::Matrix3d intrinsic_d = intrinsic.cast<double>();

  // Declare optimization problem and variables
  ceres::Problem problem;
  Eigen::Vector3d param_t = Eigen::Vector3d::Zero();
  Eigen::Vector3d param_euler(0, 0, 0);
  const Grid grid(cost_image.data, 0, cost_image.rows, 0, cost_image.cols);
  const Interpolator interpolator(grid);

  // Add parameter blocks
  problem.AddParameterBlock(param_t.data(), 3);
  problem.AddParameterBlock(param_euler.data(), 3);

  // Add boundary conditions
  {
    double lon = config.long_bound_;
    double lat = config.late_bound_;
    double hei = config.height_bound_;
    problem.SetParameterLowerBound(param_t.data(), 0, param_t.x() - lon);  // longitudinal
    problem.SetParameterUpperBound(param_t.data(), 0, param_t.x() + lon);  // longitudinal
    problem.SetParameterLowerBound(param_t.data(), 1, param_t.y() - lat);  // lateral
    problem.SetParameterUpperBound(param_t.data(), 1, param_t.y() + lat);  // lateral
    problem.SetParameterLowerBound(param_t.data(), 2, param_t.z() - hei);  // height
    problem.SetParameterUpperBound(param_t.data(), 2, param_t.z() + hei);  // height
    if (config.euler_bound_ > 0) {
      for (int axis = 0; axis < 3; ++axis) {
        problem.SetParameterLowerBound(param_euler.data(), axis, -config.euler_bound_);
        problem.SetParameterUpperBound(param_euler.data(), axis, config.euler_bound_);
      }
    } else {
      problem.SetParameterBlockConstant(param_euler.data());
    }
  }

  // Add residual blocks
  for (const pcl::PointXYZ & p : samples) {
    Eigen::Vector3d p_d = p.getVector3fMap().cast<double>();
    problem.AddResidualBlock(
      ProjectionCost::Create(interpolator, p_d, intrinsic_d, extrinsic_d, pose_d), nullptr,
      param_t.data(), param_euler.data());
  }

  // Solve the optimization problem
  ceres::Solver::Options options;
  options.max_num_iterations = config.max_iteration_;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (config.verbose_) std::cout << summary.BriefReport() << std::endl;

  // Assemble status string
  *summary_text = makeSummaryText(param_t, param_euler, summary);

  // Assemble optimized parameters
  {
    Eigen::Vector3f t(param_t.x(), param_t.y(), param_t.z());
    double R_data[9];
    ceres::EulerAnglesToRotationMatrix(param_euler.data(), 3, R_data);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(R_data);
    return pose * Sophus::SE3f{R.cast<float>(), t};
  }
}

std::string makeSummaryText(
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

}  // namespace validation