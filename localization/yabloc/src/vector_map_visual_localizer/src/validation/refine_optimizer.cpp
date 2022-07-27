#include "validation/refine_optimizer.hpp"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <ceres/rotation.h>

#include <iomanip>

namespace validation
{
using Grid = ceres::Grid2D<uchar>;
using Interpolator = ceres::BiCubicInterpolator<Grid>;

struct ProjectionCost
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template <typename T>
  bool operator()(const T * p, const T * q, T * residual) const
  {
    using Vector3d = Eigen::Vector3d;
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using QuatT = Eigen::Quaternion<T>;

    T R_data[9];
    ceres::EulerAnglesToRotationMatrix(q, 3, R_data);
    Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor> > R(R_data);

    Eigen::Map<const Vector3T> position(p);
    // Eigen::Map<const QuatT> orientation(q);

    // from_camera = (T^{-1} p)
    // where, T=pose * variable * extrinsic
    Vector3d from_biased_base_link = pose_.inverse() * point_;
    Vector3T from_base_link = R.transpose() * (from_biased_base_link - position);
    Vector3T from_camera = extrinsic_.inverse() * from_base_link;

    if (from_camera.z() < 1e-3f) {
      *residual = T(255.0);
      return true;
    }

    // NOTE: Interpolator is ROW-major
    T v;
    Vector3T pixel = intrinsic_ * from_camera / from_camera.z();
    interpolator_.Evaluate(pixel.y(), pixel.x(), &v);

    *residual = 255.0 - v;
    return true;
  }

  ProjectionCost(
    const Interpolator & interpolator, const Eigen::Vector3d & point,
    const Eigen::Matrix3d & intrinsic, const Sophus::SE3d & extrinsic, const Sophus::SE3d & pose)
  : point_(point),
    interpolator_(interpolator),
    intrinsic_(intrinsic),
    extrinsic_(extrinsic),
    pose_(pose)
  {
  }

  static ceres::CostFunction * Create(
    const Interpolator & interpolator, const Eigen::Vector3d & point,
    const Eigen::Matrix3d & intrinsic, const Sophus::SE3d & extrinsic, const Sophus::SE3d & pose)
  {
    return new ceres::AutoDiffCostFunction<ProjectionCost, 1, 3, 3>(
      new ProjectionCost(interpolator, point, intrinsic, extrinsic, pose));
  }

  const Eigen::Vector3d point_;
  const Interpolator & interpolator_;
  const Eigen::Matrix3d & intrinsic_;
  const Sophus::SE3d & extrinsic_;
  const Sophus::SE3d & pose_;
};

Sophus::SE3f refinePose(
  const Sophus::SE3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Sophus::SE3f & pose, pcl::PointCloud<pcl::PointNormal> & linesegments,
  std::string * summary_text)
{
  // Convert types from something float to double*
  Eigen::Vector3d param_t = Eigen::Vector3d::Zero();
  // Eigen::Vector4d param_q(0, 0, 0, 1);
  Eigen::Vector3d param_euler(0, 0, 0);
  const Sophus::SE3d extrinsic_d = extrinsic.cast<double>();
  const Sophus::SE3d pose_d = pose.cast<double>();
  const Eigen::Matrix3d intrinsic_d = intrinsic.cast<double>();

  // Declare optimization problem
  ceres::Problem problem;
  const Grid grid(cost_image.data, 0, cost_image.rows, 0, cost_image.cols);
  const Interpolator interpolator(grid);

  // Add parameter blocks
  problem.AddParameterBlock(param_t.data(), 3);
  // NOTE: Orientation is too sensitive to cost and it sometime might change dramatically.
  // NOTE: So I fix it temporallya and I will unfix it someday.
  // ceres::Manifold * quaternion_manifold = new ceres::EigenQuaternionManifold;
  // problem.AddParameterBlock(param_q.data(), 4);
  // problem.SetManifold(param_q.data(), quaternion_manifold);
  // problem.SetParameterBlockConstant(param_q.data());
  problem.AddParameterBlock(param_euler.data(), 3);

  // Add boundary conditions
  problem.SetParameterLowerBound(param_t.data(), 0, param_t.x() - 0.1);  // longitudinal
  problem.SetParameterUpperBound(param_t.data(), 0, param_t.x() + 0.1);  // longitudinal
  problem.SetParameterLowerBound(param_t.data(), 1, param_t.y() - 1.0);  // lateral
  problem.SetParameterUpperBound(param_t.data(), 1, param_t.y() + 1.0);  // lateral
  problem.SetParameterLowerBound(param_t.data(), 2, param_t.z() - 0.1);  // height
  problem.SetParameterUpperBound(param_t.data(), 2, param_t.z() + 0.1);  // height

  for (int axis = 0; axis < 3; ++axis) {
    problem.SetParameterLowerBound(param_euler.data(), axis, -0.2);
    problem.SetParameterUpperBound(param_euler.data(), axis, 0.2);
  }

  // Add residual blocks
  for (const pcl::PointNormal & pn : linesegments) {
    Eigen::Vector3f t = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    float l = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l; distance += 5.f) {
      Eigen::Vector3f p = pn.getVector3fMap() + t * distance;
      problem.AddResidualBlock(
        ProjectionCost::Create(interpolator, p.cast<double>(), intrinsic_d, extrinsic_d, pose_d),
        nullptr, param_t.data(), param_euler.data());
    }
  }

  // Solve the optimization problem
  ceres::Solver::Options options;
  options.max_num_iterations = 30;
  // options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  {
    std::stringstream ss;
    ss << std::showpos << std::fixed << std::setprecision(2);
    ss << "x: " << param_t(0) << std::endl;
    ss << "y: " << param_t(1) << std::endl;
    ss << "z: " << param_t(2) << std::endl;
    ss << "r: " << param_euler(0) << std::endl;
    ss << "p: " << param_euler(1) << std::endl;
    ss << "y: " << param_euler(2) << std::endl;
    *summary_text = ss.str();
  }

  // Assemble optimized parameters
  {
    Eigen::Vector3f t(param_t.x(), param_t.y(), param_t.z());
    // Eigen::Quaternionf q;
    // q.coeffs() = param_q.cast<float>();

    double R_data[9];
    ceres::EulerAnglesToRotationMatrix(param_euler.data(), 3, R_data);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(R_data);
    return pose * Sophus::SE3f{R.cast<float>(), t};
  }
}

}  // namespace validation