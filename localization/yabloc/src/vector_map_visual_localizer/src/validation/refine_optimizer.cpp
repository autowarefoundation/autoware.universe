#include "validation/refine_optimizer.hpp"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

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
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using QuatT = Eigen::Quaternion<T>;

    Eigen::Map<const Vector3T> position(p);
    Eigen::Map<const QuatT> orientation(q);

    Vector3T from_camera =
      intrinsic_ * extrinsic_.inverse() * (orientation.conjugate() * (point_ - position));

    if (from_camera.z() < 1e-3f) {
      *residual = T(255.0);
      return true;
    }

    Vector3T u = from_camera /= from_camera.z();
    T v;
    interpolator_.Evaluate(u.y(), u.x(), &v);

    *residual = 255.0 - v;
    return true;
  }

  ProjectionCost(
    const Interpolator & interpolator, const Eigen::Vector3d & point,
    const Eigen::Matrix3d & intrinsic, const Eigen::Affine3d & extrinsic)
  : point_(point), interpolator_(interpolator), intrinsic_(intrinsic), extrinsic_(extrinsic)
  {
  }

  static ceres::CostFunction * Create(
    const Interpolator & interpolator, const Eigen::Vector3d & point,
    const Eigen::Matrix3d & intrinsic, const Eigen::Affine3d & extrinsic)
  {
    return new ceres::AutoDiffCostFunction<ProjectionCost, 1, 3, 4>(
      new ProjectionCost(interpolator, point, intrinsic, extrinsic));
  }

  const Eigen::Vector3d point_;
  const Interpolator & interpolator_;
  const Eigen::Matrix3d & intrinsic_;
  const Eigen::Affine3d & extrinsic_;
};

Eigen::Affine3f refinePose(
  const Eigen::Affine3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Eigen::Affine3f & pose, pcl::PointCloud<pcl::PointNormal> & linesegments)
{
  const Grid grid(cost_image.data, 0, cost_image.rows, 0, cost_image.cols);
  const Interpolator interpolator(grid);

  Eigen::Vector3d param_t = pose.translation().cast<double>();
  Eigen::Quaterniond param_quat = Eigen::Quaternionf(pose.rotation()).cast<double>();
  Eigen::Vector4d param_q = param_quat.coeffs();

  const Eigen::Affine3d extrinsic_d = extrinsic.cast<double>();
  const Eigen::Matrix3d intrinsic_d = intrinsic.cast<double>();

  ceres::Problem problem;
  ceres::Manifold * quaternion_manifold = new ceres::EigenQuaternionManifold;

  problem.AddParameterBlock(param_t.data(), 3);
  problem.AddParameterBlock(param_q.data(), 4);
  problem.SetManifold(param_q.data(), quaternion_manifold);

  {
    Eigen::Vector3d point = linesegments.front().getVector3fMap().cast<double>();
    Eigen::Vector3d from_camera =
      intrinsic_d * extrinsic_d.inverse() * (param_quat.conjugate() * (point - param_t));

    cv::Rect2i rect(0, 0, cost_image.cols, cost_image.rows);
    if (from_camera.z() < 1e-3f) {
      std::cout << "out of image  " << from_camera.transpose() << std::endl;
    } else {
      Eigen::Vector3d u = from_camera /= from_camera.z();
      if (rect.contains(cv::Point2i(u.x(), u.y()))) {
        double score;
        interpolator.Evaluate(u.y(), u.x(), &score);

        double expected = cost_image.at<uchar>(u.y(), u.x());
        double grid_score;
        grid.GetValue(u.y(), u.x(), &grid_score);

        std::cout << "score " << score << " expected: " << expected << " grid: " << grid_score
                  << std::endl;
      } else {
        std::cout << "invalid range: " << u.transpose() << std::endl;
      }
    }
  }

  for (const pcl::PointNormal & pn : linesegments) {
    Eigen::Vector3f t = (pn.getNormalVector3fMap() - pn.getVector3fMap()).normalized();
    float l = (pn.getVector3fMap() - pn.getNormalVector3fMap()).norm();

    for (float distance = 0; distance < l; distance += 5.f) {
      Eigen::Vector3f p = pn.getVector3fMap() + t * distance;

      problem.AddResidualBlock(
        ProjectionCost::Create(interpolator, p.cast<double>(), intrinsic_d, extrinsic_d), nullptr,
        param_t.data(), param_q.data());
      break;
    }
    break;
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 10;
  // options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << '\n';

  {
    Eigen::Translation3f t(param_t.x(), param_t.y(), param_t.z());
    Eigen::Quaternionf q(param_q.w(), param_q.x(), param_q.y(), param_q.z());
    return t * q;
  }
}

}  // namespace validation