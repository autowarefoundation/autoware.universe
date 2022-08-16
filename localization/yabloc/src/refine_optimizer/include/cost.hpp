#pragma once
#include <sophus/geometry.hpp>

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <ceres/rotation.h>

namespace validation
{
using Grid = ceres::Grid2D<uint8_t>;
using Interpolator = ceres::BiCubicInterpolator<Grid>;

struct ProjectionCost
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  template <typename T>
  bool operator()(const T * p, const T * q, T * residual) const
  {
    using Vector3d = Eigen::Vector3d;
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    T R_data[9];
    ceres::EulerAnglesToRotationMatrix(q, 3, R_data);
    Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor> > R(R_data);
    Eigen::Map<const Vector3T> position(p);

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
}  // namespace validation