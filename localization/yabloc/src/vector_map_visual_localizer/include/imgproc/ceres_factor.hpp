#pragma once
#include <eigen3/Eigen/Geometry>

#include <ceres/ceres.h>

namespace imgproc::opt
{
class VanishPointFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VanishPointFactor(const Eigen::Vector3d & vp) : vp_(vp) {}

  template <typename T>
  bool operator()(const T * const q_ptr, T * residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);
    Vector3T normal = q * Vector3T::UnitZ();
    T nx = normal.x();
    T ny = normal.y();
    T nz = normal.z();
    T n2 = nx * nx + ny * ny;

    Vector3T s;
    s << -nz / nx, T(0), T(1);
    T tmp = normal.dot(s - vp_);
    residual[0] = tmp * tmp / n2;
    return true;
  }

  double operator()(const double * const q_ptr) const
  {
    double residual[1];
    (*this)(q_ptr, residual);
    return residual[0];
  }

  static ceres::CostFunction * create(const Eigen::Vector3f & vp)
  {
    // residual(1), q(4)
    return new ceres::AutoDiffCostFunction<VanishPointFactor, 1, 4>(
      new VanishPointFactor(vp.cast<double>()));
  }

private:
  Eigen::Vector3d vp_;
};

class HorizonFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HorizonFactor(const Eigen::Vector2d & vertical) : vertical_(vertical) {}

  template <typename T>
  bool operator()(const T * const q_ptr, T * residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);
    Vector3T normal = q * Vector3T::UnitZ();
    T nx = normal.x();
    T ny = normal.y();
    T n2 = nx * nx + ny * ny;
    T tmp = -ny * vertical_.x() + nx * vertical_.y();

    residual[0] = tmp * tmp / n2;
    return true;
  }

  double operator()(const double * const q_ptr) const
  {
    double residual[1];
    (*this)(q_ptr, residual);
    return residual[0];
  }

  static ceres::CostFunction * create(const Eigen::Vector2f & vp)
  {
    // residual(1), q(4)
    return new ceres::AutoDiffCostFunction<HorizonFactor, 1, 4>(
      new HorizonFactor(vp.cast<double>()));
  }

private:
  Eigen::Vector2d vertical_;
};

class ImuFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuFactor(const Eigen::Quaterniond & dq) : dq_(dq) {}
  template <typename T>
  bool operator()(const T * const q1_ptr, const T * const q2_ptr, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q2(q2_ptr);
    Eigen::Quaternion<T> dqT = dq_.template cast<T>();

    Eigen::Quaternion<T> delta_q = q1.conjugate() * q2 * dqT.conjugate();

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residual);
    residuals = delta_q.vec();
    return true;
  }

  static ceres::CostFunction * create(const Sophus::SO3f & dR)
  {
    // residual(1), q1(4) ,q2(4)
    return new ceres::AutoDiffCostFunction<ImuFactor, 1, 4, 4>(
      new ImuFactor(dR.unit_quaternion().cast<double>()));
  }

private:
  Eigen::Quaterniond dq_;
};
}  // namespace imgproc::opt