#pragma once
#include <eigen3/Eigen/Geometry>

#include <ceres/ceres.h>

namespace imgproc::opt
{
class VanishPointFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VanishPointFactor(const Eigen::Vector3d & vp, double gain) : vp_(vp), gain_(gain) {}

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
    residual[0] = gain_ * tmp * tmp / n2;
    return true;
  }

  static double eval(const Eigen::Vector3f & vp, const double * const q_ptr)
  {
    VanishPointFactor obj(vp.cast<double>(), 1.0);
    double residual[1];
    obj(q_ptr, residual);
    return residual[0];
  }

  static ceres::CostFunction * create(const Eigen::Vector3f & vp, double gain)
  {
    // residual(1), q(4)
    return new ceres::AutoDiffCostFunction<VanishPointFactor, 1, 4>(
      new VanishPointFactor(vp.cast<double>(), gain));
  }

private:
  Eigen::Vector3d vp_;
  const double gain_;
};

class HorizonFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  HorizonFactor(const Eigen::Vector2d & vertical, double gain) : vertical_(vertical), gain_(gain) {}

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

    residual[0] = gain_ * tmp * tmp / n2;
    return true;
  }

  double operator()(const double * const q_ptr) const
  {
    double residual[1];
    (*this)(q_ptr, residual);
    return residual[0];
  }

  static double eval(const Eigen::Vector2f & vertical, const double * const q_ptr)
  {
    HorizonFactor obj(vertical.cast<double>(), 1.0);
    double residual[1];
    obj(q_ptr, residual);
    return residual[0];
  }

  static ceres::CostFunction * create(const Eigen::Vector2f & vertical, double gain)
  {
    // residual(1), q(4)
    return new ceres::AutoDiffCostFunction<HorizonFactor, 1, 4>(
      new HorizonFactor(vertical.cast<double>(), gain));
  }

private:
  Eigen::Vector2d vertical_;
  const double gain_;
};

class ImuFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuFactor(const Eigen::Quaterniond & dq, double gain) : dq_(dq), gain_(gain) {}

  template <typename T>
  bool operator()(const T * const q1_ptr, const T * const q2_ptr, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q1(q1_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q2(q2_ptr);
    Eigen::Quaternion<T> dqT = dq_.template cast<T>();

    Eigen::Quaternion<T> delta_q = q1.conjugate() * q2 * dqT.conjugate();

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residual);
    residuals = gain_ * delta_q.vec();
    return true;
  }

  static double eval(
    const Sophus::SO3f & dR, const double * const q1_ptr, const double * const q2_ptr)
  {
    ImuFactor obj(dR.unit_quaternion().cast<double>(), 1.0);
    double residual[3];
    obj(q1_ptr, q2_ptr, residual);
    Eigen::Map<Eigen::Vector3d> residuals(residual);
    return residuals.norm();
  }

  static ceres::CostFunction * create(const Sophus::SO3f & dR, double gain)
  {
    // residual(3), q1(4) ,q2(4)
    return new ceres::AutoDiffCostFunction<ImuFactor, 3, 4, 4>(
      new ImuFactor(dR.unit_quaternion().cast<double>(), gain));
  }

private:
  Eigen::Quaterniond dq_;
  const double gain_;
};
}  // namespace imgproc::opt