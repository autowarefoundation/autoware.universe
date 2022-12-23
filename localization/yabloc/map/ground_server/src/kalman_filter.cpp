#include "ground_server/kalman_filter.hpp"

namespace pcdless::ground_server
{
KalmanFilter::KalmanFilter() {}

std::pair<float, Eigen::Matrix3f> KalmanFilter::get_estimate() const
{
  return {height_, rotation_.matrix()};
}

void KalmanFilter::initialize(const float & height, const Eigen::Vector3f &)
{
  height_ = height;
  rotation_ = Sophus::SO3f();
  cov_rotation_ = 0.01f * Eigen::Matrix3f::Identity();
  cov_height_ = 0.5f * Eigen::Matrix<float, 1, 1>::Identity();

  initialized = true;
}

void KalmanFilter::predict(const rclcpp::Time & stamp)
{
  if (last_stamp_) {
    const float dt = (stamp - *last_stamp_).seconds();
    cov_height_ += 0.1 * Eigen::Matrix<float, 1, 1>::Identity() * dt;
    cov_rotation_ += Eigen::Vector3f(0.01, 0.01, 0.01).asDiagonal() * dt;
  }

  last_stamp_ = stamp;
}

void KalmanFilter::measure(const float height, const Eigen::Vector3f & normal)
{
  if (!initialized) {
    throw std::runtime_error("kalman filte is not initialized yet");
  }

  // height correction
  {
    constexpr float noise_height = 0.2f;
    float error = height_ - height;
    float gain = cov_height_(0) / (cov_height_(0) + noise_height);

    height_ += gain * error;
    cov_height_ = cov_height_ - gain * cov_height_;
  }

  // rotation correction
  {
    const Eigen::Matrix3f noise_rotation_outer_product = 0.1f * Eigen::Matrix3f::Identity();
    const Eigen::Vector3f ez = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f measured = (normal.z() > 0) ? normal : (-normal);

    using Sophus::SO3f;
    Eigen::Vector3f error = SO3f::hat(rotation_ * ez) * measured;
    Eigen::Matrix3f H = -SO3f::hat(measured) * rotation_.matrix() * SO3f::hat(ez);
    Eigen::Matrix3f S = H * cov_rotation_ * H.transpose() + noise_rotation_outer_product;
    Eigen::Matrix3f K = cov_rotation_ * H.transpose() * S.inverse();

    rotation_ = rotation_ * SO3f::exp(K * error);
    cov_rotation_ = (Eigen::Matrix3f::Identity() - K * H) * cov_rotation_;
  }
}

}  // namespace pcdless::ground_server