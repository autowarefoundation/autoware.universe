#pragma once
#include <Eigen/Core>
#include <pcdless_common/ground_plane.hpp>
#include <rclcpp/time.hpp>
#include <sophus/so3.hpp>

#include <optional>

namespace pcdless::ground_server
{
class KalmanFilter
{
public:
  using Ground = common::GroundPlane;

  KalmanFilter();

  void predict(const rclcpp::Time & stamp);

  void measure(const float hegith, const Eigen::Vector3f & normal);

  std::pair<float, Eigen::Vector3f> get_estimate() const;

private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool initialized = false;

  float height_;
  Sophus::SO3f rotation_;

  Eigen::Matrix3f cov_rotation_;
  Eigen::Matrix<float, 1, 1> cov_height_;

  std::optional<rclcpp::Time> last_stamp_{std::nullopt};
};
}  // namespace pcdless::ground_server