#pragma once
#include <Eigen/Core>
#include <pcdless_common/ground_plane.hpp>
#include <rclcpp/time.hpp>

#include <optional>

namespace pcdless::ground_server
{
class LowPassFilter
{
public:
  using Ground = common::GroundPlane;

  LowPassFilter() { initialize(0); }

  void initialize(const float & height)
  {
    height_ = height;
    cov_height_ = 25.f;

    initialized = true;
  }

  void update(const float height)
  {
    cov_height_ += 0.01;

    constexpr float noise_height = 0.2f;
    float error = height - height_;
    float gain = cov_height_ / (cov_height_ + noise_height);

    height_ += gain * error;
    cov_height_ = cov_height_ - gain * cov_height_;
  }

  float get_estimate() const { return height_; }

private:
  bool initialized = false;

  float height_;
  float cov_height_;
};
}  // namespace pcdless::ground_server