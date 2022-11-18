#pragma once
#include <Eigen/Geometry>
#include <sophus/geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

namespace pcdless::common
{
class StaticTfSubscriber
{
public:
  explicit StaticTfSubscriber(rclcpp::Clock::SharedPtr clock);

  std::optional<Sophus::SE3f> se3f(
    const std::string & frame_id, const std::string & parent_frame_id = "base_link");

  std::optional<Eigen::Affine3f> operator()(
    const std::string & frame_id, const std::string & parent_frame_id = "base_link");

private:
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace pcdless::common