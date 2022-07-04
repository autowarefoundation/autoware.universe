#pragma once
#include <eigen3/Eigen/Geometry>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

namespace common
{
struct StaticTfSubscriber
{
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  StaticTfSubscriber(rclcpp::Clock::SharedPtr clock)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  std::optional<Eigen::Affine3f> operator()(
    const std::string & frame_id, const std::string & parent_frame_id = "base_link")
  {
    std::optional<Eigen::Affine3f> extrinsic_{std::nullopt};
    try {
      geometry_msgs::msg::TransformStamped ts =
        tf_buffer_->lookupTransform(parent_frame_id, frame_id, tf2::TimePointZero);
      Eigen::Vector3f p;
      p.x() = ts.transform.translation.x;
      p.y() = ts.transform.translation.y;
      p.z() = ts.transform.translation.z;

      Eigen::Quaternionf q;
      q.w() = ts.transform.rotation.w;
      q.x() = ts.transform.rotation.x;
      q.y() = ts.transform.rotation.y;
      q.z() = ts.transform.rotation.z;
      extrinsic_ = Eigen::Affine3f::Identity();
      extrinsic_->translation() = p;
      extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
    } catch (tf2::TransformException & ex) {
    }
    return extrinsic_;
  }
};

}  // namespace common