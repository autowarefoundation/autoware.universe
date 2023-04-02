#pragma once
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace pcdless::initializer
{
class ProjectorModule
{
public:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const cv::Point2i &)>;
  ProjectorModule(rclcpp::Node * node);

  bool define_project_func();

  cv::Mat project_image(const sensor_msgs::msg::Image & image_msg);

private:
  ProjectFunc project_func_ = nullptr;
  rclcpp::Logger logger_;
  common::CameraInfoSubscriber info_;
  common::StaticTfSubscriber tf_subscriber_;
};
}  // namespace pcdless::initializer