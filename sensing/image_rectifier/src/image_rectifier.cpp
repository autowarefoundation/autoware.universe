

#include "image_rectifier/image_rectifier.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace image_preprocessor
{
    ImageRectifier::ImageRectifier(const rclcpp::NodeOptions &node_options)
: rclcpp::Node("image_rectifier", node_options)
{
  queue_size_ = this->declare_parameter("queue_size", 5);
  interpolation = this->declare_parameter("interpolation", 1);
  pub_rect_ = image_transport::create_publisher(this, "image_rect",
                                                rclcpp::SensorDataQoS().get_rmw_qos_profile());

  sub_camera_ = image_transport::create_camera_subscription(
          this, "image", std::bind(
                  &ImageRectifier::imageCb,
                  this, std::placeholders::_1, std::placeholders::_2), "raw",
                  rclcpp::SensorDataQoS().get_rmw_qos_profile());

}

void ImageRectifier::imageCb(
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  model_.fromCameraInfo(info_msg);

  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat rect;

  // Rectify and publish
  model_.rectifyImage(image, rect, interpolation);

  // Allocate new rectified image message
  sensor_msgs::msg::Image::SharedPtr rect_msg =
          cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
  pub_rect_.publish(rect_msg);
}
}  // namespace image_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_preprocessor::ImageRectifier)
