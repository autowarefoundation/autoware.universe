#include "sign_detector/optflow.hpp"
#include "sign_detector/util.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/calib3d.hpp>

void OptFlowNode ::imageCallback(const sensor_msgs::msg::CompressedImage& msg)
{
  sensor_msgs::msg::Image::ConstSharedPtr image_ptr = decompressImage(msg);
  cv::Mat image = cv_bridge::toCvCopy(*image_ptr, "rgb8")->image;
  if (!info_.has_value()) return;
  cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void*)(info_->k.data()));
  cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void*)(info_->d.data()));
  cv::Mat undistorted;
  cv::undistort(image, undistorted, K, D, K);
  image = undistorted;

  cv::Size size = image.size();
  const int WIDTH = 800;
  const float SCALE = 1.0f * WIDTH / size.width;
  int HEIGHT = SCALE * size.height;
  cv::resize(image, image, cv::Size(WIDTH, HEIGHT));

  trackOptFlow(image);
  publishImage(image, this->get_clock()->now());
}

void OptFlowNode::publishImage(const cv::Mat& image, const rclcpp::Time& stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  pub_image_->publish(*raw_image.toImageMsg());
}
