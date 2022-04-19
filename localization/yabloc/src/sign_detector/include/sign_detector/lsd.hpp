#pragma once

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <lsd/lsd.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <optional>

class LineDetector : public rclcpp::Node
{
public:
  LineDetector(const std::string& image_topic, const std::string& info_topic) : Node("line_detector"), info_(std::nullopt)
  {
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, 10, std::bind(&LineDetector::imageCallback, this, std::placeholders::_1));
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 10, std::bind(&LineDetector::infoCallback, this, std::placeholders::_1));
    lsd = cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_ADV);
  }

private:
  void infoCallback(const sensor_msgs::msg::CameraInfo& msg) { info_ = msg; }

  void imageCallback(const sensor_msgs::msg::CompressedImage& msg) const
  {
    sensor_msgs::msg::Image::ConstSharedPtr image_ptr = decompressImage(msg);
    cv::Mat image = cv_bridge::toCvCopy(*image_ptr, "rgb8")->image;
    cv::Size size = image.size();

    if (info_.has_value()) {
      cv::Mat K(cv::Size(3, 3), CV_64FC1, (void*)(info_->k.data()));
      cv::Mat D(cv::Size(5, 1), CV_64FC1, (void*)(info_->d.data()));
      cv::Mat undistorted;
      cv::undistort(image, undistorted, K, D, K);
      image = undistorted;
    }

    const int WIDTH = 800;
    int HEIGHT = 1.0f * WIDTH / size.width * size.height;
    cv::resize(image, image, cv::Size(WIDTH, HEIGHT));
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    cv::Mat lines;
    lsd->detect(image, lines);
    lsd->drawSegments(image, lines);
    cv::imshow("show", image);
    cv::waitKey(1);
  }

  cv::Ptr<cv::lsd::LineSegmentDetector> lsd;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;

  sensor_msgs::msg::Image::ConstSharedPtr decompressImage(const sensor_msgs::msg::CompressedImage& compressed_img) const
  {
    cv_bridge::CvImage raw_image;
    raw_image.header = compressed_img.header;

    const std::string& format = compressed_img.format;
    const std::string encoding = format.substr(0, format.find(";"));
    raw_image.encoding = encoding;

    constexpr int DECODE_GRAY = 0;
    constexpr int DECODE_RGB = 1;

    bool encoding_is_bayer = encoding.find("bayer") != std::string::npos;
    if (encoding_is_bayer) {
      raw_image.image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_GRAY);
      if (encoding == "bayer_rggb8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerBG2BGR);
      else if (encoding == "bayer_bggr8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerRG2BGR);
      else if (encoding == "bayer_grbg8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerGB2BGR);
      else if (encoding == "bayer_gbrg8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerGR2BGR);
      else {
        std::cerr << encoding << " is not supported encoding" << std::endl;
        std::cerr << "Please implement additional decoding in " << __FUNCTION__ << std::endl;
        exit(4);
      }
      raw_image.encoding = "bgr8";
      return raw_image.toImageMsg();
    }

    raw_image.image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_RGB);
    return raw_image.toImageMsg();
  }
};