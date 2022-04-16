#pragma once

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <lsd/lsd.hpp>
#include <opencv4/opencv2/opencv.hpp>

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(const std::string& topic) : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(topic, 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::CompressedImage& msg) const
  {
    sensor_msgs::msg::Image::ConstSharedPtr image_ptr = decompressImage(msg);


    cv::Mat image = cv_bridge::toCvCopy(*image_ptr, "rgb8")->image;
    cv::Size size = image.size();

    RCLCPP_INFO_STREAM(this->get_logger(), "image size: " << size);

    const int WIDTH = 800;
    int HEIGHT = 1.0f * WIDTH / size.width * size.height;
    cv::resize(image, image, cv::Size(WIDTH, HEIGHT));
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::lsd::LineSegmentDetector> lsd = cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_ADV);
    cv::Mat lines;
    lsd->detect(image, lines);
    lsd->drawSegments(image, lines);
    cv::imshow("show", image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;

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