#include "pcdless_common/cv_decompress.hpp"

#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <iostream>

namespace pcdless::common
{
cv::Mat decompress_image(const sensor_msgs::msg::CompressedImage & compressed_img)
{
  cv::Mat raw_image;

  const std::string & format = compressed_img.format;
  const std::string encoding = format.substr(0, format.find(";"));

  constexpr int DECODE_GRAY = 0;
  constexpr int DECODE_RGB = 1;

  bool encoding_is_bayer = encoding.find("bayer") != std::string::npos;
  if (!encoding_is_bayer) {
    return cv::imdecode(cv::Mat(compressed_img.data), DECODE_RGB);
  }

  raw_image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_GRAY);
  if (encoding == "bayer_rggb8")
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BayerBG2BGR);
  else if (encoding == "bayer_bggr8")
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BayerRG2BGR);
  else if (encoding == "bayer_grbg8")
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BayerGB2BGR);
  else if (encoding == "bayer_gbrg8")
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BayerGR2BGR);
  else {
    std::cerr << encoding << " is not supported encoding" << std::endl;
    std::cerr << "Please implement additional decoding in " << __FUNCTION__ << std::endl;
    exit(EXIT_FAILURE);
  }
  return raw_image;
}

cv::Mat decompress_to_cv_mat(const sensor_msgs::msg::CompressedImage & compressed_img)
{
  return decompress_image(compressed_img);
}

cv::Mat decompress_to_cv_mat(const sensor_msgs::msg::Image & img)
{
  return cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(img), img.encoding)->image;
}

sensor_msgs::msg::Image::ConstSharedPtr decompress_to_ros_msg(
  const sensor_msgs::msg::CompressedImage & compressed_img, const std::string & encoding)
{
  cv_bridge::CvImage cv_image;
  cv_image.image = decompress_image(compressed_img);
  cv_image.encoding = encoding;
  return cv_image.toImageMsg();
}
}  // namespace pcdless::common