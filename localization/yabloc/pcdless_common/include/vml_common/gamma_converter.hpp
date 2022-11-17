#pragma once
#include <opencv4/opencv2/core.hpp>

namespace vml_common
{
class GammaConverter
{
public:
  GammaConverter(float gamma = 1.0f) { reset(gamma); }

  void reset(float gamma)
  {
    lut_ = cv::Mat(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++) {
      lut_.at<uchar>(0, i) = 256 * std::pow(i / 256.f, gamma);
    }
  }

  cv::Mat operator()(const cv::Mat & src_image) const
  {
    cv::Mat dst_image;
    cv::LUT(src_image, lut_, dst_image);
    return dst_image;
  }

private:
  cv::Mat lut_;
};
}  // namespace vml_common