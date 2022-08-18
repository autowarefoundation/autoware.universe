#pragma once
#include <opencv4/opencv2/core.hpp>

#include <std_msgs/msg/color_rgba.hpp>

namespace util
{

struct Color
{
  Color(float r, float g, float b) : r(r), g(g), b(b) {}
  Color(const std_msgs::msg::ColorRGBA & rgba) : r(rgba.r), g(rgba.g), b(rgba.b) {}
  Color(const cv::Scalar & rgb) : r(rgb[2] / 255.f), g(rgb[1] / 255.f), b(rgb[0] / 255.f) {}

  operator const cv::Scalar() const { return cv::Scalar(255 * b, 255 * g, 255 * r); }
  operator const std_msgs::msg::ColorRGBA() const
  {
    std_msgs::msg::ColorRGBA rgba;
    rgba.a = 1.0f;
    rgba.r = r;
    rgba.g = g;
    rgba.b = b;
    return rgba;
  }

  float r, g, b;
};

Color toJet(float value);
std_msgs::msg::ColorRGBA color(float red, float green, float blue, float alpha = 1.0f);
}  // namespace util