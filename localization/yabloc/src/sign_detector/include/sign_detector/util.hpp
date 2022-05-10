#pragma once
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

sensor_msgs::msg::Image::ConstSharedPtr decompressImage(const sensor_msgs::msg::CompressedImage& compressed_img);