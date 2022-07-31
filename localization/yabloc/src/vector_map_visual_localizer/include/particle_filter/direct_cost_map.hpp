#pragma once
#include <opencv4/opencv2/imgproc.hpp>

namespace particle_filter
{
cv::Mat directCostMap(const cv::Mat & cost_map, const cv::Mat & intensity);

cv::Mat visualizeDirectionMap(const cv::Mat & cost_map);

}  // namespace particle_filter