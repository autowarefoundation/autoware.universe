#pragma once
#include <opencv4/opencv2/imgproc.hpp>

namespace modularized_particle_filter
{
cv::Mat directCostMap(const cv::Mat & cost_map, const cv::Mat & intensity);

cv::Mat visualizeDirectionMap(const cv::Mat & cost_map);

}  // namespace modularized_particle_filter