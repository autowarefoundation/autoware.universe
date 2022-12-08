#pragma once
#include <opencv4/opencv2/imgproc.hpp>

namespace pcdless::common
{
cv::Mat direct_cost_map(const cv::Mat & cost_map, const cv::Mat & intensity);

cv::Mat visualize_direction_map(const cv::Mat & cost_map);

}  // namespace pcdless::common