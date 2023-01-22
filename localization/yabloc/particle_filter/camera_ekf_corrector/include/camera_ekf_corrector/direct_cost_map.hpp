#pragma once
#include <opencv4/opencv2/imgproc.hpp>

namespace pcdless::ekf_corrector
{
cv::Mat direct_cost_map(const cv::Mat & cost_map, const cv::Mat & intensity);

cv::Mat visualize_direction_map(const cv::Mat & cost_map);

}  // namespace pcdless::ekf_corrector