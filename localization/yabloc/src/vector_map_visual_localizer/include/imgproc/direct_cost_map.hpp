#pragma once
#include <opencv4/opencv2/imgproc.hpp>

namespace imgproc
{
cv::Mat directCostMap(const cv::Mat & cost_map);

cv::Mat visualizeDirectionMap(const cv::Mat & cost_map);

}  // namespace imgproc