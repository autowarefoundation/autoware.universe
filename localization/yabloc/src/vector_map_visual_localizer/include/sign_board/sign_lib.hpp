#pragma once
#include <opencv2/core.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace sign_board
{
cv::Scalar randomColor(signed long index);

std::vector<cv::Point2i> computeConvexHull(const std::vector<cv::Point2f> & src);

float distanceToSignBoard(
  const lanelet::ConstLineString3d & board, const Eigen::Vector3f & position);

}  // namespace sign_board