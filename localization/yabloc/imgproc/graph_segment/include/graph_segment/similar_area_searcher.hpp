// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <Eigen/Core>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/logger.hpp>

#include <set>

namespace pcdless::graph_segment
{
class SimilarAreaSearcher
{
public:
  SimilarAreaSearcher(float similarity_score_threshold)
  : similarity_score_threshold_(similarity_score_threshold),
    logger_(rclcpp::get_logger("similar_area_searcher"))
  {
  }

  std::set<int> search(
    const cv::Mat & rgb_image, const cv::Mat & segmented, int best_roadlike_class);

private:
  const float similarity_score_threshold_;
  rclcpp::Logger logger_;
};
}  // namespace pcdless::graph_segment
