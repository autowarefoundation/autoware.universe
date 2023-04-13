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
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include <string>

namespace refine_optimizer
{
void addText(const std::string & text, const cv::Mat & image)
{
  constexpr int fontscale = 2;
  constexpr int fontface = cv::FONT_HERSHEY_PLAIN;
  constexpr int thickness = 2;
  int cumulative_height = 10;
  int max_width = 10;

  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    int balse_line;
    cv::Size size = cv::getTextSize(line, fontface, fontscale, thickness, &balse_line);
    cumulative_height += (size.height + 5);
    max_width = std::max(max_width, size.width);
  }
  cv::rectangle(
    image, cv::Rect(40 + 5, 20 - 5, max_width, cumulative_height), cv::Scalar::all(0), -1);
  cumulative_height = 10;

  ss = std::stringstream(text);
  while (std::getline(ss, line)) {
    int balse_line;
    cv::Size size = cv::getTextSize(line, fontface, fontscale, thickness, &balse_line);
    cumulative_height += (size.height + 5);
    cv::putText(
      image, line, cv::Point2i(50, cumulative_height), fontface, fontscale, cv::Scalar(0, 255, 255),
      thickness);
  }
}

}  // namespace refine_optimizer