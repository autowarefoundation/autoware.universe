// Copyright 2024 The Autoware Contributors
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
#include "traffic_light_visualization/traffic_light_roi_visualizer/shape_draw.hpp"

#include "opencv2/core/types.hpp"

void drawCircle(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;
  cv::circle(
    params.image, cv::Point(params.position.x, params.position.y - y_offset), params.size / 2.0,
    params.color, -1);  // Filled circle
}

void drawLeftArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;
  // Start from the right, go to the left (start x > end x)
  cv::arrowedLine(
    params.image, cv::Point(params.position.x, params.position.y - y_offset),  // Start point
    cv::Point(
      params.position.x - params.size, params.position.y - y_offset),  // End point to the left
    params.color, 2, 8, 0, 0.3);
}

void drawRightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;
  // Start from the left, go to the right (start x < end x)
  cv::arrowedLine(
    params.image, cv::Point(params.position.x, params.position.y - y_offset),  // Start point
    cv::Point(
      params.position.x + params.size, params.position.y - y_offset),  // End point to the right
    params.color, 2, 8, 0, 0.3);
}

void drawStraightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;  // This adjusts the base position upwards
  // Start point is lower, and it goes upwards to a higher (smaller y-value) point
  cv::arrowedLine(
    params.image, cv::Point(params.position.x, params.position.y - y_offset),  // Start point
    cv::Point(params.position.x, params.position.y - params.size - y_offset),  // End point
    params.color, 2, 8, 0, 0.3);
}
void drawDownArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;  // This adjusts the base position upwards
  // Start point is higher, and it goes downwards to a lower (larger y-value) point
  cv::arrowedLine(
    params.image, cv::Point(params.position.x, params.position.y - y_offset),  // Start point
    cv::Point(params.position.x, params.position.y + params.size - y_offset),  // End point
    params.color, 2, 8, 0, 0.3);
}

void drawDownLeftArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 15;
  // Down-left arrow
  cv::arrowedLine(
    params.image, cv::Point(params.position.x, params.position.y - y_offset),
    cv::Point(params.position.x - params.size, params.position.y + params.size - y_offset),
    params.color, 2, 8, 0, 0.3);
}

void drawDownRightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 15;
  // Down-right arrow
  cv::arrowedLine(
    params.image, cv::Point(params.position.x, params.position.y - y_offset),
    cv::Point(params.position.x + params.size, params.position.y + params.size - y_offset),
    params.color, 2, 8, 0, 0.3);
}

void drawCross(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;
  // Cross (two lines intersecting)
  int half_size = params.size / 2;
  cv::line(
    params.image,
    cv::Point(params.position.x - half_size, params.position.y - half_size - y_offset),
    cv::Point(params.position.x + half_size, params.position.y + half_size - y_offset),
    params.color, 2);
  cv::line(
    params.image,
    cv::Point(params.position.x + half_size, params.position.y - half_size - y_offset),
    cv::Point(params.position.x - half_size, params.position.y + half_size - y_offset),
    params.color, 2);
}

void drawTrafficLightShape(
  cv::Mat & image, const std::string & shape, const cv::Point & position, const cv::Scalar & color,
  int size)
{
  static std::map<std::string, DrawFunction> shapeToFunction = {
    {"circle", drawCircle},
    {"left", drawLeftArrow},
    {"right", drawRightArrow},
    {"straight", drawStraightArrow},
    {"down", drawDownArrow},
    {"down_left", drawDownLeftArrow},
    {"down_right", drawDownRightArrow},
    {"cross", drawCross}};
  auto it = shapeToFunction.find(shape);
  if (it != shapeToFunction.end()) {
    DrawFunctionParams params{image, position, color, size};
    it->second(params);
  } else {
    std::cerr << "Unknown shape: " << shape << std::endl;
  }
}
