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
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

void drawShape(
  const DrawFunctionParams & params, const std::string & filename, bool flipHorizontally,
  bool flipVertically, int x_offset, int y_offset)
{
  std::string filepath =
    ament_index_cpp::get_package_share_directory("traffic_light_visualization") + "/images/" +
    filename;
  cv::Mat shapeImg = cv::imread(filepath, cv::IMREAD_UNCHANGED);
  if (shapeImg.empty()) {
    std::cerr << "Failed to load image: " << filepath << std::endl;
    return;
  }

  if (flipHorizontally) {
    cv::flip(shapeImg, shapeImg, 1);  // Flip horizontally
  }

  if (flipVertically) {
    cv::flip(shapeImg, shapeImg, 0);  // Flip vertically
  }

  // Resize image if needed
  double scale_factor = 0.25;  // Scale factor to reduce the size by 50%
  cv::resize(
    shapeImg, shapeImg, cv::Size(params.size, params.size), scale_factor, scale_factor,
    cv::INTER_AREA);

  // Calculate the center position including offsets
  cv::Point position(
    params.position.x + x_offset, params.position.y - shapeImg.rows / 2 + y_offset);

  // Check for image boundaries
  if (
    position.x < 0 || position.y < 0 || position.x + shapeImg.cols > params.image.cols ||
    position.y + shapeImg.rows > params.image.rows) {
    std::cerr << "Adjusted position is out of image bounds." << std::endl;
    return;
  }

  // Draw a rectangle background before placing the image
  cv::rectangle(
    params.image,
    cv::Rect(
      // width should take into account the text width
      position.x - 2, position.y - 5, shapeImg.cols + 70, shapeImg.rows + 12),
    params.color,
    -1);  // Filled rectangle

  // Create ROI on the destination image
  cv::Mat destinationROI = params.image(cv::Rect(position, cv::Size(shapeImg.cols, shapeImg.rows)));

  // Overlay the image onto the main image
  for (int y = 0; y < shapeImg.rows; ++y) {
    for (int x = 0; x < shapeImg.cols; ++x) {
      cv::Vec4b & pixel = shapeImg.at<cv::Vec4b>(y, x);
      if (pixel[3] != 0) {  // Only non-transparent pixels
        destinationROI.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel[0], pixel[1], pixel[2]);
      }
    }
  }

  // position the probability text right next to the shape
  cv::putText(
    params.image, std::to_string(static_cast<int>(round(params.probability * 100))) + "%",
    cv::Point(position.x + shapeImg.cols + 5, position.y + shapeImg.rows / 2 + 5),
    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
}

void drawCircle(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2;
  drawShape(params, "circle.png", false, false, 0, -y_offset);
}

void drawLeftArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;
  drawShape(params, "left_arrow.png", false, false, 0, -y_offset);
}

void drawRightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;
  drawShape(params, "left_arrow.png", true, false, 0, -y_offset);
}

void drawStraightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;  // This adjusts the base position upwards

  drawShape(params, "straight_arrow.png", false, false, 0, -y_offset);
}
void drawDownArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;  // This adjusts the base position upwards
  drawShape(params, "straight_arrow.png", false, true, 0, -y_offset);
}

void drawDownLeftArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 15;
  drawShape(params, "down_left_arrow.png", false, false, 0, -y_offset);
}

void drawDownRightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 15;
  drawShape(params, "down_left_arrow.png", true, false, 0, -y_offset);
}

void drawCross(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 10;

  drawShape(params, "cross.png", false, false, 0, -y_offset);
}

void drawTrafficLightShape(
  cv::Mat & image, const std::string & shape, const cv::Point & position, const cv::Scalar & color,
  int size, float probability)
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
    DrawFunctionParams params{image, position, color, size, probability};
    it->second(params);
  } else {
    cv::putText(
      image, "?", cv::Point(position.x, position.y - 5), cv::FONT_HERSHEY_COMPLEX, 0.5, color, 2);
  }
}
