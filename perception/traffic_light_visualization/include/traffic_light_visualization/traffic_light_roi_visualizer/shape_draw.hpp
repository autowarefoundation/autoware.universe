#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <functional>
#include <map>

struct DrawFunctionParams
{
  cv::Mat & image;
  cv::Point position;
  cv::Scalar color;
  int size;
};

using DrawFunction = std::function<void(const DrawFunctionParams & params)>;

void drawCircle(const DrawFunctionParams & params);
void drawLeftArrow(const DrawFunctionParams & params);
void drawRightArrow(const DrawFunctionParams & params);
void drawStraightArrow(const DrawFunctionParams & params);
void drawDownArrow(const DrawFunctionParams & params);
void drawDownLeftArrow(const DrawFunctionParams & params);
void drawDownRightArrow(const DrawFunctionParams & params);
void drawCross(const DrawFunctionParams & params);
void drawTrafficLightShape(
  cv::Mat & image, const std::string & shape, const cv::Point & position, const cv::Scalar & color,
  int size);
