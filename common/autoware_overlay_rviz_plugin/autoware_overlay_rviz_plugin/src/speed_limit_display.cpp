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

#include "speed_limit_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_rendering/render_system.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace autoware_overlay_rviz_plugin
{

SpeedLimitDisplay::SpeedLimitDisplay() : current_limit(0.0), current_speed_(0.0)
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_overlay_rviz_plugin");
  std::string font_path = package_path + "/assets/font/Quicksand/static/Quicksand-Regular.ttf";
  std::string font_path2 = package_path + "/assets/font/Quicksand/static/Quicksand-Bold.ttf";
  int fontId = QFontDatabase::addApplicationFont(
    font_path.c_str());  // returns -1 on failure (see docs for more info)
  int fontId2 = QFontDatabase::addApplicationFont(
    font_path2.c_str());  // returns -1 on failure (see docs for more info)
  if (fontId == -1 || fontId2 == -1) {
    std::cout << "Failed to load the Quicksand font.";
  }
}

void SpeedLimitDisplay::updateSpeedLimitData(
  const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg)
{
  current_limit = msg->max_velocity;
}

void SpeedLimitDisplay::updateSpeedData(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg)
{
  try {
    float speed = msg->longitudinal_velocity;
    current_speed_ = speed;
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

void SpeedLimitDisplay::drawSpeedLimitIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  // Convert color_min and color_max to QColor
  QColor colorMin = QColor("#FF9999");
  QColor colorMax = QColor("#FF3333");

  // Calculate the ratio between the current speed and the speed limit
  double speed_to_limit_ratio = current_speed_ / current_limit;

  QColor borderColor = colorMin;

  if (speed_to_limit_ratio > 0.7 && current_limit > 0.0) {
    // Adjust the interpolation to start more gradually
    // Calculate a new factor that slows down the transition
    double adjusted_speed_to_limit_ratio = (speed_to_limit_ratio - 0.7) / (1.0 - 0.7);
    double smoothFactor =
      pow(adjusted_speed_to_limit_ratio, 2);  // Use a power to make it interpolate slower

    int r = std::min(
      std::max(
        0, colorMin.red() + static_cast<int>((colorMax.red() - colorMin.red()) * smoothFactor)),
      255);
    int g = std::min(
      std::max(
        0,
        colorMin.green() + static_cast<int>((colorMax.green() - colorMin.green()) * smoothFactor)),
      255);
    int b = std::min(
      std::max(
        0, colorMin.blue() + static_cast<int>((colorMax.blue() - colorMin.blue()) * smoothFactor)),
      255);

    borderColor = QColor(r, g, b);
  }

  // Define the area for the outer circle
  QRectF outerCircleRect = backgroundRect;
  outerCircleRect.setWidth(backgroundRect.width() - 30);
  outerCircleRect.setHeight(backgroundRect.width() - 30);
  outerCircleRect.moveTopLeft(QPointF(backgroundRect.left() + 15, backgroundRect.top() + 10));

  // Now use borderColor for drawing
  painter.setPen(QPen(borderColor, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.setBrush(borderColor);
  // Draw the outer circle for the speed limit indicator
  painter.drawEllipse(outerCircleRect);

  // Define the area for the inner circle
  QRectF innerCircleRect = outerCircleRect;
  innerCircleRect.setWidth(outerCircleRect.width() / 1.25);
  innerCircleRect.setHeight(outerCircleRect.height() / 1.25);
  innerCircleRect.moveCenter(outerCircleRect.center());

  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(0, 0, 0);  // Hue, Saturation, Value

  painter.setBrush(colorFromHSV);
  painter.drawEllipse(innerCircleRect);

  int current_limit_int = std::round(current_limit * 3.6);

  // Define the text to be drawn
  QString text = QString::number(current_limit_int);

  // Set the font and color for the text
  QFont font = QFont("Quicksand", 24, QFont::Bold);

  painter.setFont(font);
  // #C2C2C2
  painter.setPen(QPen(gray, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  // Draw the text in the center of the circle
  painter.drawText(innerCircleRect, Qt::AlignCenter, text);
}

}  // namespace autoware_overlay_rviz_plugin
