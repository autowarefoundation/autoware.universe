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

#include "remaining_distance_time_display.hpp"

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

namespace autoware_mission_details_overlay_rviz_plugin
{

RemainingDistanceTimeDisplay::RemainingDistanceTimeDisplay() : 
remaining_distance_(0.0), remaining_hours_(0), remaining_minutes_(0), remaining_seconds_(0)
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_mission_details_overlay_rviz_plugin");
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

void RemainingDistanceTimeDisplay::updateRemainingDistanceTimeData(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg)
{
  try {
    remaining_distance_ = msg->remaining_distance;
    remaining_hours_    = msg->remaining_hours;
    remaining_minutes_  = msg->remaining_minutes;
    remaining_seconds_  = msg->remaining_seconds;
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

void RemainingDistanceTimeDisplay::drawRemainingDistanceTimeDisplay(
  QPainter & painter, const QRectF & backgroundRect)
{
  QFont referenceFont("Quicksand", 80, QFont::Bold);
  painter.setFont(referenceFont);
  QRect referenceRect = painter.fontMetrics().boundingRect("88");
  QPointF remainingDistReferencePos(
    backgroundRect.width() / 2 - referenceRect.width() / 2, backgroundRect.height() / 3);

  QPointF remainingTimeReferencePos(
    backgroundRect.width() / 2 - referenceRect.width() / 2, backgroundRect.height() / 1.3);

  // Remaining distance value
  QString remainingDistanceValue = QString::number(remaining_distance_, 'f', 0);
  int fontSize = 15;
  QFont remainingDistancValueFont("Quicksand", fontSize);
  painter.setFont(remainingDistancValueFont);

  QPointF remainingDistancePos(remainingDistReferencePos.x() + 100, remainingDistReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingDistancePos, remainingDistanceValue);

  // Remaining distance text
  QFont remainingDistancTextFont("Quicksand", 12);
  painter.setFont(remainingDistancTextFont);
  QString remainingDistText = "Remaining Distance: ";
  QPointF remainingDistancTextPos(
    remainingDistReferencePos.x() - 80, remainingDistReferencePos.y());
  painter.drawText(remainingDistancTextPos, remainingDistText);

  // Remaining distance unit
  QFont remainingDistancUnitFont("Quicksand", 12);
  painter.setFont(remainingDistancUnitFont);
  QString remainingDistUnitText = " m";
  QPointF remainingDistancUnitPos(
    remainingDistReferencePos.x() + 150, remainingDistReferencePos.y());
  painter.drawText(remainingDistancUnitPos, remainingDistUnitText);

  // Remaining time text
  QFont remainingTimeTextFont("Quicksand", 12);
  painter.setFont(remainingDistancTextFont);
  QString remainingTimeText = "Remaining Time: ";
  QPointF remainingTimeTextPos(remainingTimeReferencePos.x() - 80, remainingTimeReferencePos.y());
  painter.drawText(remainingTimeTextPos, remainingTimeText);

  // Remaining hours value
  QString remaininghoursValue = QString::number(remaining_hours_, 'f', 0);
  QFont remaininghoursValueFont("Quicksand", fontSize);
  painter.setFont(remaininghoursValueFont);

  QPointF remaininghoursValuePos(remainingTimeReferencePos.x() + 50, remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remaininghoursValuePos, remaininghoursValue);

  // Remaining hours separator
  QFont hoursSeparatorTextFont("Quicksand", 12);
  painter.setFont(hoursSeparatorTextFont);
  QString hoursSeparatorText = " h : ";

  QPointF hoursSeparatorTextPos(remainingTimeReferencePos.x() + 70, remainingTimeReferencePos.y());
  painter.drawText(hoursSeparatorTextPos, hoursSeparatorText);

  // Remaining minutes value
  QString remainingminutesValue = QString::number(remaining_minutes_, 'f', 0);
  QFont remainingminutesValueFont("Quicksand", fontSize);
  painter.setFont(remainingminutesValueFont);

  QPointF remainingminutesValuePos(
    remainingTimeReferencePos.x() + 100, remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingminutesValuePos, remainingminutesValue);

  // Remaining minutes separator
  QFont minutesSeparatorTextFont("Quicksand", 12);
  painter.setFont(minutesSeparatorTextFont);
  QString minutesSeparatorText = " m : ";
  QPointF minutesSeparatorTextPos(
    remainingTimeReferencePos.x() + 120, remainingTimeReferencePos.y());
  painter.drawText(minutesSeparatorTextPos, minutesSeparatorText);

  // Remaining seconds value
  QString remainingsecondsValue = QString::number(remaining_seconds_, 'f', 0);
  QFont remainingsecondsValueFont("Quicksand", fontSize);
  painter.setFont(remainingsecondsValueFont);

  QPointF remainingsecondValuePos(
    remainingTimeReferencePos.x() + 160, remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingsecondValuePos, remainingsecondsValue);

  // Remaining seconds separator
  QFont secondsSeparatorTextFont("Quicksand", 12);
  painter.setFont(secondsSeparatorTextFont);
  QString secondsSeparatorText = " s";
  QPointF secondsSeparatorTextPos(
    remainingTimeReferencePos.x() + 180, remainingTimeReferencePos.y());
  painter.drawText(secondsSeparatorTextPos, secondsSeparatorText);
}

}  // namespace autoware_mission_details_overlay_rviz_plugin
