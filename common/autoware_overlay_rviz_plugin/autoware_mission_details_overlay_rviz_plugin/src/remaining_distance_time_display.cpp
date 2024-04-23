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

RemainingDistanceTimeDisplay::RemainingDistanceTimeDisplay() : remaining_distance_(0.0), hours_(0), minutes_(0), seconds_(0)
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
  const autoware_planning_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg)
{
  try {
    remaining_distance_ = msg->remaining_distance;
    hours_   = msg->remaining_hours;
    minutes_ = msg->remaining_minutes;
    seconds_ = msg->remaining_seconds;
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

void RemainingDistanceTimeDisplay::drawRemainingDistanceTimeDisplay(QPainter & painter, const QRectF & backgroundRect)
{
  QFont referenceFont("Quicksand", 80, QFont::Bold);
  painter.setFont(referenceFont);
  QRect referenceRect = painter.fontMetrics().boundingRect("88");
  QPointF remainingDistReferencePos(
    backgroundRect.width() / 2 - referenceRect.width() / 2, backgroundRect.height() / 3);

  QPointF remainingTimeReferencePos(
    backgroundRect.width() / 2 - referenceRect.width() / 2, backgroundRect.height() / 1.3);

  QString remainingDistanceValue = QString::number(remaining_distance_, 'f', 0);
  int fontSize = 15;
  QFont remainingDistancValueFont("Quicksand", fontSize);
  painter.setFont(remainingDistancValueFont);

  // Calculate the bounding box of the remaining distance value
  QRect remainingDistancValueRect = painter.fontMetrics().boundingRect(remainingDistanceValue);

  // Top the remaining distance in the backgroundRect
  QPointF remainingDistancePos(
    remainingDistReferencePos.x() + 100 , remainingDistReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingDistancePos, remainingDistanceValue);

  QFont remainingDistancTextFont("Quicksand", 12);
  painter.setFont(remainingDistancTextFont);
  QString remainingDistText = "Remaining Distance: ";
  QRect remainingDistancTextRect = painter.fontMetrics().boundingRect(remainingDistText);
  // QPointF remainingDistancTextPos(
  //   (backgroundRect.width() / 2 - remainingDistancTextRect.width() / 2), referencePos.y() + remainingDistancTextRect.height());
    QPointF remainingDistancTextPos(remainingDistReferencePos.x() - 80, remainingDistReferencePos.y());
  painter.drawText(remainingDistancTextPos, remainingDistText);


  QFont remainingDistancUnitFont("Quicksand", 12);
  painter.setFont(remainingDistancUnitFont);
  QString remainingDistUnitText = " m";
  QRect remainingDistUnitTextRect = painter.fontMetrics().boundingRect(remainingDistUnitText);
  // QPointF remainingDistancTextPos(
  //   (backgroundRect.width() / 2 - remainingDistancTextRect.width() / 2), referencePos.y() + remainingDistancTextRect.height());
    QPointF remainingDistancUnitPos(remainingDistReferencePos.x() + 150, remainingDistReferencePos.y());
  painter.drawText(remainingDistancUnitPos, remainingDistUnitText);


  QFont remainingTimeTextFont("Quicksand", 12);
  painter.setFont(remainingDistancTextFont);
  QString remainingTimeText = "Remaining Time: ";
  QRect remainingTimeTextRect = painter.fontMetrics().boundingRect(remainingTimeText);
  // QPointF remainingDistancTextPos(
  //   (backgroundRect.width() / 2 - remainingDistancTextRect.width() / 2), referencePos.y() + remainingDistancTextRect.height());
  QPointF remainingTimeTextPos(remainingTimeReferencePos.x() - 80, remainingTimeReferencePos.y());
  painter.drawText(remainingTimeTextPos, remainingTimeText);
  
  
  QString remaininghoursValue = QString::number(hours_, 'f', 0);
  QFont remaininghoursValueFont("Quicksand", fontSize);
  painter.setFont(remaininghoursValueFont);

  // Calculate the bounding box of the remaining distance value
  QRect remaininghoursValueRect = painter.fontMetrics().boundingRect(remaininghoursValue);

  // Top the remaining distance in the backgroundRect
  QPointF remaininghoursValuePos(
    remainingTimeReferencePos.x() + 50 , remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remaininghoursValuePos, remaininghoursValue);

  QFont hoursSeparatorTextFont("Quicksand", 12);
  painter.setFont(hoursSeparatorTextFont);
  QString hoursSeparatorText = " h : ";
  QRect hoursSeparatorTextRect = painter.fontMetrics().boundingRect(hoursSeparatorText);
  // QPointF remainingDistancTextPos(
  //   (backgroundRect.width() / 2 - remainingDistancTextRect.width() / 2), referencePos.y() + remainingDistancTextRect.height());
  QPointF hoursSeparatorTextPos(remainingTimeReferencePos.x() + 70, remainingTimeReferencePos.y());
  painter.drawText(hoursSeparatorTextPos, hoursSeparatorText);

  QString remainingminutesValue = QString::number(minutes_, 'f', 0);
  QFont remainingminutesValueFont("Quicksand", fontSize);
  painter.setFont(remainingminutesValueFont);

  // Calculate the bounding box of the remaining distance value
  QRect remainingminutesValueRect = painter.fontMetrics().boundingRect(remainingminutesValue);

  // Top the remaining distance in the backgroundRect
  QPointF remainingminutesValuePos(
    remainingTimeReferencePos.x() + 100 , remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingminutesValuePos, remainingminutesValue);


  QFont minutesSeparatorTextFont("Quicksand", 12);
  painter.setFont(minutesSeparatorTextFont);
  QString minutesSeparatorText = " m : ";
  QRect minutesSeparatorTextRect = painter.fontMetrics().boundingRect(minutesSeparatorText);
  // QPointF remainingDistancTextPos(
  //   (backgroundRect.width() / 2 - remainingDistancTextRect.width() / 2), referencePos.y() + remainingDistancTextRect.height());
  QPointF minutesSeparatorTextPos(remainingTimeReferencePos.x() + 120, remainingTimeReferencePos.y());
  painter.drawText(minutesSeparatorTextPos, minutesSeparatorText);

  QString remainingsecondsValue = QString::number(seconds_, 'f', 0);
  QFont remainingsecondsValueFont("Quicksand", fontSize);
  painter.setFont(remainingsecondsValueFont);

  // Calculate the bounding box of the remaining distance value
  QRect remainingsecondsValueRect = painter.fontMetrics().boundingRect(remainingsecondsValue);

  // Top the remaining distance in the backgroundRect
  QPointF remainingsecondValuePos(
    remainingTimeReferencePos.x() + 160 , remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingsecondValuePos, remainingsecondsValue);

  QFont secondsSeparatorTextFont("Quicksand", 12);
  painter.setFont(secondsSeparatorTextFont);
  QString secondsSeparatorText = " s";
  QRect secondsSeparatorTextRect = painter.fontMetrics().boundingRect(secondsSeparatorText);
  // QPointF remainingDistancTextPos(
  //   (backgroundRect.width() / 2 - remainingDistancTextRect.width() / 2), referencePos.y() + remainingDistancTextRect.height());
  QPointF secondsSeparatorTextPos(remainingTimeReferencePos.x() + 180, remainingTimeReferencePos.y());
  painter.drawText(secondsSeparatorTextPos, secondsSeparatorText);
  
}

}  // namespace autoware_mission_details_overlay_rviz_plugin
