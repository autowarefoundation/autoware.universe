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
#include <qchar.h>
#include <qpoint.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace autoware_mission_details_overlay_rviz_plugin
{

RemainingDistanceTimeDisplay::RemainingDistanceTimeDisplay()
: remaining_distance_(0.0), remaining_hours_(0), remaining_minutes_(0), remaining_seconds_(0)
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

  // Load the wheel image
  std::string dist_image = package_path + "/assets/score_white.png";
  std::string time_image = package_path + "/assets/timelapse.png";
  distToGoalFlag.load(dist_image.c_str());
  timeToGoalFlag.load(time_image.c_str());
  scaledDistToGoalFlag =
    distToGoalFlag.scaled(32, 32, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  scaledTimeToGoalFlag =
    timeToGoalFlag.scaled(32, 32, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void RemainingDistanceTimeDisplay::updateRemainingDistanceTimeData(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg)
{
  try {
    remaining_distance_ = msg->remaining_distance;
    remaining_hours_ = msg->remaining_hours;
    remaining_minutes_ = msg->remaining_minutes;
    remaining_seconds_ = msg->remaining_seconds;
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

  // ----------------- Remaining Distance -----------------

  int fontSize = 15;
  QFont remainingDistancTimeFont("Quicksand", fontSize, QFont::Bold);
  painter.setFont(remainingDistancTimeFont);

  // Remaining distance icon
  QPointF remainingDistanceIconPos(
    remainingDistReferencePos.x() - 25, remainingDistReferencePos.y());
  painter.drawImage(
    remainingDistanceIconPos.x(),
    remainingDistanceIconPos.y() - scaledDistToGoalFlag.height() / 2.0, scaledDistToGoalFlag);

  // Remaining distance value
  QString remainingDistanceValue = QString::number(
    remaining_distance_ > 1000 ? remaining_distance_ / 1000 : remaining_distance_, 'f', 0);
  QPointF remainingDistancePos;
  switch (remainingDistanceValue.size()) {
    case 1:
      remainingDistancePos =
        QPointF(remainingDistReferencePos.x() + 100, remainingDistReferencePos.y() + 10);
      break;
    case 2:
      remainingDistancePos =
        QPointF(remainingDistReferencePos.x() + 95, remainingDistReferencePos.y() + 10);
      break;
    case 3:
      remainingDistancePos =
        QPointF(remainingDistReferencePos.x() + 90, remainingDistReferencePos.y() + 10);
      break;
    case 4:
      remainingDistancePos =
        QPointF(remainingDistReferencePos.x() + 85, remainingDistReferencePos.y() + 10);
      break;
    default:
      remainingDistancePos =
        QPointF(remainingDistReferencePos.x() + 100, remainingDistReferencePos.y() + 10);
      break;
  }
  painter.setPen(gray);
  painter.drawText(remainingDistancePos, remainingDistanceValue);

  // Remaining distance unit
  QString remainingDistUnitText = remaining_distance_ > 1000 ? "km" : "m";
  QPointF remainingDistancUnitPos(
    remainingDistReferencePos.x() + 125, remainingDistReferencePos.y() + 10);
  painter.drawText(remainingDistancUnitPos, remainingDistUnitText);

  //  ----------------- Remaining Time -----------------

  // Remaining time text
  painter.drawImage(
    remainingDistanceIconPos.x(),
    remainingDistanceIconPos.y() + scaledTimeToGoalFlag.height() / 2.0, scaledTimeToGoalFlag);

  // Remaining hours value
  QString remaininghoursValue =
    QString::number(remaining_hours_ != 0 ? remaining_hours_ : 0, 'f', 0);
  QPointF remaininghoursValuePos(remainingTimeReferencePos.x() + 17, remainingTimeReferencePos.y());
  painter.setPen(gray);
  if (remaining_hours_ != 0) painter.drawText(remaininghoursValuePos, remaininghoursValue);

  // Remaining hours separator
  QString hoursSeparatorText = "h";
  QPointF hoursSeparatorTextPos(remainingTimeReferencePos.x() + 35, remainingTimeReferencePos.y());
  if (remaining_hours_ != 0) painter.drawText(hoursSeparatorTextPos, hoursSeparatorText);

  // Remaining minutes value
  QString remainingminutesValue =
    QString::number(remaining_minutes_ != 0 ? remaining_minutes_ : 0, 'f', 0);
  QPointF remainingminutesValuePos(
    remainingTimeReferencePos.x() + 55, remainingTimeReferencePos.y());
  painter.setPen(gray);
  if (remaining_minutes_ != 0) painter.drawText(remainingminutesValuePos, remainingminutesValue);
  // Remaining minutes separator
  QString minutesSeparatorText = "m";
  QPointF minutesSeparatorTextPos(
    remainingTimeReferencePos.x() + 80, remainingTimeReferencePos.y());
  if (remaining_minutes_ != 0) painter.drawText(minutesSeparatorTextPos, minutesSeparatorText);

  // Remaining seconds value
  QString remainingsecondsValue =
    QString::number(remaining_seconds_ != 0 ? remaining_seconds_ : 0, 'f', 0);
  QPointF remainingsecondValuePos(
    remainingTimeReferencePos.x() + 100, remainingTimeReferencePos.y());
  painter.setPen(gray);
  painter.drawText(remainingsecondValuePos, remainingsecondsValue);

  // Remaining seconds separator
  QString secondsSeparatorText = "s";
  QPointF secondsSeparatorTextPos(
    remainingTimeReferencePos.x() + 125, remainingTimeReferencePos.y());
  painter.drawText(secondsSeparatorTextPos, secondsSeparatorText);
}

}  // namespace autoware_mission_details_overlay_rviz_plugin
