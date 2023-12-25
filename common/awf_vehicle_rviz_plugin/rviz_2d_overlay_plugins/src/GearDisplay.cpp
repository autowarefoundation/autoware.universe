#include "GearDisplay.h"

#include <QFontDatabase>
#include <QPainter>
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

namespace awf_2d_overlay_vehicle
{

GearDisplay::GearDisplay() : current_gear_(0)
{
  int fontId =
    QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Regular.ttf");
  int fontId2 =
    QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Bold.ttf");
  if (fontId == -1 || fontId2 == -1) {
    std::cout << "Failed to load the Quicksand font.";
  }
}

GearDisplay::~GearDisplay()
{
  // Cleanup if necessary
}

void GearDisplay::updateGearData(
  const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr & msg)
{
  current_gear_ = msg->report;  // Assuming msg->report contains the gear information
}

void GearDisplay::drawGearIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  // we deal with the different gears here
  std::string gearString;
  switch (current_gear_) {
    case autoware_auto_vehicle_msgs::msg::GearReport::NEUTRAL:
      gearString = "N";
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::LOW:
    case autoware_auto_vehicle_msgs::msg::GearReport::LOW_2:
      gearString = "L";
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::NONE:
      gearString = "P";
      break;
    // all the drive gears from DRIVE to DRIVE_20
    default:
      gearString = "D";
      break;
  }

  QFont gearFont("Quicksand", 16, QFont::Bold);
  painter.setFont(gearFont);
  QPen borderPen(Qt::white);
  borderPen.setWidth(4);
  painter.setPen(borderPen);

  int gearBoxSize = 30;
  int gearX = backgroundRect.left() + 30 + gearBoxSize;
  int gearY = backgroundRect.height() - gearBoxSize - 20;
  QRect gearRect(gearX, gearY, gearBoxSize, gearBoxSize);
  painter.setBrush(QColor(0, 0, 0, 0));
  painter.drawRoundedRect(gearRect, 5, 5);
  painter.drawText(gearRect, Qt::AlignCenter, QString::fromStdString(gearString));
}

}  // namespace awf_2d_overlay_vehicle

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awf_2d_overlay_vehicle::GearDisplay, rviz_common::Display)
