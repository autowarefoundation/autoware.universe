#include "speed_display.hpp"

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

SpeedDisplay::SpeedDisplay() : current_speed_(0.0)
{
  int fontId =
    QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Regular.ttf");
  int fontId2 =
    QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Bold.ttf");
  if (fontId == -1 || fontId2 == -1) {
    std::cout << "Failed to load the Quicksand font.";
  }
}

void SpeedDisplay::updateSpeedData(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg)
{
  try {
    // Assuming msg->state.longitudinal_velocity_mps is the field you're interested in
    float speed = msg->longitudinal_velocity;
    // we received it as a m/s value, but we want to display it in km/h
    current_speed_ = (speed * 3.6);
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

// void SpeedDisplay::processMessage(const
// autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
// {
//     try
//     {
//         current_speed_ = std::round(msg->state.longitudinal_velocity_mps * 3.6);
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << "Error in processMessage: " << e.what() << std::endl;
//     }
// }

void SpeedDisplay::drawSpeedDisplay(QPainter & painter, const QRectF & backgroundRect)
{
  QFont referenceFont("Quicksand", 80, QFont::Bold);
  painter.setFont(referenceFont);
  QRect referenceRect = painter.fontMetrics().boundingRect("88");
  QPointF referencePos(
    backgroundRect.width() / 2 - referenceRect.width() / 2 - 5, backgroundRect.height() / 2);

  QString speedNumber = QString::number(current_speed_, 'f', 0);
  int fontSize = 60;
  QFont speedFont("Quicksand", fontSize);
  painter.setFont(speedFont);

  // Calculate the bounding box of the speed number
  QRect speedNumberRect = painter.fontMetrics().boundingRect(speedNumber);

  // Center the speed number in the backgroundRect
  QPointF speedPos(
    backgroundRect.center().x() - speedNumberRect.width() / 2, backgroundRect.center().y());
  painter.setPen(Qt::white);
  painter.drawText(speedPos, speedNumber);

  QFont unitFont("Quicksand", 14);
  painter.setFont(unitFont);
  QString speedUnit = "km/h";
  QRect unitRect = painter.fontMetrics().boundingRect(speedUnit);
  QPointF unitPos(
    (backgroundRect.width() / 2 - unitRect.width() / 2), referencePos.y() + unitRect.height());
  painter.drawText(unitPos, speedUnit);
}

}  // namespace awf_2d_overlay_vehicle
