#include "TrafficDisplay.h"

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

TrafficDisplay::TrafficDisplay() : current_traffic_(0)
{
  traffic_light_image_.load(":/assets/images/traffic.png");
}

TrafficDisplay::~TrafficDisplay()
{
  // Cleanup if necessary
}

// void TrafficDisplay::updateTrafficData(const
// autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr &msg)
// {
//     current_gear_ = msg->report; // Assuming msg->report contains the gear information
// }

void TrafficDisplay::drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  // Define the area for the circle (background)
  QRectF circleRect = backgroundRect;
  circleRect.setWidth(backgroundRect.width() / 2 - 20);
  circleRect.setHeight(backgroundRect.height() - 20);
  circleRect.moveTopRight(QPointF(backgroundRect.right() - 10, backgroundRect.top() + 10));

  // #C2C2C2
  painter.setBrush(QBrush(QColor(194, 194, 194)));
  painter.drawEllipse(circleRect.center(), 30, 30);

  // Define the area for the traffic light image (should be smaller or positioned within the circle)
  QRectF imageRect =
    circleRect.adjusted(15, 15, -15, -15);  // Adjusting the rectangle to make the image smaller

  // make the image thicker
  painter.setPen(QPen(Qt::black, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  // Draw the traffic light image on top of the circle
  painter.drawImage(
    imageRect, traffic_light_image_.scaled(
                 imageRect.size().toSize(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

QImage TrafficDisplay::coloredImage(const QImage & source, const QColor & color)
{
  QImage result = source;
  QPainter p(&result);
  p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  p.fillRect(result.rect(), color);
  p.end();
  return result;
}

}  // namespace awf_2d_overlay_vehicle

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awf_2d_overlay_vehicle::TrafficDisplay, rviz_common::Display)
