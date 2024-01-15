#include "traffic_display.hpp"

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

TrafficDisplay::TrafficDisplay()
{
  traffic_light_image_.load(":/assets/images/traffic.png");
}

void TrafficDisplay::updateTrafficLightData(
  const rviz_2d_overlay_msgs::msg::TrafficSignalArrayUI::ConstSharedPtr & msg)
{
  current_traffic_ = *msg;
}

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

  painter.setBrush(QBrush(gray));
  painter.drawEllipse(circleRect.center(), 30, 30);

  // Define the area for the traffic light image (should be smaller or positioned within the circle)
  QRectF imageRect =
    circleRect.adjusted(15, 15, -15, -15);  // Adjusting the rectangle to make the image smaller

  QImage scaled_traffic_image = traffic_light_image_.scaled(
    imageRect.size().toSize(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

  if (current_traffic_.traffic_signals.size() > 0) {
    switch (current_traffic_.traffic_signals[0].elements[0].color) {
      case 1:
        painter.setBrush(QBrush(red));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      case 2:
        painter.setBrush(QBrush(yellow));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      case 3:
        painter.setBrush(QBrush(green));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      case 4:
        painter.setBrush(QBrush(gray));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
      default:
        painter.setBrush(QBrush(gray));
        painter.drawEllipse(circleRect.center(), 30, 30);
        break;
    }
  }
  // make the image thicker
  painter.setPen(QPen(Qt::black, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  painter.drawImage(imageRect, scaled_traffic_image);
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
