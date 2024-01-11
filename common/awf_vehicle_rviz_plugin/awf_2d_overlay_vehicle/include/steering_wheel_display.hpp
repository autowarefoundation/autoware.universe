#ifndef STEERING_WHEEL_DISPLAY_HPP_
#define STEERING_WHEEL_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace awf_2d_overlay_vehicle
{

class SteeringWheelDisplay
{
public:
  SteeringWheelDisplay();
  void drawSteeringWheel(QPainter & painter, const QRectF & backgroundRect);
  void updateSteeringData(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr & msg);

private:
  float steering_angle_ = 0.0f;

  QImage wheelImage;
  QImage scaledWheelImage;
  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace awf_2d_overlay_vehicle

#endif  // STEERING_WHEEL_DISPLAY_HPP_
