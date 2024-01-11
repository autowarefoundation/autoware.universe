#ifndef SPEED_DISPLAY_HPP_
#define SPEED_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace awf_2d_overlay_vehicle
{

class SpeedDisplay
{
public:
  SpeedDisplay();
  void drawSpeedDisplay(QPainter & painter, const QRectF & backgroundRect);
  void updateSpeedData(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg);

private:
  float current_speed_;  // Internal variable to store current speed
};

}  // namespace awf_2d_overlay_vehicle

#endif  // SPEED_DISPLAY_HPP_
