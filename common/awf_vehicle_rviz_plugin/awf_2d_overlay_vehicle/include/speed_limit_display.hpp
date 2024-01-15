#ifndef SPEED_LIMIT_DISPLAY_HPP_
#define SPEED_LIMIT_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace awf_2d_overlay_vehicle
{

class SpeedLimitDisplay
{
public:
  SpeedLimitDisplay();
  void drawSpeedLimitIndicator(QPainter & painter, const QRectF & backgroundRect);
  void updateSpeedLimitData(const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);

private:
  float current_limit;  // Internal variable to store current gear
  QColor gray = QColor(194, 194, 194);
};

}  // namespace awf_2d_overlay_vehicle

#endif  // SPEED_LIMIT_DISPLAY_HPP_
