#ifndef GEARDISPLAY_HPP_
#define GEARDISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

namespace awf_2d_overlay_vehicle
{

class GearDisplay
{
public:
  GearDisplay();
  void drawGearIndicator(QPainter & painter, const QRectF & backgroundRect);
  void updateGearData(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr & msg);

private:
  int current_gear_;  // Internal variable to store current gear
};

}  // namespace awf_2d_overlay_vehicle

#endif  // GEARDISPLAY_HPP_
