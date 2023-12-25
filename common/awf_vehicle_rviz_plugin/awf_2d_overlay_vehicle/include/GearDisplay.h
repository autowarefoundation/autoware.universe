#ifndef GEARDISPLAY_H_
#define GEARDISPLAY_H_
#ifndef Q_MOC_RUN
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
#endif

namespace awf_2d_overlay_vehicle
{

class GearDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  GearDisplay();
  virtual ~GearDisplay() override;
  void drawGearIndicator(QPainter & painter, const QRectF & backgroundRect);
  void updateGearData(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr & msg);

private:
  int current_gear_;  // Internal variable to store current gear
};

}  // namespace awf_2d_overlay_vehicle

#endif  // GEARDISPLAY_H_
