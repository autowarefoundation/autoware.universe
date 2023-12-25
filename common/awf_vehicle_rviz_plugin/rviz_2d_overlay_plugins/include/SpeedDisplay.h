#ifndef SPEEDDISPLAY_H_
#define SPEEDDISPLAY_H_
#ifndef Q_MOC_RUN
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
#endif

namespace awf_2d_overlay_vehicle
{

class SpeedDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  SpeedDisplay();
  virtual ~SpeedDisplay() override;
  void drawSpeedDisplay(QPainter & painter, const QRectF & backgroundRect);
  void updateSpeedData(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg);

private:
  float current_speed_;  // Internal variable to store current speed
};

}  // namespace awf_2d_overlay_vehicle

#endif  // SPEEDDISPLAY_H_
