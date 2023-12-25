#ifndef STEERINGWHEELDISPLAY_H_
#define STEERINGWHEELDISPLAY_H_
#ifndef Q_MOC_RUN
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
#endif

namespace awf_2d_overlay_vehicle
{

class SteeringWheelDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  SteeringWheelDisplay();
  virtual ~SteeringWheelDisplay() override;
  void drawSteeringWheel(QPainter & painter, const QRectF & backgroundRect);
  void updateSteeringData(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr & msg);

private:
  float steering_angle_ = 0.0f;

  QImage wheelImage;
  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace awf_2d_overlay_vehicle

#endif  // STEERINGWHEELDISPLAY_H_
