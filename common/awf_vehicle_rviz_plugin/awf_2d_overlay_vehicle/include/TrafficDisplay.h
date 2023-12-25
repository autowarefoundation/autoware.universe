#ifndef TRAFFICDISPLAY_H_
#define TRAFFICDISPLAY_H_
#ifndef Q_MOC_RUN
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>
// #include "autoware_auto_vehicle_msgs/msg/traf
#endif

namespace awf_2d_overlay_vehicle
{

class TrafficDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  TrafficDisplay();
  virtual ~TrafficDisplay() override;
  void drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect);
  // void updateTrafficLightData(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr
  // &msg);

private:
  int current_traffic_;  // Internal variable to store current gear
  QImage traffic_light_image_;

  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace awf_2d_overlay_vehicle

#endif  // TRAFFICDISPLAY_H_
