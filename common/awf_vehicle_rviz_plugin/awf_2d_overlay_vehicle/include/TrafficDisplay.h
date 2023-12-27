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

// #include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
// #include <autoware_perception_msgs/msg/traffic_signal.hpp>
// #include <autoware_perception_msgs/msg/traffic_signal_element.hpp>
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
  // void updateTrafficLightData(const
  // autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg);

private:
  int current_traffic_;  // Internal variable to store current gear
  QImage traffic_light_image_;
  // yellow #CFC353
  QColor yellow = QColor(207, 195, 83);
  // red #CF5353
  QColor red = QColor(207, 83, 83);
  // green #53CF5F
  QColor green = QColor(83, 207, 95);
  // gray #C2C2C2
  QColor gray = QColor(194, 194, 194);

  QImage coloredImage(const QImage & source, const QColor & color);
};

}  // namespace awf_2d_overlay_vehicle

#endif  // TRAFFICDISPLAY_H_
