#ifndef TURN_SIGNALS_DISPLAY_HPP_
#define TURN_SIGNALS_DISPLAY_HPP_
#include "overlay_utils.hpp"

#include <QImage>
#include <QString>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <chrono>

namespace awf_2d_overlay_vehicle
{

class TurnSignalsDisplay
{
public:
  TurnSignalsDisplay();
  void drawArrows(QPainter & painter, const QRectF & backgroundRect, const QColor & color);
  void updateTurnSignalsData(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr & msg);
  void updateHazardLightsData(
    const autoware_auto_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr & msg);

private:
  QImage arrowImage;
  QColor gray = QColor(194, 194, 194);

  int current_turn_signal_;    // Internal variable to store turn signal state
  int current_hazard_lights_;  // Internal variable to store hazard lights state
  QImage coloredImage(const QImage & source, const QColor & color);

  std::chrono::steady_clock::time_point last_toggle_time_;
  bool blink_on_ = false;
  const std::chrono::milliseconds blink_interval_{500};  // Blink interval in milliseconds
};

}  // namespace awf_2d_overlay_vehicle

#endif  // TURN_SIGNALS_DISPLAY_HPP_
