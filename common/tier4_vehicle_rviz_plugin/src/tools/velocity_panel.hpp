#ifndef RVIZ2_CUSTOM_PLUGIN_VELOCITY_PANEL_HPP_
#define RVIZ2_CUSTOM_PLUGIN_VELOCITY_PANEL_HPP_

#include "rviz_common/panel.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <memory>

namespace rviz_plugins
{

class VelocityPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit VelocityPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void onVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  void onVelocityCommand(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_report_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_velocity_command_;

  QLabel * current_vehicle_speed_display_{nullptr};
  QLabel * max_vehicle_speed_display_{nullptr};
  QLabel * current_command_speed_display_{nullptr};
  QLabel * max_command_speed_display_{nullptr};

  double vehicle_speed_max_{0.0};
  double command_speed_max_{0.0};

  static void updateLabel(QLabel * label, QString text, QString style_sheet = nullptr)
  {
    label->setText(text);
    if (style_sheet != nullptr){
      label->setStyleSheet(style_sheet);
    }
  }

  static void setLabelFormat(QLabel * label){
    label->setTextInteractionFlags(Qt::TextInteractionFlag::TextSelectableByMouse);
    label->setFont(QFont("Arial", 14, QFont::Bold));
  }
};

}  // namespace rviz_plugins

#endif  // RVIZ2_CUSTOM_PLUGIN_VELOCITY_PANEL_HPP_
