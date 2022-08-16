#pragma once

#include <QtWidgets>
#include <QBasicTimer>

#include <memory>
#include <string>
#include <vector>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include "rclcpp/rclcpp.hpp"

class QPushButton;

namespace rosbag2_rviz_plugins
{

class PlayerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlayerPanel(QWidget * parent = 0);
  virtual ~PlayerPanel();

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void toggle();

private:
  std::chrono::milliseconds service_available_timeout_;
  std::chrono::milliseconds service_timeout_;

  rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_client_;
  QPushButton * toggle_button_{nullptr};
};

} // namespace rosbag2_rviz_plugins
