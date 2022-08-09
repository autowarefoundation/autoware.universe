#pragma once

#include <QBasicTimer>
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include <memory>
#include <string>
#include <vector>

class QPushButton;

namespace initialpose_plugins
{

class InitialPosePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit InitialPosePanel(QWidget * parent = 0);
  virtual ~InitialPosePanel();

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void toggle();

private:
  QPushButton * toggle_button_{nullptr};
};

}  // namespace initialpose_plugins
