#include "initialpose_panel.hpp"

#include <QPushButton>
#include <QtWidgets>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include <memory>
#include <string>
#include <vector>

namespace initialpose_plugins
{

InitialPosePanel::InitialPosePanel(QWidget * parent) : Panel(parent)
{
  toggle_button_ = new QPushButton;
  toggle_button_->setText("PUSH");
  QObject::connect(toggle_button_, SIGNAL(clicked()), this, SLOT(toggle()));

  // Add button
  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(toggle_button_);
  main_layout->setContentsMargins(10, 10, 10, 10);

  setLayout(main_layout);
}

InitialPosePanel::~InitialPosePanel() {}

void InitialPosePanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void InitialPosePanel::save(rviz_common::Config config) const { Panel::save(config); }

void InitialPosePanel::load(const rviz_common::Config & config) { Panel::load(config); }

void InitialPosePanel::toggle()
{
  rclcpp::Logger logger = rclcpp::get_logger("rviz2");
  RCLCPP_INFO_STREAM(logger, "hoge");
}

}  // namespace initialpose_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(initialpose_plugins::InitialPosePanel, rviz_common::Panel)
