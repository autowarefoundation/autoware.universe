#include <QtWidgets>
#include <QPushButton>

#include <memory>
#include <string>
#include <vector>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include "rclcpp/rclcpp.hpp"
#include "player_panel.hpp"

class QPushButton;

namespace rosbag2_rviz_plugins
{

using rosbag2_interfaces::srv::TogglePaused;

PlayerPanel::PlayerPanel(QWidget * parent)
: Panel(parent),
  service_available_timeout_(200),
  service_timeout_(200)
{
  toggle_button_ = new QPushButton;
  toggle_button_->setText("play/pause");
  QObject::connect(toggle_button_, SIGNAL(clicked()), this, SLOT(toggle()));

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->addWidget(toggle_button_);
  main_layout->setContentsMargins(10, 10, 10, 10);
  setLayout(main_layout);
}

PlayerPanel::~PlayerPanel() {}

void PlayerPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  toggle_client_ = node->create_client<TogglePaused>("rosbag2_player/toggle_paused");
  toggle_client_->wait_for_service(service_available_timeout_);
}

void PlayerPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void PlayerPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void PlayerPanel::toggle()
{
  auto future = toggle_client_->async_send_request(std::make_shared<TogglePaused::Request>());
  future.wait_for(service_timeout_);
}

} // namespace rosbag2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_rviz_plugins::PlayerPanel, rviz_common::Panel)
