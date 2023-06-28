#include "velocity_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <Qt>

namespace rviz_plugins
{

VelocityPanel::VelocityPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Set up the layout
  QVBoxLayout* vlayout = new QVBoxLayout();
  QGridLayout* glayout = new QGridLayout();

  // Create legends
  glayout->addWidget(new QLabel("[km/h]"), 0, 0, Qt::AlignCenter);
  glayout->addWidget(new QLabel("Indicated\nSpeed"), 0, 1, Qt::AlignCenter);
  glayout->addWidget(new QLabel("Actual\nSpeed"), 0, 2, Qt::AlignCenter);

  // Create the value displays
  current_command_speed_display_ = new QLabel("-");
  setLabelFormat(current_command_speed_display_);
  current_vehicle_speed_display_ = new QLabel("-");
  setLabelFormat(current_vehicle_speed_display_);
  glayout->addWidget(new QLabel("Current: "), 1, 0, Qt::AlignRight);
  glayout->addWidget(current_command_speed_display_, 1, 1, Qt::AlignCenter);
  glayout->addWidget(current_vehicle_speed_display_, 1, 2, Qt::AlignCenter);

  max_command_speed_display_ = new QLabel("-");
  setLabelFormat(max_command_speed_display_);
  max_vehicle_speed_display_ = new QLabel("-");
  setLabelFormat(max_vehicle_speed_display_);
  glayout->addWidget(new QLabel("Max: "), 2, 0, Qt::AlignRight);
  glayout->addWidget(max_command_speed_display_, 2, 1, Qt::AlignCenter);
  glayout->addWidget(max_vehicle_speed_display_, 2, 2, Qt::AlignCenter);

  // Create the button display
  QPushButton* reset_button = new QPushButton("Reset");
  connect(reset_button, &QPushButton::clicked, this, [this]() {
    vehicle_speed_max_ = 0.0;
    command_speed_max_ = 0.0;
    updateLabel(max_command_speed_display_, QString::number(command_speed_max_, 'f', 2));
    updateLabel(max_vehicle_speed_display_, QString::number(vehicle_speed_max_, 'f', 2));
  });
  glayout->addWidget(reset_button, 3, 2);

  vlayout->addLayout(glayout);
  this->setLayout(vlayout);
}

void VelocityPanel::onInitialize()
{
  using std::placeholders::_1;

  node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  sub_velocity_report_ = node_->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1, std::bind(&VelocityPanel::onVelocityReport, this, _1));
  sub_velocity_command_ = node_->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&VelocityPanel::onVelocityCommand, this, _1));
}

void VelocityPanel::onVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
{
  // Process the velocity message and update the vehicle speed
  // 車両CANからの実車速
  const auto vehicle_speed_kmph = msg->longitudinal_velocity * 3.6;
  updateLabel(current_vehicle_speed_display_, QString::number(vehicle_speed_kmph, 'f', 2));

  if (vehicle_speed_max_ < vehicle_speed_kmph) {
    vehicle_speed_max_ = vehicle_speed_kmph;
    updateLabel(max_vehicle_speed_display_, QString::number(vehicle_speed_max_, 'f', 2));
  }
}

void VelocityPanel::onVelocityCommand(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  // Process the command message and update the actual speed
  // Autowareからの制御速度
  const auto command_speed_kmph = std::fabs(msg->longitudinal.speed) * 3.6;
  updateLabel(current_command_speed_display_, QString::number(command_speed_kmph, 'f', 2));

  if (command_speed_max_ < command_speed_kmph) {
    command_speed_max_ = command_speed_kmph;
    updateLabel(max_command_speed_display_, QString::number(command_speed_max_, 'f', 2));
  }
}

}  // namespace rviz_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rviz_plugins::VelocityPanel, rviz_common::Panel)
