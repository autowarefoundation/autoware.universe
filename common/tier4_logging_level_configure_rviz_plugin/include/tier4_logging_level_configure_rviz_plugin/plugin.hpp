#ifndef PLUGIN_HPP_
#define PLUGIN_HPP_

#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QButtonGroup>
#include <QMap>
#include <QLabel>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "logging_demo/srv/config_logger.hpp"

namespace rviz_plugin {

class LoggingLevelConfigureRvizPlugin : public rviz_common::Panel
{
  Q_OBJECT // This macro is needed for Qt to handle slots and signals

public:
  LoggingLevelConfigureRvizPlugin(QWidget* parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  QMap<QString, QButtonGroup*> buttonGroups_;
  rclcpp::Node::SharedPtr raw_node_;
  
  // logger_node_map_[target_name] = {container_name, logger_name}
  std::map<QString, std::vector<std::pair<QString, QString>>> logger_node_map_;
  
  // client_map_[container_name] = service_client
  std::unordered_map<QString, rclcpp::Client<logging_demo::srv::ConfigLogger>::SharedPtr> client_map_;
  
  // button_map_[target_name][logging_level] = Q_button_pointer
  std::unordered_map<QString, std::unordered_map<QString, QPushButton*>> button_map_;

  QStringList getContainerList();
  int getMaxModuleNameWidth(QLabel* containerLabel);
  void setLoggerNodeMap();
  void attachLoggingComponent();

private Q_SLOTS:
  void onButtonClick(QPushButton *button, const QString & name, const QString & level);
  void updateButtonColors(const QString & target_module_name, QPushButton * active_button);
  void changeLogLevel(const QString& container, const QString& level);
};

} // namespace rviz_plugin

#endif // PLUGIN_HPP_
