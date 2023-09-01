#include "target_object_type_panel.hpp"
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QColor>

TargetObjectTypePanel::TargetObjectTypePanel(QWidget* parent):
    rviz_common::Panel(parent)
{
    node_ = std::make_shared<rclcpp::Node>("matrix_display_node");
    
    matrix_widget_ = new QTableWidget(modules_.size(), targets_.size(), this);
    for(size_t i = 0; i < modules_.size(); i++) {
        matrix_widget_->setVerticalHeaderItem(i, new QTableWidgetItem(QString::fromStdString(modules_[i])));
    }
    for(size_t j = 0; j < targets_.size(); j++) {
        matrix_widget_->setHorizontalHeaderItem(j, new QTableWidgetItem(QString::fromStdString(targets_[j])));
    }

    setParamName();
    updateMatrix();

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(matrix_widget_);
    setLayout(layout);
}

void TargetObjectTypePanel::setParamName() {
// TODO: If the param naming strategy is aligned, this should be done automatically based on the modules_ and targets_.

//avoidance
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
  param_name.ns = "avoidance.target_object";
  param_name.name.emplace("car", "car.is_target");
  param_name.name.emplace("truck", "truck.is_target");
  param_name.name.emplace("bus", "bus.is_target");
  param_name.name.emplace("trailer", "trailer.is_target");
  param_name.name.emplace("unknown", "unknown.is_target");
  param_name.name.emplace("bicycle", "bicycle.is_target");
  param_name.name.emplace("motorcycle", "motorcycle.is_target");
  param_name.name.emplace("pedestrian", "pedestrian.is_target");
  param_names_.emplace("avoidance", param_name);
}


// avoidance_by_lane_change
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
  param_name.ns = "avoidance_by_lane_change.target_object";
  param_name.name.emplace("car", "car.is_target");
  param_name.name.emplace("truck", "truck.is_target");
  param_name.name.emplace("bus", "bus.is_target");
  param_name.name.emplace("trailer", "trailer.is_target");
  param_name.name.emplace("unknown", "unknown.is_target");
  param_name.name.emplace("bicycle", "bicycle.is_target");
  param_name.name.emplace("motorcycle", "motorcycle.is_target");
  param_name.name.emplace("pedestrian", "pedestrian.is_target");
  param_names_.emplace("avoidance_by_lane_change", param_name);
}

// lane_change
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
  param_name.ns = "lane_change.target_object";
  param_name.name.emplace("car", "car");
  param_name.name.emplace("truck", "truck");
  param_name.name.emplace("bus", "bus");
  param_name.name.emplace("trailer", "trailer");
  param_name.name.emplace("unknown", "unknown");
  param_name.name.emplace("bicycle", "bicycle");
  param_name.name.emplace("motorcycle", "motorcycle");
  param_name.name.emplace("pedestrian", "pedestrian");
  param_names_.emplace("lane_change", param_name);
}

// crosswalk
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner";
  param_name.ns = "crosswalk.object_filtering.target_object";
  param_name.name.emplace("car", "car");
  param_name.name.emplace("truck", "truck");
  param_name.name.emplace("bus", "bus");
  param_name.name.emplace("trailer", "trailer");
  param_name.name.emplace("unknown", "unknown");
  param_name.name.emplace("bicycle", "bicycle");
  param_name.name.emplace("motorcycle", "motorcycle");
  param_name.name.emplace("pedestrian", "pedestrian");
  param_names_.emplace("crosswalk", param_name);
}

// obstacle cruise (inside)
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
  param_name.ns = "common.cruise_obstacle_type.inside";
  param_name.name.emplace("car", "car");
  param_name.name.emplace("truck", "truck");
  param_name.name.emplace("bus", "bus");
  param_name.name.emplace("trailer", "trailer");
  param_name.name.emplace("unknown", "unknown");
  param_name.name.emplace("bicycle", "bicycle");
  param_name.name.emplace("motorcycle", "motorcycle");
  param_name.name.emplace("pedestrian", "pedestrian");
  param_names_.emplace("obstacle_cruise (inside)", param_name);
}

// obstacle cruise (outside)
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
  param_name.ns = "common.cruise_obstacle_type.outside";
  param_name.name.emplace("car", "car");
  param_name.name.emplace("truck", "truck");
  param_name.name.emplace("bus", "bus");
  param_name.name.emplace("trailer", "trailer");
  param_name.name.emplace("unknown", "unknown");
  param_name.name.emplace("bicycle", "bicycle");
  param_name.name.emplace("motorcycle", "motorcycle");
  param_name.name.emplace("pedestrian", "pedestrian");
  param_names_.emplace("obstacle_cruise (outside)", param_name);
}


// obstacle stop
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
  param_name.ns = "common.cruise_obstacle_type";
  param_name.name.emplace("car", "car");
  param_name.name.emplace("truck", "truck");
  param_name.name.emplace("bus", "bus");
  param_name.name.emplace("trailer", "trailer");
  param_name.name.emplace("unknown", "unknown");
  param_name.name.emplace("bicycle", "bicycle");
  param_name.name.emplace("motorcycle", "motorcycle");
  param_name.name.emplace("pedestrian", "pedestrian");
  param_names_.emplace("obstacle_stop", param_name);
}

// obstacle slowdown
{
  ParamNameEnableObject param_name;
  param_name.node = "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
  param_name.ns = "common.slow_down_obstacle_type";
  param_name.name.emplace("car", "car");
  param_name.name.emplace("truck", "truck");
  param_name.name.emplace("bus", "bus");
  param_name.name.emplace("trailer", "trailer");
  param_name.name.emplace("unknown", "unknown");
  param_name.name.emplace("bicycle", "bicycle");
  param_name.name.emplace("motorcycle", "motorcycle");
  param_name.name.emplace("pedestrian", "pedestrian");
  param_names_.emplace("obstacle_stop", param_name);
}
}

void TargetObjectTypePanel::updateMatrix() {

  constexpr QColor GreenYellow(173,255,47);
  // constexpr QColor FireBrick(178,34,34);


    for(size_t i = 0; i < modules_.size(); i++) {
        const auto& module = modules_[i];
        
        // Check if module exists in param_names_ 
        if (param_names_.find(module) == param_names_.end()) {
          std::cerr << module << " is not in the param names" << std::endl;
            continue;
        }

        const auto& module_params = param_names_.at(module);
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, module_params.node);
        if (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_ERROR(
            node_->get_logger(), "Failed to find parameter service for node: %s",
            module_params.node.c_str());
            continue;
        }

        for(size_t j = 0; j < targets_.size(); j++) {
            const auto& target = targets_[j];

            // Check if target exists in module's name map
            if (module_params.name.find(target) == module_params.name.end()) {
              std::cerr << target << " is not in the " << target << " param names" << std::endl;
                continue;
            }

            std::string param_name = module_params.ns + "." + module_params.name.at(target);
            auto parameter_result = parameters_client->get_parameters({param_name});


            if (!parameter_result.empty()) {
                bool value = parameter_result[0].as_bool();
                QTableWidgetItem* item = new QTableWidgetItem(value ? "O" : "X");
                item->setForeground(QBrush(Qt::black)); // set the text color to black
                item->setBackground(QBrush(value ? GreenYellow : Qt::lightGray)); // set the background color to either yellow or gray
                matrix_widget_->setItem(i, j, item);
                std::cerr << "Find parameter " << module_params.node << " " << param_name << " = " << value << std::endl;
            }
            else
            {
                std::cerr << "Failed to get parameter " << module_params.node << " " << param_name << std::endl;
                QTableWidgetItem* item = new QTableWidgetItem("n/a");
                item->setForeground(QBrush(Qt::black)); // set the text color to black
                item->setBackground(QBrush(Qt::darkGray)); // set the background color to either yellow or gray
                matrix_widget_->setItem(i, j, item);
            }
        }
    }
}


PLUGINLIB_EXPORT_CLASS(TargetObjectTypePanel, rviz_common::Panel)
