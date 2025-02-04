//
//  Copyright 2022 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "traffic_light_publish_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rviz_common/display_context.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#undef signals
namespace rviz_plugins
{
TrafficLightPublishPanel::TrafficLightPublishPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Publish Rate
  publishing_rate_input_ = new QSpinBox();
  publishing_rate_input_->setRange(1, 100);
  publishing_rate_input_->setSingleStep(1);
  publishing_rate_input_->setValue(10);
  publishing_rate_input_->setSuffix("Hz");

  // Traffic Light ID
  traffic_light_id_input_ = new QComboBox();  // init items in first onVectorMap
  traffic_light_id_input_->view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Traffic Light Confidence
  traffic_light_confidence_input_ = new QDoubleSpinBox();
  traffic_light_confidence_input_->setRange(0.0, 1.0);
  traffic_light_confidence_input_->setSingleStep(0.1);
  traffic_light_confidence_input_->setValue(1.0);

  // Traffic Light Color
  light_color_combo_ = new QComboBox();
  light_color_combo_->addItems({"RED", "AMBER", "GREEN", "WHITE", "UNKNOWN"});

  // Traffic Light Shape
  light_shape_combo_ = new QComboBox();
  light_shape_combo_->addItems(
    {"CIRCLE", "LEFT_ARROW", "RIGHT_ARROW", "UP_ARROW", "DOWN_ARROW", "DOWN_LEFT_ARROW",
     "DOWN_RIGHT_ARROW", "CROSS", "UNKNOWN"});

  // Traffic Light Status
  light_status_combo_ = new QComboBox();
  light_status_combo_->addItems({"SOLID_ON", "SOLID_OFF", "FLASHING", "UNKNOWN"});

  // Set Traffic Signals Button
  set_button_ = new QPushButton("SET");

  // Reset Traffic Signals Button
  reset_button_ = new QPushButton("RESET");

  // Publish Traffic Signals Button
  publish_button_ = new QPushButton("PUBLISH");

  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);

  traffic_table_ = new QTableWidget();
  traffic_table_->setColumnCount(5);
  traffic_table_->setHorizontalHeaderLabels({"ID", "Color", "Shape", "Status", "Confidence"});
  traffic_table_->setVerticalHeader(vertical_header);
  traffic_table_->setHorizontalHeader(horizontal_header);

  connect(publishing_rate_input_, SIGNAL(valueChanged(int)), this, SLOT(onRateChanged(int)));
  connect(set_button_, SIGNAL(clicked()), SLOT(onSetTrafficLightState()));
  connect(reset_button_, SIGNAL(clicked()), SLOT(onResetTrafficLightState()));
  connect(publish_button_, SIGNAL(clicked()), SLOT(onPublishTrafficLightState()));

  auto * h_layout_1 = new QHBoxLayout;
  h_layout_1->addWidget(new QLabel("Rate: "));
  h_layout_1->addWidget(publishing_rate_input_);
  h_layout_1->addWidget(new QLabel("ID: "));
  h_layout_1->addWidget(traffic_light_id_input_);
  h_layout_1->addWidget(new QLabel("Confidence: "));
  h_layout_1->addWidget(traffic_light_confidence_input_);

  auto * h_layout_2 = new QHBoxLayout;
  h_layout_2->addWidget(new QLabel("Traffic Light Color: "), 40);
  h_layout_2->addWidget(light_color_combo_, 60);

  auto * h_layout_3 = new QHBoxLayout;
  h_layout_3->addWidget(new QLabel("Traffic Light Shape: "), 40);
  h_layout_3->addWidget(light_shape_combo_, 60);

  auto * h_layout_4 = new QHBoxLayout;
  h_layout_4->addWidget(new QLabel("Traffic Light Status: "), 40);
  h_layout_4->addWidget(light_status_combo_, 60);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(h_layout_1);
  v_layout->addLayout(h_layout_2);
  v_layout->addLayout(h_layout_3);
  v_layout->addLayout(h_layout_4);
  v_layout->addWidget(set_button_);
  v_layout->addWidget(reset_button_);
  v_layout->addWidget(publish_button_);

  auto * h_layout_5 = new QHBoxLayout;
  h_layout_5->addLayout(v_layout);
  h_layout_5->addWidget(traffic_table_);

  setLayout(h_layout_5);
}

void TrafficLightPublishPanel::onSetTrafficLightState()
{
  const auto traffic_light_id_str = traffic_light_id_input_->currentText();
  const auto traffic_light_id = std::stoi(traffic_light_id_str.toStdString());
  const auto color = light_color_combo_->currentText();
  const auto shape = light_shape_combo_->currentText();
  const auto status = light_status_combo_->currentText();

  TrafficLightElement traffic_light;
  traffic_light.confidence = traffic_light_confidence_input_->value();

  if (color == "RED") {
    traffic_light.color = TrafficLightElement::RED;
  } else if (color == "AMBER") {
    traffic_light.color = TrafficLightElement::AMBER;
  } else if (color == "GREEN") {
    traffic_light.color = TrafficLightElement::GREEN;
  } else if (color == "WHITE") {
    traffic_light.color = TrafficLightElement::WHITE;
  } else if (color == "UNKNOWN") {
    traffic_light.color = TrafficLightElement::UNKNOWN;
  }

  if (shape == "CIRCLE") {
    traffic_light.shape = TrafficLightElement::CIRCLE;
  } else if (shape == "LEFT_ARROW") {
    traffic_light.shape = TrafficLightElement::LEFT_ARROW;
  } else if (shape == "RIGHT_ARROW") {
    traffic_light.shape = TrafficLightElement::RIGHT_ARROW;
  } else if (shape == "UP_ARROW") {
    traffic_light.shape = TrafficLightElement::UP_ARROW;
  } else if (shape == "DOWN_ARROW") {
    traffic_light.shape = TrafficLightElement::DOWN_ARROW;
  } else if (shape == "DOWN_LEFT_ARROW") {
    traffic_light.shape = TrafficLightElement::DOWN_LEFT_ARROW;
  } else if (shape == "DOWN_RIGHT_ARROW") {
    traffic_light.shape = TrafficLightElement::DOWN_RIGHT_ARROW;
  } else if (shape == "UNKNOWN") {
    traffic_light.shape = TrafficLightElement::UNKNOWN;
  }

  if (status == "SOLID_OFF") {
    traffic_light.status = TrafficLightElement::SOLID_OFF;
  } else if (status == "SOLID_ON") {
    traffic_light.status = TrafficLightElement::SOLID_ON;
  } else if (status == "FLASHING") {
    traffic_light.status = TrafficLightElement::FLASHING;
  } else if (status == "UNKNOWN") {
    traffic_light.status = TrafficLightElement::UNKNOWN;
  }

  TrafficLightGroup traffic_signal;
  traffic_signal.elements.push_back(traffic_light);
  traffic_signal.traffic_light_group_id = traffic_light_id;

  for (auto & signal : extra_traffic_signals_.traffic_light_groups) {
    if (signal.traffic_light_group_id == traffic_light_id) {
      signal = traffic_signal;
      return;
    }
  }

  extra_traffic_signals_.traffic_light_groups.push_back(traffic_signal);
}

void TrafficLightPublishPanel::onResetTrafficLightState()
{
  extra_traffic_signals_.traffic_light_groups.clear();
  enable_publish_ = false;

  publish_button_->setText("PUBLISH");
  publish_button_->setStyleSheet("background-color: #FFFFFF");
}

void TrafficLightPublishPanel::onPublishTrafficLightState()
{
  enable_publish_ = true;

  publish_button_->setText("PUBLISHING...");
  publish_button_->setStyleSheet("background-color: #FFBF00");
}

void TrafficLightPublishPanel::onInitialize()
{
  using std::placeholders::_1;
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_traffic_signals_ = raw_node_->create_publisher<TrafficLightGroupArray>(
    "/perception/traffic_light_recognition/traffic_signals", rclcpp::QoS(1));

  sub_vector_map_ = raw_node_->create_subscription<LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightPublishPanel::onVectorMap, this, _1));
  createWallTimer();

  enable_publish_ = false;
  received_vector_map_ = false;
}

void TrafficLightPublishPanel::onRateChanged(int new_rate)
{
  (void)new_rate;
  pub_timer_->cancel();
  createWallTimer();
}

void TrafficLightPublishPanel::createWallTimer()
{
  // convert rate from Hz to milliseconds
  const auto period =
    std::chrono::milliseconds(static_cast<int64_t>(1e3 / publishing_rate_input_->value()));
  pub_timer_ = raw_node_->create_wall_timer(period, [&]() { onTimer(); });
}

void TrafficLightPublishPanel::onTimer()
{
  if (enable_publish_) {
    extra_traffic_signals_.stamp = rclcpp::Clock().now();
    pub_traffic_signals_->publish(extra_traffic_signals_);
  }

  traffic_table_->setRowCount(extra_traffic_signals_.traffic_light_groups.size());

  if (extra_traffic_signals_.traffic_light_groups.empty()) {
    return;
  }

  for (size_t i = 0; i < extra_traffic_signals_.traffic_light_groups.size(); ++i) {
    const auto & signal = extra_traffic_signals_.traffic_light_groups.at(i);

    if (signal.elements.empty()) {
      continue;
    }

    auto id_label = new QLabel(QString::number(signal.traffic_light_group_id));
    id_label->setAlignment(Qt::AlignCenter);

    auto color_label = new QLabel();
    color_label->setAlignment(Qt::AlignCenter);

    const auto & light = signal.elements.front();
    switch (light.color) {
      case TrafficLightElement::RED:
        color_label->setText("RED");
        color_label->setStyleSheet("background-color: #FF0000;");
        break;
      case TrafficLightElement::AMBER:
        color_label->setText("AMBER");
        color_label->setStyleSheet("background-color: #FFBF00;");
        break;
      case TrafficLightElement::GREEN:
        color_label->setText("GREEN");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficLightElement::WHITE:
        color_label->setText("WHITE");
        color_label->setStyleSheet("background-color: #FFFFFF;");
        break;
      case TrafficLightElement::UNKNOWN:
        color_label->setText("UNKNOWN");
        color_label->setStyleSheet("background-color: #808080;");
        break;
      default:
        break;
    }

    auto shape_label = new QLabel();
    shape_label->setAlignment(Qt::AlignCenter);

    switch (light.shape) {
      case TrafficLightElement::CIRCLE:
        shape_label->setText("CIRCLE");
        break;
      case TrafficLightElement::LEFT_ARROW:
        shape_label->setText("LEFT_ARROW");
        break;
      case TrafficLightElement::RIGHT_ARROW:
        shape_label->setText("RIGHT_ARROW");
        break;
      case TrafficLightElement::UP_ARROW:
        shape_label->setText("UP_ARROW");
        break;
      case TrafficLightElement::DOWN_ARROW:
        shape_label->setText("DOWN_ARROW");
        break;
      case TrafficLightElement::DOWN_LEFT_ARROW:
        shape_label->setText("DOWN_LEFT_ARROW");
        break;
      case TrafficLightElement::DOWN_RIGHT_ARROW:
        shape_label->setText("DOWN_RIGHT_ARROW");
        break;
      case TrafficLightElement::UNKNOWN:
        shape_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto status_label = new QLabel();
    status_label->setAlignment(Qt::AlignCenter);

    switch (light.status) {
      case TrafficLightElement::SOLID_OFF:
        status_label->setText("SOLID_OFF");
        break;
      case TrafficLightElement::SOLID_ON:
        status_label->setText("SOLID_ON");
        break;
      case TrafficLightElement::FLASHING:
        status_label->setText("FLASHING");
        break;
      case TrafficLightElement::UNKNOWN:
        status_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto confidence_label = new QLabel(QString::number(light.confidence));
    confidence_label->setAlignment(Qt::AlignCenter);

    traffic_table_->setCellWidget(i, 0, id_label);
    traffic_table_->setCellWidget(i, 1, color_label);
    traffic_table_->setCellWidget(i, 2, shape_label);
    traffic_table_->setCellWidget(i, 3, status_label);
    traffic_table_->setCellWidget(i, 4, confidence_label);
  }
  traffic_table_->update();
}

void TrafficLightPublishPanel::onVectorMap(const LaneletMapBin::ConstSharedPtr msg)
{
  if (received_vector_map_) return;
  // NOTE: examples from autoware_lanelet2_map_visualizer/lanelet2_map_visualization_node.cpp
  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems =
    lanelet::utils::query::trafficLights(all_lanelets);
  std::string info = "Fetching traffic lights :";
  std::string delim = " ";
  for (auto && tl_reg_elem_ptr : tl_reg_elems) {
    auto id = static_cast<int>(tl_reg_elem_ptr->id());
    info += (std::exchange(delim, ", ") + std::to_string(id));
    traffic_light_ids_.insert(id);
  }
  RCLCPP_INFO_STREAM(raw_node_->get_logger(), info);
  received_vector_map_ = true;

  for (auto && traffic_light_id : traffic_light_ids_) {
    traffic_light_id_input_->addItem(QString::fromStdString(std::to_string(traffic_light_id)));
  }
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TrafficLightPublishPanel, rviz_common::Panel)
