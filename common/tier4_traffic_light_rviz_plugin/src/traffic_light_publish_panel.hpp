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

#ifndef TRAFFIC_LIGHT_PUBLISH_PANEL_HPP_
#define TRAFFIC_LIGHT_PUBLISH_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QSpinBox>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#endif

#include <set>

namespace rviz_plugins
{

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_perception_msgs::msg::TrafficSignal;
using autoware_perception_msgs::msg::TrafficSignalArray;
using autoware_perception_msgs::msg::TrafficSignalElement;
class TrafficLightPublishPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TrafficLightPublishPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onRateChanged(int new_rate);
  void onSetTrafficLightState();
  void onResetTrafficLightState();
  void onPublishTrafficLightState();

protected:
  void onTimer();
  void createWallTimer();
  void onVectorMap(const HADMapBin::ConstSharedPtr msg);
  void onTrafficLightSignal(const TrafficSignalArray::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_traffic_signals_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_light_marker_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_signals_;

  QSpinBox * publishing_rate_input_;
  QComboBox * traffic_light_id_input_;
  QDoubleSpinBox * traffic_light_confidence_input_;
  QComboBox * light_color_combo_;
  QComboBox * light_shape_combo_;
  QComboBox * light_status_combo_;
  QPushButton * set_button_;
  QPushButton * reset_button_;
  QPushButton * publish_button_;
  QTableWidget * traffic_table_;

  TrafficSignalArray extra_traffic_signals_;

  bool enable_publish_{false};
  std::set<lanelet::Id> traffic_light_ids_;
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};
  std::map<lanelet::Id, lanelet::AutowareTrafficLightConstPtr> tl_reg_elems_map_;
};

}  // namespace rviz_plugins

#endif  // TRAFFIC_LIGHT_PUBLISH_PANEL_HPP_
