// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "traffic_light_selector_node.hpp"

#include "sensor_msgs/msg/region_of_interest.hpp"

#include <memory>
#include <string>
#include <vector>

namespace
{
bool isInsideRoughRoi(
  const sensor_msgs::msg::RegionOfInterest & detected_roi,
  const sensor_msgs::msg::RegionOfInterest & rough_roi)
{
  return detected_roi.x_offset >= rough_roi.x_offset &&
         detected_roi.y_offset >= rough_roi.y_offset &&
         detected_roi.x_offset + detected_roi.width <= rough_roi.x_offset + rough_roi.width &&
         detected_roi.y_offset + detected_roi.height <= rough_roi.y_offset + rough_roi.height;
}

double getGenIoU(
  const sensor_msgs::msg::RegionOfInterest & roi1, const sensor_msgs::msg::RegionOfInterest & roi2)
{
  int rect1_x_min = static_cast<int>(roi1.x_offset);
  int rect1_x_max = static_cast<int>(roi1.x_offset + roi1.width);
  int rect1_y_min = static_cast<int>(roi1.y_offset);
  int rect1_y_max = static_cast<int>(roi1.y_offset + roi1.height);
  int rect2_x_min = static_cast<int>(roi2.x_offset);
  int rect2_x_max = static_cast<int>(roi2.x_offset + roi2.width);
  int rect2_y_min = static_cast<int>(roi2.y_offset);
  int rect2_y_max = static_cast<int>(roi2.y_offset + roi2.height);
  int rect1_area = roi1.width * roi1.height;
  int rect2_area = roi2.width * roi2.height;
  int x_min = std::max(rect1_x_min, rect2_x_min);
  int y_min = std::max(rect1_y_min, rect2_y_min);
  int x_max = std::min(rect1_x_max, rect2_x_max);
  int y_max = std::min(rect1_y_max, rect2_y_max);

  auto w = std::max(0, x_max - x_min);
  auto h = std::max(0, y_max - y_min);
  auto intersect = w * h;

  auto union_area = rect1_area + rect2_area - intersect;

  double iou = static_cast<double>(intersect) / static_cast<double>(union_area);

  // convex shape area

  auto con_x_min = std::min(rect1_x_min, rect2_x_min);
  auto con_y_min = std::min(rect1_y_min, rect2_y_min);
  auto con_x_max = std::max(rect1_x_max, rect2_x_max);
  auto con_y_max = std::max(rect1_y_max, rect2_y_max);

  auto con_area = (con_x_max - con_x_min + 1) * (con_y_max - con_y_min + 1);

  // GIoU calc
  double giou = iou - static_cast<double>(con_area - union_area) / static_cast<double>(con_area);

  return giou;
}
}  // namespace

namespace autoware::traffic_light
{

TrafficLightSelectorNode::TrafficLightSelectorNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_selector_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  detected_rois_sub_(this, "input/detected_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  rough_rois_sub_(this, "input/rough_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  expected_rois_sub_(this, "input/expect_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), detected_rois_sub_, rough_rois_sub_, expected_rois_sub_)
{
  debug_ = declare_parameter<bool>("debug");
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  sync_.registerCallback(std::bind(&TrafficLightSelectorNode::objectsCallback, this, _1, _2, _3));

  // Publisher
  pub_traffic_light_rois_ =
    create_publisher<TrafficLightRoiArray>("output/traffic_light_rois", rclcpp::QoS{1});
}

void TrafficLightSelectorNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & detected_traffic_light_msg,
  const TrafficLightRoiArray::ConstSharedPtr & rough_rois_msg,
  const TrafficLightRoiArray::ConstSharedPtr & expected_rois_msg)
{
  TrafficLightRoiArray output;

  // create map for traffic_roi and traffic_light_id
  std::map<int, sensor_msgs::msg::RegionOfInterest> rough_roi_map;
  for (const auto & rough_roi : rough_rois_msg->rois) {
    rough_roi_map[rough_roi.traffic_light_id] = rough_roi.roi;
  }
  std::map<int, sensor_msgs::msg::RegionOfInterest> expected_roi_map;
  for (const auto & expected_roi : expected_rois_msg->rois) {
    expected_roi_map[expected_roi.traffic_light_id] = expected_roi.roi;
  }

  // declare image to selected_roi and publish

  output.header = detected_traffic_light_msg->header;
  for (const auto & rough_roi : rough_rois_msg->rois) {
    // find expect roi
    auto expected_roi = expected_roi_map.find(rough_roi.traffic_light_id);
    double max_gen_iou = -1.0;
    TrafficLightRoi max_gen_iou_roi;
    max_gen_iou_roi.traffic_light_id = rough_roi.traffic_light_id;

    for (const auto & detected_light : detected_traffic_light_msg->feature_objects) {
      const auto detected_roi = detected_light.feature.roi;
      const auto detected_class = detected_light.object.classification.front().label;
      if (detected_class == 0) {
        continue;
      }
      if (!isInsideRoughRoi(detected_roi, rough_roi.roi)) {
        continue;
      }
      double gen_iou = getGenIoU(detected_roi, expected_roi->second);
      RCLCPP_INFO(get_logger(), "gen_iou: %f", gen_iou);
      if (gen_iou > max_gen_iou) {
        max_gen_iou = gen_iou;
        max_gen_iou_roi.roi = detected_roi;
        max_gen_iou_roi.traffic_light_type = detected_class - 1;
      }
    }
    if (max_gen_iou > -1.0) {
      output.rois.push_back(max_gen_iou_roi);
    }
  }

  pub_traffic_light_rois_->publish(output);
  if (debug_) {
  }
  return;
}
}  // namespace autoware::traffic_light

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightSelectorNode)
