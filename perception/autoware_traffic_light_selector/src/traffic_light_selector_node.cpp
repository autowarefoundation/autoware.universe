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

#include "object_recognition_utils/object_recognition_utils.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

#include <memory>
#include <string>
#include <vector>

namespace
{
  bool isInsideRoughRoi(const sensor_msgs::msg::RegionOfInterest & detected_roi, const sensor_msgs::msg::RegionOfInterest & rough_roi)
  {
    return detected_roi.x_offset >= rough_roi.x_offset &&
           detected_roi.y_offset >= rough_roi.y_offset &&
           detected_roi.x_offset + detected_roi.width <= rough_roi.x_offset + rough_roi.width &&
           detected_roi.y_offset + detected_roi.height <= rough_roi.y_offset + rough_roi.height;
  }
}

namespace autoware::traffic_light
{

TrafficLightSelectorNode::TrafficLightSelectorNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_selector_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  objects0_sub_(this, "input/detected_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  objects1_sub_(this, "input/rough_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), objects0_sub_, objects1_sub_)
{
  debug_ = declare_parameter<bool>("debug");
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(std::bind(&TrafficLightSelectorNode::objectsCallback, this, _1, _2));

  // Publisher
  pub_traffic_light_rois_ = create_publisher<TrafficLightRoiArray>("output/traffic_light_rois", rclcpp::QoS{1});

}

void TrafficLightSelectorNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & detected_traffic_light_msg,
  const TrafficLightRoiArray::ConstSharedPtr & rough_rois_msg)
{ 
  TrafficLightRoiArray output;
  output.header = detected_traffic_light_msg->header;
  for (const auto & detected_light : detected_traffic_light_msg->feature_objects)
  {
    const auto detected_roi = detected_light.feature.roi;
    for (const auto & rough_roi : rough_rois_msg->rois)
    {
      if (isInsideRoughRoi(detected_roi, rough_roi.roi))
      {
        TrafficLightRoi selected_roi;
        selected_roi.roi = detected_roi;
        selected_roi.traffic_light_id = rough_roi.traffic_light_id;
        output.rois.push_back(selected_roi);
      }
    }
  }
  if (debug_){
    pub_traffic_light_rois_->publish(output);
  }
  return;

}
}  // namespace autoware::traffic_light_selector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightSelectorNode)
