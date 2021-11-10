// Copyright 2018 Autoware Foundation. All rights reserved.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "shape_estimation/shape_estimator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

class ShapeEstimationNode : public rclcpp::Node
{
private:
  // ros
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr sub_;
  // bool use_map_correct_;

  void callback(
    const autoware_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_msg);

private:
  std::unique_ptr<ShapeEstimator> estimator_;
  bool use_vehicle_reference_yaw_;

public:
  explicit ShapeEstimationNode(const rclcpp::NodeOptions & node_options);
};

#endif  // NODE_HPP_
