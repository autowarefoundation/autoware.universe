// Copyright 2023 TIER IV, Inc.
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

// Copyright 2024 AutoCore, Inc.
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

#ifndef BYTETRACK3D__BYTETRACK3D_NODE_HPP_
#define BYTETRACK3D__BYTETRACK3D_NODE_HPP_

#include <bytetrack3D/bytetrack3D.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/dynamic_object_array.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bytetrack3D
{
using LabelMap = std::map<int, std::string>;

class ByteTrack3DNode : public rclcpp::Node
{
public:
  explicit ByteTrack3DNode(const rclcpp::NodeOptions & node_options);

private:
  void on_rect(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);

  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr objects_pub_;

  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    detection_rect_sub_;

  std::unique_ptr<bytetrack3D::ByteTrack3D> bytetrack3D_;
};

}  // namespace bytetrack3D

#endif  // BYTETRACK3D__BYTETRACK3D_NODE_HPP_
