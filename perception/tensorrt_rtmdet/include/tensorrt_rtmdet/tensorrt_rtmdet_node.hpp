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

#ifndef TENSORRT_RTMDET__TENSORRT_RTMDET_NODE_HPP
#define TENSORRT_RTMDET__TENSORRT_RTMDET_NODE_HPP

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include "tier4_perception_msgs/msg/detected_objects_with_mask.hpp"
#include "tier4_perception_msgs/msg/detected_object_with_mask.hpp"
#include "tier4_perception_msgs/msg/feature.hpp"
#include "tensorrt_rtmdet/tensorrt_rtmdet.hpp"
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>

namespace tensorrt_rtmdet {
    class TrtRTMDetNode : public rclcpp::Node {
    public:
        explicit TrtRTMDetNode(const rclcpp::NodeOptions &node_options);

    private:
        void onConnect();

        void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

        std::unique_ptr<tensorrt_rtmdet::TrtRTMDet> trt_rtmdet_;

        image_transport::Publisher debug_image_pub_;
        image_transport::Subscriber image_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithMask>::SharedPtr objects_pub_;

        std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
        std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;

        std::vector<float> mean_;
        std::vector<float> std_;
    };
}

#endif // TENSORRT_RTMDET__TENSORRT_RTMDET_NODE_HPP
