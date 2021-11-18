// Copyright 2021 Tier IV, Inc.
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

#include "lgsvl_ground_truth/odom_ground_truth.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

namespace lgsvl_ground_truth {
  
OdomGroundTruth::OdomGroundTruth(const rclcpp::NodeOptions & /*options*/) : Node("odom_ground_truth") {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/lgsvl/gnss_odom", 1, std::bind(&OdomGroundTruth::odomCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

void OdomGroundTruth::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) const {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = odom_msg->pose.pose.position.x;
    tf.transform.translation.y = odom_msg->pose.pose.position.y;
    tf.transform.translation.z = odom_msg->pose.pose.position.z;
    tf.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    tf.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    tf.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    tf.transform.rotation.w = odom_msg->pose.pose.orientation.w; 
    tf_broadcaster_->sendTransform(tf);
}

}  // namespace lgsvl_ground_truth

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(lgsvl_ground_truth::OdomGroundTruth)
