/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "naive_path_prediction/node.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

NaivePathPredictionNode::NaivePathPredictionNode(const rclcpp::NodeOptions & node_options)
: Node("naive_path_prediction_node", node_options)
{
  using std::placeholders::_1;
  sub_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "input", 1, std::bind(
      &NaivePathPredictionNode::callback, this,
      _1));
  pub_ = this->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>("objects", 1);
}

void NaivePathPredictionNode::callback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  autoware_perception_msgs::msg::DynamicObjectArray output_msg = *input_msg;

  for (size_t i = 0; i < output_msg.objects.size(); ++i) {
    autoware_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = 1.0;
    const double ep = 0.001;
    for (double dt = 0.0; dt < 3.0 + ep; dt += 0.5) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_stamped;
      pose_cov_stamped.header = output_msg.header;
      rclcpp::Duration delta_t_ = rclcpp::Duration::from_seconds(dt);
      pose_cov_stamped.header.stamp.sec = output_msg.header.stamp.sec + delta_t_.seconds();
      pose_cov_stamped.header.stamp.nanosec = output_msg.header.stamp.nanosec +
        delta_t_.nanoseconds();
      geometry_msgs::msg::Pose object_frame_pose;
      geometry_msgs::msg::Pose world_frame_pose;
      object_frame_pose.position.x =
        output_msg.objects.at(i).state.twist_covariance.twist.linear.x * dt;
      object_frame_pose.position.y =
        output_msg.objects.at(i).state.twist_covariance.twist.linear.y * dt;
      tf2::Quaternion quat;
      quat.setRPY(0.0, 0.0, 0.0);
      object_frame_pose.orientation = tf2::toMsg(quat);
      tf2::Transform tf_object2future;
      tf2::Transform tf_world2object;
      tf2::Transform tf_world2future;

      tf2::fromMsg(output_msg.objects.at(i).state.pose_covariance.pose, tf_world2object);
      tf2::fromMsg(object_frame_pose, tf_object2future);
      tf_world2future = tf_world2object * tf_object2future;
      tf2::toMsg(tf_world2future, world_frame_pose);
      pose_cov_stamped.pose.pose = world_frame_pose;

      predicted_path.path.push_back(pose_cov_stamped);
    }
    output_msg.objects.at(i).state.predicted_paths.push_back(predicted_path);
  }

  // Publish
  pub_->publish(output_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NaivePathPredictionNode)
