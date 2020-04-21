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
#include "geometry_msgs/PoseWithCovarianceStamped.h"

NaivePathPredictionNode::NaivePathPredictionNode() : nh_(""), pnh_("~")
{
  sub_ = nh_.subscribe("input", 1, &NaivePathPredictionNode::callback, this);
  pub_ = nh_.advertise<autoware_perception_msgs::DynamicObjectArray>("output", 1, true);
}

void NaivePathPredictionNode::callback(
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & input_msg)
{
  autoware_perception_msgs::DynamicObjectArray output_msg = *input_msg;

  for (size_t i = 0; i < output_msg.objects.size(); ++i) {
    autoware_perception_msgs::PredictedPath predicted_path;
    predicted_path.confidence = 1.0;
    const double ep = 0.001;
    for (double dt = 0.0; dt < 3.0 + ep; dt += 0.5) {
      geometry_msgs::PoseWithCovarianceStamped pose_cov_stamped;
      pose_cov_stamped.header = output_msg.header;
      pose_cov_stamped.header.stamp = output_msg.header.stamp + ros::Duration(dt);
      geometry_msgs::Pose object_frame_pose;
      geometry_msgs::Pose world_frame_pose;
      object_frame_pose.position.x =
        output_msg.objects.at(i).state.twist_covariance.twist.linear.x * dt;
      object_frame_pose.position.y =
        output_msg.objects.at(i).state.twist_covariance.twist.linear.y * dt;
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
  pub_.publish(output_msg);
  return;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "naive_path_prediction_node");
  NaivePathPredictionNode node;
  ros::spin();

  return 0;
}
