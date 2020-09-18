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

#include "shape_estimation/map_corrector_node.hpp"
#include "no_map_corrector.hpp"
#include "vehicle_map_corrector.hpp"

MapCorrectorNode::MapCorrectorNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  while (!vector_map_.load()) {
    ROS_ERROR("failed to load vector map");
  }
  ROS_INFO("succeeded to load vector map");
  pnh_.param<bool>("map_corrector/use_rad_filter", use_rad_filter_, true);
  pnh_.param<double>("map_corrector/rad_threshold", rad_threshold_, M_PI_2 * 0.7);
}

bool MapCorrectorNode::correct(autoware_perception_msgs::DynamicObjectWithFeatureArray & input)
{
  geometry_msgs::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_.lookupTransform(
      "map", input.header.frame_id, input.header.stamp, ros::Duration(1.0));
  } catch (tf2::TransformException & exception) {
    ROS_WARN("failed to lookup transform: %s", exception.what());
    return false;
  }
  autoware_perception_msgs::DynamicObjectWithFeatureArray output;
  output.header = input.header;

  for (const auto & input_feature_object : input.feature_objects) {
    autoware_perception_msgs::DynamicObjectWithFeature feature_object = input_feature_object;
    if (feature_object.object.state.orientation_reliable) {
      output.feature_objects.push_back(feature_object);
      continue;
    }
    std::unique_ptr<MapCorrectorInterface> corrector_ptr;
    if (
      feature_object.object.semantic.type == autoware_perception_msgs::Semantic::CAR ||
      feature_object.object.semantic.type == autoware_perception_msgs::Semantic::TRUCK ||
      feature_object.object.semantic.type == autoware_perception_msgs::Semantic::BUS) {
      corrector_ptr.reset(new VehicleMapCorrector(use_rad_filter_, rad_threshold_));
    } else {
      corrector_ptr.reset(new NoMapCorrector);
    }
    bool orientation = feature_object.object.state.orientation_reliable;
    if (corrector_ptr->correct(
          vector_map_, tf_stamped, feature_object.object.shape,
          feature_object.object.state.pose_covariance.pose, orientation)) {
      feature_object.object.state.orientation_reliable = orientation;
      output.feature_objects.push_back(feature_object);
    }
    // if (corrector_ptr->correct(feature_object.object.shape, feature_object.object.state.pose.pose,
    // bool(feature_object.object.state.pose_reliable)))
    //   output.feature_objects.push_back(feature_object);
  }

  input = output;
  return true;
}