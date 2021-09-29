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

#ifndef DETECTION_BY_TRACKER__DETECTION_BY_TRACKER_CORE_HPP_
#define DETECTION_BY_TRACKER__DETECTION_BY_TRACKER_CORE_HPP_

#include <deque>
#include <memory>
#include <vector>

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_with_feature_array.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "euclidean_cluster/euclidean_cluster.hpp"
#include "euclidean_cluster/utils.hpp"
#include "euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "shape_estimation/shape_estimator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TrackerHandler
{
private:
  std::deque<autoware_perception_msgs::msg::DynamicObjectArray> objects_buffer_;

public:
  TrackerHandler() = default;
  void onDynamicObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_objects_msg);
  bool estimateDynamicObjects(
    const rclcpp::Time & time, autoware_perception_msgs::msg::DynamicObjectArray & output);
};

class DetectionByTracker : public rclcpp::Node
{
public:
  explicit DetectionByTracker(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    objects_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr trackers_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>::SharedPtr
    initial_objects_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  TrackerHandler tracker_handler_;
  std::shared_ptr<ShapeEstimator> shape_estimator_;
  std::shared_ptr<euclidean_cluster::EuclideanClusterInterface> cluster_;

  void onObjects(
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg);

  void divideUnderSegmentedObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray & tracked_objects,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & in_objects,
    autoware_perception_msgs::msg::DynamicObjectArray & out_no_found_tracked_objects,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & out_objects);

  float optimizeUnderSegmentedObject(
    const autoware_perception_msgs::msg::DynamicObject & target_object,
    const sensor_msgs::msg::PointCloud2 & under_segmented_cluster,
    autoware_perception_msgs::msg::DynamicObjectWithFeature & output);

  void mergeOverSegmentedObjects(
    const autoware_perception_msgs::msg::DynamicObjectArray & tracked_objects,
    const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & in_objects,
    autoware_perception_msgs::msg::DynamicObjectArray & out_no_found_tracked_objects,
    autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & out_objects);
};

#endif  // DETECTION_BY_TRACKER__DETECTION_BY_TRACKER_CORE_HPP_
