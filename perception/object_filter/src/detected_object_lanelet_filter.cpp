// Copyright 2020 Tier IV, Inc.
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

#include "vector_map_filter/detected_object_filter.hpp"


#include <pcl_ros/transforms.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace detected_object_filter
{
DetectedObjectLaneletFilterNode::DetectedObjectLaneletFilterNode(const rclcpp::NodeOptions & node_options)
: Node("detected_object_lanelet_filter_node", node_options)
{
  using std::placeholders::_1;

  // Set parameters
  voxel_size_x_ = declare_parameter("voxel_size_x", 0.04);
  voxel_size_y_ = declare_parameter("voxel_size_y", 0.04);

  // Set publisher/subscriber
  map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(), std::bind(&DetectedObjectLaneletFilterNode::mapCallback, this, _1));
  object_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&DetectedObjectLaneletFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  // Set tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void DetectedObjectLaneletFilterNode::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
}


void DetectedObjectLaneletFilterNode::objectCallback(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  std::cout << "DetectedObjectLaneletFilterNode::objectCallback "  << std::endl;
  // Guard
  if (object_pub_->get_subscription_count() < 1)
      return;
  }


  autoware_auto_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  for (const auto & object : input_msg->objects) {
    const auto & position = object.kinematics.pose_with_covariance.pose.position;
    const auto object_sq_dist = position.x * position.x + position.y * position.y;
    if (position.x > 0 && position.y > -5 && position.y < 5 ) {
      output_object_msg.objects.push_back(object);
    }
  }

  // publish output msg
  object_pub_->publish(output_object_msg);

}


bool DetectedObjectLaneletFilterNode::transformPointCloud(
    const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
    PointCloud2 * out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    return false;
  }
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

}  // namespace detected_object_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(detected_object_filter::DetectedObjectLaneletFilterNode)
