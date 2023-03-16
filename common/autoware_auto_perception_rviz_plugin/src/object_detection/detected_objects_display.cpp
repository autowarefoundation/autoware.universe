// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <object_detection/detected_objects_display.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
DetectedObjectsDisplay::DetectedObjectsDisplay()
: ObjectPolygonDisplayBase(
    "/perception/object_recognition/detection/objects",
    "/perception/obstacle_segmentation/pointcloud")
{
}

void DetectedObjectsDisplay::processMessage(DetectedObjects::ConstSharedPtr msg)
{
  clear_markers();
  int id = 0;
  for (const auto & object : msg->objects) {
    // TODO(Satoshi Tanaka): fixing from string label to one string
    // Get marker for shape
    auto shape_marker = get_shape_marker_ptr(
      object.shape, object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification,
      get_line_width());
    if (shape_marker) {
      auto shape_marker_ptr = shape_marker.value();
      shape_marker_ptr->header = msg->header;
      shape_marker_ptr->id = id++;
      add_marker(shape_marker_ptr);
    }

    // Get marker for label
    auto label_marker = get_label_marker_ptr(
      object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification);
    if (label_marker) {
      auto label_marker_ptr = label_marker.value();
      label_marker_ptr->header = msg->header;
      label_marker_ptr->id = id++;
      add_marker(label_marker_ptr);
    }

    // Get marker for velocity text
    geometry_msgs::msg::Point vel_vis_position;
    vel_vis_position.x = object.kinematics.pose_with_covariance.pose.position.x - 0.5;
    vel_vis_position.y = object.kinematics.pose_with_covariance.pose.position.y;
    vel_vis_position.z = object.kinematics.pose_with_covariance.pose.position.z - 0.5;
    auto velocity_text_marker = get_velocity_text_marker_ptr(
      object.kinematics.twist_with_covariance.twist, vel_vis_position, object.classification);
    if (velocity_text_marker) {
      auto velocity_text_marker_ptr = velocity_text_marker.value();
      velocity_text_marker_ptr->header = msg->header;
      velocity_text_marker_ptr->id = id++;
      add_marker(velocity_text_marker_ptr);
    }

    // Get marker for twist
    auto twist_marker = get_twist_marker_ptr(
      object.kinematics.pose_with_covariance, object.kinematics.twist_with_covariance);
    if (twist_marker) {
      auto twist_marker_ptr = twist_marker.value();
      twist_marker_ptr->header = msg->header;
      twist_marker_ptr->id = id++;
      add_marker(twist_marker_ptr);
    }
  }
}

void DetectedObjectsDisplay::onInitialize()
{
  ObjectPolygonDisplayBase::onInitialize();
  // get access to rivz node to sub and to pub to topics
  rclcpp::Node::SharedPtr raw_node = this->context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/detected_objects_pointcloud", rclcpp::SensorDataQoS());

  sync_ptr =
    std::make_shared<Sync>(SyncPolicy(10), percepted_objects_subscription, pointcloud_subscription);
  sync_ptr->registerCallback(&DetectedObjectsDisplay::onObjectsAndObstaclePointCloud, this);
  percepted_objects_subscription.subscribe(
    raw_node, "/perception/object_recognition/detection/objects",
    rclcpp::QoS{1}.get_rmw_qos_profile()),
  pointcloud_subscription.subscribe(
    raw_node, m_default_pointcloud_topic->getTopic().toStdString(),
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
}

void DetectedObjectsDisplay::onObjectsAndObstaclePointCloud(
  const DetectedObjects::ConstSharedPtr & input_objs_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_pointcloud_msg)
{
  if (!m_publish_objs_pointcloud->getBool()) {
    return;
  }
  // Transform to pointcloud frame
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!transformObjects(
      *input_objs_msg, input_pointcloud_msg->header.frame_id, *tf_buffer, transformed_objects))
  {
    // objects_pub_->publish(*input_objects);
    return;
  }

  objs_buffer.clear();
  for (const auto & object : transformed_objects.objects) {
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels =
      object.classification;
    object_info info = {
      object.shape, object.kinematics.pose_with_covariance.pose, object.classification};
    objs_buffer.push_back(info);
  }

  // convert to pcl pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(transformed_pointcloud, *temp_cloud);
  pcl::fromROSMsg(*input_pointcloud_msg, *temp_cloud);

  // Create a new point cloud with RGB color information and copy data from input cloudb
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*temp_cloud, *colored_cloud);

  // Create Kd-tree to search neighbor pointcloud to reduce cost.
  pcl::search::Search<pcl::PointXYZRGB>::Ptr kdtree =
    pcl::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>(false);
  kdtree->setInputCloud(colored_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  if (objs_buffer.size() > 0) {
    for (auto object : objs_buffer) {
      const auto search_radius = getMaxRadius(object);
      // Search neighbor pointcloud to reduce cost.
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbor_pointcloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
      std::vector<int> indices;
      std::vector<float> distances;
      kdtree->radiusSearch(
        toPCL(object.position.position), search_radius.value(), indices, distances);
      for (const auto & index : indices) {
        neighbor_pointcloud->push_back(colored_cloud->at(index));
      }

      filterPolygon(neighbor_pointcloud, out_cloud, object);
    }
  } else {
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5, "objects buffer is empty");
    return;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr output_pointcloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*out_cloud, *output_pointcloud_msg_ptr);

  output_pointcloud_msg_ptr->header = input_pointcloud_msg->header;

  publisher->publish(*output_pointcloud_msg_ptr);
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsDisplay, rviz_common::Display)
