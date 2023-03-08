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

#include <object_detection/tracked_objects_display.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
TrackedObjectsDisplay::TrackedObjectsDisplay() : ObjectPolygonDisplayBase("tracks", "/perception/obstacle_segmentation/pointcloud") {}

void TrackedObjectsDisplay::processMessage(TrackedObjects::ConstSharedPtr msg)
{
  clear_markers();
  update_id_map(msg);

  for (const auto & object : msg->objects) {
    // Get marker for shape
    auto shape_marker = get_shape_marker_ptr(
      object.shape, object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification,
      get_line_width());
    if (shape_marker) {
      auto shape_marker_ptr = shape_marker.value();
      shape_marker_ptr->header = msg->header;
      shape_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(shape_marker_ptr);
    }

    // Get marker for label
    auto label_marker = get_label_marker_ptr(
      object.kinematics.pose_with_covariance.pose.position,
      object.kinematics.pose_with_covariance.pose.orientation, object.classification);
    if (label_marker) {
      auto label_marker_ptr = label_marker.value();
      label_marker_ptr->header = msg->header;
      label_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(label_marker_ptr);
    }

    // Get marker for id
    geometry_msgs::msg::Point uuid_vis_position;
    uuid_vis_position.x = object.kinematics.pose_with_covariance.pose.position.x - 0.5;
    uuid_vis_position.y = object.kinematics.pose_with_covariance.pose.position.y;
    uuid_vis_position.z = object.kinematics.pose_with_covariance.pose.position.z - 0.5;

    auto id_marker =
      get_uuid_marker_ptr(object.object_id, uuid_vis_position, object.classification);
    if (id_marker) {
      auto id_marker_ptr = id_marker.value();
      id_marker_ptr->header = msg->header;
      id_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(id_marker_ptr);
    }

    // Get marker for pose with covariance
    auto pose_with_covariance_marker =
      get_pose_with_covariance_marker_ptr(object.kinematics.pose_with_covariance);
    if (pose_with_covariance_marker) {
      auto pose_with_covariance_marker_ptr = pose_with_covariance_marker.value();
      pose_with_covariance_marker_ptr->header = msg->header;
      pose_with_covariance_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(pose_with_covariance_marker_ptr);
    }

    // Get marker for velocity text
    geometry_msgs::msg::Point vel_vis_position;
    vel_vis_position.x = uuid_vis_position.x - 0.5;
    vel_vis_position.y = uuid_vis_position.y;
    vel_vis_position.z = uuid_vis_position.z - 0.5;
    auto velocity_text_marker = get_velocity_text_marker_ptr(
      object.kinematics.twist_with_covariance.twist, vel_vis_position, object.classification);
    if (velocity_text_marker) {
      auto velocity_text_marker_ptr = velocity_text_marker.value();
      velocity_text_marker_ptr->header = msg->header;
      velocity_text_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(velocity_text_marker_ptr);
    }

    // Get marker for acceleration text
    geometry_msgs::msg::Point acc_vis_position;
    acc_vis_position.x = uuid_vis_position.x - 1.0;
    acc_vis_position.y = uuid_vis_position.y;
    acc_vis_position.z = uuid_vis_position.z - 1.0;
    auto acceleration_text_marker = get_acceleration_text_marker_ptr(
      object.kinematics.acceleration_with_covariance.accel, acc_vis_position,
      object.classification);
    if (acceleration_text_marker) {
      auto acceleration_text_marker_ptr = acceleration_text_marker.value();
      acceleration_text_marker_ptr->header = msg->header;
      acceleration_text_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(acceleration_text_marker_ptr);
    }

    // Get marker for twist
    auto twist_marker = get_twist_marker_ptr(
      object.kinematics.pose_with_covariance, object.kinematics.twist_with_covariance);
    if (twist_marker) {
      auto twist_marker_ptr = twist_marker.value();
      twist_marker_ptr->header = msg->header;
      twist_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(twist_marker_ptr);
    }

    // Transform to pointcloud frame
    autoware_auto_perception_msgs::msg::TrackedObjects transformed_objects;
    if (!transformObjects(
          *msg, pointcloud_frame_id_, tf_buffer_,
          transformed_objects)) {
      // objects_pub_->publish(*input_objects);
      return;
    }
    
    objects_frame_id_ = transformed_objects.header.frame_id;

    objs_buffer.clear();
    for (const auto & object : transformed_objects.objects)
    {
      std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels = object.classification;
      object_info info = {object.shape, object.kinematics.pose_with_covariance.pose, object.classification};
      objs_buffer.push_back(info);
    }
    // RCLCPP_INFO(this->get_logger(), "Update objects buffer");;
  }
}

void TrackedObjectsDisplay::onInitialize() override
{
    ObjectPolygonDisplayBase::onInitialize();
    // get access to rivz node to sub and to pub to topics 
    rclcpp::Node::SharedPtr raw_node = this->context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("output/tracked_objects_pointcloud", 10);
    // pointcloud_subscription_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   m_default_pointcloud_topic->getTopicStd(), 
    //   rclcpp::SensorDataQoS(), 
    //   std::bind(&TrackedObjectsDisplay::pointCloudCallback, 
    //   this, 
    //   std::placeholders::_1));
}

// void TrackedObjectsDisplay::pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg)
// {
//     sensor_msgs::msg::PointCloud2 transformed_pointcloud;
//     if (objects_frame_id_ != "" && input_pointcloud_msg.header.frame_id != objects_frame_id_) 
//     {
//       geometry_msgs::msg::TransformStamped transform;
//       transform = tf_buffer_->lookupTransform(
//       input_pointcloud_msg.header.frame_id, objects_frame_id_, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

//       tf2::doTransform(input_pointcloud_msg, transformed_pointcloud, transform);
      
//     } else {
//       transformed_pointcloud = input_pointcloud_msg;
//     }
    
//     pcl::PCLPointCloud2 input_pcl_cloud;
//     pcl_conversions::toPCL(transformed_pointcloud, input_pcl_cloud);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(input_pcl_cloud, *temp_cloud);
    
//     // Create a new point cloud with RGB color information and copy data from input cloudb
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//     pcl::copyPointCloud(*temp_cloud, *colored_cloud);

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//     if (objs_buffer.size() > 0) {
//       for (auto object : objs_buffer) {

//           if (object.shape.type == 0)
//         {
//           pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
//           pcl::CropBox<pcl::PointXYZRGB> cropBoxFilter (true);
//           cropBoxFilter.setInputCloud(colored_cloud);
//           float trans_x = object.kinematics.pose_with_covariance.pose.position.x;
//           float trans_y = object.kinematics.pose_with_covariance.pose.position.y;
//           float trans_z = object.kinematics.pose_with_covariance.pose.position.z;
//           float max_x = trans_x + object.shape.dimensions.x / 2.0;
//           float min_x = trans_x - object.shape.dimensions.x / 2.0;
//           float max_y = trans_y + object.shape.dimensions.y / 2.0;
//           float min_y = trans_y - object.shape.dimensions.y / 2.0;
//           float max_z = trans_z + object.shape.dimensions.z / 2.0;
//           float min_z = trans_z - object.shape.dimensions.z / 2.0; 

//           Eigen::Vector4f min_pt (min_x, min_y, min_z, 0.0f);
//           Eigen::Vector4f max_pt (max_x, max_y, max_z, 0.0f);
//           cropBoxFilter.setMin(min_pt);
//           cropBoxFilter.setMax(max_pt);
//           cropBoxFilter.filter(filtered_cloud);

//           // Define a custom color for the box polygons
//           const uint8_t r = 30, g = 44, b = 255;

//           for (auto cloud_it = filtered_cloud.begin(); cloud_it != filtered_cloud.end(); ++cloud_it)
//           {
//             cloud_it->r = r;
//             cloud_it->g = g;
//             cloud_it->b = b;
//           }

//           *out_cloud += filtered_cloud;

//         }

//       }
      
//   }

//     sensor_msgs::msg::PointCloud2::SharedPtr output_pointcloud_msg_ptr( new sensor_msgs::msg::PointCloud2);
//     // pcl::toROSMsg(*colored_cloud, *output_pointcloud_msg_ptr);
    
//     pcl::toROSMsg(*out_cloud, *output_pointcloud_msg_ptr);
//     output_pointcloud_msg_ptr->header = input_pointcloud_msg.header;
//     output_pointcloud_msg_ptr->header.frame_id = objects_frame_id_;

//     // TODO(lexavtanke) get pointcloud in frame base link and detected objects in frame map
//     // change pointcloud frame to map 
//     // update color of the points and remove all points outside deceted objects 

    
//     // RCLCPP_INFO(this->get_logger(), "Publishing pointcloud");
//     publisher_->publish(*output_pointcloud_msg_ptr);
// }

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::TrackedObjectsDisplay, rviz_common::Display)
