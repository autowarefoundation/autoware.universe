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

#include <object_detection/predicted_objects_display.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
PredictedObjectsDisplay::PredictedObjectsDisplay() : ObjectPolygonDisplayBase("tracks", "/perception/obstacle_segmentation/pointcloud")
{
  std::thread worker(&PredictedObjectsDisplay::workerThread, this);
  worker.detach();
}

void PredictedObjectsDisplay::workerThread()
{
  while (true) {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [this] { return this->msg; });

    auto tmp_msg = this->msg;
    this->msg.reset();

    lock.unlock();

    auto tmp_markers = createMarkers(tmp_msg);
    lock.lock();
    markers = tmp_markers;

    consumed = true;
  }
}

std::vector<visualization_msgs::msg::Marker::SharedPtr> PredictedObjectsDisplay::createMarkers(
  PredictedObjects::ConstSharedPtr msg)
{
  update_id_map(msg);

  std::vector<visualization_msgs::msg::Marker::SharedPtr> markers;
  objects_frame_id_ = msg->header.frame_id;
  objs_buffer.clear();

  for (const auto & object : msg->objects) {
    // Get marker for shape
    auto shape_marker = get_shape_marker_ptr(
      object.shape, object.kinematics.initial_pose_with_covariance.pose.position,
      object.kinematics.initial_pose_with_covariance.pose.orientation, object.classification,
      get_line_width());
    if (shape_marker) {
      auto shape_marker_ptr = shape_marker.value();
      shape_marker_ptr->header = msg->header;
      shape_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(shape_marker_ptr);
    }

    // Get marker for label
    auto label_marker = get_label_marker_ptr(
      object.kinematics.initial_pose_with_covariance.pose.position,
      object.kinematics.initial_pose_with_covariance.pose.orientation, object.classification);
    if (label_marker) {
      auto label_marker_ptr = label_marker.value();
      label_marker_ptr->header = msg->header;
      label_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(label_marker_ptr);
    }

    // Get marker for id
    geometry_msgs::msg::Point uuid_vis_position;
    uuid_vis_position.x = object.kinematics.initial_pose_with_covariance.pose.position.x - 0.5;
    uuid_vis_position.y = object.kinematics.initial_pose_with_covariance.pose.position.y;
    uuid_vis_position.z = object.kinematics.initial_pose_with_covariance.pose.position.z - 0.5;

    auto id_marker =
      get_uuid_marker_ptr(object.object_id, uuid_vis_position, object.classification);
    if (id_marker) {
      auto id_marker_ptr = id_marker.value();
      id_marker_ptr->header = msg->header;
      id_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(id_marker_ptr);
    }

    // Get marker for pose with covariance
    auto pose_with_covariance_marker =
      get_pose_with_covariance_marker_ptr(object.kinematics.initial_pose_with_covariance);
    if (pose_with_covariance_marker) {
      auto pose_with_covariance_marker_ptr = pose_with_covariance_marker.value();
      pose_with_covariance_marker_ptr->header = msg->header;
      pose_with_covariance_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(pose_with_covariance_marker_ptr);
    }

    // Get marker for velocity text
    geometry_msgs::msg::Point vel_vis_position;
    vel_vis_position.x = uuid_vis_position.x - 0.5;
    vel_vis_position.y = uuid_vis_position.y;
    vel_vis_position.z = uuid_vis_position.z - 0.5;
    auto velocity_text_marker = get_velocity_text_marker_ptr(
      object.kinematics.initial_twist_with_covariance.twist, vel_vis_position,
      object.classification);
    if (velocity_text_marker) {
      auto velocity_text_marker_ptr = velocity_text_marker.value();
      velocity_text_marker_ptr->header = msg->header;
      velocity_text_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(velocity_text_marker_ptr);
    }

    // Get marker for acceleration text
    geometry_msgs::msg::Point acc_vis_position;
    acc_vis_position.x = uuid_vis_position.x - 1.0;
    acc_vis_position.y = uuid_vis_position.y;
    acc_vis_position.z = uuid_vis_position.z - 1.0;
    auto acceleration_text_marker = get_acceleration_text_marker_ptr(
      object.kinematics.initial_acceleration_with_covariance.accel, acc_vis_position,
      object.classification);
    if (acceleration_text_marker) {
      auto acceleration_text_marker_ptr = acceleration_text_marker.value();
      acceleration_text_marker_ptr->header = msg->header;
      acceleration_text_marker_ptr->id = uuid_to_marker_id(object.object_id);
      add_marker(acceleration_text_marker_ptr);
    }

    // Get marker for twist
    auto twist_marker = get_twist_marker_ptr(
      object.kinematics.initial_pose_with_covariance,
      object.kinematics.initial_twist_with_covariance);
    if (twist_marker) {
      auto twist_marker_ptr = twist_marker.value();
      twist_marker_ptr->header = msg->header;
      twist_marker_ptr->id = uuid_to_marker_id(object.object_id);
      markers.push_back(twist_marker_ptr);
    }

    // Add marker for each candidate path
    int32_t path_count = 0;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      // Get marker for predicted path
      auto predicted_path_marker =
        get_predicted_path_marker_ptr(object.object_id, object.shape, predicted_path);
      if (predicted_path_marker) {
        auto predicted_path_marker_ptr = predicted_path_marker.value();
        predicted_path_marker_ptr->header = msg->header;
        predicted_path_marker_ptr->id =
          uuid_to_marker_id(object.object_id) + path_count * PATH_ID_CONSTANT;
        path_count++;
        markers.push_back(predicted_path_marker_ptr);
      }
    }

    // Add confidence text marker for each candidate path
    path_count = 0;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (predicted_path.path.empty()) {
        continue;
      }
      auto path_confidence_marker =
        get_path_confidence_marker_ptr(object.object_id, predicted_path);
      if (path_confidence_marker) {
        auto path_confidence_marker_ptr = path_confidence_marker.value();
        path_confidence_marker_ptr->header = msg->header;
        path_confidence_marker_ptr->id =
          uuid_to_marker_id(object.object_id) + path_count * PATH_ID_CONSTANT;
        path_count++;
        markers.push_back(path_confidence_marker_ptr);
      }
    }

    // add objects to buffer for pointcloud filtering
    // objs_buffer.push_back(object);
  }

  return markers;
}

void PredictedObjectsDisplay::processMessage(PredictedObjects::ConstSharedPtr msg)
{
  std::unique_lock<std::mutex> lock(mutex);

  this->msg = msg;
  condition.notify_one();
}

void PredictedObjectsDisplay::update(float wall_dt, float ros_dt)
{
  std::unique_lock<std::mutex> lock(mutex);

  if (!markers.empty()) {
    clear_markers();

    for (const auto & marker : markers) {
      add_marker(marker);
    }

    markers.clear();
  }

  lock.unlock();

  ObjectPolygonDisplayBase<autoware_auto_perception_msgs::msg::PredictedObjects>::update(
    wall_dt, ros_dt);
}

void PredictedObjectsDisplay::onInitialize() override
{
    ObjectPolygonDisplayBase::onInitialize();
    // get access to rivz node to sub and to pub to topics 
    rclcpp::Node::SharedPtr raw_node = this->context_->getRosNodeAbstraction().lock()->get_raw_node();
    publisher_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("output/predicted_objects_pointcloud", 10);
    // pointcloud_subscription_ = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   m_default_pointcloud_topic->getTopicStd(), 
    //   rclcpp::SensorDataQoS(), 
    //   std::bind(&PredictedObjectsDisplay::pointCloudCallback, 
    //   this, 
    //   std::placeholders::_1));
}

// void PredictedObjectsDisplay::pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg)
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
//           float trans_x = object.kinematics.initial_pose_with_covariance.pose.position.x;
//           float trans_y = object.kinematics.initial_pose_with_covariance.pose.position.y;
//           float trans_z = object.kinematics.initial_pose_with_covariance.pose.position.z;
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
      
//     }

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
  autoware::rviz_plugins::object_detection::PredictedObjectsDisplay, rviz_common::Display)
