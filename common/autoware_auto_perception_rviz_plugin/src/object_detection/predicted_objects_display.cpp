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
PredictedObjectsDisplay::PredictedObjectsDisplay()
: ObjectPolygonDisplayBase("predictions", "/perception/obstacle_segmentation/pointcloud")
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

void PredictedObjectsDisplay::onInitialize()
{
  ObjectPolygonDisplayBase::onInitialize();
  // get access to rviz node to sub and to pub to topics
  rclcpp::Node::SharedPtr raw_node = this->context_->getRosNodeAbstraction().lock()->get_raw_node();

  sync_ptr = std::make_shared<Sync>(
    SyncPolicy(10), perception_objects_subscription, pointcloud_subscription);
  sync_ptr->registerCallback(&PredictedObjectsDisplay::onObjectsAndObstaclePointCloud, this);

  perception_objects_subscription.subscribe(
    raw_node, "/perception/object_recognition/objects", rclcpp::QoS{1}.get_rmw_qos_profile());
  pointcloud_subscription.subscribe(
    raw_node, m_default_pointcloud_topic->getTopic().toStdString(),
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
}

bool PredictedObjectsDisplay::transformObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::PredictedObjects & output_msg)
{
  output_msg = input_msg;

  // transform to world coordinate
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      tf2::fromMsg(object.kinematics.initial_pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, object.kinematics.initial_pose_with_covariance.pose);
    }
  }
  return true;
}

void PredictedObjectsDisplay::onObjectsAndObstaclePointCloud(
  const PredictedObjects::ConstSharedPtr & input_objs_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_pointcloud_msg)
{
  if (!m_publish_objs_pointcloud->getBool()) {
    return;
  }
  // Transform to pointcloud frame
  autoware_auto_perception_msgs::msg::PredictedObjects transformed_objects;
  if (!transformObjects(
        *input_objs_msg, input_pointcloud_msg->header.frame_id, *tf_buffer, transformed_objects)) {
    return;
  }

  objs_buffer.clear();
  for (const auto & object : transformed_objects.objects) {
    std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> labels =
      object.classification;
    object_info info = {
      object.shape, object.kinematics.initial_pose_with_covariance.pose, object.classification};
    objs_buffer.push_back(info);
  }

  // convert to pcl pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(transformed_pointcloud, *temp_cloud);
  pcl::fromROSMsg(*input_pointcloud_msg, *temp_cloud);

  // Create a new point cloud with RGB color information and copy data from input cloud
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
    return;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr output_pointcloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*out_cloud, *output_pointcloud_msg_ptr);

  output_pointcloud_msg_ptr->header = input_pointcloud_msg->header;

  add_pointcloud(output_pointcloud_msg_ptr);
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::PredictedObjectsDisplay, rviz_common::Display)
