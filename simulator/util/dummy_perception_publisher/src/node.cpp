/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
 */
#include "dummy_perception_publisher/node.hpp"
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode()
: nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  std::random_device seed_gen;
  random_generator_.seed(seed_gen());
  dynamic_object_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>(
    "output/dynamic_object", 1, true);
  pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/points_raw", 1, true);

  object_sub_ =
    pnh_.subscribe("input/object", 100, &DummyPerceptionPublisherNode::objectCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), &DummyPerceptionPublisherNode::timerCallback, this);
  pnh_.param<double>("visible_range", visible_range_, double(100.0));
  pnh_.param<double>("detection_successful_rate", detection_successful_rate_, 0.8);
  pnh_.param<bool>("enable_ray_tracing", enable_ray_tracing_, true);
  pnh_.param<bool>("use_object_recognition", use_object_recognition_, true);
}

void DummyPerceptionPublisherNode::timerCallback(const ros::TimerEvent &)
{
  // output msgs
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_dynamic_object_msg;
  geometry_msgs::PoseStamped output_moved_object_pose;
  sensor_msgs::PointCloud2 output_pointcloud_msg;
  std_msgs::Header header;
  ros::Time current_time = ros::Time::now();

  tf2::Transform tf_base_link2map;
  try {
    geometry_msgs::TransformStamped ros_base_link2map;
    ros_base_link2map = tf_buffer_.lookupTransform(
      /*target*/ "base_link", /*src*/ "map", current_time, ros::Duration(0.5));
    tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
  } catch (tf2::TransformException & ex) {
    ROS_WARN_THROTTLE(5.0, "%s", ex.what());
    return;
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> v_pointcloud;
  std::vector<size_t> delete_idxs;
  static std::uniform_real_distribution<> detection_successful_random(0.0, 1.0);
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (detection_successful_rate_ < detection_successful_random(random_generator_)) continue;
    const double std_dev_x = std::sqrt(objects_.at(i).initial_state.pose_covariance.covariance[0]);
    const double std_dev_y = std::sqrt(objects_.at(i).initial_state.pose_covariance.covariance[7]);
    const double std_dev_z = std::sqrt(objects_.at(i).initial_state.pose_covariance.covariance[14]);
    const double std_dev_yaw =
      std::sqrt(objects_.at(i).initial_state.pose_covariance.covariance[35]);
    const double move_distance =
      objects_.at(i).initial_state.twist_covariance.twist.linear.x *
      (current_time.toSec() - ros::Time(objects_.at(i).header.stamp).toSec());
    tf2::Transform tf_object_origin2moved_object;
    tf2::Transform tf_map2object_origin;
    tf2::Transform tf_map2moved_object;
    {
      geometry_msgs::Transform ros_object_origin2moved_object;
      ros_object_origin2moved_object.translation.x = move_distance;
      ros_object_origin2moved_object.rotation.x = 0;
      ros_object_origin2moved_object.rotation.y = 0;
      ros_object_origin2moved_object.rotation.z = 0;
      ros_object_origin2moved_object.rotation.w = 1;
      tf2::fromMsg(ros_object_origin2moved_object, tf_object_origin2moved_object);
    }
    tf2::fromMsg(objects_.at(i).initial_state.pose_covariance.pose, tf_map2object_origin);
    tf_map2moved_object = tf_map2object_origin * tf_object_origin2moved_object;
    tf2::toMsg(tf_map2moved_object, output_moved_object_pose.pose);

    // pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    createObjectPointcloud(
      objects_.at(i).shape.dimensions.x, objects_.at(i).shape.dimensions.y,
      objects_.at(i).shape.dimensions.z, std_dev_x, std_dev_y, std_dev_z,
      tf_base_link2map * tf_map2moved_object, pointcloud_ptr);
    v_pointcloud.push_back(pointcloud_ptr);

    // dynamic object
    std::normal_distribution<> x_random(0.0, std_dev_x);
    std::normal_distribution<> y_random(0.0, std_dev_y);
    std::normal_distribution<> yaw_random(0.0, std_dev_yaw);
    tf2::Quaternion noised_quat;
    noised_quat.setRPY(0, 0, yaw_random(random_generator_));
    tf2::Transform tf_moved_object2noised_moved_object(
      noised_quat, tf2::Vector3(x_random(random_generator_), y_random(random_generator_), 0.0));
    tf2::Transform tf_base_link2noised_moved_object;
    tf_base_link2noised_moved_object =
      tf_base_link2map * tf_map2moved_object * tf_moved_object2noised_moved_object;
    autoware_perception_msgs::DynamicObjectWithFeature feature_object;
    feature_object.object.semantic = objects_.at(i).semantic;
    feature_object.object.state.pose_covariance = objects_.at(i).initial_state.pose_covariance;
    feature_object.object.state.twist_covariance = objects_.at(i).initial_state.twist_covariance;
    feature_object.object.state.orientation_reliable = false;
    feature_object.object.state.twist_reliable = false;
    feature_object.object.state.acceleration_reliable = false;
    tf2::toMsg(tf_base_link2noised_moved_object, feature_object.object.state.pose_covariance.pose);
    feature_object.object.shape = objects_.at(i).shape;
    pcl::toROSMsg(*pointcloud_ptr, feature_object.feature.cluster);
    output_dynamic_object_msg.feature_objects.push_back(feature_object);

    // check delete idx
    tf2::Transform tf_base_link2moved_object;
    tf_base_link2moved_object = tf_base_link2map * tf_map2moved_object;
    double dist = std::sqrt(
      tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
      tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
    if (visible_range_ < dist) delete_idxs.push_back(i);
  }
  // delete
  for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx) {
    objects_.erase(objects_.begin() + delete_idxs.at(delete_idx));
  }

  // merge all pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < v_pointcloud.size(); ++i) {
    for (size_t j = 0; j < v_pointcloud.at(i)->size(); ++j) {
      merged_pointcloud_ptr->push_back(v_pointcloud.at(i)->at(j));
    }
  }
  // no ground
  pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);

  // ray tracing
  if (enable_ray_tracing_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ray_traced_merged_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> ray_tracing_filter;
    ray_tracing_filter.setInputCloud(merged_pointcloud_ptr);
    ray_tracing_filter.setLeafSize(0.25, 0.25, 0.25);
    ray_tracing_filter.initializeVoxelGrid();
    for (size_t i = 0; i < v_pointcloud.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ray_traced_pointcloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
      for (size_t j = 0; j < v_pointcloud.at(i)->size(); ++j) {
        Eigen::Vector3i grid_cordinates = ray_tracing_filter.getGridCoordinates(
          v_pointcloud.at(i)->at(j).x, v_pointcloud.at(i)->at(j).y, v_pointcloud.at(i)->at(j).z);
        int grid_state;
        if (ray_tracing_filter.occlusionEstimation(grid_state, grid_cordinates) != 0)
          ROS_ERROR("ray tracing failed");
        if (grid_state == 1) {  // occluded
          continue;
        } else {  // not occluded
          ray_traced_pointcloud_ptr->push_back(v_pointcloud.at(i)->at(j));
          ray_traced_merged_pointcloud_ptr->push_back(v_pointcloud.at(i)->at(j));
        }
      }
      pcl::toROSMsg(
        *ray_traced_pointcloud_ptr,
        output_dynamic_object_msg.feature_objects.at(i).feature.cluster);
      output_dynamic_object_msg.feature_objects.at(i).feature.cluster.header.frame_id = "base_link";
      output_dynamic_object_msg.feature_objects.at(i).feature.cluster.header.stamp = current_time;
    }
    pcl::toROSMsg(*ray_traced_merged_pointcloud_ptr, output_pointcloud_msg);
  }

  // create output header
  output_moved_object_pose.header.frame_id = "map";
  output_moved_object_pose.header.stamp = current_time;
  output_dynamic_object_msg.header.frame_id = "base_link";
  output_dynamic_object_msg.header.stamp = current_time;
  output_pointcloud_msg.header.frame_id = "base_link";
  output_pointcloud_msg.header.stamp = current_time;

  // publish
  pointcloud_pub_.publish(output_pointcloud_msg);
  if (use_object_recognition_) {
    dynamic_object_pub_.publish(output_dynamic_object_msg);
  }
}

void DummyPerceptionPublisherNode::createObjectPointcloud(
  const double length, const double width, const double height, const double std_dev_x,
  const double std_dev_y, const double std_dev_z, const tf2::Transform & tf_base_link2moved_object,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud_ptr)
{
  std::normal_distribution<> x_random(0.0, std_dev_x);
  std::normal_distribution<> y_random(0.0, std_dev_y);
  std::normal_distribution<> z_random(0.0, std_dev_z);
  const double r = std::sqrt(
    tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
    tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
  auto getBaseLinkTo2DPoint = [tf_base_link2moved_object](double x, double y) -> pcl::PointXYZ {
    tf2::Transform tf_moved_object2point;
    tf2::Transform tf_base_link2point;
    geometry_msgs::Transform ros_moved_object2point;
    ros_moved_object2point.translation.x = x;
    ros_moved_object2point.translation.y = y;
    ros_moved_object2point.translation.z = 0.0;
    ros_moved_object2point.rotation.x = 0;
    ros_moved_object2point.rotation.y = 0;
    ros_moved_object2point.rotation.z = 0;
    ros_moved_object2point.rotation.w = 1;
    tf2::fromMsg(ros_moved_object2point, tf_moved_object2point);
    tf_base_link2point = tf_base_link2moved_object * tf_moved_object2point;
    pcl::PointXYZ point;
    point.x = tf_base_link2point.getOrigin().x();
    point.y = tf_base_link2point.getOrigin().y();
    point.z = tf_base_link2point.getOrigin().z();
    return point;
  };
  const double epsilon = 0.001;
  const double step = 0.05;
  const double vertical_theta_step = (1.0 / 180.0) * M_PI;
  const double vertical_min_theta = (-15.0 / 180.0) * M_PI;
  const double vertical_max_theta = (15.0 / 180.0) * M_PI;
  const double horizontal_theta_step = (0.5 / 180.0) * M_PI;
  const double horizontal_min_theta = (-180.0 / 180.0) * M_PI;
  const double horizontal_max_theta = (180.0 / 180.0) * M_PI;

  const double min_z = -1.0 * (height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  const double max_z = 1.0 * (height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  pcl::PointCloud<pcl::PointXYZ> horizontal_candidate_pointcloud;
  pcl::PointCloud<pcl::PointXYZ> horizontal_pointcloud;
  {
    const double y = -1.0 * (width / 2.0);
    for (double x = -1.0 * (length / 2.0); x <= ((length / 2.0) + epsilon); x += step) {
      horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
    }
  }
  {
    const double y = 1.0 * (width / 2.0);
    for (double x = -1.0 * (length / 2.0); x <= ((length / 2.0) + epsilon); x += step) {
      horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
    }
  }
  {
    const double x = -1.0 * (length / 2.0);
    for (double y = -1.0 * (width / 2.0); y <= ((width / 2.0) + epsilon); y += step) {
      horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
    }
  }
  {
    const double x = 1.0 * (length / 2.0);
    for (double y = -1.0 * (width / 2.0); y <= ((width / 2.0) + epsilon); y += step) {
      horizontal_candidate_pointcloud.push_back(getBaseLinkTo2DPoint(x, y));
    }
  }
  // 2D ray tracing
  size_t ranges_size =
    std::ceil((horizontal_max_theta - horizontal_min_theta) / horizontal_theta_step);
  std::vector<double> horizontal_ray_traced_2d_pointcloud;
  horizontal_ray_traced_2d_pointcloud.assign(ranges_size, std::numeric_limits<double>::infinity());
  const int no_data = -1;
  std::vector<int> horizontal_ray_traced_pointcloud_indices;
  horizontal_ray_traced_pointcloud_indices.assign(ranges_size, no_data);
  for (size_t i = 0; i < horizontal_candidate_pointcloud.points.size(); ++i) {
    double angle =
      std::atan2(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
    double range =
      std::hypot(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
    if (angle < horizontal_min_theta || angle > horizontal_max_theta) {
      continue;
    }
    int index = (angle - horizontal_min_theta) / horizontal_theta_step;
    if (range < horizontal_ray_traced_2d_pointcloud[index]) {
      horizontal_ray_traced_2d_pointcloud[index] = range;
      horizontal_ray_traced_pointcloud_indices.at(index) = i;
    }
  }
  for (const auto & pointcloud_index : horizontal_ray_traced_pointcloud_indices) {
    if (pointcloud_index != no_data) {
      // generate vertical point
      horizontal_pointcloud.push_back(horizontal_candidate_pointcloud.at(pointcloud_index));
      const double distance = std::hypot(
        horizontal_candidate_pointcloud.at(pointcloud_index).x,
        horizontal_candidate_pointcloud.at(pointcloud_index).y);
      for (double vertical_theta = vertical_min_theta;
           vertical_theta <= vertical_max_theta + epsilon; vertical_theta += vertical_theta_step) {
        const double z = distance * std::tan(vertical_theta);
        if (min_z <= z && z <= max_z + epsilon) {
          pcl::PointXYZ point;
          point.x =
            horizontal_candidate_pointcloud.at(pointcloud_index).x + x_random(random_generator_);
          point.y =
            horizontal_candidate_pointcloud.at(pointcloud_index).y + y_random(random_generator_);
          point.z = z + z_random(random_generator_);
          pointcloud_ptr->push_back(point);
        }
      }
    }
  }
}

void DummyPerceptionPublisherNode::objectCallback(
  const dummy_perception_publisher::Object::ConstPtr & msg)
{
  switch (msg->action) {
    case dummy_perception_publisher::Object::ADD: {
      tf2::Transform tf_input2map;
      tf2::Transform tf_input2object_origin;
      tf2::Transform tf_map2object_origin;
      try {
        geometry_msgs::TransformStamped ros_input2map;
        ros_input2map = tf_buffer_.lookupTransform(
          /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp, ros::Duration(0.5));
        tf2::fromMsg(ros_input2map.transform, tf_input2map);
      } catch (tf2::TransformException & ex) {
        ROS_WARN_THROTTLE(5.0, "%s", ex.what());
        return;
      }
      tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
      tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
      dummy_perception_publisher::Object object;
      object = *msg;
      tf2::toMsg(tf_map2object_origin, object.initial_state.pose_covariance.pose);
      objects_.push_back(object);
      break;
    }
    case dummy_perception_publisher::Object::DELETE: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          objects_.erase(objects_.begin() + i);
          break;
        }
      }
      break;
    }
    case dummy_perception_publisher::Object::MODIFY: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          tf2::Transform tf_input2map;
          tf2::Transform tf_input2object_origin;
          tf2::Transform tf_map2object_origin;
          try {
            geometry_msgs::TransformStamped ros_input2map;
            ros_input2map = tf_buffer_.lookupTransform(
              /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
              ros::Duration(0.5));
            tf2::fromMsg(ros_input2map.transform, tf_input2map);
          } catch (tf2::TransformException & ex) {
            ROS_WARN_THROTTLE(5.0, "%s", ex.what());
            return;
          }
          tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
          tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
          dummy_perception_publisher::Object object;
          objects_.at(i) = *msg;
          tf2::toMsg(tf_map2object_origin, objects_.at(i).initial_state.pose_covariance.pose);
          break;
        }
      }
      break;
    }
    case dummy_perception_publisher::Object::DELETEALL: {
      objects_.clear();
      break;
    }
  }
}
