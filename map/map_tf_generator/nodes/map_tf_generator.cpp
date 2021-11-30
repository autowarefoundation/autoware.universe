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
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
std::string map_frame_ = "map";
std::string viewer_frame_ = "viewer";

void Callback(const PointCloud::ConstPtr & clouds)
{
  const unsigned int sum = clouds->points.size();
  double coordinate[3] = {0, 0, 0};
  for (int i = 0; i < sum; i++) {
    coordinate[0] += clouds->points[i].x;
    coordinate[1] += clouds->points[i].y;
    coordinate[2] += clouds->points[i].z;
  }
  coordinate[0] = coordinate[0] / sum;
  coordinate[1] = coordinate[1] / sum;
  coordinate[2] = coordinate[2] / sum;

  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = map_frame_;
  static_transformStamped.child_frame_id = viewer_frame_;
  static_transformStamped.transform.translation.x = coordinate[0];
  static_transformStamped.transform.translation.y = coordinate[1];
  static_transformStamped.transform.translation.z = coordinate[2];
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static_broadcaster.sendTransform(static_transformStamped);

  ROS_INFO_STREAM(
    "broadcast static tf. map_frame:" << map_frame_ << ", viewer_frame:" << viewer_frame_
                                      << ", x:" << coordinate[0] << ", y:" << coordinate[1]
                                      << ", z:" << coordinate[2]);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "map_tf_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.getParam("map_frame", map_frame_);
  nh_private.getParam("viewer_frame", viewer_frame_);

  ros::Subscriber sub = nh.subscribe<PointCloud>("pointcloud_map", 1, &Callback);

  ros::spin();

  return 0;
};
