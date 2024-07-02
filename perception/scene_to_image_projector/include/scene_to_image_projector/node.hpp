// Copyright 2022 LeoDrive.ai
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

#ifndef SCENE_TO_IMAGE_PROJECTOR__NODE_HPP_
#define SCENE_TO_IMAGE_PROJECTOR__NODE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scene_to_image_projector
{
class SceneToImageProjectorNode : public rclcpp::Node
{
public:
  explicit SceneToImageProjectorNode(const rclcpp::NodeOptions & options);

  void image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg);
  
  void detected_objects_callback(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & msg);

  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_{};

  std::shared_ptr<autoware_auto_perception_msgs::msg::DetectedObjects> latest_detected_objects_;
  bool latest_detected_objects_received_ = false;

  std::shared_ptr<sensor_msgs::msg::CameraInfo> latest_camera_info_;
  bool camera_info_received_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  template <typename T>
  std::optional<std::vector<Eigen::Vector3d>> detected_object_corners(const T & detected_object);

  bool out_of_image(
    const geometry_msgs::msg::Pose & center_pose, const sensor_msgs::msg::CameraInfo & camera_info,
    const Eigen::Matrix4d & projection);

  template <typename T>
  static void calculate_bbox_corners(
    const T & detected_object, std::vector<Eigen::Vector3d> & corners_out);

  void draw_bounding_box(
    const std::vector<Eigen::Vector3d> & corners, cv::Mat & image,
    const Eigen::Matrix4d & projection_matrix);

  Eigen::Matrix4d get_projection_matrix(const sensor_msgs::msg::CameraInfo & camera_info_msg);

  bool project_point(
    const Eigen::Vector3d & point, const Eigen::Matrix4d & projection_matrix,
    cv::Point2d & projected_point_out);
};

}  // namespace scene_to_image_projector

#endif  // SCENE_TO_IMAGE_PROJECTOR__NODE_HPP_