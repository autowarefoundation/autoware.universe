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

#include "scene_to_image_projector/node.hpp"

// #include "perception_utils/perception_utils.hpp"  instead of this one we use following

#include <object_recognition_utils/object_recognition_utils.hpp>

#include <boost/optional.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransformAnonymous(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    // check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer.canTransform(
          target_frame_id, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return boost::none;
    }

    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return boost::none;
  }
}

}  // namespace

namespace scene_to_image_projector
{
SceneToImageProjectorNode::SceneToImageProjectorNode(const rclcpp::NodeOptions & options)
: Node("scene_to_image_projector_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO(this->get_logger(), "SceneToImageProjectorNode::SceneToImageProjectorNode");

  auto objects_type = declare_parameter<std::string>("objects_type", "None");
  auto use_trajectories = declare_parameter<bool>("use_trajectory", false);
  auto use_objects = declare_parameter<bool>("use_objects", false);
  //  auto sub_road_boundaries = declare_parameter<bool>("sub_road_boundaries", false);

  image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
  camera_info_sub_.subscribe(this, "~/input/camera_info", rmw_qos_profile_sensor_data);

  if(use_objects){
    if (objects_type == "tracked") {
      tracked_objects_sub_.subscribe(this, "~/input/objects", rclcpp::QoS{1}.get_rmw_qos_profile());

      sync_tracked_ = std::make_shared<ApproximateSyncTracked>(
        ApproximateSyncPolicyTracked(10), image_sub_, camera_info_sub_, tracked_objects_sub_);

      sync_tracked_->registerCallback(std::bind(
        &SceneToImageProjectorNode::ImageObjectCallback<
          autoware_auto_perception_msgs::msg::TrackedObjects>,
        this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else if (objects_type == "detected") {
        detected_objects_sub_.subscribe(this, "~/input/objects", rclcpp::QoS{1}.get_rmw_qos_profile());

        sync_detected_ = std::make_shared<ApproximateSyncDetected>(
        ApproximateSyncPolicyDetected(10), image_sub_, camera_info_sub_, detected_objects_sub_);

        sync_detected_->registerCallback(std::bind(
        &SceneToImageProjectorNode::ImageObjectCallback<
          autoware_auto_perception_msgs::msg::DetectedObjects>,
        this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else {
        RCLCPP_ERROR(this->get_logger(), "objects_type is not supported");
    }
  } 
  
  if (use_trajectories) {
    trajectory_sub_.subscribe(this, "~/input/trajectory", rclcpp::QoS{1}.get_rmw_qos_profile());

    sync_trajectory_ = std::make_shared<ApproximateSyncTrajectory>(
      ApproximateSyncPolicyTrajectory(10), image_sub_, camera_info_sub_, trajectory_sub_);

    sync_trajectory_->registerCallback(std::bind(
      &SceneToImageProjectorNode::TrajectoryCallback,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  image_pub_ = image_transport::create_publisher(this, "~/output/image");
}

template <typename T>
void SceneToImageProjectorNode::ImageObjectCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const typename T::ConstSharedPtr & objects_msg)
{
  RCLCPP_INFO(this->get_logger(), "SceneToImageProjectorNode::ProjectObjectsToImage");
  const auto self_transform = getTransformAnonymous(
    tf_buffer_, objects_msg->header.frame_id, camera_info_msg->header.frame_id,
    objects_msg->header.stamp);

  if (!self_transform) {
    return;
  }

  /* transform to world coordinate */
  T transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *objects_msg, camera_info_msg->header.frame_id, tf_buffer_, transformed_objects)) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input_image_msg, "bgr8");

  Eigen::Matrix4d projection = get_projection_matrix(*camera_info_msg);

  auto const & objects = transformed_objects.objects;
  for (const auto & object : objects) {
    if (
      object.classification.front().label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
      continue;
    }

    if (out_of_image(object.kinematics.pose_with_covariance.pose, *camera_info_msg, projection)) {
      continue;
    }

    auto bbox_corners = detected_object_corners(object);
    if (!bbox_corners) {
      continue;
    }

    draw_bounding_box(*bbox_corners, cv_ptr->image, projection);
  }

  image_pub_.publish(cv_ptr->toImageMsg());
}

template <typename T>
std::optional<std::vector<Eigen::Vector3d>> SceneToImageProjectorNode::detected_object_corners(
  const T & detected_object)
{
  std::vector<Eigen::Vector3d> corners;
  if (detected_object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    calculate_bbox_corners(detected_object, corners);
    return corners;
  }

  return std::nullopt;
}

template <typename T>
void SceneToImageProjectorNode::calculate_bbox_corners(
  const T & detected_object, std::vector<Eigen::Vector3d> & corners_out)
{
  auto const & pose = detected_object.kinematics.pose_with_covariance.pose;
  auto const & dimensions = detected_object.shape.dimensions;

  const std::vector<std::vector<double>> corners_template = {
    // down surface
    {1, 1, -1},
    {1, -1, -1},
    {-1, -1, -1},
    {-1, 1, -1},
    // up surface
    {1, 1, 1},
    {1, -1, 1},
    {-1, -1, 1},
    {-1, 1, 1},
  };

  const auto position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const auto orientation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  for (const auto & corner : corners_template) {
    Eigen::Vector3d corner_point(
      dimensions.x * corner.at(0) / 2.0, dimensions.y * corner.at(1) / 2.0,
      dimensions.z * corner.at(2) / 2.0);
    corners_out.emplace_back(orientation * corner_point + position);
  }
}

bool SceneToImageProjectorNode::out_of_image(
  const geometry_msgs::msg::Pose & center_pose, const sensor_msgs::msg::CameraInfo & camera_info,
  const Eigen::Matrix4d & projection)
{
  const auto position =
    Eigen::Vector3d(center_pose.position.x, center_pose.position.y, center_pose.position.z);

  cv::Point2d center_on_image;
  if (!project_point(position, projection, center_on_image)) {
    return true;
  }

  if (
    center_on_image.x <= 0 || center_on_image.x >= camera_info.width || center_on_image.y <= 0 ||
    center_on_image.y >= camera_info.height) {
    return true;
  }

  return false;
}

Eigen::Matrix4d SceneToImageProjectorNode::get_projection_matrix(
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  Eigen::Matrix4d projection;
  projection << camera_info_msg.p.at(0), camera_info_msg.p.at(1), camera_info_msg.p.at(2),
    camera_info_msg.p.at(3), camera_info_msg.p.at(4), camera_info_msg.p.at(5),
    camera_info_msg.p.at(6), camera_info_msg.p.at(7), camera_info_msg.p.at(8),
    camera_info_msg.p.at(9), camera_info_msg.p.at(10), camera_info_msg.p.at(11);

  return projection;
}

bool SceneToImageProjectorNode::project_point(
  const Eigen::Vector3d & point, const Eigen::Matrix4d & projection_matrix,
  cv::Point2d & projected_point_out)
{
  Eigen::Vector4d projected_point =
    projection_matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);

  if (projected_point.z() < 0.0) {
    return false;
  }

  Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
    projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

  projected_point_out.x = normalized_projected_point.x();
  projected_point_out.y = normalized_projected_point.y();

  return true;
}

void SceneToImageProjectorNode::draw_bounding_box(
  const std::vector<Eigen::Vector3d> & corners, cv::Mat & image,
  const Eigen::Matrix4d & projection_matrix)
{
  const std::vector<std::vector<int>> edges = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto & edge : edges) {
    cv::Point2d point_on_image_1;
    project_point(corners.at(edge.at(0)), projection_matrix, point_on_image_1);
    cv::Point2d point_on_image_2;
    project_point(corners.at(edge.at(1)), projection_matrix, point_on_image_2);
    cv::line(image, point_on_image_1, point_on_image_2, cv::Scalar(0, 0, 255), 2);
  }
}

void SceneToImageProjectorNode::TrajectoryCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const typename autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & msg)
{
  
  const auto self_transform = getTransformAnonymous(
    tf_buffer_, msg->header.frame_id, camera_info_msg->header.frame_id, msg->header.stamp);

  if (!self_transform) {  
    RCLCPP_ERROR(this->get_logger(), "Transform is not possible!");
    return;
  }

  /* transform to world coordinate */
  autoware_auto_planning_msgs::msg::Trajectory transformed_trajectory;
  if (!object_recognition_utils::transformTrajectory(
        *msg, camera_info_msg->header.frame_id, tf_buffer_, transformed_trajectory)) {
    RCLCPP_WARN(get_logger(), "There was a problem with transforming the trajectory");
    return;
  } 

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input_image_msg, "bgr8");

  Eigen::Matrix4d projection_matrix = get_projection_matrix(*camera_info_msg);

  std::vector<cv::Point2d> projected_points(transformed_trajectory.points.size());

   for(size_t i = 0; i < transformed_trajectory.points.size(); i++){
    Eigen::Vector3d position(
      transformed_trajectory.points[i].pose.position.x,
      transformed_trajectory.points[i].pose.position.y,
      transformed_trajectory.points[i].pose.position.z);
    project_point(position, projection_matrix, projected_points[i]);
  }

  auto last_point = cv::Point2d(-1, -1); // this is out of image

  for(size_t i = 0; i < projected_points.size(); i++){
    if(out_of_image(transformed_trajectory.points[i].pose, *camera_info_msg, projection_matrix)){
      continue;
    }
    if(last_point.x == -1){
      last_point = projected_points[i];
      continue;
    }
    cv::line(cv_ptr->image, last_point, projected_points[i], cv::Scalar(0, 255, 0), 2);
    last_point = projected_points[i];
  }

  image_pub_.publish(cv_ptr->toImageMsg());
}

}  // namespace scene_to_image_projector

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(scene_to_image_projector::SceneToImageProjectorNode)