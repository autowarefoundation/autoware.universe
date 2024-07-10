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

#include "autoware_scene_to_image_projector_node.hpp"
#include <object_recognition_utils/object_recognition_utils.hpp>
#include <boost/optional.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace
{
boost::optional<geometry_msgs::msg::Transform> get_transform_anonymous(
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

bool is_polygon_contained(const std::vector<cv::Point2f>& containee, const std::vector<std::vector<cv::Point2f>>& container) {   
    for(auto const& poly : container) {
        size_t counter = 0; 
        std::vector<cv::Point2f> contour;
        cv::convexHull(poly, contour);
        for(auto const& point : containee) {    
            if(cv::pointPolygonTest(contour, point, false) >= 0) counter++;
        }
        if(containee.size() - counter < 2) return true;
    }
    return false;
}

}  // namespace

namespace autoware::scene_to_image_projector
{
SceneToImageProjectorNode::SceneToImageProjectorNode(const rclcpp::NodeOptions & options)
: Node("scene_to_image_projector", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO(this->get_logger(), "SceneToImageProjectorNode::SceneToImageProjectorNode");

  auto objects_type = declare_parameter<std::string>("objects_type", "detected");
  auto use_trajectory = declare_parameter<bool>("use_trajectory", false);
  auto use_road_boundaries = declare_parameter<bool>("use_road_boundaries", false);

  show_pedestrian = declare_parameter<bool>("show_pedestrian", true);
  show_bicycle = declare_parameter<bool>("show_bicycle", true);
  show_motorcycle = declare_parameter<bool>("show_motorcycle", true);
  show_trailer = declare_parameter<bool>("show_trailer", true);
  show_bus = declare_parameter<bool>("show_bus", true);
  show_truck = declare_parameter<bool>("show_truck", true);
  show_car = declare_parameter<bool>("show_car", true);

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/output/image", 10);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "~/input/image", 10,
    std::bind(&SceneToImageProjectorNode::image_callback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/camera_info", 10,
    std::bind(&SceneToImageProjectorNode::camera_info_callback, this, std::placeholders::_1));

  if(objects_type == "detected"){
    detected_objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "~/input/objects", 10,
    std::bind(&SceneToImageProjectorNode::detected_objects_callback, this, std::placeholders::_1));
    }
  else if(objects_type == "tracked"){
    tracked_objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    "~/input/objects", 10,
    std::bind(&SceneToImageProjectorNode::tracked_objects_callback, this, std::placeholders::_1));
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "Invalid objects_type parameter");
  }

  if(use_trajectory){
    trajectory_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 10,
    std::bind(&SceneToImageProjectorNode::trajectory_callback, this, std::placeholders::_1));
  }

  if(use_road_boundaries){
    path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Path>(
    "~/input/path", 10,
    std::bind(&SceneToImageProjectorNode::path_callback, this, std::placeholders::_1));
  }
}

void SceneToImageProjectorNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  latest_camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
  camera_info_received_ = true;
}

void SceneToImageProjectorNode::detected_objects_callback(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & msg)
{
  latest_detected_objects_ = std::make_shared<autoware_perception_msgs::msg::DetectedObjects>(*msg);
  latest_detected_objects_received_ = true;
}

void SceneToImageProjectorNode::tracked_objects_callback(
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & msg)
{
  latest_tracked_objects_ = std::make_shared<autoware_perception_msgs::msg::TrackedObjects>(*msg);
  latest_tracked_objects_received_ = true;
}

void SceneToImageProjectorNode::trajectory_callback(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & msg)
{
  latest_trajectory_ = std::make_shared<autoware_planning_msgs::msg::Trajectory>(*msg);
  latest_trajectory_received_ = true;
}

void SceneToImageProjectorNode::path_callback(
  const autoware_planning_msgs::msg::Path::ConstSharedPtr & msg)
{
  latest_path_ = std::make_shared<autoware_planning_msgs::msg::Path>(*msg);
  latest_path_received_ = true;
}

void SceneToImageProjectorNode::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg)
{
  if(camera_info_received_) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input_image_msg, "bgr8");
    Eigen::Matrix4d projection = get_projection_matrix(*latest_camera_info_);

    if(latest_detected_objects_received_){
      const auto self_transform = get_transform_anonymous(
        tf_buffer_, latest_detected_objects_->header.frame_id, latest_camera_info_->header.frame_id,
        latest_detected_objects_->header.stamp);

      if (!self_transform) {
        return;
      }

      /* transform to world coordinate */
      autoware_perception_msgs::msg::DetectedObjects transformed_objects;
      if (!object_recognition_utils::transformObjects(
            *latest_detected_objects_, latest_camera_info_->header.frame_id, tf_buffer_, transformed_objects)) {
        return;
      }

      auto & objects = transformed_objects.objects;

      std::sort(objects.begin(), objects.end(), [](const auto& obj1, const auto& obj2) {
        return (obj1.kinematics.pose_with_covariance.pose.position.x * obj1.kinematics.pose_with_covariance.pose.position.x +
               obj1.kinematics.pose_with_covariance.pose.position.y * obj1.kinematics.pose_with_covariance.pose.position.y +
               obj1.kinematics.pose_with_covariance.pose.position.z * obj1.kinematics.pose_with_covariance.pose.position.z <
        obj2.kinematics.pose_with_covariance.pose.position.x * obj2.kinematics.pose_with_covariance.pose.position.x +
               obj2.kinematics.pose_with_covariance.pose.position.y * obj2.kinematics.pose_with_covariance.pose.position.y +
               obj2.kinematics.pose_with_covariance.pose.position.z * obj2.kinematics.pose_with_covariance.pose.position.z);
      });

      std::vector<std::vector<cv::Point2f>> previous_polygons;

      for (const auto & object : objects) {
        if (
          object.classification.front().label ==
          autoware_perception_msgs::msg::ObjectClassification::UNKNOWN) {
          continue;
        }

        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) &&
          !show_pedestrian) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::BICYCLE) &&
          !show_bicycle) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE) &&
          !show_motorcycle) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::TRAILER) &&
          !show_trailer) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::BUS) &&
          !show_bus) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::TRUCK) &&
          !show_truck) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::CAR) &&
          !show_car) {
          continue;
        }

        if(!projectable(object.kinematics.pose_with_covariance.pose.position, projection)) continue;

        auto bbox_corners = detected_object_corners(object);
        if (!bbox_corners) {
          continue;
        }

        draw_bounding_box(*bbox_corners, cv_ptr->image, projection, previous_polygons);
      }      
    }
    if(latest_tracked_objects_received_){
      const auto self_transform = get_transform_anonymous(
        tf_buffer_, latest_tracked_objects_->header.frame_id, latest_camera_info_->header.frame_id,
        latest_tracked_objects_->header.stamp);

      if (!self_transform) {
        return;
      }

      /* transform to world coordinate */
      autoware_perception_msgs::msg::TrackedObjects transformed_objects;
      if (!object_recognition_utils::transformObjects(
            *latest_tracked_objects_, latest_camera_info_->header.frame_id, tf_buffer_, transformed_objects)) {
        return;
      }
      
      auto & objects = transformed_objects.objects;

      std::sort(objects.begin(), objects.end(), [](const auto& obj1, const auto& obj2) {
        return (obj1.kinematics.pose_with_covariance.pose.position.x * obj1.kinematics.pose_with_covariance.pose.position.x +
               obj1.kinematics.pose_with_covariance.pose.position.y * obj1.kinematics.pose_with_covariance.pose.position.y +
               obj1.kinematics.pose_with_covariance.pose.position.z * obj1.kinematics.pose_with_covariance.pose.position.z <
        obj2.kinematics.pose_with_covariance.pose.position.x * obj2.kinematics.pose_with_covariance.pose.position.x +
               obj2.kinematics.pose_with_covariance.pose.position.y * obj2.kinematics.pose_with_covariance.pose.position.y +
               obj2.kinematics.pose_with_covariance.pose.position.z * obj2.kinematics.pose_with_covariance.pose.position.z);
      });

      std::vector<std::vector<cv::Point2f>> previous_polygons = {};

      for (const auto & object : objects) {
        if (
          object.classification.front().label ==
          autoware_perception_msgs::msg::ObjectClassification::UNKNOWN) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN) &&
          !show_pedestrian) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::BICYCLE) &&
          !show_bicycle) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE) &&
          !show_motorcycle) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::TRAILER) &&
          !show_trailer) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::BUS) &&
          !show_bus) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::TRUCK) &&
          !show_truck) {
          continue;
        }
        if (
          (object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::CAR) &&
          !show_car) {
          continue;
        }

        if(!projectable(object.kinematics.pose_with_covariance.pose.position, projection)) continue;

        auto bbox_corners = detected_object_corners(object);
        if (!bbox_corners) {
          continue;
        }

        draw_bounding_box(*bbox_corners, cv_ptr->image, projection , previous_polygons);
      }
    }
    if(latest_trajectory_received_){
      const auto self_transform = get_transform_anonymous(
        tf_buffer_, latest_trajectory_->header.frame_id, latest_camera_info_->header.frame_id, latest_trajectory_->header.stamp);

      if (!self_transform) {
        RCLCPP_WARN(this->get_logger(), "Transform is not possible!");
        return;
      }

      /* transform to world coordinate */
      autoware_planning_msgs::msg::Trajectory transformed_trajectory;
      if (!object_recognition_utils::transformTrajectory(
            *latest_trajectory_, latest_camera_info_->header.frame_id, tf_buffer_, transformed_trajectory)) {
        RCLCPP_WARN(get_logger(), "There was a problem with transforming the trajectory");
        return;
      }

      auto last_point = cv::Point2f(0,0); // this is out of image

      for(size_t i = 1; i < transformed_trajectory.points.size(); i++){
        Eigen::Vector3d position(
          transformed_trajectory.points[i].pose.position.x,
          transformed_trajectory.points[i].pose.position.y,
          transformed_trajectory.points[i].pose.position.z);
        
        cv::Point2f projected_point;

        if(!project_point(position, projection, projected_point)){
          continue;
        }

        if(last_point.x == 0){
          Eigen::Vector3d position_before(
          transformed_trajectory.points[i-1].pose.position.x,
          transformed_trajectory.points[i-1].pose.position.y,
          transformed_trajectory.points[i-1].pose.position.z);

          while(!project_point(position_before, projection, last_point)){
            position_before = 0.9 * position_before + 0.1 * position;
          }
        }
        
        cv::line(cv_ptr->image, last_point, projected_point, cv::Scalar(0, 255, 0), 8);
        last_point = projected_point;
      }
    }
    if(latest_path_received_){
      const auto self_transform = get_transform_anonymous(
        tf_buffer_, latest_path_->header.frame_id, latest_camera_info_->header.frame_id, latest_path_->header.stamp);

      if (!self_transform) {  
        RCLCPP_WARN(this->get_logger(), "Transform is not possible!");
        return;
      }

      /* transform to world coordinate */
      autoware_planning_msgs::msg::Path transformed_path;
      if (!object_recognition_utils::transformPath(
            *latest_path_, latest_camera_info_->header.frame_id, tf_buffer_, transformed_path)) {
        RCLCPP_WARN(get_logger(), "There was a problem with transforming the trajectory");
        return;
      }

      auto last_point_left = cv::Point2f(0,0); // this is out of image
      auto last_point_right = cv::Point2f(0,0); // this is out of image

      for(size_t i = 1; i < transformed_path.left_bound.size(); i++){
        Eigen::Vector3d left_position(
          transformed_path.left_bound[i].x,
          transformed_path.left_bound[i].y,
          transformed_path.left_bound[i].z);

          cv::Point2f projected_point_left;

          if(!project_point(left_position, projection, projected_point_left)){
            continue;
          }
        
        if(last_point_left.x == 0){
          Eigen::Vector3d left_position_before(
          transformed_path.left_bound[i-1].x,
          transformed_path.left_bound[i-1].y,
          transformed_path.left_bound[i-1].z);

          while(!project_point(left_position_before, projection, last_point_left)){
            left_position_before = 0.9 * left_position_before + 0.1 * left_position;
          }
        }
        cv::line(cv_ptr->image, last_point_left, projected_point_left, cv::Scalar(255, 0, 0), 8);
        last_point_left = projected_point_left;
      }

      for(size_t i = 1; i < transformed_path.right_bound.size(); i++){
        Eigen::Vector3d right_position(
          transformed_path.right_bound[i].x,
          transformed_path.right_bound[i].y,
          transformed_path.right_bound[i].z);
        
        cv::Point2f projected_point_right;

        if(!project_point(right_position, projection, projected_point_right)){
          continue;
        }

        if(last_point_right.x == 0){
          Eigen::Vector3d right_position_before(
          transformed_path.right_bound[i-1].x,
          transformed_path.right_bound[i-1].y,
          transformed_path.right_bound[i-1].z);

          while(!project_point(right_position_before, projection, last_point_right)){
            right_position_before = 0.9 * right_position_before + 0.1 * right_position;
          }
        }
        cv::line(cv_ptr->image, last_point_right, projected_point_right, cv::Scalar(255, 0, 0), 8);
        last_point_right = projected_point_right;
      }
    }
    
    image_pub_->publish(*cv_ptr->toImageMsg());
  }
  else {
    image_pub_->publish(*input_image_msg);
  }
}

template <typename T>
std::optional<std::vector<Eigen::Vector3d>> SceneToImageProjectorNode::detected_object_corners(
  const T & detected_object)
{
  std::vector<Eigen::Vector3d> corners;
  if (detected_object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
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
  cv::Point2f & projected_point_out)
{
  Eigen::Vector4d projected_point =
    projection_matrix * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);

  if (projected_point.z() <= 0.0) {
    return false;
  }

  Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
    projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

  projected_point_out.x = normalized_projected_point.x();
  projected_point_out.y = normalized_projected_point.y();

  return true;
}

bool SceneToImageProjectorNode::projectable(
  const geometry_msgs::msg::Point & point, const Eigen::Matrix4d & projection_matrix)
{
  Eigen::Vector4d projected_point =
    projection_matrix * Eigen::Vector4d(point.x, point.y, point.z, 1.0);

  return projected_point.z() > 0.0;
}

void SceneToImageProjectorNode::draw_bounding_box(
  const std::vector<Eigen::Vector3d> & corners, cv::Mat & image,
  const Eigen::Matrix4d & projection_matrix, std::vector<std::vector<cv::Point2f>> & previous_polygons)
{
  const std::vector<std::vector<int>> edges = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  std::vector<cv::Point2f> points(8);
  cv::Mat image_copy = image.clone();

  for (const auto & edge : edges) {
    cv::Point2f point_on_image_1;
    cv::Point2f point_on_image_2;
    Eigen::Vector3d point_on_image_1_3d = corners.at(edge.at(0));
    Eigen::Vector3d point_on_image_2_3d = corners.at(edge.at(1));

    if(!project_point( point_on_image_1_3d, projection_matrix, point_on_image_1) && !project_point( point_on_image_2_3d, projection_matrix, point_on_image_2)) continue; // two of them are not in the projectable area
    
    while(!project_point( point_on_image_1_3d , projection_matrix, point_on_image_1)){
      point_on_image_1_3d  = 0.99 *  point_on_image_1_3d  + 0.01 *  point_on_image_2_3d;
    }

    points[edge.at(0)] = point_on_image_1;
    
    while(!project_point( point_on_image_2_3d, projection_matrix, point_on_image_2)){
      point_on_image_2_3d = 0.99 * point_on_image_2_3d + 0.01 *  point_on_image_1_3d ;
    }

    points[edge.at(1)] = point_on_image_2;    
    
    cv::line(image_copy, point_on_image_1, point_on_image_2, cv::Scalar(0, 0, 255), 8);
  }

  if (!is_polygon_contained(points, previous_polygons)){
    previous_polygons.push_back(points);
    image = image_copy.clone();
  }
}

}  // namespace autoware::scene_to_image_projector

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::scene_to_image_projector::SceneToImageProjectorNode)