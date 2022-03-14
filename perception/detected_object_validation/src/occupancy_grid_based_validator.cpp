// Copyright 2022 Tier IV, Inc.
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

#include "occupancy_grid_based_validator/occupancy_grid_based_validator.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <boost/optional.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
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

bool transformDetectedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::DetectedObjects & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
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
    for (size_t i = 0; i < output_msg.objects.size(); ++i) {
      tf2::fromMsg(
        output_msg.objects.at(i).kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, output_msg.objects.at(i).kinematics.pose_with_covariance.pose);
      // Note: Covariance is not transformed.
    }
  }
  return true;
}

}  // namespace

namespace occupancy_grid_based_validator
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
using Shape = autoware_auto_perception_msgs::msg::Shape;

OccupancyGridBasedValidator::OccupancyGridBasedValidator(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("occupancy_grid_based_validator", node_options),
  objects_sub_(this, "~/input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile()),
  occ_grid_sub_(this, "~/input/occupancy_grid_map", rclcpp::QoS{1}.get_rmw_qos_profile()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  sync_(SyncPolicy(10), objects_sub_, occ_grid_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(
    std::bind(&OccupancyGridBasedValidator::onObjectsAndOccGrid, this, _1, _2));
  objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});
}

void OccupancyGridBasedValidator::onObjectsAndOccGrid(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & input_occ_grid)
{
  // Transform to occ grid frame
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!transformDetectedObjects(
        *input_objects, input_occ_grid->header.frame_id, tf_buffer_, transformed_objects))
    return;

  // Convert ros data type to cv::Mat
  cv::Mat occ_grid = fromOccupancyGrid(*input_occ_grid);

  // Debug
  cv::namedWindow("test2", cv::WINDOW_NORMAL);

  // Get vehicle mask image and calculate mean within mask.
  for (const auto & object : transformed_objects.objects) {
    const auto & label = object.classification.front().label;
    const bool is_vehicle = Label::CAR == label || Label::TRUCK == label || Label::BUS == label ||
                            Label::TRAILER == label;
    if (is_vehicle) {
      cv::Mat mask = getMask(*input_occ_grid, object, occ_grid);
      //  (int)cv::mean(occ_grid, mask)[0]);
      cv::imshow("test2", occ_grid);
      cv::waitKey(2);
    }
  }
}

cv::Mat OccupancyGridBasedValidator::getMask(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  cv::Mat mask = cv::Mat::zeros(occupancy_grid.info.height, occupancy_grid.info.width, CV_8UC1);
  return getMask(occupancy_grid, object, mask);
}

cv::Mat OccupancyGridBasedValidator::getMask(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const autoware_auto_perception_msgs::msg::DetectedObject & object, cv::Mat mask)
{
  const auto & resolution = occupancy_grid.info.resolution;
  const auto & origin = occupancy_grid.info.origin;
  std::vector<cv::Point2f> vertices;
  std::vector<cv::Point> pixel_vertices;
  toPolygon2d(object, vertices);

  for (const auto & vertex : vertices) {
    pixel_vertices.push_back(cv::Point2f(
      (vertex.x - origin.position.x) / resolution, (vertex.y - origin.position.y) / resolution));
  }
  cv::fillConvexPoly(mask, pixel_vertices, cv::Scalar(255));
  return mask;
}


cv::Mat OccupancyGridBasedValidator::fromOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  const auto & resolution = occupancy_grid.info.resolution;
  cv::Mat cv_occ_grid =
    cv::Mat::zeros(occupancy_grid.info.height, occupancy_grid.info.width, CV_8UC1);
  for (size_t i = 0; i < occupancy_grid.data.size(); ++i) {
    size_t y = i / occupancy_grid.info.width;
    size_t x = i % occupancy_grid.info.width;
    const auto & data = occupancy_grid.data[i];
    cv_occ_grid.at<unsigned char>(y, x) = std::min(std::max(data, static_cast<signed char>(0)), static_cast<signed char>(50)) * 2;
  }
  return cv_occ_grid;
}

void OccupancyGridBasedValidator::toPolygon2d(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  std::vector<cv::Point2f> & vertices)
{
  if (object.shape.type == Shape::BOUNDING_BOX) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    double yaw = tier4_autoware_utils::normalizeRadian(tf2::getYaw(pose.orientation));
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    Eigen::Vector2d offset0, offset1, offset2, offset3;
    offset0 = rotation *
              Eigen::Vector2d(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    offset1 = rotation *
              Eigen::Vector2d(object.shape.dimensions.x * 0.5f, -object.shape.dimensions.y * 0.5f);
    offset2 = rotation *
              Eigen::Vector2d(-object.shape.dimensions.x * 0.5f, -object.shape.dimensions.y * 0.5f);
    offset3 = rotation *
              Eigen::Vector2d(-object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    vertices.push_back(cv::Point2f(pose.position.x + offset0.x(), pose.position.y + offset0.y()));
    vertices.push_back(cv::Point2f(pose.position.x + offset1.x(), pose.position.y + offset1.y()));
    vertices.push_back(cv::Point2f(pose.position.x + offset2.x(), pose.position.y + offset2.y()));
    vertices.push_back(cv::Point2f(pose.position.x + offset3.x(), pose.position.y + offset3.y()));
  } else if (object.shape.type == Shape::CYLINDER) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5, "CYLINDER type is not supported");
  } else if (object.shape.type == Shape::POLYGON) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5, "POLYGON type is not supported");
  }
}
}  // namespace occupancy_grid_based_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_based_validator::OccupancyGridBasedValidator)
