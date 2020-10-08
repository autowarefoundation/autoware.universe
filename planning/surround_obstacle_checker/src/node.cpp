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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

#include <surround_obstacle_checker/node.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode()
: nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  // Parameters
  pnh_.param<bool>("use_pointcloud", use_pointcloud_, true);
  pnh_.param<bool>("use_dynamic_object", use_dynamic_object_, true);
  pnh_.param<double>("surround_check_distance", surround_check_distance_, 2.0);
  pnh_.param<double>("surround_check_recover_distance", surround_check_recover_distance_, 2.5);
  pnh_.param<double>("state_clear_time", state_clear_time_, 2.0);
  pnh_.param<double>("stop_state_ego_speed", stop_state_ego_speed_, 0.1);
  wheel_base_ = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");
  front_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/front_overhang");
  rear_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/rear_overhang");
  left_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/left_overhang");
  right_overhang_ = waitForParam<double>(pnh_, "/vehicle_info/right_overhang");
  wheel_tread_ = waitForParam<double>(pnh_, "/vehicle_info/wheel_tread");
  vehicle_width_ = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
  vehicle_length_ = waitForParam<double>(pnh_, "/vehicle_info/vehicle_length");
  debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(wheel_base_ + front_overhang_);
  self_poly_ = createSelfPolygon();

  // Publishers
  path_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  stop_reason_diag_pub_ =
    pnh_.advertise<diagnostic_msgs::DiagnosticStatus>("output/no_start_reason", 1);

  // Subscriber
  path_sub_ =
    pnh_.subscribe("input/trajectory", 1, &SurroundObstacleCheckerNode::pathCallback, this);
  pointcloud_sub_ =
    pnh_.subscribe("input/pointcloud", 1, &SurroundObstacleCheckerNode::pointCloudCallback, this);
  dynamic_object_sub_ =
    pnh_.subscribe("input/objects", 1, &SurroundObstacleCheckerNode::dynamicObjectCallback, this);
  current_velocity_sub_ =
    pnh_.subscribe("input/twist", 1, &SurroundObstacleCheckerNode::currentVelocityCallback, this);
}

void SurroundObstacleCheckerNode::pathCallback(
  const autoware_planning_msgs::Trajectory::ConstPtr & input_msg)
{
  if (use_pointcloud_ && !pointcloud_ptr_) {
    ROS_WARN_THROTTLE(1.0, "waiting for pointcloud info...");
    return;
  }

  if (use_dynamic_object_ && !object_ptr_) {
    ROS_WARN_THROTTLE(1.0, "waiting for dynamic object info...");
    return;
  }

  if (!current_velocity_ptr_) {
    ROS_WARN_THROTTLE(1.0, "waiting for current velocity...");
    return;
  }

  // parameter description
  autoware_planning_msgs::Trajectory output_msg = *input_msg;
  diagnostic_msgs::DiagnosticStatus no_start_reason_diag;

  // get current pose in traj frame
  geometry_msgs::Pose current_pose;
  if (!getPose(input_msg->header.frame_id, "base_link", input_msg->header.stamp, current_pose)) {
    return;
  }

  // get closest idx
  const size_t closest_idx = getClosestIdx(*input_msg, current_pose);

  // get nearest object
  double min_dist_to_obj = std::numeric_limits<double>::max();
  geometry_msgs::Point nearest_obj_point;
  getNearestObstacle(&min_dist_to_obj, &nearest_obj_point);

  // check current obstacle status (exist or not)
  const auto is_obstacle_found = isObstacleFound(min_dist_to_obj);

  // check current stop status (stop or not)
  const auto is_stopped = checkStop(input_msg->points.at(closest_idx));

  const auto is_stop_required = isStopRequired(is_obstacle_found, is_stopped);

  // insert stop velocity
  if (is_stop_required) {
    state_ = State::STOP;

    // do not start when there is a obstacle near the ego vehicle.
    ROS_WARN_STREAM_THROTTLE(
      0.5, "[surround_obstacle_checker]: "
             << "do not start because there is obstacle near the ego vehicle.");
    insertStopVelocity(closest_idx, &output_msg);

    // visualization for debug
    if (is_obstacle_found) {
      debug_ptr_->pushObstaclePoint(nearest_obj_point, PointType::NoStart);
    }
    debug_ptr_->pushPose(input_msg->points.at(closest_idx).pose, PoseType::NoStart);
    no_start_reason_diag = makeStopReasonDiag("obstacle", input_msg->points.at(closest_idx).pose);
  } else {
    state_ = State::PASS;
  }

  // publish trajectory and debug info
  path_pub_.publish(output_msg);
  stop_reason_diag_pub_.publish(no_start_reason_diag);
  debug_ptr_->publish();
}

void SurroundObstacleCheckerNode::pointCloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr & input_msg)
{
  pointcloud_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::dynamicObjectCallback(
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & input_msg)
{
  object_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::currentVelocityCallback(
  const geometry_msgs::TwistStamped::ConstPtr & input_msg)
{
  current_velocity_ptr_ = input_msg;
}

void SurroundObstacleCheckerNode::insertStopVelocity(
  const size_t closest_idx, autoware_planning_msgs::Trajectory * traj)
{
  //set zero velocity from closest idx to last idx
  for (size_t i = closest_idx; i < traj->points.size(); i++) {
    traj->points.at(i).twist.linear.x = 0.0;
  }
}

bool SurroundObstacleCheckerNode::getPose(
  const std::string & source, const std::string & target, const ros::Time & time,
  geometry_msgs::Pose & pose)
{
  try {
    // get transform from source to target
    geometry_msgs::TransformStamped ros_src2tgt;
    ros_src2tgt = tf_buffer_.lookupTransform(source, target, time, ros::Duration(0.1));
    // convert geometry_msgs::Transform to geometry_msgs::Pose
    tf2::Transform transform;
    tf2::fromMsg(ros_src2tgt.transform, transform);
    tf2::toMsg(transform, pose);
  } catch (tf2::TransformException & ex) {
    ROS_WARN_STREAM_THROTTLE(
      0.5, "[surround_obstacle_checker]: cannot get tf from " << source << " to " << target);
    return false;
  }
  return true;
}

bool SurroundObstacleCheckerNode::convertPose(
  const geometry_msgs::Pose & pose, const std::string & source, const std::string & target,
  const ros::Time & time, geometry_msgs::Pose & conv_pose)
{
  geometry_msgs::TransformStamped ros_src2tgt;
  tf2::Transform src2tgt;
  tf2::Transform src2obj;
  tf2::Transform tgt2obj;

  try {
    // get transform from source to target
    ros_src2tgt = tf_buffer_.lookupTransform(source, target, time, ros::Duration(0.1));
    tf2::fromMsg(ros_src2tgt.transform, src2tgt);
  } catch (tf2::TransformException & ex) {
    ROS_WARN_STREAM_THROTTLE(
      0.5, "[surround obstacle chekcer] cannot get tf from " << source << " to " << target);
    return false;
  }

  tf2::fromMsg(pose, src2obj);
  tgt2obj = src2tgt.inverse() * src2obj;
  tf2::toMsg(tgt2obj, conv_pose);
  return true;
}

size_t SurroundObstacleCheckerNode::getClosestIdx(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Pose current_pose)
{
  double min_dist = std::numeric_limits<double>::max();
  size_t min_dist_idx = 0;
  bool is_init = false;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const double x = traj.points.at(i).pose.position.x - current_pose.position.x;
    const double y = traj.points.at(i).pose.position.y - current_pose.position.y;
    const double dist = std::hypot(x, y);
    if (dist < min_dist) {
      min_dist_idx = i;
      min_dist = dist;
    }
  }
  return min_dist_idx;
}

void SurroundObstacleCheckerNode::getNearestObstacle(
  double * min_dist_to_obj, geometry_msgs::Point * nearest_obj_point)
{
  if (use_pointcloud_) {
    getNearestObstacleByPointCloud(min_dist_to_obj, nearest_obj_point);
  }

  if (use_dynamic_object_) {
    getNearestObstacleByDynamicObject(min_dist_to_obj, nearest_obj_point);
  }
}

void SurroundObstacleCheckerNode::getNearestObstacleByPointCloud(
  double * min_dist_to_obj, geometry_msgs::Point * nearest_obj_point)
{
  // waint to transform pointcloud
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp,
      ros::Duration(0.5));
  } catch (tf2::TransformException & ex) {
    ROS_WARN_STREAM_THROTTLE(
      0.5, "[surround obstacle chekcer] failed to get base_link to "
             << pointcloud_ptr_->header.frame_id << " transform.");
    return;
  }

  Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  sensor_msgs::PointCloud2 transformed_pointcloud;
  pcl_ros::transformPointCloud(affine_matrix, *pointcloud_ptr_, transformed_pointcloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_pointcloud, *pcl);

  for (const auto p : *pcl) {
    //create boost point
    Point2d boost_p(p.x, p.y);

    //calc distance
    const double dist_to_obj = boost::geometry::distance(self_poly_, boost_p);

    //get minimum distance to obj
    if (dist_to_obj < *min_dist_to_obj) {
      *min_dist_to_obj = dist_to_obj;
      nearest_obj_point->x = p.x;
      nearest_obj_point->y = p.y;
      nearest_obj_point->z = p.z;
    }
  }
}

void SurroundObstacleCheckerNode::getNearestObstacleByDynamicObject(
  double * min_dist_to_obj, geometry_msgs::Point * nearest_obj_point)
{
  const auto obj_frame = object_ptr_->header.frame_id;
  const auto obj_time = object_ptr_->header.stamp;
  for (const auto obj : object_ptr_->objects) {
    //change frame of obj_pose to base_link
    geometry_msgs::Pose pose_baselink;
    if (!convertPose(
          obj.state.pose_covariance.pose, obj_frame, "base_link", obj_time, pose_baselink)) {
      return;
    }

    //create obj polygon
    Polygon2d obj_poly;
    if (obj.shape.type == autoware_perception_msgs::Shape::POLYGON) {
      obj_poly = createObjPolygon(pose_baselink, obj.shape.footprint);
    } else {
      obj_poly = createObjPolygon(pose_baselink, obj.shape.dimensions);
    }

    //calc distance
    const double dist_to_obj = boost::geometry::distance(self_poly_, obj_poly);

    //get minimum distance to obj
    if (dist_to_obj < *min_dist_to_obj) {
      *min_dist_to_obj = dist_to_obj;
      *nearest_obj_point = obj.state.pose_covariance.pose.position;
    }
  }
}

bool SurroundObstacleCheckerNode::isObstacleFound(const double min_dist_to_obj)
{
  const auto is_obstacle_inside_range = min_dist_to_obj < surround_check_distance_;
  const auto is_obstacle_outside_range = min_dist_to_obj > surround_check_recover_distance_;

  if (state_ == State::PASS) {
    return is_obstacle_inside_range;
  }

  if (state_ == State::STOP) {
    return !is_obstacle_outside_range;
  }

  throw std::runtime_error("invalid state");
}

bool SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_stopped)
{
  if (!is_stopped) {
    return false;
  }

  if (is_obstacle_found) {
    last_obstacle_found_time_ = std::make_shared<const ros::Time>(ros::Time::now());
    return true;
  }

  if (state_ != State::STOP) {
    return false;
  }

  // Keep stop state
  if (last_obstacle_found_time_) {
    const auto elapsed_time = ros::Time::now() - *last_obstacle_found_time_;
    if (elapsed_time.toSec() <= state_clear_time_) {
      return true;
    }
  }

  last_obstacle_found_time_ = {};
  return false;
}

bool SurroundObstacleCheckerNode::checkStop(
  const autoware_planning_msgs::TrajectoryPoint & closest_point)
{
  if (std::fabs(current_velocity_ptr_->twist.linear.x) > stop_state_ego_speed_) {
    //ego vehicle has high velocity now. not stop.
    return false;
  }

  const double closest_vel = closest_point.twist.linear.x;
  const double closest_acc = closest_point.accel.linear.x;

  if (closest_vel * closest_acc < 0) {
    //ego vehicle is about to stop (during deceleration). not stop.
    return false;
  }

  return true;
}

Polygon2d SurroundObstacleCheckerNode::createSelfPolygon()
{
  double front = front_overhang_ + wheel_base_;
  double rear = -rear_overhang_;
  double left = wheel_tread_ / 2.0 + left_overhang_ / 2.0;
  double right = -(wheel_tread_ / 2.0 + right_overhang_ / 2.0);

  Polygon2d poly;
  boost::geometry::exterior_ring(poly) = boost::assign::list_of<Point2d>(front, left)(front, right)(
    rear, right)(rear, left)(front, left);
  return poly;
}

Polygon2d SurroundObstacleCheckerNode::createObjPolygon(
  const geometry_msgs::Pose & pose, const geometry_msgs::Vector3 & size)
{
  //rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = size.x;
  const double w = size.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<Point2d>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

Polygon2d SurroundObstacleCheckerNode::createObjPolygon(
  const geometry_msgs::Pose & pose, const geometry_msgs::Polygon & footprint)
{
  //rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  for (const auto point : footprint.points) {
    const Point2d point2d(point.x, point.y);
    obj_poly.outer().push_back(point2d);
  }

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

diagnostic_msgs::DiagnosticStatus SurroundObstacleCheckerNode::makeStopReasonDiag(
  const std::string no_start_reason, const geometry_msgs::Pose & stop_pose)
{
  diagnostic_msgs::DiagnosticStatus no_start_reason_diag;
  diagnostic_msgs::KeyValue no_start_reason_diag_kv;
  no_start_reason_diag.level = diagnostic_msgs::DiagnosticStatus::OK;
  no_start_reason_diag.name = "no_start_reason";
  no_start_reason_diag.message = no_start_reason;
  no_start_reason_diag_kv.key = "no_start_pose";
  no_start_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  no_start_reason_diag.values.push_back(no_start_reason_diag_kv);
  return no_start_reason_diag;
}

std::string SurroundObstacleCheckerNode::jsonDumpsPose(const geometry_msgs::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}
