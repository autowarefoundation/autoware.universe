// Copyright 2023 Autoware Foundation
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
#include "nerf_based_localizer.hpp"

#include "nerf/stop_watch.hpp"
#include "nerf/utils.hpp"

#include <Eigen/Eigen>
#include <experimental/filesystem>
#include <rclcpp/rclcpp.hpp>

#include <torch/torch.h>

#include <sstream>

geometry_msgs::msg::Pose transform_pose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::Quaterniond R1(
    transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y,
    transform.transform.rotation.z);
  Eigen::Vector3d t1(
    transform.transform.translation.x, transform.transform.translation.y,
    transform.transform.translation.z);

  Eigen::Quaterniond R2(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d t2(pose.position.x, pose.position.y, pose.position.z);

  Eigen::Quaterniond R = R2 * R1;
  Eigen::Vector3d t = R2._transformVector(t1) + t2;

  geometry_msgs::msg::Pose result_pose;
  result_pose.orientation.x = R.x();
  result_pose.orientation.y = R.y();
  result_pose.orientation.z = R.z();
  result_pose.orientation.w = R.w();
  result_pose.position.x = t.x();
  result_pose.position.y = t.y();
  result_pose.position.z = t.z();

  return result_pose;
}

NerfBasedLocalizer::NerfBasedLocalizer(
  const std::string & name_space, const rclcpp::NodeOptions & options)
: Node("nerf_based_localizer", name_space, options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  tf2_broadcaster_(*this),
  map_frame_("map"),
  particle_num_(this->declare_parameter<int>("particle_num")),
  output_covariance_(this->declare_parameter<double>("output_covariance")),
  iteration_num_(this->declare_parameter<int>("iteration_num")),
  learning_rate_(this->declare_parameter<float>("learning_rate")),
  is_activated_(true),
  optimization_mode_(this->declare_parameter<int>("optimization_mode"))
{
  LocalizerParam param;
  param.train_result_dir = this->declare_parameter<std::string>("train_result_dir");
  param.render_pixel_num = this->declare_parameter<int>("render_pixel_num");
  param.noise_position_x = this->declare_parameter<float>("noise_position_x");
  param.noise_position_y = this->declare_parameter<float>("noise_position_y");
  param.noise_position_z = this->declare_parameter<float>("noise_position_z");
  param.noise_rotation_x = this->declare_parameter<float>("noise_rotation_x");
  param.noise_rotation_y = this->declare_parameter<float>("noise_rotation_y");
  param.noise_rotation_z = this->declare_parameter<float>("noise_rotation_z");
  param.resize_factor = this->declare_parameter<int>("resize_factor");
  param.sample_num_per_ray = this->declare_parameter<int>("sample_num_per_ray");
  localizer_ = Localizer(param);

  initial_pose_with_covariance_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/input/pose", 100,
      std::bind(&NerfBasedLocalizer::callback_initial_pose, this, std::placeholders::_1));

  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "~/input/image", rclcpp::SensorDataQoS().keep_last(0),
    std::bind(&NerfBasedLocalizer::callback_image, this, std::placeholders::_1));

  // create publishers
  nerf_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/pose", 10);
  nerf_pose_with_covariance_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/output/pose_with_covariance", 10);
  nerf_score_publisher_ = this->create_publisher<std_msgs::msg::Float32>("~/output/score", 10);
  nerf_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("~/output/image", 10);

  service_ = this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "~/service/optimize_pose",
    std::bind(&NerfBasedLocalizer::service, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "~/service/trigger_node",
    std::bind(
      &NerfBasedLocalizer::service_trigger_node, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  RCLCPP_DEBUG(this->get_logger(), "nerf_based_localizer is created.");
}

void NerfBasedLocalizer::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    const builtin_interfaces::msg::Time & t_front =
      initial_pose_msg_ptr_array_.front()->header.stamp;
    const builtin_interfaces::msg::Time & t_msg = initial_pose_msg_ptr->header.stamp;
    if (t_front.sec > t_msg.sec || (t_front.sec == t_msg.sec && t_front.nanosec > t_msg.nanosec)) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
    if (initial_pose_msg_ptr_array_.size() > 1) {
      initial_pose_msg_ptr_array_.pop_front();
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "initial_pose_with_covariance is not in map frame.");
    std::exit(1);
  }
}

void NerfBasedLocalizer::callback_image(const sensor_msgs::msg::Image::ConstSharedPtr image_msg_ptr)
{
  target_frame_ = image_msg_ptr->header.frame_id;
  image_msg_ptr_array_.push_back(image_msg_ptr);
  if (image_msg_ptr_array_.size() > 1) {
    image_msg_ptr_array_.pop_front();
  }

  if (!is_activated_) {
    RCLCPP_ERROR(this->get_logger(), "NerfBasedLocalizer is not activated in callback_image.");
    return;
  }

  if (initial_pose_msg_ptr_array_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "initial_pose_with_covariance is not received.");
    return;
  }

  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_base_link =
    initial_pose_msg_ptr_array_.back();
  initial_pose_msg_ptr_array_.pop_back();

  // Process
  const auto [pose_msg, image_msg, score_msg] = localize(pose_base_link->pose.pose, *image_msg_ptr);

  // (1) publish nerf_pose
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.frame_id = map_frame_;
  pose_stamped_msg.header.stamp = image_msg_ptr->header.stamp;
  pose_stamped_msg.pose = pose_msg;
  nerf_pose_publisher_->publish(pose_stamped_msg);

  // (2) publish nerf_pose_with_covariance
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg;
  pose_with_cov_msg.header.frame_id = map_frame_;
  pose_with_cov_msg.header.stamp = image_msg_ptr->header.stamp;
  pose_with_cov_msg.pose.pose = pose_msg;
  pose_with_cov_msg.pose.covariance[0] = output_covariance_;
  pose_with_cov_msg.pose.covariance[7] = output_covariance_;
  pose_with_cov_msg.pose.covariance[14] = output_covariance_;
  pose_with_cov_msg.pose.covariance[21] = output_covariance_ * 10;
  pose_with_cov_msg.pose.covariance[28] = output_covariance_ * 10;
  pose_with_cov_msg.pose.covariance[35] = output_covariance_ * 10;
  nerf_pose_with_covariance_publisher_->publish(pose_with_cov_msg);

  // (3) publish score
  nerf_score_publisher_->publish(score_msg);

  // (4) publish image
  nerf_image_publisher_->publish(image_msg);
}

void NerfBasedLocalizer::service(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  RCLCPP_DEBUG(this->get_logger(), "start NerfBasedLocalizer::service");

  if (image_msg_ptr_array_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "image is not received.");
    res->success = false;
    return;
  }

  // Get the oldest image
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg_ptr = image_msg_ptr_array_.back();

  // Process
  const auto [pose_msg, image_msg, score_msg] =
    localize(req->pose_with_covariance.pose.pose, *image_msg_ptr);

  res->success = true;
  res->pose_with_covariance.header.frame_id = map_frame_;
  res->pose_with_covariance.header.stamp = image_msg_ptr->header.stamp;
  res->pose_with_covariance.pose.pose = pose_msg;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;

  RCLCPP_DEBUG(this->get_logger(), "finish NerfBasedLocalizer::service");
}

std::tuple<geometry_msgs::msg::Pose, sensor_msgs::msg::Image, std_msgs::msg::Float32>
NerfBasedLocalizer::localize(
  const geometry_msgs::msg::Pose & pose_msg, const sensor_msgs::msg::Image & image_msg)
{
  Timer timer;
  timer.start();

  // Get data of image_ptr
  // Accessing header information
  const std_msgs::msg::Header header = image_msg.header;

  // Accessing image properties
  const uint32_t width = image_msg.width;
  const uint32_t height = image_msg.height;

  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "Image received. width: " << width << ", height: " << height);

  // Accessing image data
  torch::Tensor image_tensor = torch::tensor(image_msg.data);
  image_tensor = image_tensor.view({height, width, 3});
  image_tensor = image_tensor.index({Slc(0, 850)});
  image_tensor = image_tensor.to(torch::kFloat32);
  image_tensor /= 255.0;
  image_tensor = image_tensor.flip(2);  // BGR to RGB
  image_tensor =
    utils::resize_image(image_tensor, localizer_.infer_height(), localizer_.infer_width());
  image_tensor = image_tensor.to(torch::kCUDA);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_camera;
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform("base_link", target_frame_, tf2::TimePointZero);
    pose_camera.pose.pose = transform_pose(pose_msg, transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
  }

  const geometry_msgs::msg::Pose pose = pose_camera.pose.pose;

  Eigen::Quaternionf quat_in(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3f rot_in = quat_in.toRotationMatrix();

  torch::Tensor initial_pose = torch::eye(4);
  initial_pose[0][0] = rot_in(0, 0);
  initial_pose[0][1] = rot_in(0, 1);
  initial_pose[0][2] = rot_in(0, 2);
  initial_pose[0][3] = pose.position.x;
  initial_pose[1][0] = rot_in(1, 0);
  initial_pose[1][1] = rot_in(1, 1);
  initial_pose[1][2] = rot_in(1, 2);
  initial_pose[1][3] = pose.position.y;
  initial_pose[2][0] = rot_in(2, 0);
  initial_pose[2][1] = rot_in(2, 1);
  initial_pose[2][2] = rot_in(2, 2);
  initial_pose[2][3] = pose.position.z;
  initial_pose = initial_pose.to(torch::kCUDA);
  initial_pose = initial_pose.to(torch::kFloat32);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "pose_before:\n" << initial_pose);

  initial_pose = localizer_.camera2nerf(initial_pose);

  // run NeRF
  torch::Tensor optimized_pose;
  std::vector<Particle> particles;

  if (optimization_mode_ == 0) {
    const float noise_coeff = 1.0f;
    particles = localizer_.optimize_pose_by_random_search(
      initial_pose, image_tensor, particle_num_, noise_coeff);
    optimized_pose = Localizer::calc_average_pose(particles);
  } else {
    std::vector<torch::Tensor> optimized_poses = localizer_.optimize_pose_by_differential(
      initial_pose, image_tensor, iteration_num_, learning_rate_);
    optimized_pose = optimized_poses.back();
  }

  torch::Tensor nerf_image = localizer_.render_image(optimized_pose);
  const float score = utils::calc_loss(nerf_image, image_tensor);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "score = " << score);

  optimized_pose = localizer_.nerf2camera(optimized_pose);

  RCLCPP_DEBUG_STREAM(this->get_logger(), "pose_after:\n" << optimized_pose);

  geometry_msgs::msg::Pose result_pose_camera;
  result_pose_camera.position.x = optimized_pose[0][3].item<float>();
  result_pose_camera.position.y = optimized_pose[1][3].item<float>();
  result_pose_camera.position.z = optimized_pose[2][3].item<float>();
  Eigen::Matrix3f rot_out;
  rot_out << optimized_pose[0][0].item<float>(), optimized_pose[0][1].item<float>(),
    optimized_pose[0][2].item<float>(), optimized_pose[1][0].item<float>(),
    optimized_pose[1][1].item<float>(), optimized_pose[1][2].item<float>(),
    optimized_pose[2][0].item<float>(), optimized_pose[2][1].item<float>(),
    optimized_pose[2][2].item<float>();
  Eigen::Quaternionf quat_out(rot_out);
  result_pose_camera.orientation.x = quat_out.x();
  result_pose_camera.orientation.y = quat_out.y();
  result_pose_camera.orientation.z = quat_out.z();
  result_pose_camera.orientation.w = quat_out.w();

  geometry_msgs::msg::Pose result_pose_base_link;
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform(target_frame_, "base_link", tf2::TimePointZero);
    result_pose_base_link = transform_pose(result_pose_camera, transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
  }

  nerf_image = nerf_image * 255;
  nerf_image = nerf_image.to(torch::kUInt8);
  nerf_image = nerf_image.to(torch::kCPU);
  nerf_image = nerf_image.contiguous();
  sensor_msgs::msg::Image nerf_image_msg;
  nerf_image_msg.header = header;
  nerf_image_msg.width = nerf_image.size(1);
  nerf_image_msg.height = nerf_image.size(0);
  nerf_image_msg.step = nerf_image.size(1) * 3;
  nerf_image_msg.encoding = "rgb8";
  nerf_image_msg.data.resize(nerf_image.numel());
  std::copy(
    nerf_image.data_ptr<uint8_t>(), nerf_image.data_ptr<uint8_t>() + nerf_image.numel(),
    nerf_image_msg.data.begin());

  std_msgs::msg::Float32 score_msg;
  score_msg.data = score;

  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = result_pose_base_link.position.x;
  transform.transform.translation.y = result_pose_base_link.position.y;
  transform.transform.translation.z = result_pose_base_link.position.z;
  transform.transform.rotation = result_pose_base_link.orientation;
  transform.header = header;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = "nerf_base_link";
  tf2_broadcaster_.sendTransform(transform);

  RCLCPP_DEBUG_STREAM(get_logger(), "localize time: " << timer.elapsed_milli_seconds());

  return std::make_tuple(result_pose_base_link, nerf_image_msg, score_msg);
}

void NerfBasedLocalizer::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  RCLCPP_DEBUG_STREAM(
    this->get_logger(), "service_trigger " << req->data << " is arrived to NerfBasedLocalizer.");

  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_msg_ptr_array_.clear();
    image_msg_ptr_array_.clear();
  }
  res->success = true;
}
