// Copyright 2023 TIER IV, Inc.
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

#include "modularized_particle_filter/prediction/predictor.hpp"

#include "modularized_particle_filter/common/mean.hpp"
#include "modularized_particle_filter/common/prediction_util.hpp"
#include "modularized_particle_filter/prediction/resampler.hpp"

#include <Eigen/Core>
#include <sophus/geometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <iostream>
#include <numeric>

namespace pcdless::modularized_particle_filter
{
Predictor::Predictor()
: Node("predictor"),
  visualize_(declare_parameter<bool>("visualize", false)),
  number_of_particles_(declare_parameter("num_of_particles", 500)),
  resampling_interval_seconds_(declare_parameter("resampling_interval_seconds", 1.0f)),
  static_linear_covariance_(declare_parameter("static_linear_covariance", 0.01)),
  static_angular_covariance_(declare_parameter("static_angular_covariance", 0.01)),
  use_dynamic_noise_(declare_parameter("use_dynamic_noise", false))
{
  const double prediction_rate{declare_parameter("prediction_rate", 50.0f)};

  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Publishers
  predicted_particles_pub_ = create_publisher<ParticleArray>("predicted_particles", 10);
  pose_pub_ = create_publisher<PoseStamped>("pose", 10);
  pose_cov_pub_ = create_publisher<PoseCovStamped>("pose_with_covariance", 10);

  // Subscribers
  using std::placeholders::_1;
  auto on_gnss_pose = std::bind(&Predictor::on_gnss_pose, this, _1);
  auto initial_cb = std::bind(&Predictor::on_initial_pose, this, _1);
  auto twist_cb = std::bind(&Predictor::on_twist, this, _1);
  auto twist_cov_cb = std::bind(&Predictor::on_twist_cov, this, _1);
  auto particle_cb = std::bind(&Predictor::on_weighted_particles, this, _1);
  auto height_cb = [this](std_msgs::msg::Float32 m) -> void { this->ground_height_ = m.data; };

  gnss_sub_ = create_subscription<PoseStamped>("initial_gnss_pose", 1, on_gnss_pose);
  initialpose_sub_ = create_subscription<PoseCovStamped>("initialpose", 1, initial_cb);
  twist_sub_ = create_subscription<TwistStamped>("twist", 10, twist_cb);
  twist_cov_sub_ = create_subscription<TwistCovStamped>("twist_cov", 10, twist_cov_cb);
  particles_sub_ = create_subscription<ParticleArray>("weighted_particles", 10, particle_cb);
  height_sub_ = create_subscription<std_msgs::msg::Float32>("height", 10, height_cb);

  if (visualize_) visualizer_ = std::make_shared<ParticleVisualizer>(*this);

  if (declare_parameter("is_swap_mode", false)) {
    swap_mode_adaptor_ = std::make_unique<SwapModeAdaptor>(this);
  }

  // Timer callback
  auto cb_timer = std::bind(&Predictor::on_timer, this);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Rate(prediction_rate).period(), std::move(cb_timer));
}

void Predictor::on_gnss_pose(const PoseStamped::ConstSharedPtr pose)
{
  PoseCovStamped pose_cov;
  pose_cov.header = pose->header;
  pose_cov.pose.pose = pose->pose;
  pose_cov.pose.covariance[6 * 0 + 0] = 0.25;
  pose_cov.pose.covariance[6 * 1 + 1] = 0.25;
  pose_cov.pose.covariance[6 * 5 + 5] = 0.04;

  if (particle_array_opt_.has_value()) return;

  initialize_particles(pose_cov);
}

void Predictor::on_initial_pose(const PoseCovStamped::ConstSharedPtr initialpose)
{
  initialize_particles(*initialpose);
}

void Predictor::initialize_particles(const PoseCovStamped & initialpose)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "initialize_particles");
  modularized_particle_filter_msgs::msg::ParticleArray particle_array{};
  particle_array.header = initialpose.header;
  particle_array.id = 0;
  particle_array.particles.resize(number_of_particles_);

  Eigen::Matrix2d cov;
  cov(0, 0) = initialpose.pose.covariance[6 * 0 + 0];
  cov(0, 1) = initialpose.pose.covariance[6 * 0 + 1];
  cov(1, 0) = initialpose.pose.covariance[6 * 1 + 0];
  cov(1, 1) = initialpose.pose.covariance[6 * 1 + 1];

  const float yaw{static_cast<float>(tf2::getYaw(initialpose.pose.pose.orientation))};
  for (size_t i{0}; i < particle_array.particles.size(); i++) {
    geometry_msgs::msg::Pose pose{initialpose.pose.pose};

    Eigen::Vector2d noise = util::nrand_2d(cov);
    pose.position.x += noise.x();
    pose.position.y += noise.y();

    float noised_yaw =
      util::normalize_radian(yaw + util::nrand(sqrt(initialpose.pose.covariance[6 * 5 + 5])));
    tf2::Quaternion q;
    q.setRPY(0, 0, noised_yaw);
    pose.orientation = tf2::toMsg(q);

    particle_array.particles[i].pose = pose;
    particle_array.particles[i].weight = 1.0;
  }
  particle_array_opt_ = particle_array;

  // We have to initialize resampler every particles initialization,
  // because resampler has particles resampling history and it will be outdate.
  resampler_ptr_ =
    std::make_shared<RetroactiveResampler>(resampling_interval_seconds_, number_of_particles_, 100);
}

void Predictor::on_twist(const TwistStamped::ConstSharedPtr twist)
{
  TwistCovStamped twist_covariance;
  twist_covariance.header = twist->header;
  twist_covariance.twist.twist = twist->twist;
  twist_covariance.twist.covariance.at(0) = static_linear_covariance_;
  twist_covariance.twist.covariance.at(7) = 1e4;
  twist_covariance.twist.covariance.at(14) = 1e4;
  twist_covariance.twist.covariance.at(21) = 1e4;
  twist_covariance.twist.covariance.at(28) = 1e4;
  twist_covariance.twist.covariance.at(35) = static_angular_covariance_;
  twist_opt_ = twist_covariance;
}

void Predictor::on_twist_cov(const TwistCovStamped::ConstSharedPtr twist_cov)
{
  twist_opt_ = *twist_cov;
}

geometry_msgs::msg::Pose se3_to_pose_msg(const Sophus::SE3f & se3)
{
  const Eigen::Vector3f t = se3.translation();
  const Eigen::Quaternionf q = se3.unit_quaternion();

  geometry_msgs::msg::Pose pose;
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}

Sophus::SE3f pose_msg_to_se3(const geometry_msgs::msg::Pose & pose)
{
  auto pos = pose.position;
  auto ori = pose.orientation;

  Eigen::Vector3f t(pos.x, pos.y, pos.z);
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  return Sophus::SE3f(q, t);
}

void Predictor::update_with_dynamic_noise(
  ParticleArray & particle_array, const TwistCovStamped & twist)
{
  rclcpp::Time current_time{this->now()};
  rclcpp::Time msg_time{particle_array.header.stamp};

  const float dt = static_cast<float>((current_time - msg_time).seconds());
  particle_array.header.stamp = current_time;

  if (dt < 0.0f || dt > 1.0f) {
    // NOTE: sometime particle_array.header.stamp is ancient due to lagged pose_initializer
    RCLCPP_WARN_STREAM(get_logger(), "time stamp is wrong? " << dt);
    return;
  }

  const float linear_x = twist.twist.twist.linear.x;
  const float angular_z = twist.twist.twist.angular.z;
  const float std_linear_x = std::sqrt(twist.twist.covariance[6 * 0 + 0]);
  const float std_angular_z = std::sqrt(twist.twist.covariance[6 * 5 + 5]);

  const float truncated_gain = std::clamp(std::sqrt(std::abs(linear_x)), 0.1f, 1.0f);
  const float truncated_linear_std = std::clamp(std_linear_x * linear_x, 0.1f, 2.0f);

  using util::nrand;
  for (size_t i{0}; i < particle_array.particles.size(); i++) {
    Sophus::SE3f se3_pose = pose_msg_to_se3(particle_array.particles[i].pose);
    Eigen::Matrix<float, 6, 1> noised_xi;
    noised_xi.setZero();
    noised_xi(0) = linear_x + nrand(truncated_linear_std);
    noised_xi(5) = angular_z + nrand(std_angular_z * truncated_gain);
    se3_pose *= Sophus::SE3f::exp(noised_xi * dt);

    geometry_msgs::msg::Pose pose = se3_to_pose_msg(se3_pose);
    pose.position.z = ground_height_;
    particle_array.particles[i].pose = pose;
  }
}

void Predictor::update_with_static_noise(
  ParticleArray & particle_array, const TwistCovStamped & twist)
{
  rclcpp::Time current_time{this->now()};
  const float dt =
    static_cast<float>((current_time - rclcpp::Time(particle_array.header.stamp)).seconds());
  if (dt < 0.0f) {
    RCLCPP_WARN_STREAM(get_logger(), "time stamp is wrong? " << dt);
    return;
  }

  particle_array.header.stamp = current_time;
  for (size_t i{0}; i < particle_array.particles.size(); i++) {
    geometry_msgs::msg::Pose pose{particle_array.particles[i].pose};

    const float stddev_vx = static_cast<float>(std::sqrt(twist.twist.covariance[6 * 0 + 0]));
    const float stddev_wz = static_cast<float>(std::sqrt(twist.twist.covariance[6 * 5 + 5]));

    const float roll{0.0f};
    const float pitch{0.0f};
    const float yaw{static_cast<float>(tf2::getYaw(pose.orientation))};
    const float vx{static_cast<float>(twist.twist.twist.linear.x) + util::nrand(stddev_vx)};
    const float wz{static_cast<float>(twist.twist.twist.angular.z) + util::nrand(stddev_wz)};
    tf2::Quaternion q;
    q.setRPY(roll, pitch, util::normalize_radian(yaw + wz * dt));

    pose.orientation = tf2::toMsg(q);
    pose.position.x += vx * std::cos(yaw) * dt;
    pose.position.y += vx * std::sin(yaw) * dt;
    pose.position.z = ground_height_;

    particle_array.particles[i].pose = pose;
  }
}

void Predictor::on_timer()
{
  // TODO: Refactor
  if (swap_mode_adaptor_) {
    if (swap_mode_adaptor_->should_keep_update() == false) {
      return;
    }
    if (swap_mode_adaptor_->should_call_initialize()) {
      initialize_particles(swap_mode_adaptor_->init_pose());
    }
  }

  if (!particle_array_opt_.has_value()) return;
  if (!twist_opt_.has_value()) return;

  ParticleArray particle_array = particle_array_opt_.value();
  TwistCovStamped twist{twist_opt_.value()};

  if (use_dynamic_noise_) {
    update_with_dynamic_noise(particle_array, twist);
  } else {
    update_with_static_noise(particle_array, twist);
  }

  predicted_particles_pub_->publish(particle_array);

  rclcpp::Time current_time{this->now()};
  geometry_msgs::msg::Pose meaned_pose{mean_pose(particle_array)};
  publish_mean_pose(meaned_pose, current_time);

  if (visualize_) visualizer_->publish(particle_array);

  particle_array_opt_ = particle_array;
}

void Predictor::on_weighted_particles(const ParticleArray::ConstSharedPtr weighted_particles_ptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "on_weighted_particle start");

  // Since the weighted_particles is generated from messages published from this node,
  // the particle_array must have an entity in this function.
  // Therefore, we need not to check particle_array_opt.has_value().
  ParticleArray particle_array{particle_array_opt_.value()};

  OptParticleArray retroactive_weighted_particles{
    resampler_ptr_->add_weight_retroactively(particle_array, *weighted_particles_ptr)};
  if (retroactive_weighted_particles.has_value()) {
    // TODO: Why do you copy only particles. Why do not copy all members including header and id
    particle_array.particles = retroactive_weighted_particles.value().particles;
  }

  OptParticleArray resampled_particles{resampler_ptr_->resample(particle_array)};
  if (resampled_particles.has_value()) {
    particle_array = resampled_particles.value();
  }
  particle_array_opt_ = particle_array;
}

void Predictor::publish_mean_pose(
  const geometry_msgs::msg::Pose & mean_pose, const rclcpp::Time & stamp)
{
  // Publish pose
  {
    PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = mean_pose;
    pose_pub_->publish(pose_stamped);
  }

  // Publish pose with covariance
  {
    // TODO: Use particle distribution
    PoseCovStamped pose_cov_stamped;
    pose_cov_stamped.header.stamp = stamp;
    pose_cov_stamped.header.frame_id = "map";
    pose_cov_stamped.pose.pose = mean_pose;
    pose_cov_stamped.pose.covariance[6 * 0 + 0] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 1 + 1] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 2 + 2] = 0.255;
    pose_cov_stamped.pose.covariance[6 * 3 + 3] = 0.00625;
    pose_cov_stamped.pose.covariance[6 * 4 + 4] = 0.00625;
    pose_cov_stamped.pose.covariance[6 * 5 + 5] = 0.00625;

    pose_cov_pub_->publish(pose_cov_stamped);
  }

  // Publish TF
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = particle_array_opt_->header.stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "particle_filter";
    transform.transform.translation.x = mean_pose.position.x;
    transform.transform.translation.y = mean_pose.position.y;
    transform.transform.translation.z = mean_pose.position.z;
    transform.transform.rotation = mean_pose.orientation;
    tf2_broadcaster_->sendTransform(transform);
  }
}

Predictor::SwapModeAdaptor::SwapModeAdaptor(rclcpp::Node * node)
{
  auto on_ekf_pose = [this](const PoseCovStamped & pose) -> void { init_pose_opt_ = pose; };
  auto on_image = [this](const Image & msg) -> void {
    stamp_opt_ = rclcpp::Time(msg.header.stamp);
  };

  sub_image_ = node->create_subscription<Image>("image", 1, on_image);
  sub_pose_ = node->create_subscription<PoseCovStamped>("pose_cov", 1, on_ekf_pose);
  clock_ = node->get_clock();

  state_is_active = false;
  state_is_activating = false;
}

bool Predictor::SwapModeAdaptor::should_keep_update()
{
  if (!stamp_opt_.has_value()) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("swap_adaptor"), "system should stop");
    state_is_active = false;
    return false;
  }

  double dt = (clock_->now() - stamp_opt_.value()).seconds();
  if (dt > 3) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("swap_adaptor"), "system should stop");
    state_is_active = false;
    return false;
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("swap_adaptor"), "system should keep estimation");
    state_is_activating = true;
    return true;
  }
}

bool Predictor::SwapModeAdaptor::should_call_initialize()
{
  if (state_is_activating && init_pose_opt_.has_value()) {
    if (!state_is_active) {
      state_is_active = true;
      state_is_activating = false;
      return true;
    }
  }
  return false;
}

}  // namespace pcdless::modularized_particle_filter