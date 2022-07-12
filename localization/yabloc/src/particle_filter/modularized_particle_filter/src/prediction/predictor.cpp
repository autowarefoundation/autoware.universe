#include "modularized_particle_filter/prediction/predictor.hpp"

#include "modularized_particle_filter/prediction/prediction_util.hpp"
#include "modularized_particle_filter/prediction/resampler.hpp"

#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <memory>
#include <numeric>

Predictor::Predictor()
: Node("predictor"),
  visualize_(declare_parameter<bool>("visualize", false)),
  number_of_particles_(declare_parameter("num_of_particles", 500)),
  resampling_interval_seconds_(declare_parameter("resampling_interval_seconds", 1.0f)),
  static_linear_covariance(declare_parameter("static_linear_covariance", 0.01)),
  static_angular_covariance(declare_parameter("static_angular_covariance", 0.01))
{
  const double prediction_rate{declare_parameter("prediction_rate", 50.0f)};

  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Publishers
  predicted_particles_pub_ = create_publisher<ParticleArray>("predicted_particles", 10);
  pose_pub_ = create_publisher<PoseStamped>("particle_pose", 10);

  // Subscribers
  using std::placeholders::_1;
  auto gnss_cb = std::bind(&Predictor::gnssposeCallback, this, _1);
  auto initial_cb = std::bind(&Predictor::initialposeCallback, this, _1);
  auto twist_cb = std::bind(&Predictor::twistCallback, this, _1);
  auto particle_cb = std::bind(&Predictor::weightedParticlesCallback, this, _1);
  auto height_cb = [this](std_msgs::msg::Float32 m) -> void { this->ground_height_ = m.data; };

  gnss_sub_ = create_subscription<PoseStamped>("pose", 1, gnss_cb);
  initialpose_sub_ = create_subscription<PoseWithCovarianceStamped>("initialpose", 1, initial_cb);
  twist_sub_ = create_subscription<TwistStamped>("twist", 10, twist_cb);
  particles_sub_ = create_subscription<ParticleArray>("weighted_particles", 10, particle_cb);
  height_sub_ = create_subscription<std_msgs::msg::Float32>("height", 10, height_cb);

  if (visualize_) visualizer_ = std::make_shared<ParticleVisualizer>(*this);

  // Timer callback
  auto chrono_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0f / prediction_rate));
  timer_ = this->create_wall_timer(chrono_period, std::bind(&Predictor::timerCallback, this));
}

void Predictor::gnssposeCallback(const PoseStamped::ConstSharedPtr pose)
{
  if (particle_array_opt_.has_value()) return;
  PoseWithCovarianceStamped pose_cov;
  pose_cov.header = pose->header;
  pose_cov.pose.pose = pose->pose;
  pose_cov.pose.covariance[0] = 0.25;
  pose_cov.pose.covariance[6 * 1 + 1] = 0.25;
  pose_cov.pose.covariance[6 * 5 + 1] = 0.04;
  initializeParticles(pose_cov);
}

void Predictor::initialposeCallback(const PoseWithCovarianceStamped::ConstSharedPtr initialpose)
{
  PoseWithCovarianceStamped pose = *initialpose;
  pose.pose.covariance[0] = 0.25;
  pose.pose.covariance[6 * 1 + 1] = 0.25;
  pose.pose.covariance[6 * 5 + 1] = 0.04;
  initializeParticles(pose);
}

void Predictor::initializeParticles(const PoseWithCovarianceStamped & initialpose)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "initiposeCallback");
  modularized_particle_filter_msgs::msg::ParticleArray particle_array{};
  particle_array.header = initialpose.header;
  particle_array.id = 0;
  particle_array.particles.resize(number_of_particles_);

  const float roll{0.0f};
  const float pitch{0.0f};
  const float yaw{static_cast<float>(tf2::getYaw(initialpose.pose.pose.orientation))};
  for (size_t i{0}; i < particle_array.particles.size(); i++) {
    geometry_msgs::msg::Pose pose{initialpose.pose.pose};
    pose.position.x += prediction_util::nrand(std::sqrt(initialpose.pose.covariance[0]));
    pose.position.y += prediction_util::nrand(std::sqrt(initialpose.pose.covariance[6 * 1 + 1]));
    float noised_yaw = prediction_util::normalizeRadian(
      yaw + prediction_util::nrand(sqrt(initialpose.pose.covariance[6 * 5 + 5])));
    tf2::Quaternion q;
    q.setRPY(roll, pitch, noised_yaw);
    pose.orientation = tf2::toMsg(q);

    particle_array.particles[i].pose = pose;
    particle_array.particles[i].weight = 1.0;
  }
  particle_array_opt_ = particle_array;

  resampler_ptr_ =
    std::make_shared<RetroactiveResampler>(resampling_interval_seconds_, number_of_particles_);
}

void Predictor::twistCallback(const TwistStamped::ConstSharedPtr twist)
{
  TwistWithCovarianceStamped twist_covariance;
  twist_covariance.header = twist->header;
  twist_covariance.twist.twist = twist->twist;
  twist_covariance.twist.covariance.at(0) = static_linear_covariance;
  twist_covariance.twist.covariance.at(7) = 1e4;
  twist_covariance.twist.covariance.at(14) = 1e4;
  twist_covariance.twist.covariance.at(21) = 1e4;
  twist_covariance.twist.covariance.at(28) = 1e4;
  twist_covariance.twist.covariance.at(35) = static_angular_covariance;
  twist_opt_ = twist_covariance;
}

void Predictor::timerCallback()
{
  if (!particle_array_opt_.has_value()) return;
  if (!twist_opt_.has_value()) return;

  modularized_particle_filter_msgs::msg::ParticleArray particle_array = particle_array_opt_.value();
  geometry_msgs::msg::TwistWithCovarianceStamped twist{twist_opt_.value()};

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

    const float roll{0.0f};
    const float pitch{0.0f};
    const float yaw{static_cast<float>(tf2::getYaw(pose.orientation))};
    const float vx{
      twist.twist.twist.linear.x +
      prediction_util::nrand(16 * std::sqrt(twist.twist.covariance[0]))};
    const float wz{
      twist.twist.twist.angular.z +
      prediction_util::nrand(1 * std::sqrt(twist.twist.covariance[5 * 6 + 5]))};
    tf2::Quaternion q;
    q.setRPY(roll, pitch, prediction_util::normalizeRadian(yaw + wz * dt));

    pose.orientation = tf2::toMsg(q);
    pose.position.x += vx * std::cos(yaw) * dt;
    pose.position.y += vx * std::sin(yaw) * dt;
    pose.position.z = ground_height_;

    particle_array.particles[i].pose = pose;
  }

  predicted_particles_pub_->publish(particle_array);

  geometry_msgs::msg::Pose mean_pose{calculateMeanPose(particle_array)};
  publishMeanPose(mean_pose, current_time);

  if (visualize_) visualizer_->publish(particle_array);

  particle_array_opt_ = particle_array;
}

void Predictor::weightedParticlesCallback(
  const ParticleArray::ConstSharedPtr weighted_particles_ptr)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "weightedParticleCallback is called");
  modularized_particle_filter_msgs::msg::ParticleArray particle_array{particle_array_opt_.value()};

  std::optional<modularized_particle_filter_msgs::msg::ParticleArray>
    retroactive_weighted_particles{
      resampler_ptr_->retroactiveWeighting(particle_array, weighted_particles_ptr)};
  if (retroactive_weighted_particles.has_value()) {
    particle_array.particles = retroactive_weighted_particles.value().particles;
  }

  std::optional<modularized_particle_filter_msgs::msg::ParticleArray> resampled_particles{
    resampler_ptr_->resampling(particle_array)};
  if (resampled_particles.has_value()) {
    particle_array = resampled_particles.value();
  }

  particle_array_opt_ = particle_array;
}

geometry_msgs::msg::Pose Predictor::calculateMeanPose(const ParticleArray & particle_array)
{
  modularized_particle_filter_msgs::msg::ParticleArray normalized_particle_array{particle_array};
  geometry_msgs::msg::Pose mean_pose;

  auto minmax_weight = std::minmax_element(
    normalized_particle_array.particles.begin(), normalized_particle_array.particles.end(),
    [](auto p1, auto p2) { return p1.weight < p2.weight; });

  const float num_of_particles_inv{1.0f / normalized_particle_array.particles.size()};
  const float dif_weight{minmax_weight.second->weight - minmax_weight.first->weight};

  for (auto & particle : normalized_particle_array.particles) {
    if (dif_weight != 0.0f) {
      particle.weight = (particle.weight - minmax_weight.first->weight) / dif_weight;
    } else {
      particle.weight = num_of_particles_inv;
    }
  }

  double sum_weight{std::accumulate(
    normalized_particle_array.particles.begin(), normalized_particle_array.particles.end(), 0.0,
    [](double weight, modularized_particle_filter_msgs::msg::Particle & particle) {
      return weight + particle.weight;
    })};
  if (std::isinf(sum_weight)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "sum_weight: " << sum_weight);
  }

  std::vector<double> rolls{};
  std::vector<double> pitches{};
  std::vector<double> yaws{};
  std::vector<double> weights{};
  for (modularized_particle_filter_msgs::msg::Particle particle :
       normalized_particle_array.particles) {
    double weight{1.0 / normalized_particle_array.particles.size()};
    if (0.0f < sum_weight) weight = particle.weight / sum_weight;

    mean_pose.position.x += particle.pose.position.x * weight;
    mean_pose.position.y += particle.pose.position.y * weight;
    mean_pose.position.z += particle.pose.position.z * weight;

    double yaw{0.0}, pitch{0.0}, roll{0.0};
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);

    rolls.push_back(roll);
    pitches.push_back(pitch);
    yaws.push_back(yaw);
    weights.push_back(weight);
  }

  const double mean_roll{prediction_util::meanRadian(rolls, weights)};
  const double mean_pitch{prediction_util::meanRadian(pitches, weights)};
  const double mean_yaw{prediction_util::meanRadian(yaws, weights)};
  tf2::Quaternion q;
  q.setRPY(mean_roll, mean_pitch, mean_yaw);
  mean_pose.orientation = tf2::toMsg(q);
  return mean_pose;
}

void Predictor::publishMeanPose(
  const geometry_msgs::msg::Pose & mean_pose, const rclcpp::Time & stamp)
{
  PoseStamped pose_stamped;
  pose_stamped.header.stamp = stamp;
  pose_stamped.header.frame_id = "map";

  pose_stamped.pose = mean_pose;
  pose_pub_->publish(pose_stamped);

  geometry_msgs::msg::TransformStamped transform{};
  transform.header.stamp = particle_array_opt_->header.stamp;
  transform.header.frame_id = "map";
  transform.child_frame_id = "particle_filter";
  transform.transform.translation.x = mean_pose.position.x;
  transform.transform.translation.y = mean_pose.position.y;
  transform.transform.translation.z = mean_pose.position.z;
  transform.transform.rotation = mean_pose.orientation;
  tf2_broadcaster_->sendTransform(transform);
}
