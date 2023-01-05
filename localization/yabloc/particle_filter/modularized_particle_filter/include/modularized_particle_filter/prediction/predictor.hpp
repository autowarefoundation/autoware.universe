#ifndef MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_

#include "modularized_particle_filter/common/visualize.hpp"
#include "modularized_particle_filter/prediction/init_area.hpp"
#include "modularized_particle_filter/prediction/resampler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <optional>

namespace pcdless::modularized_particle_filter
{
class Predictor : public rclcpp::Node
{
public:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistCovStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using OptParticleArray = std::optional<ParticleArray>;

  Predictor();

private:
  // Subscriber
  rclcpp::Subscription<PoseCovStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<TwistCovStamped>::SharedPtr twist_cov_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<ParticleArray>::SharedPtr particles_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_init_area_;

  // Publisher
  rclcpp::Publisher<ParticleArray>::SharedPtr predicted_particles_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  const bool visualize_;
  const int number_of_particles_;
  const float resampling_interval_seconds_;
  const float static_linear_covariance_;
  const float static_angular_covariance_;
  const bool use_dynamic_noise_;
  float ground_height_;

  std::shared_ptr<ParticleVisualizer> visualizer_{nullptr};
  std::shared_ptr<RetroactiveResampler> resampler_ptr_{nullptr};
  std::optional<ParticleArray> particle_array_opt_{std::nullopt};
  std::optional<TwistCovStamped> twist_opt_{std::nullopt};
  std::optional<InitArea> init_area_;

  // Callback
  void on_gnss_pose(const PoseStamped::ConstSharedPtr initialpose);
  void on_initial_pose(const PoseCovStamped::ConstSharedPtr initialpose);
  void on_twist(const TwistStamped::ConstSharedPtr twist);
  void on_twist_cov(const TwistCovStamped::ConstSharedPtr twist_cov);
  void on_weighted_particles(const ParticleArray::ConstSharedPtr weighted_particles);
  void on_init_area(const PointCloud2 & msg);
  void on_timer();

  void initialize_particles(const PoseCovStamped & initialpose);

  void update_with_static_noise(ParticleArray & particle_array, const TwistCovStamped & twist);
  void update_with_dynamic_noise(ParticleArray & particle_array, const TwistCovStamped & twist);

  void publish_mean_pose(const geometry_msgs::msg::Pose & mean_pose, const rclcpp::Time & stamp);
};

}  // namespace pcdless::modularized_particle_filter
#endif  // MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_
