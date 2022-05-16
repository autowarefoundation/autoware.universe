#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/circular_buffer.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <optional>

class CameraPoseCorrector : public rclcpp::Node
{
public:
  CameraPoseCorrector();

private:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  // Publisher
  rclcpp::Publisher<ParticleArray>::SharedPtr weighted_particle_pub_;

  rclcpp::Time prev_time_;
  int particles_buffer_size_;
  boost::circular_buffer<ParticleArray> particles_circular_buffer_;
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_
