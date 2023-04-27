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

#pragma once
#include <Eigen/Core>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <cmath>
#include <random>

namespace yabloc::ekf_corrector
{
extern std::random_device seed_gen;
extern std::default_random_engine engine;

Eigen::Vector2d nrand_2d(const Eigen::Matrix2d & cov);

class NormalDistribution2d
{
public:
  NormalDistribution2d(const Eigen::Matrix2d & cov);
  std::pair<double, Eigen::Vector2d> operator()() const;

private:
  Eigen::Vector2d std_;
  Eigen::Matrix2d rotation_;
};

template <typename T = float>
T nrand(T cov)
{
  std::normal_distribution<T> dist(0.0, std::sqrt(cov));
  return dist(engine);
}

geometry_msgs::msg::PoseWithCovariance debug_debayes_distribution(
  const geometry_msgs::msg::PoseWithCovariance & post,
  const geometry_msgs::msg::PoseWithCovariance & prior);

geometry_msgs::msg::PoseWithCovariance debayes_distribution(
  const geometry_msgs::msg::PoseWithCovariance & post,
  const geometry_msgs::msg::PoseWithCovariance & prior);

struct MeanResult
{
  geometry_msgs::msg::Pose pose_;
  Eigen::Matrix3f cov_xyz_;
  float cov_theta_;
};

MeanResult compile_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);

}  // namespace yabloc::ekf_corrector