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

#include "ndt_scan_matcher/tree-structured_parzen_estimator.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

TreeStructuredParzenEstimator::TreeStructuredParzenEstimator(
  const double x_stddev, const double y_stddev, const double z_stddev, const double roll_stddev,
  const double pitch_stddev)
: x_stddev_(x_stddev),
  y_stddev_(y_stddev),
  z_stddev_(z_stddev),
  roll_stddev_(roll_stddev),
  pitch_stddev_(pitch_stddev)
{
}

void TreeStructuredParzenEstimator::add_trial(const Trial & trial)
{
  trials_.push_back(trial);
  std::sort(trials_.begin(), trials_.end(), [](const Trial & lhs, const Trial & rhs) {
    return lhs.score > rhs.score;
  });
}

TreeStructuredParzenEstimator::Input TreeStructuredParzenEstimator::get_next_input()
{
  if (trials_.empty()) {
    // return all 0
    return Input{};
  }
  static std::mt19937 engine(std::random_device{}());

  std::uniform_real_distribution<double> dist_uni(-M_PI, M_PI);
  std::normal_distribution<double> dist_norm(0.0, 1.0);

  Input best_input;
  double best_score = -1e9;
  for (int64_t i = 0; i < 10; i++) {
    Input input{};
    if (trials_.size() < 10) {
      // Only for yaw, use uniform distribution instead of normal distribution
      input.x = dist_norm(engine) * x_stddev_;
      input.y = dist_norm(engine) * y_stddev_;
      input.z = dist_norm(engine) * z_stddev_;
      input.roll = dist_norm(engine) * roll_stddev_;
      input.pitch = dist_norm(engine) * pitch_stddev_;
      input.yaw = dist_uni(engine);

      // fixed roll and pitch in [-pi, pi]
      input.roll = fix_angle(input.roll);
      input.pitch = fix_angle(input.pitch);
    } else {
      const int64_t n = trials_.size();
      const int64_t good_num = kRate * n;
      const int64_t index = engine() % good_num;
      const Input & base = trials_[index].input;
      input.x = base.x + dist_norm(engine) * kCov;
      input.y = base.y + dist_norm(engine) * kCov;
      input.z = base.z + dist_norm(engine) * kCov;
      input.roll = base.roll + dist_norm(engine) * kCov;
      input.pitch = base.pitch + dist_norm(engine) * kCov;
      input.yaw = base.yaw + dist_uni(engine) * kCov;

      // fixed angle
      input.roll = fix_angle(input.roll);
      input.pitch = fix_angle(input.pitch);
      input.yaw = fix_angle(input.yaw);
    }
    const double score = acquisition_function(input);
    if (score > best_score) {
      best_score = score;
      best_input = input;
    }
  }
  return best_input;
}

TreeStructuredParzenEstimator::Trial TreeStructuredParzenEstimator::get_best_trial() const
{
  Trial best_trial;
  best_trial.score = -1e9;
  for (const Trial & trial : trials_) {
    if (trial.score > best_trial.score) {
      best_trial = trial;
    }
  }
  return best_trial;
}

double TreeStructuredParzenEstimator::acquisition_function(const Input & input)
{
  // Until a certain number of trials are accumulated, random
  if (trials_.size() < 10) {
    return 1.0;
  }

  // The upper kRate is good, the rest is bad.
  const int64_t n = trials_.size();
  const int64_t good_num = kRate * n;

  // The upper KDE and the lower KDE are calculated respectively, and the ratio is the score of the
  // acquisition function.
  double upper = 0.0;
  double lower = 0.0;
  const Input sigma{kCov, kCov, kCov, kCov, kCov, kCov};
  for (int64_t i = 0; i < n; i++) {
    const double p = gauss(input, trials_[i].input, sigma);
    if (i < good_num) {
      upper += p;
    } else {
      lower += p;
    }
  }

  const double r = upper / lower;
  return r;
}

double TreeStructuredParzenEstimator::gauss(
  const Input & input, const Input & mu, const Input & sigma)
{
  const double sqrt_2pi = std::sqrt(2 * M_PI);
  double result = 1.0;

  auto f = [&](const double diff, const double sigma) {
    const double exponent = -(diff * diff) / (2 * sigma * sigma);
    return std::exp(exponent) / (sqrt_2pi * sigma);
  };

  const double diff_x = input.x - mu.x;
  result *= f(diff_x, sigma.x);

  const double diff_y = input.y - mu.y;
  result *= f(diff_y, sigma.y);

  const double diff_z = input.z - mu.z;
  result *= f(diff_z, sigma.z);

  const double diff_roll = fix_angle(input.roll - mu.roll);
  result *= f(diff_roll, sigma.roll);

  const double diff_pitch = fix_angle(input.pitch - mu.pitch);
  result *= f(diff_pitch, sigma.pitch);

  const double diff_yaw = fix_angle(input.yaw - mu.yaw);
  result *= f(diff_yaw, sigma.yaw);

  return result;
}

double TreeStructuredParzenEstimator::fix_angle(const double angle)
{
  double fixed_angle = angle;
  while (fixed_angle > M_PI) {
    fixed_angle -= 2 * M_PI;
  }
  while (fixed_angle < -M_PI) {
    fixed_angle += 2 * M_PI;
  }
  return fixed_angle;
}
