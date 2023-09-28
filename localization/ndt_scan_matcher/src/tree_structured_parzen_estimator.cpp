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

#include "ndt_scan_matcher/tree_structured_parzen_estimator.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

TreeStructuredParzenEstimator::TreeStructuredParzenEstimator(
  const int64_t n_startup_trials, const double x_stddev, const double y_stddev,
  const double z_stddev, const double roll_stddev, const double pitch_stddev)
: engine_(std::random_device{}()),
  dist_uniform_(-M_PI, M_PI),
  dist_normal_(0.0, 1.0),
  good_num_(0),
  n_startup_trials_(n_startup_trials),
  x_stddev_(x_stddev),
  y_stddev_(y_stddev),
  z_stddev_(z_stddev),
  roll_stddev_(roll_stddev),
  pitch_stddev_(pitch_stddev),
  base_stddev_({x_stddev_, y_stddev_, z_stddev_, roll_stddev_, pitch_stddev_, yaw_stddev_})
{
}

void TreeStructuredParzenEstimator::add_trial(const Trial & trial)
{
  trials_.push_back(trial);
  std::sort(trials_.begin(), trials_.end(), [](const Trial & lhs, const Trial & rhs) {
    return lhs.score > rhs.score;
  });
  good_num_ = std::count_if(trials_.begin(), trials_.end(), [this](const Trial & trial) {
    return trial.score > MIN_GOOD_SCORE;
  });
  good_num_ = std::min(good_num_, static_cast<int64_t>(trials_.size() * MAX_GOOD_RATE));
}

TreeStructuredParzenEstimator::Input TreeStructuredParzenEstimator::get_next_input()
{
  if (trials_.empty()) {
    // The first attempt returns all zeros.
    // This means that the input `base_pose` is used as is.
    // Do this because the input `base_pose` may be a good candidate.
    return Input{};
  } else if (static_cast<int64_t>(trials_.size()) < n_startup_trials_) {
    // Random sampling based on prior until the number of trials reaches `n_startup_trials_`.
    return sample_from_prior();
  }

  std::vector<double> scores;
  for (int64_t i = 0; i < good_num_; i++) {
    const double diff = trials_[i].score - MIN_GOOD_SCORE;
    scores.push_back(std::exp(diff));
  }
  constexpr double PRIOR_WEIGHT = 1.0;
  scores.push_back(PRIOR_WEIGHT);
  std::discrete_distribution<int64_t> dist_good(scores.begin(), scores.end());

  Input best_input;
  double best_score = -1e9;
  for (int64_t i = 0; i < N_EI_CANDIDATES; i++) {
    Input input{};
    const int64_t index = dist_good(engine_);
    if (index == good_num_) {
      input = sample_from_prior();
    } else {
      const Input & base = trials_[index].input;

      // Linear interpolation so that it becomes 1.0 or more at TARGET_SCORE and 4.0 at
      // MIN_GOOD_SCORE.
      const double score_diff = std::max(TARGET_SCORE - trials_[index].score, 0.0);
      const double linear = 1.0 + (score_diff / (TARGET_SCORE - MIN_GOOD_SCORE)) * 3.0;
      const double coeff = BASE_STDDEV_COEFF * linear / 20;
      input.x = base.x + dist_normal_(engine_) * coeff * x_stddev_;
      input.y = base.y + dist_normal_(engine_) * coeff * y_stddev_;
      input.z = base.z + dist_normal_(engine_) * coeff * z_stddev_;
      input.roll = base.roll + dist_normal_(engine_) * coeff * roll_stddev_;
      input.pitch = base.pitch + dist_normal_(engine_) * coeff * pitch_stddev_;
      input.yaw = base.yaw + dist_normal_(engine_) * coeff * yaw_stddev_;

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

double TreeStructuredParzenEstimator::acquisition_function(const Input & input)
{
  const int64_t n = trials_.size();

  // The upper KDE and the lower KDE are calculated respectively, and the ratio is the score of the
  // acquisition function.
  std::vector<double> upper_logs;
  std::vector<double> lower_logs;

  // Scott's rule
  const double coeff_upper = BASE_STDDEV_COEFF * std::pow(good_num_, -1.0 / 10);
  const double coeff_lower = BASE_STDDEV_COEFF * std::pow(n - good_num_, -1.0 / 10);
  const Input sigma_upper = base_stddev_ * coeff_upper;
  const Input sigma_lower = base_stddev_ * coeff_lower;

  std::vector<double> weights;
  constexpr double PRIOR_WEIGHT = 1.0;
  double upper_sum = PRIOR_WEIGHT;
  double lower_sum = 0.0;
  for (int64_t i = 0; i < n; i++) {
    const double v = trials_[i].score - MIN_GOOD_SCORE;
    const double w = std::min(std::abs(v), 1.0);
    if (i < good_num_) {
      weights.push_back(w);
      upper_sum += w;
    } else {
      weights.push_back(w);
      lower_sum += w;
    }
  }

  for (int64_t i = 0; i < n; i++) {
    if (i < good_num_) {
      const double log_p = log_gaussian_pdf(input, trials_[i].input, sigma_upper);
      const double w = weights[i] / upper_sum;
      const double log_w = std::log(w);
      upper_logs.push_back(log_p + log_w);
    } else {
      const double log_p = log_gaussian_pdf(input, trials_[i].input, sigma_lower);
      const double w = weights[i] / lower_sum;
      const double log_w = std::log(w);
      lower_logs.push_back(log_p + log_w);
    }
  }

  // prior
  // Only yaw has a large variance in order to have a pseudo-uniform distribution.
  const Input zeros{};
  const Input sigma{x_stddev_, y_stddev_, z_stddev_, roll_stddev_, pitch_stddev_, 10.0};
  const double log_p = log_gaussian_pdf(input, zeros, sigma);
  const double log_w = std::log(PRIOR_WEIGHT / upper_sum);
  upper_logs.push_back(log_p + log_w);

  auto log_sum_exp = [](const std::vector<double> & log_vec) {
    const double max = *std::max_element(log_vec.begin(), log_vec.end());
    double sum = 0.0;
    for (const double log_v : log_vec) {
      sum += std::exp(log_v - max);
    }
    return max + std::log(sum);
  };

  const double upper = log_sum_exp(upper_logs);
  const double lower = log_sum_exp(lower_logs);
  const double r = upper - lower;
  return r;
}

double TreeStructuredParzenEstimator::log_gaussian_pdf(
  const Input & input, const Input & mu, const Input & sigma)
{
  const double log_2pi = std::log(2.0 * M_PI);
  auto log_gaussian_pdf_1d = [&](const double diff, const double sigma) {
    return -0.5 * log_2pi - std::log(sigma) - (diff * diff) / (2.0 * sigma * sigma);
  };

  double result = 0.0;

  const double diff_x = input.x - mu.x;
  result += log_gaussian_pdf_1d(diff_x, sigma.x);

  const double diff_y = input.y - mu.y;
  result += log_gaussian_pdf_1d(diff_y, sigma.y);

  const double diff_z = input.z - mu.z;
  result += log_gaussian_pdf_1d(diff_z, sigma.z);

  const double diff_roll = fix_angle(input.roll - mu.roll);
  result += log_gaussian_pdf_1d(diff_roll, sigma.roll);

  const double diff_pitch = fix_angle(input.pitch - mu.pitch);
  result += log_gaussian_pdf_1d(diff_pitch, sigma.pitch);

  const double diff_yaw = fix_angle(input.yaw - mu.yaw);
  result += log_gaussian_pdf_1d(diff_yaw, sigma.yaw);

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

TreeStructuredParzenEstimator::Input TreeStructuredParzenEstimator::sample_from_prior()
{
  Input input{};
  // Only for yaw, use uniform distribution instead of normal distribution
  input.x = dist_normal_(engine_) * x_stddev_;
  input.y = dist_normal_(engine_) * y_stddev_;
  input.z = dist_normal_(engine_) * z_stddev_;
  input.roll = dist_normal_(engine_) * roll_stddev_;
  input.pitch = dist_normal_(engine_) * pitch_stddev_;
  input.yaw = dist_uniform_(engine_);

  // fixed roll and pitch in [-pi, pi]
  input.roll = fix_angle(input.roll);
  input.pitch = fix_angle(input.pitch);
  return input;
}
