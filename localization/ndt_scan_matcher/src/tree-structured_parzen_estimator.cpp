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
  int64_t num_variables, int64_t num_samples)
: num_variables_(num_variables), num_samples_(num_samples)
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
  static std::mt19937 engine(std::random_device{}());

  constexpr double kMin = -1.0 + kEpsilon;
  constexpr double kMax = 1.0 - kEpsilon;
  std::uniform_real_distribution<double> dist_uni(kMin, kMax);
  std::normal_distribution<double> dist_norm(0.0, 1.0);

  Input best_input;
  double best_score = -1e9;
  for (int64_t i = 0; i < 10; i++) {
    Input input(num_variables_);
    if (trials_.size() < 10) {
      for (int64_t j = 0; j < num_variables_; j++) {
        input[j] = dist_uni(engine);
      }
    } else {
      const int64_t n = trials_.size();
      const int64_t good_num = kRate * n;
      const int64_t index = engine() % good_num;
      for (int64_t j = 0; j < num_variables_; j++) {
        input[j] = trials_[index].input[j] + dist_norm(engine) * kCov;
        input[j] = std::clamp(input[j], kMin, kMax);
      }
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
  const Input sigma(input.size(), kCov);
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
  if (input.size() != mu.size() || mu.size() != sigma.size()) {
    // Error handling: dimensions mismatch
    throw std::runtime_error("Input, mu, and sigma must have the same size.");
  }

  const double sqrt_2pi = std::sqrt(2 * M_PI);
  double result = 1.0;

  for (size_t i = 0; i < input.size(); ++i) {
    double diff = input[i] - mu[i];
    double exponent = -(diff * diff) / (2 * sigma[i] * sigma[i]);
    result *= (std::exp(exponent) / (sqrt_2pi * sigma[i]));
  }

  return result;
}
