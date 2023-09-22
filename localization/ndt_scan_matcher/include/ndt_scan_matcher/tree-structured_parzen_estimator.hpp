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

#ifndef NDT_SCAN_MATCHER__TREE - STRUCTURED_PARZEN_ESTIMATOR_HPP_
#define NDT_SCAN_MATCHER__TREE -STRUCTURED_PARZEN_ESTIMATOR_HPP_

/*
A implementation of tree-structured parzen estimator
Search num_variables double variables in (-1, 1)
*/

#include <cstdint>
#include <random>
#include <vector>

class TreeStructuredParzenEstimator
{
public:
  struct Input
  {
    double x, y, z, roll, pitch, yaw;
  };
  using Score = double;
  struct Trial
  {
    Input input;
    Score score;
    int64_t id;
  };

  TreeStructuredParzenEstimator() = delete;
  TreeStructuredParzenEstimator(
    const int64_t n_startup_trials, const double x_stddev, const double y_stddev,
    const double z_stddev, const double roll_stddev, const double pitch_stddev);
  void add_trial(const Trial & trial);
  Input get_next_input();

private:
  static constexpr double MAX_GOOD_RATE = 0.10;
  static constexpr double BASE_STDDEV_COEFF = 0.25;
  static constexpr double TARGET_SCORE = 6.0;
  static constexpr double MIN_GOOD_SCORE = 1.8;
  static constexpr int64_t N_EI_CANDIDATES = 100;

  double acquisition_function(const Input & input);
  double gauss(const Input & input, const Input & mu, const Input & sigma);
  double fix_angle(const double angle);

  std::vector<Trial> trials_;
  int64_t good_num_;
  const int64_t n_startup_trials_;
  const double x_stddev_;
  const double y_stddev_;
  const double z_stddev_;
  const double roll_stddev_;
  const double pitch_stddev_;

  // Only for yaw, uniform distribution instead of normal distribution is used as initial
  // distribution. So yaw_stddev value is not set from the constructor. However, the basic
  // stddev is needed for kernel density estimation, so it is given as a fixed value.
  // The value is determined empirically to 30 degrees.
  const double yaw_stddev_ = M_PI / 6.0;
};

#endif  // NDT_SCAN_MATCHER__TREE-STRUCTURED_PARZEN_ESTIMATOR_HPP_
