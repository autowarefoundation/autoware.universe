// Copyright 2024 TIER IV, Inc.
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

#ifndef TENSORRT_MTR__TRAJECTORY_HPP_
#define TENSORRT_MTR__TRAJECTORY_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

namespace trt_mtr
{

constexpr size_t PredictedStateDim = 7;

/**
 * @brief A class to represent a predicted state.
 */
struct PredictedState
{
  // TODO(ktro2828): set other values too
  explicit PredictedState(const float * state)
  : x_(state[0]), y_(state[1]), vx_(state[5]), vy_(state[6])
  {
  }

  static size_t dim() { return PredictedStateDim; }

  float x() const { return x_; }
  float y() const { return y_; }
  float vx() const { return vx_; }
  float vy() const { return vy_; }

private:
  float x_, y_, vx_, vy_;
};  // struct PredictedState

/**
 * @brief A class to represent waypoints for a single mode.
 */
struct PredictedMode
{
  PredictedMode(const float score, const float * waypoints, const size_t num_future)
  : score_(score), num_future_(num_future)
  {
    for (size_t t = 0; t < num_future_; ++t) {
      const auto start_ptr = waypoints + t * state_dim();
      std::vector<float> state(start_ptr, start_ptr + state_dim());
      waypoints_.emplace_back(state.data());
    }
  }

  static size_t state_dim() { return PredictedStateDim; }
  float score() const { return score_; }
  size_t num_future() const { return num_future_; }

  /**
   * @brief Get the waypoints.
   *
   * @return const std::vector<PredictedState>&
   */
  const std::vector<PredictedState> & get_waypoints() const { return waypoints_; }

private:
  float score_;
  size_t num_future_;
  std::vector<PredictedState> waypoints_;
};  // struct PredictedMode

/**
 * @brief A class to represent predicted trajectory for a single target agent.
 */
struct PredictedTrajectory
{
  PredictedTrajectory(
    const float * scores, const float * trajectories, const size_t num_mode,
    const size_t num_future)
  : num_mode_(num_mode), num_future_(num_future)
  {
    for (size_t m = 0; m < num_mode_; ++m) {
      const auto score = *(scores + m);
      const auto start_ptr = trajectories + m * num_future_ * state_dim();
      std::vector<float> waypoints(start_ptr, start_ptr + num_future_ * state_dim());
      modes_.emplace_back(score, waypoints.data(), num_future_);
    }

    // sort by score
    sort_by_score();
  }

  static size_t state_dim() { return PredictedStateDim; }
  size_t num_mode() const { return num_mode_; }
  size_t num_future() const { return num_future_; }

  /**
   * @brief Return predicted modes. Modes are sorted in descending order based on their scores.
   *
   * @return const std::vector<PredictedMode>&
   */
  const std::vector<PredictedMode> & get_modes() const { return modes_; }

private:
  /**
   * @brief Sort modes in descending order based on their scores.
   */
  void sort_by_score()
  {
    std::sort(modes_.begin(), modes_.end(), [](const auto & mode1, const auto & mode2) {
      return mode1.score() > mode2.score();
    });
  }

  size_t num_mode_;
  size_t num_future_;
  std::vector<PredictedMode> modes_;
};  // struct PredictedTrajectory
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__TRAJECTORY_HPP_
