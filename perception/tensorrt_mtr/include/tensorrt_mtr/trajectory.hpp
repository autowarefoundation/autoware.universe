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

  static const size_t Dim = PredictedStateDim;

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
  : score(score), NumFuture(num_future)
  {
    for (size_t t = 0; t < NumFuture; ++t) {
      const auto start_ptr = waypoints + t * StateDim;
      std::vector<float> state(start_ptr, start_ptr + StateDim);
      waypoints_.emplace_back(state.data());
    }
  }

  static const size_t StateDim = PredictedStateDim;

  float score;
  size_t NumFuture;

  /**
   * @brief Get the waypoints.
   *
   * @return const std::vector<PredictedState>&
   */
  const std::vector<PredictedState> & getWaypoints() const { return waypoints_; }

private:
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
  : NumMode(num_mode), NumFuture(num_future)
  {
    for (size_t m = 0; m < NumMode; ++m) {
      const auto score = *(scores + m);
      const auto start_ptr = trajectories + m * NumFuture * StateDim;
      std::vector<float> waypoints(start_ptr, start_ptr + NumFuture * StateDim);
      modes_.emplace_back(score, waypoints.data(), NumFuture);
    }

    // sort by score
    sortByScore();
  }

  static const size_t StateDim = PredictedStateDim;

  const size_t NumMode;
  const size_t NumFuture;

  /**
   * @brief Return predicted modes. Modes are sorted in descending order based on their scores.
   *
   * @return const std::vector<PredictedMode>&
   */
  const std::vector<PredictedMode> & getModes() const { return modes_; }

private:
  /**
   * @brief Sort modes in descending order based on their scores.
   */
  void sortByScore()
  {
    std::sort(modes_.begin(), modes_.end(), [](const auto & mode1, const auto & mode2) {
      return mode1.score > mode2.score;
    });
  }

  std::vector<PredictedMode> modes_;
};  // struct PredictedTrajectory
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__TRAJECTORY_HPP_
