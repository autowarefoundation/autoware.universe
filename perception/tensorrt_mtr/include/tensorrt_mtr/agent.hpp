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

#ifndef TENSORRT_MTR__AGENT_HPP_
#define TENSORRT_MTR__AGENT_HPP_

#include <array>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace trt_mtr
{
constexpr size_t AgentStateDim = 12;

enum AgentLabel { VEHICLE = 0, PEDESTRIAN = 1, CYCLIST = 2 };

/**
 * @brief A class to represent a single state of an agent.
 */
struct AgentState
{
  /**
   * @brief Construct a new instance filling all elements by `0.0f`.
   */
  AgentState() : data_({0.0f}) {}

  /**
   * @brief Construct a new instance with specified values.
   *
   * @param x X position.
   * @param y Y position.
   * @param z Z position.
   * @param length Length of the bbox.
   * @param width Width of the bbox.
   * @param height Height of the bbox.
   * @param yaw Heading yaw angle [rad].
   * @param vx Velocity heading x direction in object's coordinates system.
   * @param vy Velocity heading y direction in object's coordinates system.
   * @param ax Acceleration heading x direction in object's coordinates system.
   * @param ay Acceleration heading y direction in object's coordinates system.
   * @param is_valid `1.0f` if valid, otherwise `0.0f`.
   */
  AgentState(
    const float x, const float y, const float z, const float length, const float width,
    const float height, const float yaw, const float vx, const float vy, const float ax,
    const float ay, const float is_valid)
  : data_({x, y, z, length, width, height, yaw, vx, vy, ax, ay, is_valid})
  {
  }

  static const size_t Dim = AgentStateDim;

  /**
   * @brief Construct a new instance filling all elements by `0.0f`.
   *
   * @return AgentState
   */
  static AgentState empty() noexcept { return AgentState(); }

  /**
   * @brief Return the address pointer of data array.
   *
   * @return float*
   */
  float * data_ptr() noexcept { return data_.data(); }

private:
  std::array<float, Dim> data_;
};

/**
 * @brief A class to represent the state history of an agent.
 */
struct AgentHistory
{
  /**
   * @brief Construct a new Agent History filling the latest state by input state.
   *
   * @param state Object current state.
   * @param object_id Object ID.
   * @param current_time Current timestamp.
   * @param max_time_length History length.
   */
  AgentHistory(
    AgentState & state, const std::string & object_id, const float current_time,
    const size_t max_time_length)
  : data_((max_time_length - 1) * StateDim),
    object_id_(object_id),
    latest_time_(current_time),
    max_time_length_(max_time_length)
  {
    const auto s_ptr = state.data_ptr();
    for (size_t d = 0; d < StateDim; ++d) {
      data_.push_back(*(s_ptr + d));
    }
  }

  /**
   * @brief Construct a new Agent History filling all elements by zero.
   *
   * @param object_id Object ID.
   * @param max_time_length History time length.
   */
  AgentHistory(const std::string & object_id, const size_t max_time_length)
  : data_(max_time_length * StateDim),
    object_id_(object_id),
    latest_time_(-std::numeric_limits<float>::max()),
    max_time_length_(max_time_length)
  {
  }

  static const size_t StateDim = AgentStateDim;

  /**
   * @brief Returns ID of the object.
   *
   * @return const std::string&
   */
  const std::string & object_id() const { return object_id_; }

  /**
   * @brief Return the history length.
   *
   * @return size_t History length.
   */
  size_t length() const { return max_time_length_; }

  /**
   * @brief Return the last timestamp when non-empty state was pushed.
   *
   * @return float
   */
  float latest_time() const { return latest_time_; }

  /**
   * @brief Update history with input state and latest time.
   *
   * @param current_time The current timestamp.
   * @param state The current agent state.
   */
  void update(const float current_time, AgentState & state) noexcept
  {
    // remove the state at the oldest timestamp
    data_.erase(data_.begin(), data_.begin() + StateDim);

    const auto s = state.data_ptr();
    for (size_t d = 0; d < StateDim; ++d) {
      data_.push_back(*(s + d));
    }
    latest_time_ = current_time;
  }

  /**
   * @brief Update history with all-zeros state, but latest time is not updated.
   *
   */
  void update_empty() noexcept
  {
    // remove the state at the oldest timestamp
    data_.erase(data_.begin(), data_.begin() + StateDim);

    const auto s = AgentState::empty().data_ptr();
    for (size_t d = 0; d < StateDim; ++d) {
      data_.push_back(*(s + d));
    }
  }

  /**
   * @brief Return the address pointer of data array.
   *
   * @return float* The pointer of data array.
   */
  float * data_ptr() noexcept { return data_.data(); }

  /**
   * @brief Check whether the latest valid state is too old or not.
   *
   * @param current_time Current timestamp.
   * @param threshold Time difference threshold value.
   * @return true If the difference is greater than threshold.
   * @return false Otherwise
   */
  bool is_ancient(const float current_time, const float threshold) const
  {
    /* TODO: Raise error if the current time is smaller than latest */
    return current_time - latest_time_ >= threshold;
  }

  /**
   * @brief Check whether the latest state data is valid.
   *
   * @return true If the end of element is 1.0f.
   * @return false Otherwise.
   */
  bool is_valid_latest() const { return data_.at(StateDim * max_time_length_ - 1) == 1.0f; }

private:
  std::vector<float> data_;
  const std::string object_id_;
  float latest_time_;
  const size_t max_time_length_;
};

/**
 * @brief A class containing whole state histories of all agent.
 */
struct AgentData
{
  /**
   * @brief Construct a new instance.
   *
   * @param histories An array of histories for each object.
   * @param sdc_index An index of ego.
   * @param target_index Indices of target agents.
   * @param label_index An array of label indices for each object.
   * @param timestamps An array of timestamps.
   */
  AgentData(
    std::vector<AgentHistory> & histories, const int sdc_index,
    const std::vector<int> & target_index, const std::vector<int> & label_index,
    const std::vector<float> & timestamps)
  : TargetNum(target_index.size()),
    AgentNum(histories.size()),
    TimeLength(timestamps.size()),
    sdc_index(sdc_index),
    target_index(target_index),
    label_index(label_index),
    timestamps(timestamps)
  {
    data_.reserve(AgentNum * TimeLength * StateDim);
    for (auto & history : histories) {
      const auto data_ptr = history.data_ptr();
      for (size_t t = 0; t < TimeLength; ++t) {
        for (size_t d = 0; d < StateDim; ++d) {
          data_.push_back(*(data_ptr + t * StateDim + d));
        }
      }
    }

    target_data_.reserve(TargetNum * StateDim);
    target_label_index.reserve(TargetNum);
    for (const auto & idx : target_index) {
      target_label_index.emplace_back(label_index.at(idx));
      const auto target_ptr = histories.at(idx).data_ptr();
      for (size_t d = 0; d < StateDim; ++d) {
        target_data_.push_back(*(target_ptr + (TimeLength - 1) * StateDim + d));
      }
    }

    ego_data_.reserve(TimeLength * StateDim);
    const auto ego_data_ptr = histories.at(sdc_index).data_ptr();
    for (size_t t = 0; t < TimeLength; ++t) {
      for (size_t d = 0; d < StateDim; ++d) {
        ego_data_.push_back(*(ego_data_ptr + t * StateDim + d));
      }
    }
  }

  const size_t TargetNum;
  const size_t AgentNum;
  const size_t TimeLength;
  const size_t StateDim = AgentStateDim;
  const size_t ClassNum = 3;  // TODO(ktro2828): Do not use magic number.

  int sdc_index;
  std::vector<int> target_index;
  std::vector<int> label_index;
  std::vector<int> target_label_index;
  std::vector<float> timestamps;

  /**
   * @brief Return the data shape.
   *
   * @return std::tuple<size_t, size_t, size_t> (AgentNum, TimeLength, StateDim).
   */
  std::tuple<size_t, size_t, size_t> shape() const { return {AgentNum, TimeLength, StateDim}; }

  /**
   * @brief Return the address pointer of data array.
   *
   * @return float* The pointer of data array.
   */
  float * data_ptr() noexcept { return data_.data(); }

  /**
   * @brief Return the address pointer of data array for target agents.
   *
   * @return float* The pointer of data array for target agents.
   */
  float * target_data_ptr() noexcept { return target_data_.data(); }

  /**
   * @brief Return the address pointer of data array for ego vehicle.
   *
   * @return float* The pointer of data array for ego vehicle.
   */
  float * ego_data_ptr() noexcept { return ego_data_.data(); }

private:
  std::vector<float> data_;
  std::vector<float> target_data_;
  std::vector<float> ego_data_;
};

std::vector<std::string> getLabelNames(const std::vector<int> & label_index)
{
  std::vector<std::string> label_names;
  label_names.reserve(label_index.size());
  for (const auto & idx : label_index) {
    switch (idx) {
      case 0:
        label_names.emplace_back("VEHICLE");
        break;
      case 1:
        label_names.emplace_back("PEDESTRIAN");
        break;
      case 2:
        label_names.emplace_back("CYCLIST");
        break;
      default:
        std::ostringstream msg;
        msg << "Error invalid label index: " << idx;
        throw std::runtime_error(msg.str());
        break;
    }
  }
  return label_names;
}

}  // namespace trt_mtr
#endif  // TENSORRT_MTR__AGENT_HPP_
