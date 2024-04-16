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

#include <algorithm>
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
  : data_({x, y, z, length, width, height, yaw, vx, vy, ax, ay, is_valid}),
    x_(x),
    y_(y),
    z_(z),
    length_(length),
    width_(width),
    height_(height),
    yaw_(yaw),
    vx_(vx),
    vy_(vy),
    ax_(ax),
    is_valid_(is_valid)
  {
  }

  /**
   * @brief Construct a new instance with a pointer.
   *
   * @param ptr
   */
  explicit AgentState(const std::vector<float>::const_iterator & itr)
  {
    std::copy(itr, itr + dim(), data_.begin());
  }

  static size_t dim() { return AgentStateDim; }

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

  float x() const { return x_; }
  float y() const { return y_; }
  float z() const { return z_; }
  float length() const { return length_; }
  float width() const { return width_; }
  float height() const { return height_; }
  float yaw() const { return yaw_; }
  float vx() const { return vx_; }
  float vy() const { return vy_; }
  float ax() const { return ax_; }
  float ay() const { return ay_; }
  bool is_valid() const { return is_valid_ == 1.0f; }

private:
  std::array<float, AgentStateDim> data_;
  float x_{0.0f}, y_{0.0f}, z_{0.0f}, length_{0.0f}, width_{0.0f}, height_{0.0f}, yaw_{0.0f},
    vx_{0.0f}, vy_{0.0f}, ax_{0.0f}, ay_{0.0f}, is_valid_{0.0f};
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
    AgentState & state, const std::string & object_id, const size_t label_index,
    const float current_time, const size_t max_time_length)
  : data_((max_time_length - 1) * state_dim()),
    object_id_(object_id),
    label_index_(label_index),
    latest_time_(current_time),
    max_time_length_(max_time_length)
  {
    const auto s_ptr = state.data_ptr();
    for (size_t d = 0; d < state_dim(); ++d) {
      data_.push_back(*(s_ptr + d));
    }
  }

  /**
   * @brief Construct a new Agent History filling all elements by zero.
   *
   * @param object_id Object ID.
   * @param max_time_length History time length.
   */
  AgentHistory(
    const std::string & object_id, const size_t label_index, const size_t max_time_length)
  : data_(max_time_length * state_dim()),
    object_id_(object_id),
    label_index_(label_index),
    latest_time_(-std::numeric_limits<float>::max()),
    max_time_length_(max_time_length)
  {
  }

  static size_t state_dim() { return AgentStateDim; }

  /**
   * @brief Return the history length.
   *
   * @return size_t History length.
   */
  size_t length() const { return max_time_length_; }

  /**
   * @brief Return the data size of history `T * D`.
   *
   * @return size_t
   */
  size_t size() const { return max_time_length_ * state_dim(); }

  /**
   * @brief Return the shape of history matrix ordering in `(T, D)`.
   *
   * @return std::tuple<size_t, size_t>
   */
  std::tuple<size_t, size_t> shape() const { return {max_time_length_, state_dim()}; }

  /**
   * @brief Return the object id.
   *
   * @return const std::string&
   */
  const std::string & object_id() const { return object_id_; }

  size_t label_index() const { return label_index_; }

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
    data_.erase(data_.begin(), data_.begin() + state_dim());

    const auto s = state.data_ptr();
    for (size_t d = 0; d < state_dim(); ++d) {
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
    data_.erase(data_.begin(), data_.begin() + state_dim());

    const auto s = AgentState::empty().data_ptr();
    for (size_t d = 0; d < state_dim(); ++d) {
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
  bool is_valid_latest() const { return get_latest_state().is_valid(); }

  /**
   * @brief Get the latest agent state.
   *
   * @return AgentState
   */
  AgentState get_latest_state() const
  {
    const auto & latest_itr = (data_.begin() + state_dim() * (max_time_length_ - 1));
    return AgentState(latest_itr);
  }

private:
  std::vector<float> data_;
  const std::string object_id_;
  const size_t label_index_;
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
    std::vector<AgentHistory> & histories, const size_t sdc_index,
    const std::vector<size_t> & target_index, const std::vector<size_t> & label_index,
    const std::vector<float> & timestamps)
  : num_target_(target_index.size()),
    num_agent_(histories.size()),
    time_length_(timestamps.size()),
    sdc_index_(sdc_index),
    target_index_(target_index),
    label_index_(label_index),
    timestamps_(timestamps)
  {
    data_.reserve(num_agent_ * time_length_ * state_dim());
    for (auto & history : histories) {
      const auto data_ptr = history.data_ptr();
      for (size_t t = 0; t < time_length_; ++t) {
        for (size_t d = 0; d < state_dim(); ++d) {
          data_.push_back(*(data_ptr + t * state_dim() + d));
        }
      }
    }

    target_data_.reserve(num_target_ * state_dim());
    target_label_index_.reserve(num_target_);
    for (const auto & idx : target_index) {
      target_label_index_.emplace_back(label_index.at(idx));
      const auto target_ptr = histories.at(idx).data_ptr();
      for (size_t d = 0; d < state_dim(); ++d) {
        target_data_.push_back(*(target_ptr + (time_length_ - 1) * state_dim() + d));
      }
    }

    ego_data_.reserve(time_length_ * state_dim());
    const auto ego_data_ptr = histories.at(sdc_index).data_ptr();
    for (size_t t = 0; t < time_length_; ++t) {
      for (size_t d = 0; d < state_dim(); ++d) {
        ego_data_.push_back(*(ego_data_ptr + t * state_dim() + d));
      }
    }
  }

  static size_t state_dim() { return AgentStateDim; }
  static size_t num_class() { return 3; }  // TODO(ktro2828): Do not use magic number.
  size_t num_target() const { return num_target_; }
  size_t num_agent() const { return num_agent_; }
  size_t time_length() const { return time_length_; }
  int sdc_index() const { return sdc_index_; }
  const std::vector<size_t> & target_index() const { return target_index_; }
  const std::vector<size_t> & label_index() const { return label_index_; }
  const std::vector<size_t> & target_label_index() const { return target_label_index_; }
  const std::vector<float> & timestamps() const { return timestamps_; }

  size_t size() const { return num_agent_ * time_length_ * state_dim(); }
  size_t input_dim() const { return num_agent_ + time_length_ + num_class() + 3; }

  /**
   * @brief Return the data shape ordering in (N, T, D).
   *
   * @return std::tuple<size_t, size_t, size_t>
   */
  std::tuple<size_t, size_t, size_t> shape() const
  {
    return {num_agent_, time_length_, state_dim()};
  }

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
  size_t num_target_;
  size_t num_agent_;
  size_t time_length_;
  int sdc_index_;
  std::vector<size_t> target_index_;
  std::vector<size_t> label_index_;
  std::vector<size_t> target_label_index_;
  std::vector<float> timestamps_;
  std::vector<float> data_;
  std::vector<float> target_data_;
  std::vector<float> ego_data_;
};

std::vector<std::string> getLabelNames(const std::vector<size_t> & label_index)
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
