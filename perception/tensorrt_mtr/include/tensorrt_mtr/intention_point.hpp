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

#ifndef TENSORRT_MTR__INTENTION_POINT_HPP_
#define TENSORRT_MTR__INTENTION_POINT_HPP_

#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace trt_mtr
{

constexpr size_t IntentionPointDim = 2;

/**
 * @brief A class to load and store intention points.
 */
struct IntentionPoint
{
public:
  /**
   * @brief Construct a new Intention.
   *
   * @param data_map Map of intention points hashed by label names.
   * @param num_cluster The number of clusters.
   */
  IntentionPoint(
    const std::unordered_map<std::string, std::vector<float>> data_map, const size_t num_cluster)
  : data_map_(data_map), num_cluster_(num_cluster)
  {
  }

  static size_t state_dim() { return IntentionPointDim; }

  /**
   * @brief Construct a new Intention from csv file.
   *
   * @param csv_filepath Path to csv file.
   * @param num_cluster The number of clusters.
   */
  IntentionPoint(const std::string csv_filepath, const size_t num_cluster)
  : num_cluster_(num_cluster)
  {
    std::ifstream file(csv_filepath);
    if (!file.is_open()) {
      std::ostringstream err_msg;
      err_msg << "Error opening file: " << csv_filepath << ". Please check if the file exists.";
      throw std::runtime_error(err_msg.str());
    }

    std::vector<std::tuple<float, float, std::string>> buffer;
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      float x, y;
      std::string label;

      ss >> x;
      ss.ignore();
      ss >> y;
      ss.ignore();
      std::getline(ss, label, ',');

      buffer.emplace_back(x, y, label);
    }
    file.close();

    for (const auto & [x, y, label] : buffer) {
      data_map_[label].emplace_back(x);
      data_map_[label].emplace_back(y);
    }
  }

  /**
   * @brief Load intention points for specified label names.
   *
   * @param label_names Array of label names for all agents, in shape [N].
   * @return std::vector<float> Array of all points in shape, [N * num_cluster * 2].
   */
  std::vector<float> get_points(const std::vector<std::string> & label_names)
  {
    std::vector<float> points;
    points.reserve(label_names.size() * num_cluster_ * state_dim());
    for (const auto & name : label_names) {
      const auto & label_points = data_map_.at(name);
      for (const auto & p : label_points) {
        points.emplace_back(p);
      }
    }
    return points;
  }

  // Return the size of intension point `K*D`.
  size_t size() const { return num_cluster_ * state_dim(); }

  // Return the number of clusters contained in intention points `K`.
  size_t num_cluster() const { return num_cluster_; }

private:
  std::unordered_map<std::string, std::vector<float>> data_map_;
  size_t num_cluster_;
};
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__INTENTION_POINT_HPP_
