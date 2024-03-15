// Copyright 2024 The Autoware Contributors
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// The reaction_analyzer namespace contains utility functions for the Reaction Analyzer tool.
namespace reaction_analyzer
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief Sorts the test results by their median value.
 *
 * @param test_results An unordered map containing the test results.
 * @return A vector of tuples. Each tuple contains a string (the test name), a vector of doubles
 * (the test results), and a double (the median value).
 */
std::vector<std::tuple<std::string, std::vector<double>, double>> sort_results_by_median(
  const std::unordered_map<std::string, std::vector<double>> test_results);

/**
 * @brief Creates a subscription option with a new callback group.
 *
 * @param node A pointer to the node for which the subscription options are being created.
 * @return The created SubscriptionOptions.
 */
rclcpp::SubscriptionOptions create_subscription_options(rclcpp::Node * node);

/**
 * @brief Splits a string by a given delimiter.
 *
 * @param str The string to be split.
 * @param delimiter The delimiter to split the string by.
 * @return A vector of strings, each of which is a segment of the original string split by the
 * delimiter.
 */
std::vector<std::string> split(const std::string & str, char delimiter);

/**
 * @brief Get the index of the trajectory point that is a certain distance away from the current
 * point.
 *
 * This function iterates over the trajectory from the current point and returns the index of the
 * first point that is at least the specified distance away.
 *
 * @param traj The trajectory to search.
 * @param curr_id The index of the current point in the trajectory.
 * @param distance The distance to search for a point.
 * @return The index of the point in the trajectory that is at least the specified distance away
 * from the current point.
 */
size_t get_index_after_distance(
  const Trajectory & traj, const size_t curr_id, const double distance);
}  // namespace reaction_analyzer

#endif  // UTILS_HPP_
