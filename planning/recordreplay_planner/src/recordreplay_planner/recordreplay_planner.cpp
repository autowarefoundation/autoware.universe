// Copyright 2020-2021 Embotech AG, Zurich, Switzerland, Arm Ltd. Inspired by Christopher Ho
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

#include "recordreplay_planner/recordreplay_planner.hpp"

#include <geometry_msgs/msg/point32.hpp>
#include <time_utils/time_utils.hpp>
#include <motion_common/motion_common.hpp>
#include <common/types.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <map>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::uchar8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using Csv = std::vector<std::vector<std::string>>;
using Association = std::map<std::string /* label */, int32_t /* row */>;
namespace
{
std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void deleteHeadSpace(std::string & string)
{
  while (string.find_first_of(' ') == 0) {
    string.erase(string.begin());
    if (string.empty()) {
      break;
    }
  }
}

void deleteUnit(std::string & string)
{
  size_t start_pos, end_pos;
  start_pos = string.find_first_of('[');
  end_pos = string.find_last_of(']');
  if (start_pos != std::string::npos && end_pos != std::string::npos && start_pos < end_pos) {
    string.erase(start_pos, (end_pos + 1) - start_pos);
  }
}

bool loadData(
  const std::string & file_name, Association & label_row_association_map,
  Csv & file_data)
{
  file_data.clear();
  label_row_association_map.clear();
  std::ifstream ifs(file_name);

  // open file
  if (!ifs) {
    std::cerr << "Could not load " << file_name << std::endl;
    return false;
  }

  // create label-row association map
  std::string line;
  if (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    for (size_t i = 0; i < str_vec.size(); ++i) {
      deleteUnit(str_vec.at(i));
      deleteHeadSpace(str_vec.at(i));
      label_row_association_map[str_vec.at(i)] = static_cast<int>(i);
    }
  } else {
    std::cerr << "cannot create association map" << std::endl;
    return false;
  }

  // create file data
  while (std::getline(ifs, line)) {
    std::vector<std::string> str_vec = split(line, ',');
    file_data.push_back(str_vec);
  }

  return true;
}
}  // namespace
namespace motion
{
namespace planning
{
namespace recordreplay_planner
{
using geometry_msgs::msg::Point32;

RecordReplayPlanner::RecordReplayPlanner() {}

// These may do more in the future
bool8_t RecordReplayPlanner::is_recording() const noexcept
{
  return m_recordreplaystate == RecordReplayState::RECORDING;
}

bool8_t RecordReplayPlanner::is_replaying() const noexcept
{
  return m_recordreplaystate == RecordReplayState::REPLAYING;
}

void RecordReplayPlanner::start_recording() noexcept
{
  m_recordreplaystate = RecordReplayState::RECORDING;
}

void RecordReplayPlanner::stop_recording() noexcept
{
  m_recordreplaystate = RecordReplayState::IDLE;
}

void RecordReplayPlanner::start_replaying() noexcept
{
  m_recordreplaystate = RecordReplayState::REPLAYING;
}

void RecordReplayPlanner::stop_replaying() noexcept
{
  m_recordreplaystate = RecordReplayState::IDLE;
}

void RecordReplayPlanner::clear_record() noexcept
{
  m_record_buffer.clear();
}

std::size_t RecordReplayPlanner::get_record_length() const noexcept
{
  return m_record_buffer.size();
}


void RecordReplayPlanner::set_heading_weight(float64_t heading_weight)
{
  if (heading_weight < 0.0) {
    throw std::domain_error{"Negative weights do not make sense"};
  }
  m_heading_weight = heading_weight;
}

float64_t RecordReplayPlanner::get_heading_weight()
{
  return m_heading_weight;
}

void RecordReplayPlanner::set_min_record_distance(float64_t min_record_distance)
{
  if (min_record_distance < 0.0) {
    throw std::domain_error{"Negative minumum distance do not make sense"};
  }
  m_min_record_distance = min_record_distance;
}

float64_t RecordReplayPlanner::get_min_record_distance() const
{
  return m_min_record_distance;
}

void RecordReplayPlanner::set_skip_first_velocity(const bool8_t skip_first_velocity)
{
  m_skip_first_velocity = skip_first_velocity;
}

bool RecordReplayPlanner::record_state(const State & state_to_record)
{
  if (m_record_buffer.empty()) {
    m_record_buffer.push_back(state_to_record);
    return true;
  }

  auto previous_state = m_record_buffer.back();
  auto distance_sq =
    (state_to_record.state.pose.position.x - previous_state.state.pose.position.x) *
    (state_to_record.state.pose.position.x - previous_state.state.pose.position.x) +
    (state_to_record.state.pose.position.y - previous_state.state.pose.position.y) *
    (state_to_record.state.pose.position.y - previous_state.state.pose.position.y);

  if (distance_sq >= (m_min_record_distance * m_min_record_distance) ) {
    m_record_buffer.push_back(state_to_record);
    return true;
  } else {
    return false;
  }
}

const Trajectory & RecordReplayPlanner::plan(const State & current_state)
{
  return from_record(current_state);
}


std::size_t RecordReplayPlanner::get_closest_state(const State & current_state)
{
  // Find the closest point to the current state in the stored states buffer
  const auto distance_from_current_state =
    [this, &current_state](State & other_state) {
      const auto s1 = current_state.state, s2 = other_state.state;
      return (s1.pose.position.x - s2.pose.position.x) * (s1.pose.position.x - s2.pose.position.x) +
             (s1.pose.position.y - s2.pose.position.y) * (s1.pose.position.y - s2.pose.position.y) +
             m_heading_weight * std::abs(
        ::motion::motion_common::to_angle(
          s1.pose.orientation - s2.pose.orientation));
    };
  const auto comparison_function =
    [&distance_from_current_state](State & one, State & two)
    {return distance_from_current_state(one) < distance_from_current_state(two);};


  const auto minimum_index_iterator =
    std::min_element(
    std::begin(m_record_buffer), std::end(m_record_buffer),
    comparison_function);
  auto minimum_idx = std::distance(std::begin(m_record_buffer), minimum_index_iterator);

  return static_cast<std::size_t>(minimum_idx);
}


const Trajectory & RecordReplayPlanner::from_record(const State & current_state)
{
  // Find out where on the recorded buffer we should start replaying
  m_traj_start_idx = get_closest_state(current_state);

  // Determine how long the published trajectory will be
  auto & trajectory = m_trajectory;
  const auto record_length = get_record_length();
  std::size_t publication_len;

  // Determine the final point in the trajectory and the length of the published trajectory
  if (!m_enable_loop) {
    // If not a loop, we first determine how much we
    // can record - this will either be the distance from
    // the start index to the end of the recording, or
    // the maximum length of recording, whichever is shorter.
    publication_len =
      std::min(
      record_length - m_traj_start_idx, trajectory.points.max_size());
  } else {
    publication_len =
      std::min(record_length, trajectory.points.max_size());
  }

  m_traj_end_idx = m_traj_start_idx + publication_len;

  // We wrap the end index back to the front of the recording if necessary
  if (m_traj_end_idx > record_length) {
    m_traj_end_idx -= record_length;
  }

  trajectory.points.resize(publication_len);

  // Assemble the trajectory as desired
  trajectory.header = current_state.header;

  const auto t0 = time_utils::from_message(m_record_buffer[m_traj_start_idx].header.stamp);

  if (!m_enable_loop || m_traj_start_idx < m_traj_end_idx) {
    // If not looping, we simply copy points between start index and end index
    // in record buffer into the return trajectory, adjusting timestamps
    // such that they're relative to current time
    for (std::size_t i = 0; i < publication_len; ++i) {
      // Make the time spacing of the points match the recorded timing
      trajectory.points[i] = m_record_buffer[m_traj_start_idx + i].state;
      trajectory.points[i].time_from_start = time_utils::to_message(
        time_utils::from_message(m_record_buffer[m_traj_start_idx + i].header.stamp) - t0);
    }
  } else {
    // We need two loops here - first fills from current position to end of original traj,
    // second from beginning of original traj to current pos
    auto t1 = t0;  // This will be used to correct timings in the second loop

    for (std::size_t i = m_traj_start_idx; i < record_length; ++i) {
      auto idx = i - m_traj_start_idx;
      trajectory.points[idx] = m_record_buffer[i].state;
      t1 = time_utils::from_message(m_record_buffer[i].header.stamp);
      trajectory.points[idx].time_from_start = time_utils::to_message(t1 - t0);
    }
    // At this stage, t1 is relative time of the final point in the record buffer

    for (std::size_t i = 0U; i < m_traj_end_idx; ++i) {
      auto idx = i + (record_length - m_traj_start_idx);
      trajectory.points[idx] = m_record_buffer[i].state;

      // This should be updated to have similar timestamp modificaton
      // but it is unclear what function the timestamps carry out
    }
  }

  // Adjust time stamp from velocity
  float32_t t = 0.0;
  for (std::size_t i = 1; i < publication_len; ++i) {
    const auto & p0 = trajectory.points[i - 1];
    const auto & p1 = trajectory.points[i];
    const auto dx = p1.pose.position.x - p0.pose.position.x;
    const auto dy = p1.pose.position.y - p0.pose.position.y;
    const auto v =
      0.5 * static_cast<float64_t>(p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    t += static_cast<float32_t>(std::sqrt(dx * dx + dy * dy) / std::max(std::fabs(v), 1.0e-5));
    float32_t t_s = 0;
    float32_t t_ns = std::modf(t, &t_s) * 1.0e9f;
    trajectory.points[i].time_from_start.sec = static_cast<int32_t>(t_s);
    trajectory.points[i].time_from_start.nanosec = static_cast<uint32_t>(t_ns);
  }

  // Prevent a non-0 velocity at the first trajectory point to avoid some controllers getting stuck
  if (trajectory.points.size() > 1 && trajectory.points[0].longitudinal_velocity_mps < 1.0e-3f) {
    trajectory.points[0].longitudinal_velocity_mps = trajectory.points[1].longitudinal_velocity_mps;
  }
  // Mark the last point along the trajectory as "stopping" by setting all rates,
  // accelerations and velocities to zero. TODO(s.me) this is by no means
  // guaranteed to be dynamically feasible. One could implement a proper velocity
  // profile here in the future.
  if (!m_enable_loop && m_traj_end_idx > m_traj_start_idx) {
    const auto traj_last_idx = publication_len - 1U;
    trajectory.points[traj_last_idx].longitudinal_velocity_mps = 0.0;
    trajectory.points[traj_last_idx].lateral_velocity_mps = 0.0;
    trajectory.points[traj_last_idx].acceleration_mps2 = 0.0;
    trajectory.points[traj_last_idx].heading_rate_rps = 0.0;
  }

  return trajectory;
}

void RecordReplayPlanner::writeTrajectoryBufferToFile(const std::string & record_path)
{
  if (record_path.empty()) {
    throw std::runtime_error("record_path cannot be empty");
  }

  std::ofstream ofs;
  ofs.open(record_path, std::ios::trunc);
  if (!ofs.is_open()) {
    throw std::runtime_error("Could not open file.");
  }
  ofs << "t_sec, t_nanosec, x, y, orientation_x, orientation_y, orientation_z, orientation_w, " <<
    "longitudinal_velocity_mps, lateral_velocity_mps, acceleration_mps2, heading_rate_rps, " <<
    "front_wheel_angle_rad, rear_wheel_angle_rad" << std::endl;

  for (const auto & trajectory_point : m_record_buffer) {
    const auto & s = trajectory_point.state;
    const auto & t = s.time_from_start;
    ofs << t.sec << ", " << t.nanosec << ", " << s.pose.position.x << ", " << s.pose.position.y <<
      ", " << s.pose.orientation.x << ", " << s.pose.orientation.y << ", " <<
      s.pose.orientation.z << ", " << s.pose.orientation.w << ", " <<
      s.longitudinal_velocity_mps << ", " << s.lateral_velocity_mps << ", " <<
      s.acceleration_mps2 << ", " << s.heading_rate_rps << ", " << s.front_wheel_angle_rad <<
      ", " << s.rear_wheel_angle_rad << std::endl;
  }
  ofs.close();
}

void RecordReplayPlanner::readTrajectoryBufferFromFile(const std::string & replay_path)
{
  if (replay_path.empty()) {
    throw std::runtime_error("replay_path cannot be empty");
  }

  // Clear current trajectory deque
  clear_record();

  Csv file_data;
  Association map;  // row labeled Association map
  if (!loadData(replay_path, map, file_data)) {
    std::cerr << "failed to open file : " << replay_path << std::endl;
    return;
  }

  for (size_t i = 0; i < file_data.size(); ++i) {
    State s;
    for (size_t j = 0; j < file_data.at(i).size(); ++j) {
      const int _j = static_cast<int>(j);
      if (map.at("t_sec") == _j) {
        s.state.time_from_start.sec = std::stoi(file_data.at(i).at(j));
      } else if (map.at("t_nanosec") == _j) {
        s.state.time_from_start.nanosec = static_cast<uint32_t>(std::stoi(file_data.at(i).at(j)));
      } else if (map.at("x") == _j) {
        s.state.pose.position.x = std::stof(file_data.at(i).at(j));
      } else if (map.at("y") == _j) {
        s.state.pose.position.y = std::stof(file_data.at(i).at(j));
      } else if (map.at("orientation_x") == _j) {
        s.state.pose.orientation.x = std::stof(file_data.at(i).at(j));
      } else if (map.at("orientation_y") == _j) {
        s.state.pose.orientation.y = std::stof(file_data.at(i).at(j));
      } else if (map.at("orientation_z") == _j) {
        s.state.pose.orientation.z = std::stof(file_data.at(i).at(j));
      } else if (map.at("orientation_w") == _j) {
        s.state.pose.orientation.w = std::stof(file_data.at(i).at(j));
      } else if (map.at("longitudinal_velocity_mps") == _j) {
        s.state.longitudinal_velocity_mps = std::stof(file_data.at(i).at(j));
      } else if (map.at("lateral_velocity_mps") == _j) {
        s.state.lateral_velocity_mps = std::stof(file_data.at(i).at(j));
      } else if (map.at("acceleration_mps2") == _j) {
        s.state.acceleration_mps2 = std::stof(file_data.at(i).at(j));
      } else if (map.at("heading_rate_rps") == _j) {
        s.state.heading_rate_rps = std::stof(file_data.at(i).at(j));
      } else if (map.at("front_wheel_angle_rad") == _j) {
        s.state.front_wheel_angle_rad = std::stof(file_data.at(i).at(j));
      } else if (map.at("rear_wheel_angle_rad") == _j) {
        s.state.rear_wheel_angle_rad = std::stof(file_data.at(i).at(j));
      }
    }
    record_state(s);
  }
}

void RecordReplayPlanner::set_loop(const bool8_t loop)
{
  m_enable_loop = loop;
}

bool8_t RecordReplayPlanner::get_loop() const
{
  return m_enable_loop;
}

bool8_t RecordReplayPlanner::reached_goal(
  const State & current_state,
  const float64_t & distance_thresh,
  const float64_t & angle_thresh) const
{
  if (m_trajectory.points.empty()) {
    return false;
  }

  const auto & goal_state = m_trajectory.points.back();
  const auto & ego_state = current_state.state;
  const auto distance2d = std::hypot(
    ego_state.pose.position.x - goal_state.pose.position.x,
    ego_state.pose.position.y - goal_state.pose.position.y);
  const auto angle_diff_rad = std::abs(
    ::motion::motion_common::to_angle(ego_state.pose.orientation - goal_state.pose.orientation));
  if (distance2d < distance_thresh &&
    angle_diff_rad < angle_thresh)
  {
    return true;
  }
  return false;
}

// Uses code similar to RecordReplayPlanner::reached_goal(),
// but does not check the heading alignment
bool8_t RecordReplayPlanner::is_loop(
  const float64_t & distance_thresh) const
{
  bool retval = false;

  // Can only be a loop if the path is longer than 1 in length
  if (get_record_length() > 1) {
    const auto & goal_state = m_record_buffer.front().state;
    const auto & start_state = m_record_buffer.back().state;
    const auto distance2d =
      std::hypot(
      start_state.pose.position.x - goal_state.pose.position.x,
      start_state.pose.position.y - goal_state.pose.position.y);

    retval = distance2d < distance_thresh;
  }

  return retval;
}

}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace motion
