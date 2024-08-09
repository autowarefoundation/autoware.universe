// Copyright 2023 Tier IV, Inc.
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

#ifndef MRM_PULL_OVER_MANAGER__CSV_PARSER_HPP_
#define MRM_PULL_OVER_MANAGER__CSV_PARSER_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace mrm_pull_over_manager
{
class CsvPoseParser
{
public:
  explicit CsvPoseParser(const std::string & filepath) : filepath_(filepath) {}

  std::map<lanelet::Id, geometry_msgs::msg::Pose> parse()
  {
    std::map<lanelet::Id, geometry_msgs::msg::Pose> pose_id_map;
    std::ifstream file(filepath_);

    if (!file.is_open()) {
      std::cerr << "Failed to open the CSV file." << std::endl;
      return pose_id_map;
    }

    // Skip header line
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
      std::istringstream ss(line);
      std::string field;
      double x, y, z, yaw;
      int lane_id;  // Not used in this example

      std::getline(ss, field, ',');
      x = std::stod(field);

      std::getline(ss, field, ',');
      y = std::stod(field);

      std::getline(ss, field, ',');
      z = std::stod(field);

      std::getline(ss, field, ',');
      yaw = std::stod(field);

      std::getline(ss, field, ',');
      lane_id = std::stoi(field);

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      yaw_to_quaternion(yaw, pose.orientation);

      pose_id_map[lane_id] = pose;
    }

    return pose_id_map;
  }

private:
  std::string filepath_;

  void yaw_to_quaternion(double yaw, geometry_msgs::msg::Quaternion & quaternion)
  {
    quaternion.z = sin(yaw * 0.5);
    quaternion.w = cos(yaw * 0.5);
  }
};

int main()
{
  const std::string filepath = "path_to_your_csv.csv";
  CsvPoseParser parser(filepath);
  auto poses = parser.parse();
  // ... Use the poses as needed
  return 0;
}
}  // namespace mrm_pull_over_manager

#endif  // MRM_PULL_OVER_MANAGER__CSV_PARSER_HPP_
