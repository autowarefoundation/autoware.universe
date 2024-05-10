// Copyright 2024 Tier IV, Inc.
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

/**
 * @file cgroup_setter.hpp
 * @brief Cgroup setter class
 */

#ifndef CGROUP_SETTER_HPP
#define CGROUP_SETTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <string>
#include <utility>


class CgroupSetter : public rclcpp::Node 
{
public:
    explicit CgroupSetter(const rclcpp::NodeOptions& options);

private:
    void checkCgroup(diagnostic_updater::DiagnosticStatusWrapper & stat);
    void checkProcessAndAddToCgroup();
    std::string executeCommand(const std::string& cmd);
    bool addToCgroup(const std::string& cgroupPath, const std::string& pid);
    bool checkPIDExists(const std::string& filePath, const std::string & pid);

    std::map<std::pair<std::string, std::string>, bool> cgroup_map_;
    char hostname_[256];
    diagnostic_updater::Updater updater_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string base_path_;
};

#endif // CGROUP_SETTER_HPP