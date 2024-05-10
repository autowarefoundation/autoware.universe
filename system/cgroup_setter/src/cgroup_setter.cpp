// Copyright 2024 Autoware Foundation
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
 * @file cgroup_setter.cpp
 * @brief Cgroup　setter class
 */

#include "cgroup_setter.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <boost/process.hpp>
#include <yaml-cpp/yaml.h>

namespace bp = boost::process;

CgroupSetter::CgroupSetter(const rclcpp::NodeOptions & options)
: Node("cgroup_setter_node", options),
  updater_(this)
{
  try {
    std::string yaml_path = this->declare_parameter<std::string>("cgroup_setting_config_path");
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (config["base_path"]) {
        base_path_ = config["base_path"].as<std::string>();
    } else {
        RCLCPP_ERROR(this->get_logger(), "base_path is not set in the config file.");
        return;
    }

    if (!config["settings"]) {
        RCLCPP_ERROR(this->get_logger(), "settings is not set in the config file.");
        return;
    }
    
    for (auto setting : config["settings"]) {
        if (!setting["directory"] || !setting["search_word"]) {
        RCLCPP_ERROR(this->get_logger(), "directory or search_word is not set in the config file.");
        return;
        }

        for (auto word : setting["search_word"]) {
        std::pair<std::string, std::string> tmp_pair = std::make_pair(setting["directory"].as<std::string>(), word.as<std::string>());
        cgroup_map_[tmp_pair] = false;
        }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load the config file.");
    return;
  }

  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Cgroup Setting", this, &CgroupSetter::checkCgroup);

  // Timer
  using namespace std::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), 1s, std::bind(&CgroupSetter::checkProcessAndAddToCgroup, this));
}

void CgroupSetter::checkCgroup(diagnostic_updater::DiagnosticStatusWrapper & stat) {
    bool allOK = true;
    for (auto& entry : cgroup_map_) {
        if (entry.second) {
            stat.add(entry.first.first + " " + entry.first.second, "OK");
        } else {
            allOK = false;
            stat.add(entry.first.first + " " + entry.first.second, "NG");
        }
    }
    if (allOK) {
        if (timer_->is_active()) timer->cancel();
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "All processes are added to cgroup.");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Some processes are not added to cgroup.");
    }
}

void CgroupSetter::checkProcessAndAddToCgroup() {
    for (auto& entry : cgroup_map_) {
        if (entry.second) {
            continue;
        }
        std::string word = entry.first.second;
        std::string result = executeCommand(word);
        if (!result.empty()) {
            std::istringstream iss(result);
            std::string pid;
            bool allAdded = true;
            while (std::getline(iss, pid, '\n')) {
                if (!pid.empty() && addToCgroup(entry.first.first, pid)) {
                    if (checkPIDExists(base_path_ + "/" + entry.first.first + "/cgroup.procs", pid)) {
                        RCLCPP_INFO(this->get_logger(), "Added all PIDs to cgroup. %s %s", entry.first.second.c_str(), pid.c_str());
                    } else {
                        allAdded = false;
                        RCLCPP_ERROR(this->get_logger(), "Failed to add PID %s to cgroup. %s %s", pid.c_str(), entry.first.second.c_str(), result.c_str());
                    }
                } else {
                    allAdded = false;
                    RCLCPP_ERROR(this->get_logger(), "Failed to add PID %s to cgroup. %s %s", pid.c_str(), entry.first.second.c_str(), result.c_str());
                }
            }
            if (allAdded) {
                entry.second = true;     
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get PID. %s %s", entry.first.second.c_str(), result.c_str());
        }
    }
}

std::string CgroupSetter::executeCommand(const std::string& search_word) {
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(this->get_logger(), "pipe2 error");
    return "";
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(this->get_logger(), "pipe2 error");
    return "";
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};
  auto cmd=bp::search_path("pgrep");
  std::vector<std::string> args;
  args.push_back("-f");
  args.push_back(search_word);
  bp::child c(cmd, bp::args=args, bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    RCLCPP_ERROR(this->get_logger(), os.str().c_str());
    return "";
  } else {
    std::ostringstream os;
    os << is_out.rdbuf();
    std::string output = os.str();
    return output;
  }
}

bool CgroupSetter::checkPIDExists(const std::string& filePath, const std::string & pid) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s", filePath.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line == pid) {
            return true;
        }
    }
    return false;
}

bool CgroupSetter::addToCgroup(const std::string& cgroupPath, const std::string& pid) {
    std::string cgroupProcFile = base_path_ + "/" + cgroupPath + "/cgroup.procs";
    std::ofstream ofs(cgroupProcFile, std::ofstream::app);
    if (!ofs) {
        std::cerr << "Failed to open " << cgroupProcFile << std::endl;
        ofs.close();
        return false;
    }
    ofs << pid;
    if (!ofs) {
        std::cerr << "Failed to write to " << cgroupProcFile << std::endl;
        ofs.close();
        return false;
    }
    ofs.close();
    return true;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto options = rclcpp::NodeOptions();
    auto node = std::make_shared<CgroupSetter>(options);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
}