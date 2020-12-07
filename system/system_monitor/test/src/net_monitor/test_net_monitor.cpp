// Copyright 2020 Autoware Foundation
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

#include "gtest/gtest.h"
#include "ros/ros.h"
#include "system_monitor/net_monitor/net_monitor.hpp"
#include "boost/algorithm/string.hpp"
#include "boost/filesystem.hpp"
#include <string>
#include <vector>

static constexpr const char * DOCKER_ENV = "/.dockerenv";

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::DiagnosticStatus;

class TestNetMonitor : public NetMonitor
{
  friend class NetMonitorTestSuite;

public:
  TestNetMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : NetMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_msg)
  {
    array_ = *diag_msg;
  }

  void changeUsageWarn(float usage_warn) { usage_warn_ = usage_warn; }

  const std::vector<std::string> getDeviceParams(void) { return device_params_; }
  void clearDeviceParams(void) { device_params_.clear(); }

  void update(void) { updater_.force_update(); }

  const std::string removePrefix(const std::string & name)
  {
    return boost::algorithm::erase_all_copy(name, prefix_);
  }

  bool findDiagStatus(const std::string & name, DiagStatus & status)  // NOLINT
  {
    for (int i = 0; i < array_.status.size(); ++i) {
      if (removePrefix(array_.status[i].name) == name) {
        status = array_.status[i];
        return true;
      }
    }
    return false;
  }

private:
  diagnostic_msgs::DiagnosticArray array_;
  const std::string prefix_ = ros::this_node::getName().substr(1) + ": ";
};

class NetMonitorTestSuite : public ::testing::Test
{
public:
  NetMonitorTestSuite() : nh_(""), pnh_("~") {}

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestNetMonitor> monitor_;
  ros::Subscriber sub_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestNetMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestNetMonitor::diagCallback, monitor_.get());
  }

  void TearDown(void) {}

  bool findValue(const DiagStatus status, const std::string & key, std::string & value)  // NOLINT
  {
    for (auto itr = status.values.begin(); itr != status.values.end(); ++itr) {
      if (itr->key == key) {
        value = itr->value;
        return true;
      }
    }
    return false;
  }
};

TEST_F(NetMonitorTestSuite, usageWarnTest)
{
  // Verify normal behavior
  {
    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageWarn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
    // Skip test if process runs inside docker
    // Don't know what interface should be monitored.
    if (!fs::exists(DOCKER_ENV)) ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageWarn(0.95);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(NetMonitorTestSuite, usageInvalidDeviceParameterTest)
{
  // Clear list
  monitor_->clearDeviceParams();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("Network Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "invalid device parameter");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "NetMonitorTestNode");

  return RUN_ALL_TESTS();
}
