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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <system_monitor/ntp_monitor/ntp_monitor.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/process.hpp>
#include <string>

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::DiagnosticStatus;

char ** argv_;

class TestNTPMonitor : public NTPMonitor
{
  friend class NTPMonitorTestSuite;

public:
  TestNTPMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : NTPMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_msg)
  {
    array_ = *diag_msg;
  }

  void changeOffsetWarn(float offset_warn) { offset_warn_ = offset_warn; }
  void changeOffsetError(float offset_error) { offset_error_ = offset_error; }

  void setNtpdateExists(bool ntpdate_exists) { ntpdate_exists_ = ntpdate_exists; }

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

class NTPMonitorTestSuite : public ::testing::Test
{
public:
  NTPMonitorTestSuite() : nh_(""), pnh_("~")
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    ntpdate_ = exe_dir_ + "/ntpdate";
  }

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestNTPMonitor> monitor_;
  ros::Subscriber sub_;
  std::string exe_dir_;
  std::string ntpdate_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestNTPMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestNTPMonitor::diagCallback, monitor_.get());

    // Remove dummy executable if exists
    if (fs::exists(ntpdate_)) fs::remove(ntpdate_);
  }

  void TearDown(void)
  {
    // Remove dummy executable if exists
    if (fs::exists(ntpdate_)) fs::remove(ntpdate_);
  }

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

  void modifyPath(void)
  {
    // Modify PATH temporarily
    auto env = boost::this_process::environment();
    std::string new_path = env["PATH"].to_string();
    new_path.insert(0, (boost::format("%1%:") % exe_dir_).str());
    env["PATH"] = new_path;
  }
};

TEST_F(NTPMonitorTestSuite, offsetWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeOffsetWarn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeOffsetWarn(0.05);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(NTPMonitorTestSuite, offsetErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeOffsetError(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeOffsetError(5.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(NTPMonitorTestSuite, offsetNtpdateNotFoundTest)
{
  // Set flag false
  monitor_->setNtpdateExists(false);

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "ntpdate error");
  ASSERT_TRUE(findValue(status, "ntpdate", value));
  ASSERT_STREQ(
    value.c_str(),
    "Command 'ntpdate' not found, but can be installed with: sudo apt install ntpdate");
}

TEST_F(NTPMonitorTestSuite, offsetNtpdateErrorTest)
{
  // Symlink ntpdate1 to ntpdate
  fs::create_symlink(exe_dir_ + "/ntpdate1", ntpdate_);

  // Modify PATH temporarily
  modifyPath();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "ntpdate error");
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "NTPMonitorTestNode");

  return RUN_ALL_TESTS();
}
