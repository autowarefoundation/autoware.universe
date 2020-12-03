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
#include <system_monitor/gpu_monitor/tegra_gpu_monitor.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <string>

static constexpr const char * TEST_FILE = "test";

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::DiagnosticStatus;

class TestGPUMonitor : public GPUMonitor
{
  friend class GPUMonitorTestSuite;

public:
  TestGPUMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : GPUMonitor(nh, pnh) {}

  void diagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr & diag_msg)
  {
    array_ = *diag_msg;
  }

  void addTempName(const std::string & path) { temps_.emplace_back(path, path); }
  void clearTempNames(void) { temps_.clear(); }

  void addLoadName(const std::string & path) { loads_.emplace_back(path, path); }
  void clearLoadNames(void) { loads_.clear(); }

  void addFreqName(const std::string & path) { freqs_.emplace_back(path, path); }
  void clearFreqNames(void) { freqs_.clear(); }

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

class GPUMonitorTestSuite : public ::testing::Test
{
public:
  GPUMonitorTestSuite() : nh_(""), pnh_("~") {}

protected:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<TestGPUMonitor> monitor_;
  ros::Subscriber sub_;

  void SetUp(void)
  {
    monitor_ = std::make_unique<TestGPUMonitor>(nh_, pnh_);
    sub_ = nh_.subscribe("/diagnostics", 1000, &TestGPUMonitor::diagCallback, monitor_.get());

    // Remove test file if exists
    if (fs::exists(TEST_FILE)) fs::remove(TEST_FILE);
  }

  void TearDown(void)
  {
    // Remove test file if exists
    if (fs::exists(TEST_FILE)) fs::remove(TEST_FILE);
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
};

TEST_F(GPUMonitorTestSuite, tempWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addTempName(TEST_FILE);

  // Verify warning
  {
    // Write warning level
    std::ofstream ofs(TEST_FILE);
    ofs << 90000 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 89900 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(GPUMonitorTestSuite, tempErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addTempName(TEST_FILE);

  // Verify error
  {
    // Write error level
    std::ofstream ofs(TEST_FILE);
    ofs << 95000 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 89900 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(GPUMonitorTestSuite, tempTemperatureFilesNotFoundTest)
{
  // Clear list
  monitor_->clearTempNames();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "temperature files not found");
}

TEST_F(GPUMonitorTestSuite, tempFileOpenErrorTest)
{
  // Add test file to list
  monitor_->addTempName(TEST_FILE);

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "file open error");
  ASSERT_TRUE(findValue(status, "file open error", value));
  ASSERT_STREQ(value.c_str(), TEST_FILE);
}

TEST_F(GPUMonitorTestSuite, usageWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addLoadName(TEST_FILE);

  // Verify warning
  {
    // Write warning level
    std::ofstream ofs(TEST_FILE);
    ofs << 900 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 890 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(GPUMonitorTestSuite, usageErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addLoadName(TEST_FILE);

  // Verify error
  {
    // Write error level
    std::ofstream ofs(TEST_FILE);
    ofs << 1000 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 890 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(GPUMonitorTestSuite, usageTemperatureFilesNotFoundTest)
{
  // Clear list
  monitor_->clearLoadNames();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "load files not found");
}

TEST_F(GPUMonitorTestSuite, usageFileOpenErrorTest)
{
  // Add test file to list
  monitor_->addLoadName(TEST_FILE);

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "file open error");
  ASSERT_TRUE(findValue(status, "file open error", value));
  ASSERT_STREQ(value.c_str(), TEST_FILE);
}

TEST_F(GPUMonitorTestSuite, freqTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Frequency", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(GPUMonitorTestSuite, freqFrequencyFilesNotFoundTest)
{
  // Clear list
  monitor_->clearFreqNames();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Frequency", status));

  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "frequency files not found");
}

// for coverage
class DummyGPUMonitor : public GPUMonitorBase
{
  friend class GPUMonitorTestSuite;

public:
  DummyGPUMonitor(const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : GPUMonitorBase(nh, pnh)
  {
  }
  void update(void) { updater_.force_update(); }
};

TEST_F(GPUMonitorTestSuite, dummyGPUMonitorTest)
{
  std::unique_ptr<DummyGPUMonitor> monitor = std::make_unique<DummyGPUMonitor>(nh_, pnh_);
  // Publish topic
  monitor->update();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "GPUMonitorTestNode");

  return RUN_ALL_TESTS();
}
