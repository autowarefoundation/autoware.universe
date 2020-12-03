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
#include <system_monitor/gpu_monitor/nvml_gpu_monitor.hpp>
#include <boost/algorithm/string.hpp>
#include <string>

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

  void addGPU(const gpu_info & info) { gpus_.push_back(info); }
  void clearGPU(void) { gpus_.clear(); }

  void changeTempWarn(float temp_warn) { temp_warn_ = temp_warn; }
  void changeTempError(float temp_error) { temp_error_ = temp_error; }

  void changeGPUUsageWarn(float gpu_usage_warn) { gpu_usage_warn_ = gpu_usage_warn; }
  void changeGPUUsageError(float gpu_usage_error) { gpu_usage_error_ = gpu_usage_error; }

  void changeMemoryUsageWarn(float memory_usage_warn) { memory_usage_warn_ = memory_usage_warn; }
  void changeMemoryUsageError(float memory_usage_error)
  {
    memory_usage_error_ = memory_usage_error;
  }

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
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeTempWarn(0.0);

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
    // Change back to normal
    monitor_->changeTempWarn(90.0);

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
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify error
  {
    // Change error level
    monitor_->changeTempError(0.0);

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
    // Change back to normal
    monitor_->changeTempError(95.0);

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

TEST_F(GPUMonitorTestSuite, gpuUsageWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeGPUUsageWarn(0.0);

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
    // Change back to normal
    monitor_->changeGPUUsageWarn(0.90);

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

TEST_F(GPUMonitorTestSuite, gpuUsageErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify error
  {
    // Change error level
    monitor_->changeGPUUsageError(0.0);

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
    // Change back to normal
    monitor_->changeGPUUsageError(1.00);

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

TEST_F(GPUMonitorTestSuite, gpuMemoryUsageWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeMemoryUsageWarn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeMemoryUsageWarn(0.95);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(GPUMonitorTestSuite, gpuMemoryUsageErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify error
  {
    // Change error level
    monitor_->changeMemoryUsageError(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeMemoryUsageError(0.99);

    // Publish topic
    monitor_->update();

    // Give time to publish
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(GPUMonitorTestSuite, throttlingTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Thermal Throttling", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(GPUMonitorTestSuite, gpuNotFoundTest)
{
  // Clear list
  monitor_->clearGPU();

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "gpu not found");

  ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "gpu not found");

  ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "gpu not found");

  ASSERT_TRUE(monitor_->findDiagStatus("GPU Thermal Throttling", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "gpu not found");
}

TEST_F(GPUMonitorTestSuite, illigalDeviceHandleTest)
{
  // Clear list
  monitor_->clearGPU();
  // Add blank device
  gpu_info gpu;
  monitor_->addGPU(gpu);

  // Publish topic
  monitor_->update();

  // Give time to publish
  ros::WallDuration(0.5).sleep();
  ros::spinOnce();

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("GPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "Failed to retrieve the current temperature");

  ASSERT_TRUE(monitor_->findDiagStatus("GPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "Failed to retrieve the current utilization rates");

  ASSERT_TRUE(monitor_->findDiagStatus("GPU Memory Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(
    status.message.c_str(), "Failed to retrieve the amount of used, free and total memory");

  ASSERT_TRUE(monitor_->findDiagStatus("GPU Thermal Throttling", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "Failed to retrieve the current clock speeds");
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
