/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/publisher.h"
#include "microstrain_3dm.h"
#include "microstrain_mips/status_msg.h"
#include "ros/ros.h"

#include <string>

namespace microstrain_mips {
class RosDiagnosticUpdater : private diagnostic_updater::Updater {
 public:
  RosDiagnosticUpdater(Microstrain::Microstrain* device);

  void generalDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void packetDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void portDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void imuDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void statusCallback(const microstrain_mips::status_msg::ConstPtr& status);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;

  microstrain_mips::status_msg last_status_;
};
}  // namespace microstrain_mips
