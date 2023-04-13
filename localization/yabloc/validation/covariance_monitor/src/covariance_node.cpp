// Copyright 2023 TIER IV, Inc.
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

#include "covariance_monitor/covariance_monitor.hpp"

#include <pcdless_common/pose_conversions.hpp>

#include <iomanip>
#include <sstream>

namespace pcdless::covariance_monitor
{
CovarianceMonitor::CovarianceMonitor() : Node("covariance_monitor")
{
  using std::placeholders::_1, std::placeholders::_2;
  auto cb_synchro = std::bind(&CovarianceMonitor::particle_and_pose, this, _1, _2);
  synchro_subscriber_ = std::make_shared<common::SynchroSubscriber<ParticleArray, PoseStamped>>(
    this, "particles", "particle_pose");
  synchro_subscriber_->set_callback(cb_synchro);

  pub_diagnostic_ = create_publisher<String>("cov_diag", 10);
  pub_pose_cov_stamped_ = create_publisher<PoseCovStamped>("pose_with_cov", 10);
}

void CovarianceMonitor::particle_and_pose(const ParticleArray & particles, const PoseStamped & pose)
{
  auto ori = pose.pose.orientation;
  Eigen::Quaternionf orientation(ori.w, ori.x, ori.y, ori.z);
  Eigen::Vector3f std = compute_std(particles, orientation);

  std::streamsize default_precision = std::cout.precision();

  std::stringstream ss;
  ss << "--- Particles Status ---" << std::endl;
  ss << "count: " << particles.particles.size() << std::endl;
  ss << "std: " << std::fixed << std::setprecision(2) << std.x() << ", " << std.y() << ", "
     << std.z() << std::setprecision(default_precision) << std::endl;

  publish_pose_cov_stamped(pose, std.cwiseAbs2());

  String msg;
  msg.data = ss.str();
  pub_diagnostic_->publish(msg);
}

void CovarianceMonitor::publish_pose_cov_stamped(
  const PoseStamped & pose, const Eigen::Vector3f & covariance)
{
  PoseCovStamped msg;
  msg.header = pose.header;
  msg.pose.pose = pose.pose;
  msg.pose.covariance.at(0) = covariance.x();
  msg.pose.covariance.at(7) = covariance.y();
  msg.pose.covariance.at(14) = covariance.z();
  pub_pose_cov_stamped_->publish(msg);
}

Eigen::Vector3f CovarianceMonitor::compute_std(
  const ParticleArray & array, const Eigen::Quaternionf & orientation) const
{
  if (array.particles.empty()) return Eigen::Vector3f::Zero();

  float invN = 1.f / array.particles.size();
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    mean += affine.translation();
  }
  mean *= invN;

  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    d = orientation.conjugate() * d;
    sigma += (d * d.transpose()) * invN;
  }

  return sigma.diagonal().cwiseMax(1e-4f).cwiseSqrt();
}

}  // namespace pcdless::covariance_monitor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::covariance_monitor::CovarianceMonitor>());
  rclcpp::shutdown();
  return 0;
}