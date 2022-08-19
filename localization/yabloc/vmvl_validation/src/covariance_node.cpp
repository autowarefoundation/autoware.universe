#include "vmvl_validation/covariance_monitor.hpp"

#include <vml_common/util.hpp>

#include <iomanip>
#include <sstream>

namespace vmvl_validation
{
CovarianceMonitor::CovarianceMonitor() : Node("covariance_monitor")
{
  using std::placeholders::_1, std::placeholders::_2;
  auto cb_synchro = std::bind(&CovarianceMonitor::particleAndPose, this, _1, _2);
  synchro_subscriber_ = std::make_shared<SynchroSubscriber<ParticleArray, PoseStamped>>(
    this, "/predicted_particles", "/particle_pose");
  synchro_subscriber_->setCallback(cb_synchro);

  pub_diagnostic_ = create_publisher<String>("/covariance_diag", 10);
  pub_pose_cov_stamped_ = create_publisher<PoseCovStamped>("/pose_with_covariance", 10);
}

void CovarianceMonitor::particleAndPose(const ParticleArray & particles, const PoseStamped & pose)
{
  auto ori = pose.pose.orientation;
  Eigen::Quaternionf orientation(ori.w, ori.x, ori.y, ori.z);
  Eigen::Vector3f std = computeStd(particles, orientation);

  std::streamsize default_precision = std::cout.precision();

  std::stringstream ss;
  ss << "--- Particles Status ---" << std::endl;
  ss << "count: " << particles.particles.size() << std::endl;
  ss << "std: " << std::fixed << std::setprecision(2) << std.x() << ", " << std.y() << ", "
     << std.z() << std::setprecision(default_precision) << std::endl;

  publishPoseCovStamped(pose, std.cwiseAbs2());

  String msg;
  msg.data = ss.str();
  pub_diagnostic_->publish(msg);
}

void CovarianceMonitor::publishPoseCovStamped(
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

Eigen::Vector3f CovarianceMonitor::computeStd(
  const ParticleArray & array, const Eigen::Quaternionf & orientation) const
{
  if (array.particles.empty()) return Eigen::Vector3f::Zero();

  float invN = 1.f / array.particles.size();
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = vml_common::pose2Affine(p.pose);
    mean += affine.translation();
  }
  mean *= invN;

  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = vml_common::pose2Affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    d = orientation.conjugate() * d;
    sigma += (d * d.transpose()) * invN;
  }

  return sigma.diagonal().cwiseMax(1e-4f).cwiseSqrt();
}

}  // namespace vmvl_validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vmvl_validation::CovarianceMonitor>());
  rclcpp::shutdown();
  return 0;
}