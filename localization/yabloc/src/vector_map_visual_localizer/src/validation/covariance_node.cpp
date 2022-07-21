#include "common/util.hpp"
#include "validation/covariance_monitor.hpp"

#include <iomanip>
#include <sstream>

namespace validation
{
CovarianceMonitor::CovarianceMonitor() : Node("covariance_monitor")
{
  using std::placeholders::_1, std::placeholders::_2;
  auto cb_synchro = std::bind(&CovarianceMonitor::particleAndPose, this, _1, _2);
  synchro_subscriber_ = std::make_shared<SynchroSubscriber<ParticleArray, PoseStamped>>(
    this, "/predicted_particles", "/particle_pose");
  synchro_subscriber_->setCallback(cb_synchro);

  pub_diagnostic_ = create_publisher<String>("/covariance_diag", 10);
  pub_pose_cov_stamped_ = create_publisher<PoseCovStamped>("/pose_with_covariane", 10);
}

void CovarianceMonitor::particleAndPose(const ParticleArray & particles, const PoseStamped & pose)
{
  Eigen::Vector3f eigens = computeEigens(particles);

  std::streamsize default_precision = std::cout.precision();

  std::stringstream ss;
  ss << "--- Particles Status ---" << std::endl;
  ss << "count: " << particles.particles.size() << std::endl;
  ss << "eigens: " << std::fixed << std::setprecision(2) << eigens.x() << ", " << eigens.y() << ", "
     << eigens.z() << std::setprecision(default_precision) << std::endl;

  publishPoseCovStamped(pose, eigens);

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

Eigen::Vector3f CovarianceMonitor::computeEigens(const ParticleArray & array) const
{
  if (array.particles.empty()) return Eigen::Vector3f::Zero();

  float invN = 1.f / array.particles.size();
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = util::pose2Affine(p.pose);
    mean += affine.translation();
  }
  mean *= invN;

  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = util::pose2Affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    sigma += (d * d.transpose()) * invN;
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(sigma, Eigen::ComputeThinV | Eigen::ComputeThinU);
  return svd.singularValues().cwiseSqrt();
}

}  // namespace validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::CovarianceMonitor>());
  rclcpp::shutdown();
  return 0;
}