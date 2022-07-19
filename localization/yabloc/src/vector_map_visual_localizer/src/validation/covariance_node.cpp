#include "common/util.hpp"
#include "validation/covariance_monitor.hpp"

#include <iomanip>
#include <sstream>

namespace validation
{
CovarianceMonitor::CovarianceMonitor() : Node("covariance_monitor")
{
  using std::placeholders::_1;
  auto cb_array = std::bind(&CovarianceMonitor::particleCallback, this, _1);

  pub_diagnostic_ = create_publisher<String>("/covariance_diag", 10);
  sub_particle_ = create_subscription<ParticleArray>("/predicted_particles", 10, cb_array);
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

void CovarianceMonitor::particleCallback(const ParticleArray & array)
{
  Eigen::Vector3f eigens = computeEigens(array);

  std::streamsize default_precision = std::cout.precision();

  std::stringstream ss;
  ss << "--- Particles Status ---" << std::endl;
  ss << "count: " << array.particles.size() << std::endl;
  ss << "eigens: " << std::fixed << std::setprecision(2) << eigens.x() << ", " << eigens.y() << ", "
     << eigens.z() << std::setprecision(default_precision) << std::endl;

  String msg;
  msg.data = ss.str();
  pub_diagnostic_->publish(msg);
}

}  // namespace validation

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::CovarianceMonitor>());
  rclcpp::shutdown();
  return 0;
}