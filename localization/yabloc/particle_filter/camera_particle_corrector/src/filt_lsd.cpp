#include "camera_particle_corrector/camera_particle_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <pcdless_common/pose_conversions.hpp>

namespace pcdless::modularized_particle_filter
{
namespace
{
// TODO: unify
float abs_cos(const Eigen::Vector3f & t, float deg)
{
  Eigen::Vector2f x(t.x(), t.y());
  Eigen::Vector2f y(std::cos(deg * M_PI / 180.0), std::sin(deg * M_PI / 180.0));
  x.normalize();
  return std::abs(x.dot(y));
}

cv::Point2f cv2pt(const Eigen::Vector3f v)
{
  const float METRIC_PER_PIXEL = 0.1;
  const float IMAGE_RADIUS = 400;
  return {-v.y() / METRIC_PER_PIXEL + IMAGE_RADIUS, -v.x() / METRIC_PER_PIXEL + IMAGE_RADIUS};
}

}  // namespace

CameraParticleCorrector::LineSegments CameraParticleCorrector::filt(
  const LineSegments & lines, const LineSegments & reliable)
{
  LineSegments::Ptr filted = pcl::make_shared<LineSegments>();
  cv::Mat debug_image = cv::Mat::zeros(800, 800, CV_8UC3);

  const Sophus::SE3f pose = common::pose_to_se3(latest_pose_.value().pose);
  for (const auto & line : lines) {
    const Eigen::Vector3f p1 = line.getVector3fMap();
    const Eigen::Vector3f p2 = line.getNormalVector3fMap();
    const float length = (p1 - p2).norm();
    const Eigen::Vector3f tangent = (p1 - p2).normalized();

    float score = 0;
    int count = 0;
    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f px = pose * (p1 + tangent * distance);
      cv::Vec2b v2 = cost_map_.at2(px.topRows(2));
      float cos = abs_cos(pose.so3() * tangent, v2[1]);
      score += (cos * v2[0]);
      count++;
    }

    if (score / count > 80) {
      filted->push_back(line);
      cv::line(debug_image, cv2pt(p1), cv2pt(p2), cv::Scalar(255, 0, 0), 2);
    } else {
      cv::line(debug_image, cv2pt(p1), cv2pt(p2), cv::Scalar(0, 0, 255), 1);
    }
  }
  for (const auto & line : reliable) {
    const Eigen::Vector3f p1 = line.getVector3fMap();
    const Eigen::Vector3f p2 = line.getNormalVector3fMap();
    cv::line(debug_image, cv2pt(p1), cv2pt(p2), cv::Scalar(0, 255, 0), 2);
  }

  cv::imshow("debug", debug_image);
  cv::waitKey(10);

  return *filted;
}
}  // namespace pcdless::modularized_particle_filter