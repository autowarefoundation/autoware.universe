#include "common/color.hpp"
#include "common/util.hpp"
#include "imgproc/orientation_optimizer.hpp"
#include "imgproc/vanish.hpp"

#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

namespace imgproc
{
VanishPoint::VanishPoint() : Node("vanish_point"), tf_subscriber_(get_clock())
{
  using std::placeholders::_1;
  auto cb_image = std::bind(&VanishPoint::callbackImage, this, _1);
  auto cb_imu = std::bind(&VanishPoint::callbackImu, this, _1);

  sub_imu_ = create_subscription<Imu>("/sensing/imu/tamagawa/imu_raw", 10, cb_imu);
  sub_image_ =
    create_subscription<Image>("/sensing/camera/traffic_light/image_raw/compressed", 10, cb_image);
  sub_info_ = create_subscription<CameraInfo>(
    "/sensing/camera/traffic_light/camera_info", 10,
    [this](const CameraInfo & msg) -> void { this->info_ = msg; });

  ransac_vanish_point_ = std::make_shared<RansacVanishPoint>(this);
  optimizer_ = std::make_shared<opt::Optimizer>(this);
}

void VanishPoint::callbackImu(const Imu & msg) { imu_buffer_.push_back(msg); }

void VanishPoint::projectOnPlane(const Sophus::SO3f & rotation, const cv::Mat & lines)
{
  // Prepare vairables
  auto opt_camera_ex = tf_subscriber_("traffic_light_left_camera/camera_optical_link", "base_link");
  const Eigen::Matrix3d Kd = Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).transpose();
  const Eigen::Matrix3f K = Kd.cast<float>();
  const float max_range = 20;
  const float image_size = 800;
  const Eigen::Matrix3f Kinv = K.inverse();
  const Eigen::Vector3f t = opt_camera_ex->translation();
  const Eigen::Quaternionf q(opt_camera_ex->rotation());

  // Prepare functions
  auto toCvPoint = [max_range, image_size](const Eigen::Vector3f & v) -> cv ::Point2i {
    cv::Point pt;
    pt.x = -v.y() / max_range * image_size * 0.5f + image_size / 2;
    pt.y = -v.x() / max_range * image_size * 0.5f + image_size;
    return pt;
  };

  auto project_func = [Kinv, q, t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f u_bearing = (q * Kinv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };

  // Project all lins on ground plane
  cv::Mat image = cv::Mat::zeros(cv::Size(image_size, image_size), CV_8UC3);
  for (int i = 0; i < lines.rows; i++) {
    cv::Mat xy_xy(lines.row(i));
    Eigen::Vector3f from, to;
    from << xy_xy.at<float>(0), xy_xy.at<float>(1), 1;
    to << xy_xy.at<float>(2), xy_xy.at<float>(3), 1;
    std::optional<Eigen::Vector3f> opt1 = project_func(from);
    std::optional<Eigen::Vector3f> opt2 = project_func(to);
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;
    cv::line(image, toCvPoint(*opt1), toCvPoint(*opt2), cv::Scalar(0, 255, 255), 1);
  }
  cv::imshow("projected", image);
}

void VanishPoint::rotateImage(const cv::Mat & image, const Sophus::SO3f & rot)
{
  const Eigen::Matrix3d Kd = Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).transpose();
  const Eigen::Matrix3f K = Kd.cast<float>();
  const Eigen::Matrix3f Kinv = K.inverse();

  const cv::Size size = image.size();
  cv::Mat xmap = cv::Mat::zeros(size, CV_32FC1);
  cv::Mat ymap = cv::Mat::zeros(size, CV_32FC1);

  for (int i = 0; i < size.width; i++) {
    for (int j = 0; j < size.height; j++) {
      Eigen::Vector3f u(i, j, 1);
      Eigen::Vector3f v = K * (rot * (Kinv * u));
      v = v / v.z();

      xmap.at<float>(j, i) = v.x();
      ymap.at<float>(j, i) = v.y();
    }
  }

  cv::Mat rotated_image;
  cv::remap(image, rotated_image, xmap, ymap, cv::INTER_LINEAR);
  cv::imshow("rotation corected", rotated_image);
}

Sophus::SO3f VanishPoint::integral(const rclcpp::Time & image_stamp)
{
  auto opt_ex = tf_subscriber_("base_link", "tamagawa/imu_link");
  if (!opt_ex.has_value()) return Sophus::SO3f();
  const Sophus::SO3f ex_rot(opt_ex->rotation());

  Sophus::SO3f integrated_rot;
  while (!imu_buffer_.empty()) {
    Imu msg = imu_buffer_.front();
    rclcpp::Time stamp = msg.header.stamp;
    if (stamp > image_stamp) break;

    if (!last_imu_stamp_.has_value()) {
      last_imu_stamp_ = stamp;
      continue;
    }
    const float dt = (stamp - last_imu_stamp_.value()).seconds();

    Eigen::Vector3f w;
    w << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
    integrated_rot *= Sophus::SO3f::exp(ex_rot * w * dt);

    imu_buffer_.pop_front();
    last_imu_stamp_ = stamp;
  }
  return integrated_rot;
}

void VanishPoint::drawHorizontalLine(
  const cv::Mat & image, const Sophus::SO3f & rot, const cv::Scalar & color, int thick)
{
  const Eigen::Vector3f normal = rot * Eigen::Vector3f::UnitZ();
  const Eigen::Matrix3d Kd = Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).transpose();
  const Eigen::Matrix3f K = Kd.cast<float>();
  const int W = info_->width;
  const float cx = K(0, 2);
  const float fx = K(0, 0);

  // The argument `rot` is rotation in coordinate of normalized camera, where intrinsic parameter is
  // not considered. If normal is [0,-1,0], then it points upper side of image and it means
  // horizontal plane is literally horizontal. n\cdot [x,y,z]=0

  // This function assumes that horizontal line accross image almost horizontally.
  // So, all we need to compute is the left point and right point of line.
  // The left point is [0, v1] and the right point is [width, v2].
  // [0, v1] is projected intersection of horizontal plane and left side line in  normalized camera
  // coordinate. [0, v1, 1]=Kp, p is the intersection.

  auto intersection = [&normal](
                        const Eigen::Vector3f & s, const Eigen::Vector3f & t) -> Eigen::Vector3f {
    float lambda = -normal.dot(s) / (normal.dot(t) + 1e-6f);
    return s + lambda * t;
  };
  Eigen::Vector3f pl = intersection({(-cx) / fx, 0, 1}, Eigen::Vector3f::UnitY());
  Eigen::Vector3f pr = intersection({(W - cx) / fx, 0, 1}, Eigen::Vector3f::UnitY());
  float v1 = (K * pl)(1);
  float v2 = (K * pr)(1);
  cv::line(image, cv::Point(0, v1), cv::Point2i(W, v2), color, thick);

  for (int i = 0; i < 20; i++) {
    cv::Scalar color = util::toJet(i / 20.f);
    Eigen::Vector3f azimuth(std::sin(M_PI * i / 10), std::cos(M_PI * i / 10), 0);
    azimuth = rot * azimuth;
    if (azimuth.z() < 1e-3f) continue;
    Eigen::Vector3f u = K * azimuth;
    u = u / u.z();
    cv::circle(image, cv::Point(u.x(), u.y()), 4, color, -1);
  }
}

void VanishPoint::drawGridLine(const cv::Mat & image)
{
  const int H = image.rows;
  const int W = image.cols;
  const int N = 5;
  for (int i = 0; i < N - 1; i++) {
    cv::line(
      image, cv::Point(0, (i + 1) * H / N), cv::Point2i(W, (i + 1) * H / N),
      cv::Scalar(255, 255, 0), 1);
  }
}

void VanishPoint::drawCrossLine(
  const cv::Mat & image, const cv::Point2f & vp, const cv::Scalar & color)
{
  const int H = image.rows;
  const int W = image.cols;
  cv::line(image, cv::Point(vp.x, 0), cv::Point2i(vp.x, H), color, 1);
  cv::line(image, cv::Point(0, vp.y), cv::Point2i(W, vp.y), color, 1);
  cv::circle(image, vp, 5, color, 1, cv::LINE_8);
}

void VanishPoint::callbackImage(const Image & msg)
{
  // Prepare coodinate transformation
  auto opt_imu_ex =
    tf_subscriber_("traffic_light_left_camera/camera_optical_link", "tamagawa/imu_link");
  auto opt_camera_ex = tf_subscriber_("base_link", "traffic_light_left_camera/camera_optical_link");
  const Sophus::SO3f init_rot = Sophus::SO3f(opt_camera_ex->rotation());
  const Eigen::Matrix3d Kd = Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).transpose();
  const Eigen::Matrix3f K = Kd.cast<float>();

  // Return soon if at least one transformation is not ready
  if (!opt_imu_ex.has_value()) return;
  if (!opt_camera_ex.has_value()) return;
  if (!info_.has_value()) return;

  // Obtain relative rotation from last frame
  Sophus::SO3f dR = integral(msg.header.stamp);

  // Detect vanishing point using LSD & RANSAC
  cv::Mat image = util::decompress2CvMat(msg);
  cv::Mat lines = ransac_vanish_point_->lsd(image);
  OptPoint2f vanish = ransac_vanish_point_->estimate(lines);
  ransac_vanish_point_->drawActiveLines(image);
  if (vanish.has_value()) drawCrossLine(image, vanish.value(), cv::Scalar(0, 255, 0));
  // drawHorizontalLine(image, init_rot, cv::Scalar(200, 255, 0), 1);

  // Transform vanishing point from the image coordinate to camera coordinate
  std::optional<Eigen::Vector3f> vp_image = std::nullopt;
  if (vanish.has_value()) {
    vp_image = K.inverse() * Eigen::Vector3f(vanish->x, vanish->y, 1);
  }

  // Optimize using graph optimizer
  Sophus::SO3f graph_opt_rot =
    optimizer_->optimize(dR, vp_image, Eigen::Vector2f::UnitY(), init_rot);
  drawHorizontalLine(image, graph_opt_rot, cv::Scalar(255, 0, 0));

  // Visualize
  // projectOnPlane(graph_opt_rot.inverse(), lines);
  Sophus::SO3f correction = opt::extractNominalRotation(init_rot, graph_opt_rot);
  rotateImage(image, correction);
  cv::imshow("vanishing point", image);
  cv::waitKey(1);
}

}  // namespace imgproc