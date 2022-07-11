#include "common/util.hpp"
#include "imgproc/orientation_optimizer.hpp"
#include "imgproc/vanish.hpp"

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

  {
    RansacVanishParam param;
    param.max_iteration_ = declare_parameter<int>("max_iteration", 500);
    param.sample_count_ = declare_parameter<int>("sample_count", 4);
    param.inlier_ratio_ = declare_parameter<float>("inlier_ratio", 0.2);
    param.error_threshold_ = declare_parameter<float>("error_threshold", 0.996);
    param.lsd_sigma_ = declare_parameter<float>("lsd_sigma", 1.5);

    ransac_vanish_point_ = std::make_shared<RansacVanishPoint>(param);
  }
}

void VanishPoint::callbackImu(const Imu & msg) { imu_buffer_.push_back(msg); }

Sophus::SO3f VanishPoint::integral(const rclcpp::Time & image_stamp)
{
  Sophus::SO3f integrated_rot;

  auto opt_ex =
    tf_subscriber_("traffic_light_left_camera/camera_optical_link", "tamagawa/imu_link");
  if (!opt_ex.has_value()) return integrated_rot;
  Sophus::SO3f ex_rot(opt_ex->rotation());

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
    integrated_rot *= Sophus::SO3f::exp(ex_rot.inverse() * w * dt);

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

  // The argument `normal` is in coordinate of normalized camera, where intrinsic parameter is not
  // considered. If normal is [0,-1,0], then it points upper side of image and it means horizontal
  // plane is literally horizontal.
  // n\cdot [x,y,z]=0

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
}

void VanishPoint::drawVerticalLine(
  const cv::Mat & image, const cv::Point2f & vp, const Eigen::Vector2f & tangent,
  const cv::Scalar & color)
{
  const int H = image.rows;

  Eigen::Vector2f t = tangent;
  if (t.y() < 0) t = -t;

  float lambda1 = -vp.y / (t.y() + 1e-6f);
  float lambda2 = (H - vp.y) / (t.y() + 1e-6f);
  float u1 = lambda1 * tangent.x() + vp.x;
  float u2 = lambda2 * tangent.x() + vp.x;
  cv::line(image, cv::Point(u1, 0), cv::Point2i(u2, H), color, 1);
}

void VanishPoint::callbackImage(const Image & msg)
{
  auto opt_imu_ex =
    tf_subscriber_("traffic_light_left_camera/camera_optical_link", "tamagawa/imu_link");
  auto opt_camera_ex = tf_subscriber_("base_link", "traffic_light_left_camera/camera_optical_link");

  if (!opt_imu_ex.has_value()) return;
  if (!opt_camera_ex.has_value()) return;
  if (!info_.has_value()) return;

  Sophus::SO3f dR = integral(msg.header.stamp);

  cv::Mat image = util::decompress2CvMat(msg);
  cv::Point2f vanish = (*ransac_vanish_point_)(image);
  ransac_vanish_point_->drawActiveLines(image);
  drawVerticalLine(image, vanish, Eigen::Vector2f::UnitY());

  // Visualize estimated vanishing point
  Sophus::SO3f init_rot = Sophus::SO3f(opt_camera_ex->rotation());
  drawHorizontalLine(image, init_rot, cv::Scalar(0, 255, 255));

  const Eigen::Matrix3d Kd = Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).transpose();
  const Eigen::Matrix3f K = Kd.cast<float>();
  Eigen::Vector3f vp = K.inverse() * Eigen::Vector3f(vanish.x, vanish.y, 1);
  Sophus::SO3f opt_rot = opt::optimizeOnce(init_rot, vp, Eigen::Vector2f::UnitY());

  drawHorizontalLine(image, opt_rot, cv::Scalar(0, 255, 0));

  // Sophus::SO3f graph_opt_rot = optimizer_.optimize(dR, vp, Eigen::Vector2f::UnitY(), init_rot);
  // drawHorizontalLine(image, graph_opt_rot, cv::Scalar(255, 0, 255));

  // Pure measurement vanishing point
  cv::circle(image, vanish, 5, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
  cv::circle(image, vanish, 20, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
  cv::imshow("lsd", image);
  cv::waitKey(1);

  RCLCPP_INFO_STREAM(get_logger(), "nice vanish point " << vanish);
}

}  // namespace imgproc