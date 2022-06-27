#include "common/util.hpp"
#include "particle_filter/sign_corrector.hpp"
#include "sign_detector/ll2_util.hpp"

#include <opencv4/opencv2/imgproc.hpp>

namespace particle_filter
{
SignCorrector::SignCorrector() : rclcpp::Node("sign_corrector")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();
  auto cb_map = std::bind(&SignCorrector::mapCallback, this, _1);
  auto cb_particle = std::bind(&SignCorrector::particleCallback, this, _1);
  auto cb_pose = std::bind(&SignCorrector::poseCallback, this, _1);
  auto cb_info = std::bind(&SignCorrector::infoCallback, this, _1);
  auto cb_image = std::bind(&SignCorrector::imageCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_particle_ = create_subscription<ParticleArray>("/predicted_particles", 10, cb_particle);
  sub_pose_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_pose);
  sub_info_ = create_subscription<CameraInfo>("/camera_info", 10, cb_info);
  sub_image_ = create_subscription<Image>("/image", 10, cb_image);

  pub_image_ = create_publisher<Image>("/sign_image", 10);
}

void SignCorrector::mapCallback(const HADMapBin & msg) { lanelet_map_ = fromBinMsg(msg); }

void SignCorrector::particleCallback(const ParticleArray & particles)
{
  if (linestrings_ == nullptr) return;
  if (!camera_info_.has_value()) return;
  if (!camera_extrinsic_.has_value()) return;
  if (linestrings_->empty()) return;

  RCLCPP_WARN_STREAM(get_logger(), "There are visible sign-board around here");

  const rclcpp::Time stamp = particles.header.stamp;

  cv::Mat image = cv::Mat::zeros(cv::Size(camera_info_->width, camera_info_->height), CV_8UC3);
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(camera_info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();

  for (const Particle & p : particles.particles) {
    Eigen::Affine3f transform = util::pose2Affine(p.pose);
    auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
      Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
      if (from_camera.z() < 1e-3f) return std::nullopt;
      Eigen::Vector3f uv1 = from_camera /= from_camera.z();
      return cv::Point2i(uv1.x(), uv1.y());
    };

    for (const auto pn : *linestrings_) {
      auto from_opt = project(pn.getVector3fMap());
      auto to_opt = project(pn.getNormalVector3fMap());
      if (from_opt.has_value() & to_opt.has_value())
        cv::line(image, from_opt.value(), to_opt.value(), cv::Scalar(0, 0, 255), 1);
    }
  }

  util::publishImage(*pub_image_, image, stamp);
}

void SignCorrector::poseCallback(const PoseStamped & msg) { extractNearSign(msg); }

void SignCorrector::extractNearSign(const PoseStamped & pose_stamped)
{
  if (lanelet_map_ == nullptr) return;

  linestrings_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  const rclcpp::Time stamp = pose_stamped.header.stamp;
  auto pos = pose_stamped.pose.position;
  Eigen::Vector3f position;
  position << pos.x, pos.y, pos.z;

  const std::unordered_set<std::string> visible_labels = {"sign-board"};

  for (lanelet::LineString3d & line : lanelet_map_->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      Eigen::Vector3f p_vec(p.x(), p.y(), 0);

      if (((p_vec - position).norm() < 50) & from.has_value()) {
        pcl::PointNormal pn;
        pn.x = from->x();
        pn.y = from->y();
        pn.z = from->z();
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = p.z();
        linestrings_->push_back(pn);
      }
      from = p;
    }
  }
}

void SignCorrector::infoCallback(const CameraInfo & msg)
{
  camera_info_ = msg;
  listenExtrinsicTf(msg.header.frame_id);
}

void SignCorrector::imageCallback(const Image & msg)
{
  // cv::Mat image = util::decompress2CvMat(msg);
  // cv::Mat edge_image;
  // int size = 3;
  // cv::GaussianBlur(image, image, cv::Size(2 * size + 1, 2 * size + 1), 0, 0);
  // cv::Canny(image, edge_image, 20, 3 * 20, 3);

  // edge_image = cv::Mat::ones(image.size(), CV_8UC1) * 255 - edge_image;
  // std::vector<std::vector<cv::Point>> contours;
  // std::vector<cv::Vec4i> hierarchy;
  // cv::findContours(edge_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

  // cv::cvtColor(edge_image, edge_image, cv::COLOR_GRAY2BGR);
  // cv::drawContours(edge_image, contours, -1, cv::Scalar(0, 0, 255), 1);
  // util::publishImage(*pub_image_, edge_image, msg.header.stamp);
}

void SignCorrector::listenExtrinsicTf(const std::string & frame_id)
{
  try {
    geometry_msgs::msg::TransformStamped ts =
      tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero);
    Eigen::Vector3f p;
    p.x() = ts.transform.translation.x;
    p.y() = ts.transform.translation.y;
    p.z() = ts.transform.translation.z;

    Eigen::Quaternionf q;
    q.w() = ts.transform.rotation.w;
    q.x() = ts.transform.rotation.x;
    q.y() = ts.transform.rotation.y;
    q.z() = ts.transform.rotation.z;

    camera_extrinsic_ = Eigen::Affine3f::Identity();
    camera_extrinsic_->translation() = p;
    camera_extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
  } catch (tf2::TransformException & ex) {
  }
}
}  // namespace particle_filter