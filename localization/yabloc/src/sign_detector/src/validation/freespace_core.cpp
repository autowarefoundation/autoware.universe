#include "common/util.hpp"
#include "sign_detector/ll2_util.hpp"
#include "validation/freespace.hpp"

#include <opencv4/opencv2/imgproc.hpp>

namespace validation
{
FreeSpace::FreeSpace() : rclcpp::Node("freespace")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();
  auto cb_map = std::bind(&FreeSpace::mapCallback, this, _1);
  auto cb_particle = std::bind(&FreeSpace::particleCallback, this, _1);
  auto cb_pose = std::bind(&FreeSpace::poseCallback, this, _1);
  auto cb_info = std::bind(&FreeSpace::infoCallback, this, _1);
  auto cb_image = std::bind(&FreeSpace::imageCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_particle_ = create_subscription<ParticleArray>("/predicted_particles", 10, cb_particle);
  sub_pose_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_pose);
  sub_info_ = create_subscription<CameraInfo>("/camera_info", 10, cb_info);
  sub_image_ = create_subscription<Image>("/image", 10, cb_image);

  pub_image_ = create_publisher<Image>("/freespace_image", 10);

  segmentation_ = cv::ximgproc::modified::createGraphSegmentation();
}

void FreeSpace::mapCallback(const HADMapBin & msg) { lanelet_map_ = fromBinMsg(msg); }

void FreeSpace::particleCallback(const ParticleArray & particles)
{
  if (linestrings_ == nullptr) return;
  if (!camera_info_.has_value()) return;
  if (!camera_extrinsic_.has_value()) return;

  const rclcpp::Time stamp = particles.header.stamp;
  cv::Mat image = cv::Mat::zeros(cv::Size(camera_info_->width, camera_info_->height), CV_8UC1);
  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(camera_info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();

  for (const Particle & p : particles.particles) {
    auto tmp = p.pose;
    tmp.position.z = 0;
    Eigen::Affine3f transform = util::pose2Affine(tmp);
    auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
      Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
      if (from_camera.z() < 1e-3f) return std::nullopt;
      Eigen::Vector3f uv1 = from_camera /= from_camera.z();
      return cv::Point2i(uv1.x(), uv1.y());
    };

    for (const pcl::PointNormal & pn : *linestrings_) {
      auto opt_from = project(pn.getVector3fMap());
      auto opt_to = project(pn.getNormalVector3fMap());
      if (!opt_from.has_value() || !opt_to.has_value()) continue;

      incrementAlongLine(image, opt_from.value(), opt_to.value());
    }
  }

  cv::applyColorMap(image, image, cv::COLORMAP_JET);

  if (segmented_.size() == image.size()) {
    cv::addWeighted(segmented_, 0.5, image, 0.8, 1, image);
  }

  util::publishImage(*pub_image_, image, stamp);
}

void FreeSpace::poseCallback(const PoseStamped & msg) { extractNearLanelet(msg); }

void FreeSpace::extractNearLanelet(const PoseStamped & pose_stamped)
{
  if (lanelet_map_ == nullptr) return;

  linestrings_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  const rclcpp::Time stamp = pose_stamped.header.stamp;
  auto pos = pose_stamped.pose.position;
  Eigen::Vector3f position;
  position << pos.x, pos.y, 0;  // pos.z;

  const std::unordered_set<std::string> visible_labels = {
    "zebra_marking", "virtual", "line_thin", "line_thick", "pedestrian_marking", "stop_line"};

  auto ori = pose_stamped.pose.orientation;
  float theta = 2.0f * std::atan2(ori.z, ori.w);
  Eigen::Vector2f self_direction(std::cos(theta), std::sin(theta));

  std::unordered_set<lanelet::Id> forward_lane_indices;
  for (lanelet::Lanelet & lane : lanelet_map_->laneletLayer) {
    auto front = lane.centerline2d().front();
    auto back = lane.centerline2d().back();

    Eigen::Vector2f lane_direction;
    lane_direction << front.x() - back.x(), front.y() - back.y();
    lane_direction.normalize();

    if (lane_direction.dot(self_direction) > 0) continue;
    forward_lane_indices.insert(lane.leftBound().id());
    forward_lane_indices.insert(lane.rightBound().id());
  }

  for (lanelet::LineString3d & line : lanelet_map_->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    if (forward_lane_indices.count(line.id()) == 0) continue;

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      Eigen::Vector3f p_vec(p.x(), p.y(), 0);

      if (((p_vec - position).norm() < 50) & from.has_value()) {
        pcl::PointNormal pn;
        pn.x = from->x();
        pn.y = from->y();
        pn.z = 0;
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = 0;
        linestrings_->push_back(pn);
      }
      from = p;
    }
  }
}

void FreeSpace::infoCallback(const CameraInfo & msg)
{
  camera_info_ = msg;
  listenExtrinsicTf(msg.header.frame_id);
}
void FreeSpace::imageCallback(const Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);
  cv::Mat dst_image;
  segmentation_->processImage(resized, dst_image);
  cv::resize(dst_image, dst_image, image.size(), 0, 0, cv::INTER_NEAREST);

  auto color_mapping = [](int segment_id) -> cv::Scalar {
    double base = (double)(segment_id)*0.618033988749895 + 0.24443434;
    return cv::Scalar(std::fmod(base, 1.2) * 360, 0.95 * 255, 0.80 * 255);
  };

  cv::COLOR_BGR2HSV;
  segmented_ = cv::Mat::zeros(dst_image.size(), CV_8UC3);

  uint * p;
  uchar * p2;
  for (int i = 0; i < segmented_.rows; i++) {
    p = dst_image.ptr<uint>(i);
    p2 = segmented_.ptr<uchar>(i);
    for (int j = 0; j < segmented_.cols; j++) {
      cv::Scalar color = color_mapping(p[j]);
      p2[j * 3] = (uchar)color[0];
      p2[j * 3 + 1] = (uchar)color[1];
      p2[j * 3 + 2] = (uchar)color[2];
    }
  }
  cv::cvtColor(segmented_, segmented_, cv::COLOR_HSV2BGR);
}

void FreeSpace::listenExtrinsicTf(const std::string & frame_id)
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

void FreeSpace::incrementAlongLine(cv::Mat image, const cv::Point2i & from, const cv::Point2i & to)
{
  if (image.elemSize() != 1) {
    std::cerr << "In addLine() image.elemSize() must be 1" << std::endl;
    exit(1);
  }

  cv::LineIterator iterator(image, from, to, cv::LINE_4, true);
  const int count = iterator.count;
  for (int i = 0; i < count; i++, ++iterator) {
    uchar * ptr = *iterator;
    ptr[0] = std::min(255, int(ptr[0]) + 5);
  }
}

}  // namespace validation