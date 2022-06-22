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
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_particle_ = create_subscription<ParticleArray>("/predicted_particles", 10, cb_particle);
  sub_pose_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_pose);
  sub_info_ = create_subscription<CameraInfo>("/camera_info", 10, cb_info);

  pub_image_ = create_publisher<Image>("/freespace_image", 10);
}

void FreeSpace::mapCallback(const HADMapBin & msg)
{
  lanelet_map_ = fromBinMsg(msg);
  RCLCPP_INFO_STREAM(get_logger(), "lanelet: " << lanelet_map_->laneletLayer.size());
  RCLCPP_INFO_STREAM(get_logger(), "line: " << lanelet_map_->lineStringLayer.size());
  RCLCPP_INFO_STREAM(get_logger(), "point: " << lanelet_map_->pointLayer.size());
}

void FreeSpace::particleCallback(const ParticleArray &) {}

void FreeSpace::poseCallback(const PoseStamped & msg) { execute(msg); }

void FreeSpace::execute(const PoseStamped & pose_stamped)
{
  if (lanelet_map_ == nullptr) return;
  if (!camera_info_.has_value()) return;
  if (!camera_extrinsic_.has_value()) return;

  cv::Mat image = cv::Mat::zeros(cv::Size(camera_info_->width, camera_info_->height), CV_8UC3);
  auto pos = pose_stamped.pose.position;

  const rclcpp::Time stamp = pose_stamped.header.stamp;
  Eigen::Vector3f position;
  position << pos.x, pos.y, 0;  // pos.z;

  Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(camera_info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f T = camera_extrinsic_.value();
  auto tmp_pose = pose_stamped.pose;
  tmp_pose.position.z = 0;
  Eigen::Affine3f transform = util::pose2Affine(tmp_pose);

  auto project = [K, T, transform](const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  for (const auto lanelet : lanelet_map_->laneletLayer) {
    auto itr = lanelet.centerline2d().begin();
    auto end = lanelet.centerline2d().end();
    Eigen::Vector3f from(itr->x(), itr->y(), 0);

    while (itr != end) {
      Eigen::Vector3f to(itr->x(), itr->y(), 0);

      if ((position - from).norm() > 50) {
        from = to;
        itr++;
        continue;
      }

      auto from_opt = project(from);
      auto to_opt = project(to);
      if (from_opt.has_value() && to_opt.has_value())
        cv::line(image, from_opt.value(), to_opt.value(), cv::Scalar(255, 255, 255), 3);
      from = to;
      itr++;
    }
  }

  util::publishImage(*pub_image_, image, stamp);
}

void FreeSpace::infoCallback(const CameraInfo & msg)
{
  camera_info_ = msg;
  listenExtrinsicTf(msg.header.frame_id);
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

}  // namespace validation