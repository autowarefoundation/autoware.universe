#include "sign_board/sign_detector.hpp"

#include "common/util.hpp"
#include "map/ll2_util.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <sign_board/sign_lib.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace sign_board
{
SignDetector::SignDetector() : AbstCorrector("sign_detector"), tf_subscriber_(get_clock())
{
  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  auto cb_map = std::bind(&SignDetector::callbackVectorMap, this, _1);
  auto cb_info = std::bind(&SignDetector::callbackInfo, this, _1);
  auto cb_image = std::bind(&SignDetector::callbackImage, this, _1);
  sub_vector_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_info_ = create_subscription<CameraInfo>("/src_info", 10, cb_info);
  sub_image_ = create_subscription<Image>("/src_image", 10, cb_image);
}

void SignDetector::callbackVectorMap(const HADMapBin & map_bin)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(map_bin);

  const std::set<std::string> sign_board_labels = {"sign-board"};
  const auto & ls_layer = lanelet_map->lineStringLayer;
  sign_boards_ = extractSpecifiedLineString(ls_layer, sign_board_labels);
}

SignDetector::SignBoards SignDetector::extractSpecifiedLineString(
  const lanelet::LineStringLayer & line_string_layer, const std::set<std::string> & visible_labels)
{
  lanelet::ConstLineStrings3d line_strings;
  for (const lanelet::ConstLineString3d & line : line_string_layer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    line_strings.push_back(line);
  }
  return line_strings;
}

void SignDetector::callbackImage(const Image & msg)
{
  // RCLCPP_INFO_STREAM(get_logger(), "callbackImage is called");

  const rclcpp::Time stamp = msg.header.stamp;
  auto array = getSyncronizedParticleArray(stamp);
  if (!array.has_value()) return;
  if (!info_.has_value()) return;
  if (!camera_extrinsic_.has_value()) return;
  if (sign_boards_.empty()) return;

  cv::Mat image = util::decompress2CvMat(msg);
  Eigen::Affine3f mean_affine = util::pose2Affine(meanPose(array.value()));

  std::vector<Contour> contours;
  for (const auto board : sign_boards_) {
    float distance = distanceToSignBoard(board, mean_affine.translation());
    if (distance > 80) continue;
    std::optional<Contour> opt_c = extractSignBoardContour(array.value(), board);
    if (opt_c.has_value()) contours.push_back(opt_c.value());
  }

  for (const auto c : contours) cv::polylines(image, c.polygon, false, randomColor(c.id), 1);
  cv::imshow("test", image);
  cv::waitKey(5);
}

std::optional<SignDetector::Contour> SignDetector::extractSignBoardContour(
  const ParticleArray & array, const SignBoard board)
{
  const Eigen::Matrix3f K =
    Eigen::Map<Eigen::Matrix<double, 3, 3>>(info_->k.data()).cast<float>().transpose();

  // Because sign boards are convex polygons and perspective projection preserves their convexity,
  // to obtain the road sign on a image, we can find the convex hull of the projected vertex.
  Eigen::Affine3f T = camera_extrinsic_.value();

  auto project = [K, T](
                   const Eigen::Affine3f & transform,
                   const Eigen::Vector3f & xyz) -> std::optional<cv::Point2i> {
    Eigen::Vector3f from_camera = K * T.inverse() * transform.inverse() * xyz;
    if (from_camera.z() < 1e-3f) return std::nullopt;
    Eigen::Vector3f uv1 = from_camera /= from_camera.z();
    return cv::Point2i(uv1.x(), uv1.y());
  };

  std::vector<cv::Point2f> projected;
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = util::pose2Affine(p.pose);
    for (const auto & p : board) {
      auto opt_p = project(affine, {p.x(), p.y(), p.z()});
      if (opt_p.has_value()) projected.push_back(opt_p.value());
    }
  }

  if (projected.empty()) return std::nullopt;

  std::vector<cv::Point2i> convex_hull = computeConvexHull(projected);
  return Contour{board.id(), convex_hull};
}

void SignDetector::callbackInfo(const CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_(info_->header.frame_id, "base_link");
}

}  // namespace sign_board

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sign_board::SignDetector>());
  rclcpp::shutdown();
  return 0;
}
