#include "sign_board/sign_detector.hpp"

#include "common/util.hpp"
#include "map/ll2_util.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/view/transform.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

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

float SignDetector::distanceToSignBoard(
  const lanelet::ConstLineString3d & board, const Eigen::Vector3f & position)
{
  int vertex_count = 0;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (auto p : board) {
    centroid += Eigen::Vector3f(p.x(), p.y(), p.z());
    vertex_count++;
  }
  centroid /= static_cast<float>(vertex_count);
  return (centroid - position).norm();
}

void SignDetector::callbackImage(const Image & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "callbackImage is called");

  const rclcpp::Time stamp = msg.header.stamp;
  auto array = getSyncronizedParticleArray(stamp);
  if (!array.has_value()) return;
  if (!info_.has_value()) return;
  if (!camera_extrinsic_.has_value()) return;
  if (sign_boards_.empty()) return;

  using Pose = geometry_msgs::msg::Pose;

  cv::Mat image = util::decompress2CvMat(msg);
  Eigen::Affine3f affine = util::pose2Affine(meanPose(array.value()));

  // Search nearest sign-boards
  using namespace ranges;
  auto lambda = std::bind(
    &SignDetector::distanceToSignBoard, this, std::placeholders::_1, affine.translation());
  auto distances = sign_boards_ | ranges::views::transform(lambda);
  int step = std::distance(distances.begin(), ranges::min_element(distances));
  lanelet::ConstLineString3d nearest_sign = *std::next(sign_boards_.begin(), step);

  float shortest_distance = distances.at(step);
  RCLCPP_INFO_STREAM(
    get_logger(), "nearest sign-boards: " << nearest_sign.id() << " " << shortest_distance);

  drawSignBoardContour(image, array.value(), nearest_sign, affine);
}

std::vector<cv::Point2i> computeConvexHull(const std::vector<cv::Point2f> & src)
{
  namespace bg = boost::geometry;

  typedef bg::model::d2::point_xy<double> point;
  typedef bg::model::polygon<point> polygon;

  polygon poly;
  // std::cout << "raw: ";
  for (const auto & p : src) {
    bg::append(poly, point(p.x, p.y));
    // std::cout << p << " ";
  }
  // std::cout << std::endl;

  polygon hull;
  bg::convex_hull(poly, hull);

  std::vector<cv::Point2i> dst;
  std::cout << "convex: ";
  for (auto p : hull.outer()) {
    cv::Point2f cvp(p.x(), p.y());
    std::cout << cvp << " ";
    dst.push_back(cvp);
  }
  std::cout << std::endl;
  return dst;
}

void SignDetector::drawSignBoardContour(
  const cv::Mat & image, const ParticleArray & array, const SignBoard board,
  const Eigen::Affine3f & affine)
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

  if (projected.empty()) return;

  std::vector<cv::Point2i> contour = computeConvexHull(projected);
  if (contour.empty()) return;

  cv::polylines(image, contour, false, cv::Scalar(0, 255, 255), 2);
  cv::imshow("test", image);
  cv::waitKey(5);
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
