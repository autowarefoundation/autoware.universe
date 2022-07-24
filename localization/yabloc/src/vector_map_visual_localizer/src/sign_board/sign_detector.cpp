#include "sign_board/sign_detector.hpp"

#include "common/util.hpp"
#include "map/ll2_util.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/view/transform.hpp>

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
  if (sign_boards_.empty()) return;

  using Pose = geometry_msgs::msg::Pose;

  cv::Mat image = util::decompress2CvMat(msg);
  cv::imshow("test", image);
  cv::waitKey(5);

  RCLCPP_INFO_STREAM(get_logger(), "search nearest sign-boards");

  Eigen::Affine3f affine = util::pose2Affine(meanPose(array.value()));
  std::cout << affine.matrix() << std::endl;

  // Search nearest sign-boards
  using namespace ranges;
  auto lambda = std::bind(
    &SignDetector::distanceToSignBoard, this, std::placeholders::_1, affine.translation());
  auto distances = sign_boards_ | ranges::views::transform(lambda);
  int step = std::distance(distances.begin(), ranges::min_element(distances));
  lanelet::ConstLineString3d nearest_sign = *std::next(sign_boards_.begin(), step);

  std::cout << nearest_sign.front().x() << " " << nearest_sign.front().y() << " "
            << nearest_sign.front().z() << std::endl;
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
