#include "camera_pose_initializer/lane_image.hpp"

#include <opencv2/imgproc.hpp>

#include <boost/assert.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/range/adaptors.hpp>

namespace pcdless
{
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point_t;
typedef bg::model::box<point_t> box_t;
typedef bg::model::polygon<point_t> polygon_t;

LaneImage::LaneImage(lanelet::LaneletMapPtr map) : map_(map) {}

cv::Point2i to_cv_point(const Eigen::Vector3f & v)
{
  const float image_size_ = 800;
  const float max_range_ = 20;

  cv::Point pt;
  pt.x = -v.y() / max_range_ * image_size_ * 0.5f + image_size_ / 2;
  pt.y = -v.x() / max_range_ * image_size_ * 0.5f + image_size_;
  return pt;
}

void draw_lane(cv::Mat & image, const polygon_t & polygon)
{
  std::vector<cv::Point> contour;
  for (auto p : polygon.outer()) {
    cv::Point2i pt = to_cv_point({p.x(), p.y(), 0});
    contour.push_back(pt);
  }

  std::vector<std::vector<cv::Point> > contours;
  contours.push_back(contour);
  cv::drawContours(image, contours, -1, cv::Scalar(255, 0, 0), -1);
}

void draw_line(cv::Mat & image, const lanelet::LineString2d & line, geometry_msgs::msg::Point xyz)
{
  std::vector<cv::Point> contour;
  for (auto p : line) {
    cv::Point2i pt = to_cv_point({p.x() - xyz.x, p.y() - xyz.y, 0});
    contour.push_back(pt);
  }
  cv::polylines(image, contour, false, cv::Scalar(0, 0, 255), 2);
}

cv::Mat LaneImage::get_image(const Pose & pose)
{
  const auto xyz = pose.position;
  box_t box(point_t(-20, -20), point_t(20, 20));

  cv::Mat image = cv::Mat::zeros(cv::Size(800, 800), CV_8UC3);

  std::vector<lanelet::Lanelet> joint_lanes;
  for (auto lanelet : map_->laneletLayer) {
    polygon_t polygon;
    for (auto right : lanelet.rightBound2d()) {
      polygon.outer().push_back(point_t(right.x() - xyz.x, right.y() - xyz.y));
    }
    for (auto left : boost::adaptors::reverse(lanelet.leftBound2d())) {
      polygon.outer().push_back(point_t(left.x() - xyz.x, left.y() - xyz.y));
    }

    if (!bg::disjoint(box, polygon)) {
      joint_lanes.push_back(lanelet);
      draw_lane(image, polygon);
    }
  }
  for (auto lanelet : joint_lanes) {
    draw_line(image, lanelet.rightBound2d(), xyz);
    draw_line(image, lanelet.leftBound2d(), xyz);
  }

  return image;
}

}  // namespace pcdless