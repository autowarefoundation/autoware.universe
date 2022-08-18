#include "sign_board/sign_lib.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <random>

namespace sign_board
{
cv::Scalar randomColor(signed long index)
{
  std::mt19937 engine(std::hash<signed long>()(index));
  unsigned char r = engine();
  unsigned char g = engine();
  unsigned char b = engine();
  return cv::Scalar(r, g, b);
}

float distanceToSignBoard(
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

std::vector<cv::Point2i> computeConvexHull(const std::vector<cv::Point2f> & src)
{
  namespace bg = boost::geometry;
  using point = bg::model::d2::point_xy<float>;
  using polygon = bg::model::polygon<point>;

  polygon poly;
  for (const auto & p : src) bg::append(poly, point(p.x, p.y));

  polygon hull;
  bg::convex_hull(poly, hull);

  std::vector<cv::Point2i> dst;
  for (auto p : hull.outer()) dst.push_back(cv::Point2i(p.x(), p.y()));
  return dst;
}
}  // namespace sign_board