// Copyright 2023-2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/universe_utils/geometry/geometry.hpp"

#include "autoware/universe_utils/geometry/gjk_2d.hpp"

#include <Eigen/Geometry>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/convert.h>

namespace tf2
{
void fromMsg(const geometry_msgs::msg::PoseStamped & msg, tf2::Stamped<tf2::Transform> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.pose, tmp);
  out.setData(tmp);
}
}  // namespace tf2

namespace autoware::universe_utils
{
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Quaternion & quat)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose)
{
  return getRPY(pose.orientation);
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose)
{
  return getRPY(pose.pose);
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  return getRPY(pose.pose.pose);
}

geometry_msgs::msg::Quaternion createQuaternion(
  const double x, const double y, const double z, const double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

// Revival of tf::createQuaternionFromRPY
// https://answers.ros.org/question/304397/recommended-way-to-construct-quaternion-from-rollpitchyaw-with-tf2/
geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

double calcElevationAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dz = p_to.z - p_from.z;
  const double dist_2d = calcDistance2d(p_from, p_to);
  return std::atan2(dz, dist_2d);
}

/**
 * @brief calculate azimuth angle of two points.
 * @details This function returns the azimuth angle of the position of the two input points
 *          with respect to the origin of their coordinate system.
 *          If x and y of the two points are the same, the calculation result will be unstable.
 * @param p_from source point
 * @param p_to target point
 * @return -pi < azimuth angle < pi.
 */
double calcAzimuthAngle(
  const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

geometry_msgs::msg::Pose transform2pose(const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x;
  pose.position.y = transform.translation.y;
  pose.position.z = transform.translation.z;
  pose.orientation = transform.rotation;
  return pose;
}

geometry_msgs::msg::PoseStamped transform2pose(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose = transform2pose(transform.transform);
  return pose;
}

geometry_msgs::msg::Transform pose2transform(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;
  return transform;
}

geometry_msgs::msg::TransformStamped pose2transform(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = pose.header;
  transform.transform = pose2transform(pose.pose);
  transform.child_frame_id = child_frame_id;
  return transform;
}

Point3d transformPoint(const Point3d & point, const geometry_msgs::msg::Transform & transform)
{
  const auto & translation = transform.translation;
  const auto & rotation = transform.rotation;

  const Eigen::Translation3d T(translation.x, translation.y, translation.z);
  const Eigen::Quaterniond R(rotation.w, rotation.x, rotation.y, rotation.z);

  const Eigen::Vector3d transformed(T * R * point);

  return Point3d{transformed.x(), transformed.y(), transformed.z()};
}

Point2d transformPoint(const Point2d & point, const geometry_msgs::msg::Transform & transform)
{
  Point3d point_3d{point.x(), point.y(), 0};
  const auto transformed = transformPoint(point_3d, transform);
  return Point2d{transformed.x(), transformed.y()};
}

Eigen::Vector3d transformPoint(const Eigen::Vector3d & point, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.translation.z = pose.position.z;
  transform.rotation = pose.orientation;

  Point3d p = transformPoint(Point3d(point.x(), point.y(), point.z()), transform);
  return Eigen::Vector3d(p.x(), p.y(), p.z());
}

geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Vector3d vec = Eigen::Vector3d(point.x, point.y, point.z);
  auto transformed_vec = transformPoint(vec, pose);

  geometry_msgs::msg::Point transformed_point;
  transformed_point.x = transformed_vec.x();
  transformed_point.y = transformed_vec.y();
  transformed_point.z = transformed_vec.z();
  return transformed_point;
}

geometry_msgs::msg::Point32 transformPoint(
  const geometry_msgs::msg::Point32 & point32, const geometry_msgs::msg::Pose & pose)
{
  const auto point =
    geometry_msgs::build<geometry_msgs::msg::Point>().x(point32.x).y(point32.y).z(point32.z);
  const auto transformed_point = autoware::universe_utils::transformPoint(point, pose);
  return geometry_msgs::build<geometry_msgs::msg::Point32>()
    .x(transformed_point.x)
    .y(transformed_point.y)
    .z(transformed_point.z);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform = transform;

  return transformPose(pose, transform_stamped);
}

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & pose_transform)
{
  tf2::Transform transform;
  tf2::convert(pose_transform, transform);

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.transform = tf2::toMsg(transform);

  return transformPose(pose, transform_msg);
}

// Transform pose in world coordinates to local coordinates
/*
geometry_msgs::msg::Pose inverseTransformPose(
const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
tf2::Transform tf;
tf2::fromMsg(transform, tf);
geometry_msgs::msg::TransformStamped transform_stamped;
transform_stamped.transform = tf2::toMsg(tf.inverse());

return transformPose(pose, transform_stamped);
}
*/

// Transform pose in world coordinates to local coordinates
geometry_msgs::msg::Pose inverseTransformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Transform & transform)
{
  tf2::Transform tf;
  tf2::fromMsg(transform, tf);
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform = tf2::toMsg(tf.inverse());

  return transformPose(pose, transform_stamped);
}

// Transform pose in world coordinates to local coordinates
geometry_msgs::msg::Pose inverseTransformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & transform_pose)
{
  tf2::Transform transform;
  tf2::convert(transform_pose, transform);

  return inverseTransformPose(pose, tf2::toMsg(transform));
}

// Transform point in world coordinates to local coordinates
Eigen::Vector3d inverseTransformPoint(
  const Eigen::Vector3d & point, const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();

  const Eigen::Vector3d local_origin(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Vector3d local_point = R.transpose() * point - R.transpose() * local_origin;

  return local_point;
}

// Transform point in world coordinates to local coordinates
geometry_msgs::msg::Point inverseTransformPoint(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Vector3d local_vec =
    inverseTransformPoint(Eigen::Vector3d(point.x, point.y, point.z), pose);
  geometry_msgs::msg::Point local_point;
  local_point.x = local_vec.x();
  local_point.y = local_vec.y();
  local_point.z = local_vec.z();
  return local_point;
}

double calcCurvature(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3)
{
  // Calculation details are described in the following page
  // https://en.wikipedia.org/wiki/Menger_curvature
  const double denominator =
    calcDistance2d(p1, p2) * calcDistance2d(p2, p3) * calcDistance2d(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / denominator;
}

/**
 * @brief Calculate offset pose. The offset values are defined in the local coordinate of the input
 * pose.
 */
geometry_msgs::msg::Pose calcOffsetPose(
  const geometry_msgs::msg::Pose & p, const double x, const double y, const double z,
  const double yaw)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Transform transform;
  transform.translation = createTranslation(x, y, z);
  transform.rotation = createQuaternionFromYaw(yaw);
  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(p, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, pose);
  return pose;
}

/**
 * @brief Judge whether twist covariance is valid.
 *
 * @param twist_with_covariance source twist with covariance
 * @return If all element of covariance is 0, return false.
 */
//
bool isTwistCovarianceValid(const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance)
{
  for (const auto & c : twist_with_covariance.covariance) {
    if (c != 0.0) {
      return true;
    }
  }
  return false;
}

bool intersects_convex(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  return gjk::intersects(convex_polygon1, convex_polygon2);
}

// Alternatives for Boost.Geometry ----------------------------------------------------------------

namespace alt
{
Point fromGeom(const geometry_msgs::msg::Point & point)
{
  Point _point;
  tf2::fromMsg(point, _point);
  return _point;
}

Polygon fromGeom(const std::vector<geometry_msgs::msg::Point> & polygon)
{
  Polygon _polygon;
  for (const auto & point : polygon) {
    _polygon.push_back(fromGeom(point));
  }
  return _polygon;
}

Point fromBoost(const Point2d & point)
{
  return Point(point.x(), point.y(), 0.0);
}

Polygon fromBoost(const Polygon2d & polygon)
{
  Polygon _polygon;
  for (const auto & point : polygon.outer()) {
    _polygon.push_back(fromBoost(point));
  }
  correct(_polygon);
  return _polygon;
}

Point2d toBoost(const Point & point)
{
  return Point2d(point.x(), point.y());
}

Polygon2d toBoost(const Polygon & polygon)
{
  Polygon2d _polygon;
  for (const auto & point : polygon) {
    _polygon.outer().push_back(toBoost(point));
  }
  return _polygon;
}
}  // namespace alt

std::optional<double> area(const alt::Polygon & poly)
{
  if (poly.size() < 3) {
    return std::nullopt;
  }

  double area = 0.;
  for (size_t i = 0; i < poly.size(); ++i) {
    area += poly.at((i + 1) % poly.size()).cross(poly.at(i)).z() / 2;
  }

  return area;
}

std::optional<alt::Polygon> convexHull(const alt::PointList & points)
{
  if (points.size() < 3) {
    return std::nullopt;
  }

  // QuickHull algorithm

  const auto p_minmax_itr =
    std::minmax_element(points.begin(), points.end(), [](const auto & a, const auto & b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
  const auto & p_min = *p_minmax_itr.first;
  const auto & p_max = *p_minmax_itr.second;

  alt::Polygon hull;

  auto make_hull = [&hull](
                     auto self, const alt::Point & p1, const alt::Point & p2,
                     const alt::PointList & points) {
    if (points.empty()) {
      return;
    }

    const auto farthest = *std::max_element(
      points.begin(), points.end(),
      [&](const auto & a, const auto & b) { return distance(p1, p2, a) < distance(p1, p2, b); });

    const auto subsets_1 = divideBySegment(points, p1, farthest);
    const auto subsets_2 = divideBySegment(points, farthest, p2);

    self(self, p1, farthest, subsets_1.at(0));
    hull.push_back(farthest);
    self(self, farthest, p2, subsets_2.at(0));
  };

  const auto [above_points, below_points] = divideBySegment(points, p_min, p_max);
  hull.push_back(p_min);
  make_hull(make_hull, p_min, p_max, above_points);
  hull.push_back(p_max);
  make_hull(make_hull, p_max, p_min, below_points);
  correct(hull);

  return hull;
}

void correct(alt::Polygon & poly)
{
  if (poly.size() < 3) {
    return;
  }

  // sort points in clockwise order with respect to the first point
  std::sort(poly.begin() + 1, poly.end(), [&](const auto & a, const auto & b) {
    return (a - poly.front()).cross(b - poly.front()).z() < 0;
  });

  if (
    std::abs(poly.front().x() - poly.back().x()) <= std::numeric_limits<double>::epsilon() &&
    std::abs(poly.front().y() - poly.back().y()) <= std::numeric_limits<double>::epsilon()) {
    poly.pop_back();
  }
}

std::optional<bool> coveredBy(const alt::Point & point, const alt::Polygon & poly)
{
  const auto is_point_within = within(point, poly);
  if (!is_point_within) {
    return std::nullopt;
  } else if (*is_point_within) {
    return true;
  }

  if (touches(point, poly).value_or(false)) {
    return true;
  }

  return false;
}

std::optional<bool> disjoint(const alt::Polygon & poly1, const alt::Polygon & poly2)
{
  const auto is_equal = equals(poly1, poly2);
  if (is_equal.value_or(false)) {
    return false;
  } else if (!is_equal) {
    return std::nullopt;
  }

  if (intersects(poly1, poly2).value_or(false)) {
    return false;
  }

  for (const auto & point : poly1) {
    if (touches(point, poly2).value_or(false)) {
      return false;
    }
  }

  return true;
}

double distance(const alt::Point & point, const alt::Point & seg_start, const alt::Point & seg_end)
{
  const auto seg_vec = seg_end - seg_start;
  const auto point_vec = point - seg_start;

  const double seg_vec_norm = seg_vec.length();
  const double seg_point_dot = seg_vec.dot(point_vec);

  if (seg_vec_norm <= std::numeric_limits<double>::epsilon() || seg_point_dot < 0) {
    return point_vec.length();
  } else if (seg_point_dot > std::pow(seg_vec_norm, 2)) {
    return (point - seg_end).length();
  } else {
    return std::abs(seg_vec.cross(point_vec).z()) / seg_vec_norm;
  }
}

double distance(const alt::Point & point, const alt::Polygon & poly)
{
  if (coveredBy(point, poly).value_or(false)) {
    return 0.0;
  }

  // TODO(mitukou1109): Use plane sweep method to improve performance?
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poly.size(); ++i) {
    min_distance =
      std::min(min_distance, distance(point, poly.at(i), poly.at((i + 1) % poly.size())));
  }

  return min_distance;
}

std::array<alt::PointList, 2> divideBySegment(
  const alt::PointList & points, const alt::Point & seg_start, const alt::Point & seg_end)
{
  alt::PointList above_points, below_points;

  for (const auto & point : points) {
    if (isAbove(point, seg_start, seg_end)) {
      above_points.push_back(point);
    } else {
      below_points.push_back(point);
    }
  }

  return {above_points, below_points};
}

std::optional<bool> equals(const alt::Polygon & poly1, const alt::Polygon & poly2)
{
  if (poly1.size() < 3 || poly2.size() < 3) {
    return std::nullopt;
  }

  return std::all_of(poly1.begin(), poly1.end(), [&](const auto & a) {
    return std::any_of(poly2.begin(), poly2.end(), [&](const auto & b) {
      return std::abs(a.x() - b.x()) <= std::numeric_limits<double>::epsilon() &&
             std::abs(a.y() - b.y()) <= std::numeric_limits<double>::epsilon();
    });
  });
}

std::optional<alt::Point> intersect(
  const alt::Point & seg1_start, const alt::Point & seg1_end, const alt::Point & seg2_start,
  const alt::Point & seg2_end)
{
  const auto v1 = seg1_end - seg1_start;
  const auto v2 = seg2_end - seg2_start;

  const auto det = v1.cross(v2).z();
  if (std::abs(det) <= std::numeric_limits<double>::epsilon()) {
    return std::nullopt;
  }

  const auto v12 = seg2_end - seg1_end;
  const double t = v2.cross(v12).z() / det;
  const double s = v1.cross(v12).z() / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return std::nullopt;
  }

  return t * seg1_start + (1.0 - t) * seg1_end;
}

std::optional<bool> intersects(const alt::Polygon & poly1, const alt::Polygon & poly2)
{
  // if the polygons are equal, return true
  const auto is_equal = equals(poly1, poly2);
  if (!is_equal || *is_equal) {
    return is_equal;
  }

  // GJK algorithm

  auto find_support_vertex =
    [](const alt::Polygon & poly1, const alt::Polygon & poly2, const tf2::Vector3 & direction) {
      auto find_farthest_vertex = [](const alt::Polygon & poly, const tf2::Vector3 & direction) {
        return std::max_element(poly.begin(), poly.end(), [&](const auto & a, const auto & b) {
          return direction.dot(a) <= direction.dot(b);
        });
      };
      return *find_farthest_vertex(poly1, direction) - *find_farthest_vertex(poly2, -direction);
    };

  tf2::Vector3 direction = {1.0, 0.0, 0.0};
  auto a = find_support_vertex(poly1, poly2, direction);
  direction = -a;
  auto b = find_support_vertex(poly1, poly2, direction);
  if (b.dot(direction) <= 0.0) {
    return false;
  }

  direction = (b - a).cross(-a).cross(b - a);
  while (true) {
    auto c = find_support_vertex(poly1, poly2, direction);
    if (c.dot(direction) <= 0.0) {
      return false;
    }

    auto n_ca = (b - c).cross(a - c).cross(a - c);
    if (n_ca.dot(-c) > 0.0) {
      b = c;
      direction = n_ca;
    } else {
      auto n_cb = (a - c).cross(b - c).cross(b - c);
      if (n_cb.dot(-c) > 0.0) {
        a = c;
        direction = n_cb;
      } else {
        break;
      }
    }
  }

  return true;
}

bool isAbove(const alt::Point & point, const alt::Point & seg_start, const alt::Point & seg_end)
{
  return (seg_end - seg_start).cross(point - seg_start).z() > 0;
}

std::optional<bool> isClockwise(const alt::Polygon & poly)
{
  if (const auto s = area(poly)) {
    return *s > 0;
  } else {
    return std::nullopt;
  }
}

std::optional<bool> touches(const alt::Point & point, const alt::Polygon & poly)
{
  if (poly.size() < 3) {
    return std::nullopt;
  }

  for (size_t i = 0; i < poly.size(); ++i) {
    // check if the point is on each edge of the polygon
    if (
      distance(point, poly.at(i), poly.at((i + 1) % poly.size())) <=
      std::numeric_limits<double>::epsilon()) {
      return true;
    }
  }

  return false;
}

std::optional<bool> within(const alt::Point & point, const alt::Polygon & poly)
{
  if (poly.size() < 3) {
    return std::nullopt;
  }

  long winding_number = 0;
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto & p1 = poly.at(i);
    const auto & p2 = poly.at((i + 1) % poly.size());

    // check if the point is to the left of the edge
    auto x_dist_to_edge = [&]() { return (p2 - p1).cross(point - p1).z() / (p2 - p1).y(); };

    if (p1.y() <= point.y() && p2.y() > point.y()) {  // upward edge
      if (x_dist_to_edge() >= 0) {
        winding_number++;
      }
    } else if (p1.y() > point.y() && p2.y() <= point.y()) {  // downward edge
      if (x_dist_to_edge() >= 0) {
        winding_number--;
      }
    }
  }

  return winding_number != 0;
}

std::optional<bool> within(
  const alt::Polygon & poly_contained, const alt::Polygon & poly_containing)
{
  if (equals(poly_contained, poly_containing).value_or(false)) {
    return true;
  }

  // check if all points of poly_contained are within poly_containing
  for (const auto & point : poly_contained) {
    const auto is_point_within = within(point, poly_containing);
    if (!is_point_within) {
      return std::nullopt;
    } else if (!*is_point_within) {
      return false;
    }
  }

  return true;
}

}  // namespace autoware::universe_utils
