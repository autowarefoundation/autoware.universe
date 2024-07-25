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

// NOTE: much faster than boost::geometry::intersects()
std::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  // calculate intersection point
  const double det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  if (det == 0.0) {
    return std::nullopt;
  }

  const double t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  const double s = ((p2.y - p1.y) * (p4.x - p2.x) + (p1.x - p2.x) * (p4.y - p2.y)) / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point intersect_point;
  intersect_point.x = t * p1.x + (1.0 - t) * p2.x;
  intersect_point.y = t * p1.y + (1.0 - t) * p2.y;
  intersect_point.z = t * p1.z + (1.0 - t) * p2.z;
  return intersect_point;
}

bool intersects_convex(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  return gjk::intersects(convex_polygon1, convex_polygon2);
}

// Alternatives for Boost.Geometry ----------------------------------------------------------------

namespace alt
{
Point2d from_geom(const geometry_msgs::msg::Point & point)
{
  return {point.x, point.y};
}

Point2d from_boost(const autoware::universe_utils::Point2d & point)
{
  return {point.x(), point.y()};
}

ConvexPolygon2d from_boost(const autoware::universe_utils::Polygon2d & polygon)
{
  PointList points;
  for (const auto & point : polygon.outer()) {
    points.push_back(from_boost(point));
  }

  ConvexPolygon2d _polygon(points);
  correct(_polygon);
  return _polygon;
}

autoware::universe_utils::Point2d to_boost(const Point2d & point)
{
  return {point.x(), point.y()};
}

autoware::universe_utils::Polygon2d to_boost(const ConvexPolygon2d & polygon)
{
  autoware::universe_utils::Polygon2d _polygon;
  for (const auto & vertex : polygon.vertices()) {
    _polygon.outer().push_back(to_boost(vertex));
  }
  return _polygon;
}
}  // namespace alt

double area(const alt::ConvexPolygon2d & poly)
{
  const auto & vertices = poly.vertices();

  double area = 0.;
  for (size_t i = 1; i < vertices.size() - 1; ++i) {
    area += (vertices.at(i + 1) - vertices.front()).cross(vertices.at(i) - vertices.front()) / 2;
  }

  return area;
}

void correct(alt::ConvexPolygon2d & poly)
{
  auto & vertices = poly.vertices();

  // sort points in clockwise order with respect to the first point
  std::sort(vertices.begin() + 1, vertices.end(), [&](const auto & a, const auto & b) {
    return (a - vertices.front()).cross(b - vertices.front()) < 0;
  });

  if (equals(vertices.front(), vertices.back())) {
    vertices.pop_back();
  }
}

bool equals(const alt::Point2d & point1, const alt::Point2d & point2)
{
  return std::abs(point1.x() - point2.x()) <= std::numeric_limits<double>::epsilon() &&
         std::abs(point1.y() - point2.y()) <= std::numeric_limits<double>::epsilon();
}

bool equals(const alt::ConvexPolygon2d & poly1, const alt::ConvexPolygon2d & poly2)
{
  return std::all_of(poly1.vertices().begin(), poly1.vertices().end(), [&](const auto & a) {
    return std::any_of(poly2.vertices().begin(), poly2.vertices().end(), [&](const auto & b) {
      return equals(a, b);
    });
  });
}

bool intersects(
  const alt::Point2d & seg1_start, const alt::Point2d & seg1_end, const alt::Point2d & seg2_start,
  const alt::Point2d & seg2_end)
{
  const auto v1 = seg1_end - seg1_start;
  const auto v2 = seg2_end - seg2_start;

  const auto det = v1.cross(v2);
  if (std::abs(det) <= std::numeric_limits<double>::epsilon()) {
    return false;
  }

  const auto v12 = seg2_end - seg1_end;
  const double t = v2.cross(v12) / det;
  const double s = v1.cross(v12) / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return false;
  }

  return true;
}

bool intersects(const alt::ConvexPolygon2d & poly1, const alt::ConvexPolygon2d & poly2)
{
  if (equals(poly1, poly2)) {
    return true;
  }

  // GJK algorithm

  auto find_support_vector = [](
                               const alt::ConvexPolygon2d & poly1,
                               const alt::ConvexPolygon2d & poly2,
                               const alt::Vector2d & direction) {
    auto find_farthest_vertex =
      [](const alt::ConvexPolygon2d & poly, const alt::Vector2d & direction) {
        return std::max_element(
          poly.vertices().begin(), poly.vertices().end(),
          [&](const auto & a, const auto & b) { return direction.dot(a) <= direction.dot(b); });
      };
    return *find_farthest_vertex(poly1, direction) - *find_farthest_vertex(poly2, -direction);
  };

  alt::Vector2d direction = {1.0, 0.0};
  auto a = find_support_vector(poly1, poly2, direction);
  direction = -a;
  auto b = find_support_vector(poly1, poly2, direction);
  if (b.dot(direction) <= 0.0) {
    return false;
  }

  direction = (b - a).vector_triple(-a, b - a);
  while (true) {
    auto c = find_support_vector(poly1, poly2, direction);
    if (c.dot(direction) <= 0.0) {
      return false;
    }

    auto n_ca = (b - c).vector_triple(a - c, a - c);
    if (n_ca.dot(-c) > 0.0) {
      b = c;
      direction = n_ca;
    } else {
      auto n_cb = (a - c).vector_triple(b - c, b - c);
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

bool within(const alt::Point2d & point, const alt::ConvexPolygon2d & poly)
{
  const auto & vertices = poly.vertices();
  int64_t winding_number = 0;
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto & p1 = vertices.at(i);
    const auto & p2 = vertices.at((i + 1) % vertices.size());

    double cross = (p2 - p1).cross(point - p1);
    if (std::abs(cross) <= std::numeric_limits<double>::epsilon()) {
      return false;
    }

    if (p1.y() <= point.y() && p2.y() > point.y() && cross > 0) {  // the point is to the left of
                                                                   // upward edge
      winding_number++;
    } else if (p1.y() > point.y() && p2.y() <= point.y() && cross < 0) {  // the point is to the
                                                                          // left of downward edge
      winding_number--;
    }
  }

  return winding_number != 0;
}

bool within(
  const alt::ConvexPolygon2d & poly_contained, const alt::ConvexPolygon2d & poly_containing)
{
  if (equals(poly_contained, poly_containing)) {
    return true;
  }

  // check if all points of poly_contained are within poly_containing
  for (const auto & vertex : poly_contained.vertices()) {
    if (!within(vertex, poly_containing)) {
      return false;
    }
  }

  return true;
}

}  // namespace autoware::universe_utils
