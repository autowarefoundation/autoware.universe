// Copyright 2023 TIER IV, Inc.
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

void correct(Polygon & poly)
{
  if (poly.size() < 3) {
    return;
  }

  // sort points in clockwise order with respect to the first point

  const auto min_y_point = *std::min_element(
    poly.begin(), poly.end(), [](const auto & a, const auto & b) { return a.y() < b.y(); });

  auto arg = [min_y_point](const auto & p) {
    const auto p_vec = p - min_y_point;
    if (p_vec.length() < std::numeric_limits<double>::epsilon()) {
      return 0.0;
    }
    return std::atan2(p_vec.y(), p_vec.x());
  };
  const auto ref_arg =
    std::atan2(poly.at(0).y() - min_y_point.y(), poly.at(0).x() - min_y_point.x());

  std::sort(poly.begin() + 1, poly.end(), [ref_arg, arg](const auto & a, const auto & b) {
    const auto dt_a = ref_arg - arg(a);
    const auto dt_b = ref_arg - arg(b);
    return (dt_a > 0 ? dt_a : dt_a + 2 * M_PI) < (dt_b > 0 ? dt_b : dt_b + 2 * M_PI);
  });
}

// NOTE: much faster than boost::geometry::intersects()
std::optional<Point> intersect(
  const Point & seg1_start, const Point & seg1_end, const Point & seg2_start,
  const Point & seg2_end)
{
  const auto v1 = seg1_end - seg1_start;
  const auto v2 = seg2_end - seg2_start;

  // calculate intersection point
  const auto det = v1.cross(v2).z();
  if (std::abs(det) < std::numeric_limits<double>::epsilon()) {
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

std::optional<PointList> intersect(const Polygon & poly1, const Polygon & poly2)
{
  if (poly1.size() < 3 || poly2.size() < 3) {
    return std::nullopt;
  }

  // TODO(mitukou1109): Use plane sweep method to improve performance
  // check if all edges of poly1 intersect with those of poly2
  PointList intersect_points;
  for (size_t i = 0; i < poly1.size(); ++i) {
    for (size_t j = 0; j < poly2.size(); ++j) {
      const auto intersect_point = intersect(
        poly1.at(i), poly1.at((i + 1) % poly1.size()), poly2.at(j),
        poly2.at((j + 1) % poly2.size()));
      if (intersect_point) {
        intersect_points.push_back(*intersect_point);
      }
    }
  }

  if (intersect_points.empty()) {
    return std::nullopt;
  }

  const auto unique_points_itr = std::unique(
    intersect_points.begin(), intersect_points.end(), [](const auto & a, const auto & b) {
      return (a - b).length() < std::numeric_limits<double>::epsilon();
    });
  intersect_points.erase(unique_points_itr, intersect_points.end());

  return intersect_points;
}

std::optional<bool> within(const Point & point, const Polygon & poly)
{
  // check if the polygon is valid
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

std::optional<bool> within(const Polygon & poly_contained, const Polygon & poly_containing)
{
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

std::optional<bool> disjoint(const Polygon & poly1, const Polygon & poly2)
{
  if (poly1.size() < 3 || poly2.size() < 3) {
    return std::nullopt;
  }

  return !intersect(poly1, poly2).has_value();
}

double distance(const Point & point, const Point & seg_start, const Point & seg_end)
{
  const auto seg_vec = seg_end - seg_start;
  const auto point_vec = point - seg_start;

  const double seg_vec_norm = seg_vec.length();
  const double seg_point_dot = seg_vec.dot(point_vec);

  if (seg_vec_norm < std::numeric_limits<double>::epsilon() || seg_point_dot < 0) {
    return point_vec.length();
  } else if (seg_point_dot > std::pow(seg_vec_norm, 2)) {
    return (point - seg_end).length();
  } else {
    return std::abs(seg_vec.cross(point_vec).z()) / seg_vec_norm;
  }
}

double distance(const Point & point, const Polygon & poly)
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

std::optional<bool> coveredBy(const Point & point, const Polygon & poly)
{
  const auto is_point_within = within(point, poly);
  if (!is_point_within) {
    return std::nullopt;
  } else if (*is_point_within) {
    return true;
  }

  for (size_t i = 0; i < poly.size(); ++i) {
    if (
      distance(point, poly.at(i), poly.at((i + 1) % poly.size())) <
      std::numeric_limits<double>::epsilon()) {
      return true;
    }
  }

  return false;
}

bool isAbove(const Point & point, const Point & seg_start, const Point & seg_end)
{
  return (seg_end - seg_start).cross(point - seg_start).z() > 0;
}

std::array<PointList, 2> divideBySegment(
  const PointList & points, const Point & seg_start, const Point & seg_end)
{
  PointList above_points, below_points;

  for (const auto & point : points) {
    if (isAbove(point, seg_start, seg_end)) {
      above_points.push_back(point);
    } else {
      below_points.push_back(point);
    }
  }

  return {above_points, below_points};
}

std::optional<Polygon> convexHull(const PointList & points)
{
  if (points.size() < 3) {
    return std::nullopt;
  }

  // quick hull algorithm

  const auto p_minmax_itr =
    std::minmax_element(points.begin(), points.end(), [](const auto & a, const auto & b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
  const auto & p_min = *p_minmax_itr.first;
  const auto & p_max = *p_minmax_itr.second;

  Polygon hull;

  auto make_hull = [&hull](
                     auto self, const Point & p1, const Point & p2, const PointList & points) {
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

  return hull;
}

std::optional<double> area(const Polygon & poly)
{
  if (poly.size() < 3) {
    return std::nullopt;
  }

  double area = 0.;
  for (size_t i = 0; i < poly.size(); ++i) {
    area += poly.at(i).cross(poly.at((i + 1) % poly.size())).z() / 2;
  }

  return std::abs(area);
}

}  // namespace autoware::universe_utils
