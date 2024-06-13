// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "lib_mission_planner/helper_functions.hpp"

#include "mission_planner_messages/msg/road_segments.hpp"
#include "rclcpp/rclcpp.hpp"

#include "db_msgs/msg/lanelets_stamped.hpp"

namespace lib_mission_planner
{

double NormalizePsi(const double psi)
{
  // remove multiples of 2 PI by using modulo on absolute value and apply sign
  double psi_out = copysign(fmod(fabs(psi), M_PI * 2.0), psi);

  // restrict psi to [-pi, pi[
  if (psi_out >= M_PI)
    psi_out -= 2.0 * M_PI;
  else if (psi_out < -M_PI)
    psi_out += 2.0 * M_PI;

  return psi_out;
}

std::vector<double> GetPsiForPoints(const std::vector<geometry_msgs::msg::Point> & points)
{
  int num_points = points.size();
  std::vector<double> tang_vecs(num_points * 2);

  // get heading vector for first point
  tang_vecs[0] = points[1].x - points[0].x;
  tang_vecs[1] = points[1].y - points[0].y;

  // use one point before and after the targeted one for heading vector (more
  // stable/robust than relying on a single pair of points)
  for (int i = 1; i < num_points - 1; i++) {
    tang_vecs[2 * i] = points[i + 1].x - points[i - 1].x;
    tang_vecs[2 * i + 1] = points[i + 1].y - points[i - 1].y;
  }

  // get heading vector for last point
  tang_vecs[2 * (num_points - 1)] = points[num_points - 1].x - points[num_points - 2].x;
  tang_vecs[2 * (num_points - 1) + 1] = points[num_points - 1].y - points[num_points - 2].y;

  // calculate heading angles with atan2
  std::vector<double> psi_points(num_points);
  for (int i = 0; i < num_points; i++) {
    psi_points[i] = NormalizePsi(std::atan2(tang_vecs[2 * i + 1], tang_vecs[2 * i]));
  }

  return psi_points;
}

Pose2D::Pose2D()
{
  this->set_xy(0.0, 0.0);
  this->set_psi(0.0);
}
Pose2D::Pose2D(const double x, const double y, const double psi /* = 0.0 */)
{
  this->set_xy(x, y);
  this->set_psi(psi);
}
double Pose2D::get_x() const
{
  return this->xy_(0);
}
double Pose2D::get_y() const
{
  return this->xy_(1);
}
Eigen::Vector2d Pose2D::get_xy() const
{
  return this->xy_;
}
double Pose2D::get_psi() const
{
  return this->psi_;
}
geometry_msgs::msg::Point Pose2D::get_point() const
{
  geometry_msgs::msg::Point tmp_point;
  tmp_point.x = this->xy_(0);
  tmp_point.y = this->xy_(1);
  return tmp_point;
}
void Pose2D::set_x(const double x)
{
  this->xy_(0) = x;
}
void Pose2D::set_y(const double y)
{
  this->xy_(1) = y;
}
void Pose2D::set_xy(const double x, const double y)
{
  this->xy_ = {x, y};
}
void Pose2D::set_xy(const Eigen::Vector2d xy)
{
  this->xy_ = xy;
}
void Pose2D::set_psi(const double psi)
{
  this->psi_ = psi;
}

Pose2D TransformToNewCosy2D(const Pose2D cosy_rel, const Pose2D pose_prev)
{
  Pose2D pose_out;

  pose_out.set_xy(
    Eigen::Rotation2D<double>(-cosy_rel.get_psi()) * Eigen::Translation2d(-cosy_rel.get_xy()) *
    pose_prev.get_xy());
  pose_out.set_psi(NormalizePsi(pose_prev.get_psi() - cosy_rel.get_psi()));

  return pose_out;
}

double GetYawFromQuaternion(const double x, const double y, const double z, const double w)
{
  tf2::Quaternion tmp_quat(x, y, z, w);
  tf2::Matrix3x3 m(tmp_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

// Declare a static logger
static rclcpp::Logger static_logger = rclcpp::get_logger("static_logger");

mission_planner_messages::msg::RoadSegments ConvertLaneletsStamped2RoadSegments(
  const db_msgs::msg::LaneletsStamped & msg)
{
  // Initialize road segments message
  mission_planner_messages::msg::RoadSegments road_segments;

  // Fill message header and pose
  road_segments.header = msg.header;
  road_segments.pose = msg.pose;

  // Convert lanelets to segments if lanelets are not empty
  if (!msg.lanelets.empty()) {
    for (const db_msgs::msg::Lanelet & lanelet : msg.lanelets) {
      // Initialize a segment
      mission_planner_messages::msg::Segment segment;

      // Fill the segment with basic information
      segment.id = lanelet.id;
      segment.successor_lanelet_id = lanelet.successor_lanelet_id;
      segment.neighboring_lanelet_id = lanelet.neighboring_lanelet_id;

      // Copy linestrings data
      for (int i = 0; i < 2; ++i) {
        // Copy points from the original linestring to the new one if points are not empty
        if (!lanelet.linestrings[i].points.empty()) {
          segment.linestrings[i].poses.reserve(lanelet.linestrings[i].points.size());

          for (const db_msgs::msg::DBPoint & point : lanelet.linestrings[i].points) {
            segment.linestrings[i].poses.push_back(point.pose);
          }
        } else {
          RCLCPP_WARN(
            static_logger,
            "Linestring does not contain points (ConvertLaneletsStamped2RoadSegments)!");
        }
      }

      // Add the filled segment to the road_segments message
      road_segments.segments.push_back(segment);
    }
  }

  return road_segments;
}

}  // namespace lib_mission_planner
