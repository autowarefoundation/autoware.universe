#include "include/goal_pose.hpp"

#include <QDebug>

#include <qpoint.h>

GoalPose::GoalPose()
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_minimap_overlay_rviz_plugin");
  std::string goal_image_path = package_path + "/icons/goal.png";
  goal_image_ = QImage(goal_image_path.c_str());
  goal_image_ = goal_image_.scaled(30, 30, Qt::KeepAspectRatio);
}

void GoalPose::setGoalPosition(double local_x, double local_y, double origin_lat, double origin_lon)
{
  std::tie(goal_lat_, goal_lon_) = localToGeographic(local_x, local_y, origin_lat, origin_lon);
}

std::pair<double, double> GoalPose::localToGeographic(
  double local_x, double local_y, double origin_lat, double origin_lon)
{
  int zone;
  bool northp;
  double origin_x, origin_y, gamma, k;

  // Convert origin coordinates to UTM
  GeographicLib::UTMUPS::Forward(
    origin_lat, origin_lon, zone, northp, origin_x, origin_y, gamma, k);

  // Add local coordinates to origin UTM coordinates
  double goal_x = origin_x + local_x;
  double goal_y = origin_y + local_y;

  // Convert UTM coordinates back to geographic coordinates
  double goal_lat, goal_lon;
  GeographicLib::UTMUPS::Reverse(zone, northp, goal_x, goal_y, goal_lat, goal_lon);

  // Print the calculated goal latitude and longitude
  qDebug() << "Goal latitude: " << goal_lat;
  qDebug() << "Goal longitude: " << goal_lon;

  return {goal_lat, goal_lon};
}

std::pair<int, int> GoalPose::getTileOffsets(
  double lat, double lon, int zoom, const QRectF & backgroundRect)
{
  double x = (lon + 180.0) / 360.0 * std::pow(2.0, zoom);
  double y =
    (1.0 - std::log(std::tan(lat * M_PI / 180.0) + 1.0 / std::cos(lat * M_PI / 180.0)) / M_PI) /
    2.0 * std::pow(2.0, zoom);

  int x_tile = static_cast<int>(std::floor(x));
  int y_tile = static_cast<int>(std::floor(y));

  int x_offset = static_cast<int>((x - x_tile) * 256);
  int y_offset = static_cast<int>((y - y_tile) * 256);

  // Adjusting to align with the center of the background rectangle
  int centerX = static_cast<int>(backgroundRect.width() / 2);
  int centerY = static_cast<int>(backgroundRect.height() / 2);

  return {centerX + x_offset, centerY + y_offset - 100};
}

void GoalPose::draw(QPainter & painter, const QRectF & backgroundRect, int zoom)
{
  auto [goal_x_pixel, goal_y_pixel] = getTileOffsets(goal_lat_, goal_lon_, zoom, backgroundRect);

  QPointF goalPositionInOverlay(
    QPointF(goal_x_pixel, goal_y_pixel) -
    QPointF(goal_image_.width() / 2.0, goal_image_.height() / 2.0));

  painter.drawImage(goalPositionInOverlay, goal_image_);
}

// Getter methods for the goal latitude and longitude
double GoalPose::getGoalLatitude() const
{
  return goal_lat_;
}

double GoalPose::getGoalLongitude() const
{
  return goal_lon_;
}
