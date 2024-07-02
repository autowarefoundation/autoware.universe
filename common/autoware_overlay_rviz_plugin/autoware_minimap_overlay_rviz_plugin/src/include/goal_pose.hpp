#ifndef GOAL_POSE_HPP_
#define GOAL_POSE_HPP_

#include "GeographicLib/UTMUPS.hpp"

#include <QImage>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tuple>

class GoalPose
{
public:
  GoalPose();
  void setGoalPosition(double local_x, double local_y, double origin_lat, double origin_lon);
  void draw(QPainter & painter, const QRectF & backgroundRect, int zoom);
  // Getter methods for the goal latitude and longitude
  double getGoalLatitude() const;
  double getGoalLongitude() const;

private:
  double goal_lat_;
  double goal_lon_;
  QImage goal_image_;

  std::pair<double, double> localToGeographic(
    double local_x, double local_y, double origin_lat, double origin_lon);
  std::pair<int, int> getTileOffsets(
    double lat, double lon, int zoom, const QRectF & backgroundRect);
};

#endif  // GOAL_POSE_HPP_
