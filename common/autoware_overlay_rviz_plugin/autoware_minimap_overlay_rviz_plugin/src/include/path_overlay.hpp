#ifndef PATH_OVERLAY_HPP_
#define PATH_OVERLAY_HPP_

#include "GeographicLib/UTMUPS.hpp"

#include <QPainter>
#include <QRectF>

#include <tuple>
#include <vector>

struct PathPoint
{
  double x;
  double y;
};

class PathOverlay
{
public:
  PathOverlay();
  void setVehiclePosition(double lat, double lon);
  void setPathPoints(
    const std::vector<PathPoint> & path_points, double origin_lat, double origin_lon);
  void draw(QPainter & painter, const QRectF & backgroundRect, int zoom);

private:
  double vehicle_lat_;
  double vehicle_lon_;
  std::vector<std::pair<double, double>> path_points_geo_;

  std::pair<double, double> localToGeographic(
    double local_x, double local_y, double origin_lat, double origin_lon);
  std::pair<int, int> getTileOffsets(
    double lat, double lon, int zoom, const QRectF & backgroundRect);
};

#endif  // PATH_OVERLAY_HPP_
