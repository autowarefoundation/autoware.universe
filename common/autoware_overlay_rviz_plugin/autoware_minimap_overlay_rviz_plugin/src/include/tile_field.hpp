#ifndef TILE_FIELD_HPP_
#define TILE_FIELD_HPP_

#include "tile.hpp"

#include <QImage>
#include <QObject>

#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>

class TileField : public QObject
{
  Q_OBJECT

public:
  TileField(QObject * parent = nullptr);
  ~TileField();

  void initializeTiles(int center_x_tile, int center_y_tile);
  void fetchTiles(int zoom, int center_x_tile, int center_y_tile);
  void updateTiles(int center_x_tile, int center_y_tile);
  QImage getTileFieldImage();
  std::pair<int, int> getTileOffsets(double lat, double lon);
  std::pair<double, double> latLonToTile(double lat, double lon, int zoom);

  int long2tilex(double lon, int z);
  int lat2tiley(double lat, int z);
  double tilex2long(int x, int z);
  double tiley2lat(int y, int z);

Q_SIGNALS:
  void tilesUpdated();

private Q_SLOTS:
  void onTileFetched();

private:
  int zoom_ = 15;
  int center_x_tile_;
  int center_y_tile_;
  std::map<std::string, Tile *> tiles_;
  QImage tile_field_image_;
  std::mutex tile_mutex_;
};

#endif  // TILE_FIELD_HPP_
