#include "include/tile_field.hpp"

#include <QPainter>
#include <QUrl>

#include <cmath>

TileField::TileField(QObject * parent) : QObject(parent), center_x_tile_(0), center_y_tile_(0)
{
}

TileField::~TileField()
{
  std::lock_guard<std::mutex> lock(tile_mutex_);
  for (auto & tile : tiles_) {
    delete tile.second;
  }
}

void TileField::fetchTiles(int zoom, int center_x_tile, int center_y_tile)
{
  std::lock_guard<std::mutex> lock(tile_mutex_);
  zoom_ = zoom != 0 ? zoom : zoom_;
  center_x_tile_ = center_x_tile;
  center_y_tile_ = center_y_tile;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      int x_tile = center_x_tile_ + dx;
      int y_tile = center_y_tile_ + dy;
      std::string tile_key =
        std::to_string(zoom) + "/" + std::to_string(x_tile) + "/" + std::to_string(y_tile) + ".png";

      if (tiles_.find(tile_key) == tiles_.end()) {
        Tile * tile = new Tile(zoom, x_tile, y_tile);
        tiles_[tile_key] = tile;
        connect(tile, &Tile::tileFetched, this, &TileField::onTileFetched);
        tile->fetch();
      }
    }
  }
}

void TileField::initializeTiles(int center_x_tile, int center_y_tile)
{
  std::lock_guard<std::mutex> lock(tile_mutex_);
  center_x_tile_ = center_x_tile;
  center_y_tile_ = center_y_tile;
  updateTiles(center_x_tile_, center_y_tile_);
}

void TileField::updateTiles(int new_center_x_tile, int new_center_y_tile)
{
  std::lock_guard<std::mutex> lock(tile_mutex_);
  if (new_center_x_tile == center_x_tile_ && new_center_y_tile == center_y_tile_) {
    return;
  }

  center_x_tile_ = new_center_x_tile;
  center_y_tile_ = new_center_y_tile;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      int x_tile = center_x_tile_ + dx;
      int y_tile = center_y_tile_ + dy;

      auto tile_key = QString("%1/%2/%3.png").arg(zoom_).arg(x_tile).arg(y_tile).toStdString();

      if (tiles_.find(tile_key) == tiles_.end()) {
        Tile * tile = new Tile(zoom_, x_tile, y_tile);
        tiles_[tile_key] = tile;
        connect(tile, &Tile::tileFetched, this, &TileField::onTileFetched);
        tile->fetch();
      }
    }
  }
}

QImage TileField::getTileFieldImage()
{
  std::lock_guard<std::mutex> lock(tile_mutex_);
  if (tile_field_image_.isNull()) {
    int tile_size = 256;  // Assuming tile size is 256x256 pixels
    tile_field_image_ = QImage(tile_size * 3, tile_size * 3, QImage::Format_ARGB32);
  }

  QPainter painter(&tile_field_image_);
  painter.fillRect(tile_field_image_.rect(), Qt::transparent);

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      int x_tile = center_x_tile_ + dx;
      int y_tile = center_y_tile_ + dy;

      auto tile_key = QString("%1/%2/%3.png").arg(zoom_).arg(x_tile).arg(y_tile).toStdString();

      auto tile_it = tiles_.find(tile_key);
      if (tile_it != tiles_.end() && !tile_it->second->getImage().isNull()) {
        QRectF target((dx + 1) * 256, (dy + 1) * 256, 256, 256);
        QRectF source(0, 0, 256, 256);
        painter.drawImage(target, tile_it->second->getImage(), source);
        // Draw the tile key as text for debugging
        painter.drawText(target, Qt::AlignCenter, QString::fromStdString(tile_key));
      }
    }
  }

  return tile_field_image_;
}

void TileField::onTileFetched()
{
  Q_EMIT tilesUpdated();
}

int TileField::long2tilex(double lon, int z)
{
  return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
}

int TileField::lat2tiley(double lat, int z)
{
  double latrad = lat * M_PI / 180.0;
  return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z)));
}

double TileField::tilex2long(int x, int z)
{
  return x / (double)(1 << z) * 360.0 - 180;
}

double TileField::tiley2lat(int y, int z)
{
  double n = M_PI - 2.0 * M_PI * y / (double)(1 << z);
  return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

std::pair<int, int> TileField::getTileOffsets(double lat, double lon)
{
  double x = (lon + 180.0) / 360.0 * std::pow(2.0, zoom_);
  double y =
    (1.0 - std::log(std::tan(lat * M_PI / 180.0) + 1.0 / std::cos(lat * M_PI / 180.0)) / M_PI) /
    2.0 * std::pow(2.0, zoom_);

  int x_tile = static_cast<int>(std::floor(x));
  int y_tile = static_cast<int>(std::floor(y));

  int x_offset = static_cast<int>((x - x_tile) * 256);
  int y_offset = static_cast<int>((y - y_tile) * 256);

  return {x_offset, y_offset};
}

std::pair<double, double> TileField::latLonToTile(double lat, double lon, int zoom)
{
  double x = (lon + 180.0) / 360.0 * std::pow(2.0, zoom);
  double y =
    (1.0 - std::log(std::tan(lat * M_PI / 180.0) + 1.0 / std::cos(lat * M_PI / 180.0)) / M_PI) /
    2.0 * std::pow(2.0, zoom);

  return {x, y};
}
