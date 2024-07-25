#include "include/tile_provider.hpp"

std::string TileProvider::getUrlTemplate(TileProvider::Provider provider)
{
  switch (provider) {
    case OpenStreetMap:
      return "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
    default:
      return "";
  }
}
