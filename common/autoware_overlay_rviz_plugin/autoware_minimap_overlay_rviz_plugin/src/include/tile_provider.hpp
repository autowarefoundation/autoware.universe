#ifndef TILE_PROVIDER_HPP_
#define TILE_PROVIDER_HPP_

#include <string>

class TileProvider
{
public:
  enum Provider { OpenStreetMap };

  static std::string getUrlTemplate(Provider provider);
};

#endif  // TILE_PROVIDER_HPP_
