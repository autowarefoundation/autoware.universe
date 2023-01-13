#pragma once

namespace pcdless::modularized_particle_filter
{
struct FastCosSin
{
  FastCosSin(int bin = 90)
  {
    for (int i = 0; i < bin + 1; ++i) {
      cos_.push_back(std::cos(i * M_PI / 180.f));
    }
  }
  float cos(float deg) const
  {
    while (deg < 0) {
      deg += 360;
    }
    while (deg > 360) {
      deg -= 360;
    }
    if (deg < 90) {
      return cos_.at(int(deg));
    } else if (deg < 180) {
      return -cos_.at(int(180 - deg));
    } else if (deg < 270) {
      return -cos_.at(int(deg - 180));
    } else {
      return cos_.at(int(360 - deg));
    }
  }

  float sin(float deg) const { return cos(deg - 90.f); }

private:
  std::vector<float> cos_;
};
}  // namespace pcdless::modularized_particle_filter