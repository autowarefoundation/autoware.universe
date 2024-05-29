#include <iostream>
#include "Geodesic30.hpp"

int main() {
  {
    GeographicLib::Geodesic30<double> g(6.4e6, 0.01);
    {
      double lat2, lon2, azi2;
      g.Direct(10, 30, 20, 1e6, lat2, lon2, azi2);
      std::cout << lat2 << " " << lon2 << " " << azi2 << "\n";
    }
    {
      double s12, azi1, azi2;
      g.Inverse(-20, 0, 10, 70, s12, azi1, azi2);
      std::cout << s12 << " " << azi1 << " " << azi2 << "\n";
    }
  }
  /*
  {
    GeographicLib::Geodesic30<long double> g(6.4e6L, 0.01L);
    {
      long double lat2, lon2, azi2;
      g.Direct(10, 30, 20, 1e6L, lat2, lon2, azi2);
      std::cout << lat2 << " " << lon2 << " " << azi2 << "\n";
    }
    {
      long double s12, azi1, azi2;
      g.Inverse(-20, 0, 10, 70, s12, azi1, azi2);
      std::cout << s12 << " " << azi1 << " " << azi2 << "\n";
    }
  }
  */
  return 0;
}
