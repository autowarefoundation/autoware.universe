#include <iostream>
#include <iomanip>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/Gnomonic.hpp>
#include <GeographicLib/Geodesic.hpp>

class vector3 {
public:
  GeographicLib::Math::real _x, _y, _z;
  vector3(GeographicLib::Math::real x,
          GeographicLib::Math::real y,
          GeographicLib::Math::real z = 1) throw()
    : _x(x)
    , _y(y)
    , _z(z) {}
  vector3 cross(const vector3& b) const throw() {
    return vector3(_y * b._z - _z * b._y,
                   _z * b._x - _x * b._z,
                   _x * b._y - _y * b._x);
  }
  void norm() throw() {
    _x /= _z;
    _y /= _z;
    _z = 1;
  }
};

int main() {
  GeographicLib::Utility::set_digits();
  GeographicLib::Math::real lata1, lona1, lata2, lona2;
  GeographicLib::Math::real latb1, lonb1, latb2, lonb2;
  std::cin >> lata1 >> lona1 >> lata2 >> lona2
           >> latb1 >> lonb1 >> latb2 >> lonb2;
  const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
  const GeographicLib::Gnomonic gn(geod);
  GeographicLib::Math::real
    lat0 = (lata1 + lata2 + latb1 + latb2)/4,
    // Possibly need to deal with longitudes wrapping around
    lon0 = (lona1 + lona2 + lonb1 + lonb2)/4;
  std::cout << std::setprecision(16);
  std::cout << "Initial guess " << lat0 << " " << lon0 << "\n";
  GeographicLib::Math::real s120 = 1, s121;
  for (int i = 0; i < 20; ++i) {
    GeographicLib::Math::real xa1, ya1, xa2, ya2;
    GeographicLib::Math::real xb1, yb1, xb2, yb2;
    gn.Forward(lat0, lon0, lata1, lona1, xa1, ya1);
    gn.Forward(lat0, lon0, lata2, lona2, xa2, ya2);
    gn.Forward(lat0, lon0, latb1, lonb1, xb1, yb1);
    gn.Forward(lat0, lon0, latb2, lonb2, xb2, yb2);
    // See Hartley and Zisserman, Multiple View Geometry, Sec. 2.2.1
    vector3 va1(xa1, ya1); vector3 va2(xa2, ya2);
    vector3 vb1(xb1, yb1); vector3 vb2(xb2, yb2);
    // la is homogeneous representation of line A1,A2
    // lb is homogeneous representation of line B1,B2
    vector3 la = va1.cross(va2);
    vector3 lb = vb1.cross(vb2);
    // p0 is homogeneous representation of intersection of la and lb
    vector3 p0 = la.cross(lb);
    p0.norm();
    GeographicLib::Math::real lat1, lon1;
    gn.Reverse(lat0, lon0, p0._x, p0._y, lat1, lon1);
    geod.Inverse(lat1, lon1, lat0, lon1, s121);
    std::cout << "Increment " << s121 << " ratio " << s121/s120 << "\n";
    lat0 = lat1;
    lon0 = lon1;
    s120 = s121;
  }
  std::cout << "Final result " << lat0 << " " << lon0 << "\n";
  GeographicLib::Math::real azi1, azi2;
  geod.Inverse(lata1, lona1, lat0, lon0, azi1, azi2);
  std::cout << "Azimuths on line A " << azi2 << " ";
  geod.Inverse(lat0, lon0, lata2, lona2, azi1, azi2);
  std::cout << azi1 << "\n";
  geod.Inverse(latb1, lonb1, lat0, lon0, azi1, azi2);
  std::cout << "Azimuths on line B " << azi2 << " ";
  geod.Inverse(lat0, lon0, latb2, lonb2, azi1, azi2);
  std::cout << azi1 << "\n";
}
