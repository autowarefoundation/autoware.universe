/**
 * \file ConicTest.cpp
 * \brief Command line utility for testing transverse Mercator projections
 *
 * Copyright (c) Charles Karney (2008-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include "GeographicLib/LambertConformalConic.hpp"
#include "GeographicLib/AlbersEqualArea.hpp"
#include "GeographicLib/Constants.hpp"
#include "GeographicLib/Geodesic.hpp"
#include "GeographicLib/DMS.hpp"
#include <string>
#include <limits>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (disable: 4127)
#endif

GeographicLib::Math::real
dist(GeographicLib::Math::real a, GeographicLib::Math::real f,
     GeographicLib::Math::real lat0, GeographicLib::Math::real lon0,
     GeographicLib::Math::real lat1, GeographicLib::Math::real lon1) {
  using namespace GeographicLib;
  typedef Math::real real;
  using std::cos; using std::sin; using std::sqrt; using std::hypot;
  real
    phi = lat0 * Math::degree(),
    e2 = f * (2 - f),
    sinphi = sin(phi),
    n = 1/sqrt(1 - e2 * sinphi * sinphi),
      // See Wikipedia article on latitude
    hlon = cos(phi) * n,
    hlat = (1 - e2) * n * n * n,
    dlon = lon1 - lon0;
  if (dlon >= 180) dlon -= 360;
  else if (dlon < -180) dlon += 360;
  return a * Math::degree() *
    hypot((lat1 - lat0) * hlat, dlon * hlon);
}

class TestData {
  // Read test data with one line of buffering
public:
  typedef GeographicLib::Math::real real;
private:
  std::istream& _is;
  bool _usesaved;               // Should GetNext use saved values?
  bool _valid;                  // Are there saved values?
  real _lat0, _lat, _lon, _x, _y, _k;
  TestData& operator=(const TestData&);
public:
  TestData(std::istream& is) : _is(is), _usesaved(false), _valid(false) {}
  bool GetNext(real& lat0, real& lat, real& lon,
               real& x, real& y, real& k) {
    if (_usesaved)
      _usesaved = false;
    else {
      // Avoid a warning about void* changed to bool
      _valid = (_is >> _lat0 >> _lat >> _lon >> _x >> _y >> _k) ? true : false;
      if (!_valid)
        return false;
    }
    lat0 = _lat0; lat = _lat; lon = _lon; x = _x; y = _y; k = _k;
    return true;
  }
  bool BackUp() {
    if (!_valid || _usesaved)
      return false;             // Can't backup up again
    else {
      _usesaved = true;         // Set flag for GetNext
      return true;
    }
  }
};

int usage(int retval) {
  ( retval ? std::cerr : std::cout ) <<
"ConicTest -l -s\n\
\n\
Checks conic projections\n";
  return retval;
}

int main(int argc, char* argv[]) {
  using namespace GeographicLib;
  using namespace std;
  typedef Math::real real;
  bool lambert = true;
  bool albers = false;
  bool checkstdlats = false;
  real a = Constants::WGS84_a(), f = Constants::WGS84_f();
  for (int m = 1; m < argc; ++m) {
    string arg(argv[m]);
    if (arg == "-l") {
      lambert = true;
      albers = false;
    } else if (arg == "-a") {
      lambert = false;
      albers = true;
    } else if (arg == "-s") {
      checkstdlats = true;
    } else if (arg == "-e") {
      if (m + 2 >= argc) return usage(1);
      try {
        a = Utility::val<real>(string(argv[m + 1]));
        f = Utility::fract<real>(string(argv[m + 2]));
      }
      catch (const exception& e) {
        cerr << "Error decoding arguments of -e: " << e.what() << "\n";
        return 1;
      }
      m += 2;
      if (f > 1) f = 1/f;
    } else
      return usage(arg != "-h");
  }

  try {
    if (checkstdlats) {         // stdin contains lat1 lat2 lat0 k0
      cout << setprecision(17);
      real quant = real(1e12);
      while (true) {
        real lat1, lat2, lat0, k0;
        if (!(cin >> lat1 >> lat2 >> lat0 >> k0))
          break;
        int
          sign1 = lat1 < 0 ? -1 : 1,
          sign2 = lat2 < 0 ? -1 : 1;
        lat1 = real(floor(sign1 * lat1 * quant + 0.5L));
        lat2 = real(floor(sign2 * lat2 * quant + 0.5L));
        real
          colat1 = (90 * quant - lat1) / quant,
          colat2 = (90 * quant - lat2) / quant;
        lat1 /= quant;
        lat2 /= quant;
        real
          sinlat1 = sign1 * (lat1 < 45 ? sin(lat1 * Math::degree())
                             : cos(colat1 * Math::degree())),
          sinlat2 = sign2 * (lat2 < 45 ? sin(lat2 * Math::degree())
                             : cos(colat2 * Math::degree())),
          coslat1 = (lat1 < 45 ? cos(lat1 * Math::degree())
                     : sin(colat1 * Math::degree())),
          coslat2 = (lat2 < 45 ? cos(lat2 * Math::degree())
                     : sin(colat2 * Math::degree()));
        lat1 *= sign1;
        lat2 *= sign2;
        const LambertConformalConic lam(a, f, /* real(lat1), real(lat2), */
                                        real(sinlat1), real(coslat1),
                                        real(sinlat2), real(coslat2),
                                        real(1));
        const AlbersEqualArea alb(a, f, /* real(lat1), real(lat2), */
                                        real(sinlat1), real(coslat1),
                                        real(sinlat2), real(coslat2),
                                        real(1));
        real
          lat0a = albers ? alb.OriginLatitude() : lam.OriginLatitude();
          //k0a = albers ? alb.CentralScale() : lam.CentralScale();
        if (!(abs(lat0a-lat0) <= 4.5e-14))
          cout << lat1 << " " << lat2 << " " << lat0
               << " " << lat0a << " " << lat0a - lat0 << "\n";
      }
    } else { // Check projection
      // stdin contains lat0 lat lon x y k
      TestData testset(cin);
      cout << setprecision(8);
      while (true) {
        real lat0, lat, lon, x, y, k;
        if (!testset.GetNext(lat0, lat, lon, x, y, k))
          break;
        if (!testset.BackUp())
          break;
        int
          sign0 = lat0 < 0 ? -1 : 1;
        real quant = real(1e12);
        real
          lat00 = real(floor(sign0 * lat0 * quant + 0.5L)),
          colat00 = (90 * quant - lat00) / quant;
        lat00 /= quant;
        real
          sinlat0 = real(sign0 * (lat00 < 45 ?
                                  sin(lat00 * Math::degree()) :
                                  cos(colat00 * Math::degree()))),
          coslat0 = real(lat00 < 45 ? cos(lat00 * Math::degree())
                         : sin(colat00 * Math::degree()));
        const LambertConformalConic lcc(a, f,
                                        sinlat0, coslat0, sinlat0, coslat0,
                                        real(1));
        const AlbersEqualArea alb(a, f,
                                  sinlat0, coslat0, sinlat0, coslat0,
                                  real(1));
        real maxerrf = 0, maxerrr = 0, maxerrkf = 0, maxerrkr = 0;
        real latf = 0, lonf = 0, latr = 0, lonr = 0,
          latkf = 0, lonkf = 0, latkr = 0, lonkr = 0;
        // cout << "New lat0: " << lat0 << "\n";
        while (true) {
          real lat0x;
          if (!testset.GetNext(lat0x, lat, lon, x, y, k))
            break;
          if (lat0 != lat0x) {
            testset.BackUp();
            break;
          }
          real latb, lonb, xa, ya, gammaa, gammab, ka, kb;
          if (albers)
            alb.Forward(real(0), real(lat), real(lon), xa, ya, gammaa, ka);
          else
            lcc.Forward(real(0), real(lat), real(lon), xa, ya, gammaa, ka);
          real errf = real(hypot(real(xa) - x, real(ya) - y));
          if (lambert)
            errf /= real(k);
          real errkf = real(abs(real(ka) - k)/k);
          if (albers)
            alb.Reverse(real(0), real(x), real(y), latb, lonb, gammab, kb);
          else
            lcc.Reverse(real(0), real(x), real(y), latb, lonb, gammab, kb);
          real errr = real(dist(real(a), real(f),
                                lat, lon, real(latb), real(lonb)));
          /*
          cout << latb << " " << lonb << " " << xa << " " << ya << " "
                    << ka << " " << kb << " "
                    << gammaa << " " << gammab << "\n";
          */
          real errkr = real(abs(real(kb) - k)/k);
          if (!(errf <= maxerrf)) {
            maxerrf = errf;
            latf = real(lat);
            lonf = real(lon);
          }
          if (!(errr <= maxerrr)) {
            maxerrr = errr;
            latr = real(lat);
            lonr = real(lon);
          }
          if (!(errkf <= maxerrkf || abs(lat) >= 89)) {
            maxerrkf = errkf;
            latkf = real(lat);
            lonkf = real(lon);
          }
          if (!(errkr <= maxerrkr || abs(lat) >= 89)) {
            maxerrkr = errkr;
            latkr = real(lat);
            lonkr = real(lon);
          }
          cout << lat0 << " " << lat << " " << lon << " "
               << errf << " " << errr << " "
               << errkf << " " << errkr << "\n";
        }
        cout << "Max errf: " << lat0 << " "
             << maxerrf << " " << latf << " " << lonf << "\n";
        cout << "Max errr: " << lat0 << " "
             << maxerrr << " " << latr << " " << lonr << "\n";
        cout << "Max errkf: " << lat0 << " "
             << maxerrkf << " " << latkf << " " << lonkf << "\n";
        cout << "Max errkr: " << lat0 << " "
             << maxerrkr << " " << latkr << " " << lonkr << "\n";
      }
    }
  }
  catch (const exception& e) {
    cout << "ERROR: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
