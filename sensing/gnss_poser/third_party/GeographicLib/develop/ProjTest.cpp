/**
 * \file ProjTest.cpp
 * \brief Command line utility for testing transverse Mercator projections
 *
 * Copyright (c) Charles Karney (2008-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include "GeographicLib/LambertConformalConic.hpp"
#include "GeographicLib/PolarStereographic.hpp"
#include "GeographicLib/TransverseMercator.hpp"
#include "GeographicLib/TransverseMercatorExact.hpp"
#include "GeographicLib/Constants.hpp"
#include <string>
#include <limits>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <stdexcept>

class DataFile {
private:
  typedef GeographicLib::Math::real real;
  std::istream* _istr;
  std::ifstream _fstr;
  //  String headers
  //    COORDINATES
  //    PROJECTION
  //    DATUM
  std::string _coords, _proj, _datum;
  //  Numeric headers
  //    CENTRAL MERIDIAN
  //    FALSE EASTING
  //    FALSE NORTHING
  //    LATITUDE OF TRUE SCALE
  //    LONGITUDE DOWN FROM POLE
  //    ORIGIN LATITUDE
  //    SCALE FACTOR
  //    STANDARD PARALLEL ONE
  //    STANDARD PARALLEL TWO
  real _lon0, _fe, _fn, _latts, _lonfp, _lat0, _k0, _lat1, _lat2;
  DataFile(const DataFile&);
  DataFile& operator=(const DataFile&);
public:
  DataFile(const std::string& file) {
    if (!(file.empty() || file == "-")) {
      _fstr.open(file.c_str());
      if (!_fstr.good())
        throw std::out_of_range("Cannot open " + file);
    }

    _istr = (file.empty() || file == "-") ? &std::cin : &_fstr;
    _coords = _proj = _datum = "NONE";
    _lon0 = _fe = _fn = _latts = _lonfp = _lat0 = _k0 = _lat1 = _lat2
      = GeographicLib::Math::NaN<real>();
    std::string s;
    while (getline(*_istr, s)) {
      if (s.empty() || s[0] == '#')
        continue;
      if (s.substr(0, 13) == "END OF HEADER")
        break;
      std::string::size_type p = s.find(':');
      if (p == std::string::npos)
        continue;
      std::string key(s, 0, p);
      p = s.find_first_not_of(" \t\v\r\n\f", p + 1);
      if (p == std::string::npos)
        continue;
      s = s.substr(p);
      p = s.find_last_not_of(" \t\v\r\n\f");
      s = s.substr(0, p + 1);
      std::istringstream is(s);
      if (key == "COORDINATES") {
        _coords = s;
      } else if (key == "PROJECTION") {
        _proj = s;
      } else if (key == "DATUM") {
        _datum = s;
      } else if (key == "CENTRAL MERIDIAN") {
        is >> _lon0;
      } else if (key == "FALSE EASTING") {
        is >> _fe;
      } else if (key == "FALSE NORTHING") {
        is >> _fn;
      } else if (key == "LATITUDE OF TRUE SCALE") {
        is >> _latts;
      } else if (key == "LONGITUDE DOWN FROM POLE") {
        is >> _lonfp;
      } else if (key == "ORIGIN LATITUDE") {
        is >> _lat0;
      } else if (key == "SCALE FACTOR") {
        is >> _k0;
      } else if (key == "STANDARD PARALLEL ONE") {
        is >> _lat1;
      } else if (key == "STANDARD PARALLEL TWO") {
        is >> _lat2;
      }
    }
  }
  bool Next(real &x, real &y) {
    char c;
    // Avoid a warning about void* changed to bool
    return (*_istr >> x >> c >> y) ? true : false;
  }
  bool Next(real &x, real &y, real &z) {
    char c, d;
    // Avoid a warning about void* changed to bool
    return (*_istr >> x >> c >> y >> d >> z) ? true : false;
  }
  const std::string& coords() const { return _coords; }
  const std::string& proj() const { return _proj; }
  const std::string& datum() const { return _datum; }
  real lon0() const { return _lon0; }
  real fe() const { return _fe; }
  real fn() const { return _fn; }
  real latts() const { return _latts; }
  real lonfp() const { return _lonfp; }
  real lat0() const { return _lat0; }
  real k0() const { return _k0; }
  real lat1() const { return _lat1; }
  real lat2() const { return _lat2; }
};

GeographicLib::Math::real
dist(GeographicLib::Math::real a, GeographicLib::Math::real f,
     GeographicLib::Math::real lat0, GeographicLib::Math::real lon0,
     GeographicLib::Math::real lat1, GeographicLib::Math::real lon1) {
  using namespace GeographicLib;
  using std::cos; using std::sin; using std::sqrt; using std::hypot;
  typedef Math::real real;
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

int usage(int retval) {
  ( retval ? std::cerr : std::cout ) <<
"ProjTest latlonfile projfile\n\
\n\
Checks projections against NGS GoldData files\n";
  return retval;
}

int main(int argc, char* argv[]) {
  using namespace GeographicLib;
  typedef Math::real real;

  if (argc != 3)
    return usage(1);
  try {
    real eps = 2*std::numeric_limits<real>::epsilon();

    enum { undef = 0, tm = 1, ps = 2, lcc = 3 };
    int m = 0;
    std::string geof(argv[++m]);
    DataFile geo(geof);
    std::string projf(argv[++m]);
    DataFile proj(projf);
    if (geo.coords() != "Geodetic")
      throw std::out_of_range("Unsupported coordinates " + geo.coords());
    real a, f;
    if (geo.datum() == "WGE") {
      a = Constants::WGS84_a();
      f = Constants::WGS84_f();
    } else if (geo.datum() == "Test_sphere") {
      a = 20000000/Math::pi();
      f = 0;
    } else if (geo.datum() == "Test_SRMmax") {
      a = 6400000;
      f = 1/real(150);
    } else
      throw std::out_of_range("Unsupported datum " + geo.datum());
    if (proj.datum() != geo.datum())
      throw std::out_of_range("Datum mismatch " +
                              geo.datum() + " " + proj.datum());
    real fe, fn, lat0, lat1, lat2, lon0, k1;
    int type = undef;
    if (proj.proj() == "Lambert Conformal Conic (1 parallel)") {
      fe = proj.fe();
      fn = proj.fn();
      k1 = proj.k0();
      lon0 = proj.lon0();
      lat1 = lat2 = lat0 = proj.lat0();
      type = lcc;
    } else if (proj.proj() == "Lambert Conformal Conic (2 parallel)") {
      fe = proj.fe();
      fn = proj.fn();
      k1 = 1;
      lon0 = proj.lon0();
      lat0 = proj.lat0();
      lat1 = proj.lat1();
      lat2 = proj.lat2();
      type = lcc;
    } else if (proj.proj() == "Mercator") {
      using std::isfinite;
      fe = proj.fe();
      fn = proj.fn();
      k1 = proj.k0();
      if (!isfinite(k1))
        k1 = 1;
      lon0 = proj.lon0();
      lat0 = 0;
      lat1 = proj.latts();
      if (!isfinite(lat1))
        lat1 = 0;
      lat2 = -lat1;
      type = lcc;
    } else if (proj.proj() == "Polar Stereographic") {
      using std::isfinite;
      fe = proj.fe();
      fn = proj.fn();
      lon0 = proj.lonfp();
      lat0 = 90;
      k1 = proj.k0();
      if (!isfinite(k1))
        k1 = 1;
      real latts = proj.latts();
      if (isfinite(latts)) {
        if (latts < 0)
          lat0 = -lat0;
        LambertConformalConic tx(a, f, lat0, 1);
        real x, y, gam, k;
        tx.Forward(0, latts, 10, x, y, gam, k);
        k1 = 1/k;
      }
      lat1 = lat2 = lat0;
      type = lon0 == 0 ? ps : lcc;
    } else if (proj.proj() == "Transverse Mercator") {
      fe = proj.fe();
      fn = proj.fn();
      lon0 = proj.lon0();
      k1 = proj.k0();
      lat0 = lat1 = lat2 = proj.lat0();
      type = tm;
    } else
      throw std::out_of_range("Unsupported projection " + proj.proj());
    LambertConformalConic tx(a, f, lat1, lat2, k1);
    PolarStereographic txa(a, f, k1);
    TransverseMercator txb(a, f, k1);
    TransverseMercatorExact txc(a, f == 0 ? real(0.1)/eps : f, k1);
    std::cout << a << " 1/" << 1/f << " " << k1 << " " << type << "\n";
    real x0, y0, gam, k;
    if (type == lcc)
      tx.Forward(lon0, lat0, lon0, x0, y0, gam, k);
    else if (type == tm)
      txb.Forward(lon0, lat0, lon0, x0, y0, gam, k);
    else
      x0 = y0 = 0;
    real lata, lona, xa, ya;
    unsigned count = 0;
    real maxerrx = 0, maxerry = 0, maxerr = 0, maxerrk = 0, maxerrr = 0;
    std::cout << std::fixed << std::setprecision(7);
    while (geo.Next(lata, lona) && proj.Next(xa, ya)) {
      using std::abs; using std::hypot;
      ++count;
      // Suppress bogus uninitialized warnings for lat and lon
      real lat = 0, lon = 0, x, y, xx, yy;
      x = xa - fe + x0;
      y = ya - fn + y0;
      xx = x/k1;
      yy = y/k1;
      switch (type) {
      case lcc:
        tx.Reverse(lon0, x, y, lat, lon, gam, k);
        break;
      case ps:
        txa.Reverse(lat1 > 0, x, y, lat, lon, gam, k);
        break;
      case tm:
        txb.Reverse(lon0, x, y, lat, lon, gam, k);
        break;
      default: break;           // To suppress warning
      }
      real errr = dist(a, f, lata, lona, lat, lon);
      maxerrr = std::max(errr, maxerrr);
      if (!(errr < 1e-6))
        std::cout << "REV: "
                  << lata << " "
                  << lona << " "
                  << xx << " "
                  << yy << " "
                  << errr << "\n";

      switch (type) {
      case lcc:
        tx.Forward(lon0, lata, lona, x, y, gam, k);
        break;
      case ps:
        txa.Forward(lat1 > 0, lata, lona, x, y, gam, k);
        break;
      case tm:
        txb.Forward(lon0, lata, lona, x, y, gam, k);
        break;
      default: break;           // To suppress warning
      }
      x -= x0;
      y -= y0;
      x += fe;
      y += fn;
      real
        errx = abs(x - xa),
        erry = abs(y - ya),
        err = hypot(errx, erry),
        errk = err/std::max(real(1),k);
      std::ostringstream sx, sxa, sy, sya;
      sx << std::fixed << std::setprecision(6) << x;
      sxa << std::fixed << std::setprecision(6) << xa;
      sy << std::fixed << std::setprecision(6) << y;
      sya << std::fixed << std::setprecision(6) << ya;
      // if (sx.str() != sxa.str() || sy.str() != sya.str())
      if (!(errk < 1e-6))
        std::cout << "FOR: "
                  << lata << " "
                  << lona << " "
                  << xx << " "
                  << yy << " "
                  << errk << "\n";
      maxerrx = std::max(errx, maxerrx);
      maxerry = std::max(erry, maxerry);
      maxerr = std::max(err, maxerr);
      maxerrk = std::max(errk, maxerrk);
    }
    std::cout << count << " records; maxerr "
              << maxerrk << " " << maxerrr << "\n";
  }
  catch (const std::exception& e) {
    std::cout << "ERROR: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
