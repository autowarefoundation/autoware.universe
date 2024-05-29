/**
 * \file GeodTest.cpp
 **********************************************************************/

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "GeographicLib/Geodesic.hpp"
#include "GeographicLib/GeodesicLine.hpp"
#include "GeographicLib/GeodesicExact.hpp"
#include "GeographicLib/GeodesicLineExact.hpp"
#include "GeographicLib/Constants.hpp"
#include <GeographicLib/Utility.hpp>

#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <limits>

using namespace std;
using namespace GeographicLib;

int usage(int retval) {
  ( retval ? cerr : cout ) <<
"GeodTest [ -a | -c | -t0 | -t1 | -t2 | -t3 | -h ]\n\
\n\
Check GeographicLib::Geodesic class.\n\
-a (default) accuracy test (reads test data on standard input)\n\
-E accuracy test with GeodesicExact (reads test data on standard input)\n\
-F accuracy test with GeodesicExact (reads test data on standard input\n\
   first line gives a and f)\n\
-c coverage test (reads test data on standard input)\n\
-t0 time GeodecicLine with distances using synthetic data\n\
-t1 time GeodecicLine with angles using synthetic data\n\
-t2 time Geodecic::Direct using synthetic data\n\
-t3 time Geodecic::Inverse with synthetic data\n\
-T0 time GeodecicLineExact with distances using synthetic data\n\
-T1 time GeodecicLineExact with angles using synthetic data\n\
-T2 time GeodecicExact::Direct using synthetic data\n\
-T3 time GeodecicExact::Inverse with synthetic data\n\
\n\
-c requires an instrumented version of Geodesic.\n";
  return retval;
}

Math::real angdiff(Math::real a1, Math::real a2) {
  Math::real d = a2 - a1;
  if (d >= 180)
    d -= 360;
  else if (d < -180)
    d += 360;
  return d;
}

Math::real azidiff(Math::real lat,
                   Math::real lon1, Math::real lon2,
                   Math::real azi1, Math::real azi2) {
  Math::real
    phi = lat * Math::degree(),
    alpha1 = azi1 * Math::degree(),
    alpha2 = azi2 * Math::degree(),
    dlam = angdiff(lon1, lon2) * Math::degree();
  Math::real res = sin(alpha2-alpha1)*cos(dlam)
    -cos(alpha2-alpha1)*sin(dlam)*sin(phi)
    // -sin(alpha1)*cos(alpha2)*(1-cos(dlam))*cos(phi)*cos(phi)
    ;
  return res;
}

Math::real dist(Math::real a, Math::real f,
                Math::real lat0, Math::real lon0,
                Math::real lat1, Math::real lon1) {
  //  typedef GeographicLib::Math::real real;
  //  real s12;
  //  GeographicLib::Geodesic::
  //    WGS84.Inverse(real(lat0), real(lon0), real(lat1), real(lon1), s12);
  //  return Math::real(s12);
  a *= Math::degree();
  if (abs(lat0 + lat1) > Math::real(179.998)) {
    // Near pole, transform into polar coordinates
    Math::real
      r0 = 90 - abs(lat0),
      r1 = 90 - abs(lat1),
      lam0 = lon0 * Math::degree(),
      lam1 = lon1 * Math::degree();
    return (a / (1 - f)) *
      hypot(r0 * cos(lam0) - r1 * cos(lam1), r0 * sin(lam0) - r1 * sin(lam1));
  } else {
    // Otherwise use cylindrical formula
    Math::real
      phi = lat0 * Math::degree(),
      cphi = abs(lat0) <= 45 ? cos(phi)
      : sin((90 - abs(lat0)) * Math::degree()),
      e2 = f * (2 - f),
      sinphi = sin(phi),
      n = 1/sqrt(1 - e2 * sinphi * sinphi),
      // See Wikipedia article on latitude
      degreeLon = a * cphi * n,
      degreeLat = a * (1 - e2) * n * n * n,
      dlon = angdiff(lon1, lon0),
      dlat = lat1 - lat0;
    dlat *= degreeLat;
    dlon *= degreeLon;
    return hypot(dlat, dlon);
  }
}

// err[0] error in position of point 2 for the direct problem.
// err[1] error in azimuth at point 2 for the direct problem.
// err[2] error in m12 for the direct problem & inverse (except near conjugacy)
// err[3] error in s12 for the inverse problem.
// err[4] error in the azimuths for the inverse problem scaled by m12.
// err[5] consistency of the azimuths for the inverse problem.
// err[6] area error direct & inverse (except near conjugacy)
template<class test>
void GeodError(const test& tgeod,
               Math::real lat1, Math::real lon1, Math::real azi1,
               Math::real lat2, Math::real lon2, Math::real azi2,
               Math::real s12, Math::real /*a12*/,
               Math::real m12, Math::real S12,
               vector<Math::real>& err) {
  Math::real tlat1, tlon1, tazi1, tlat2, tlon2, tazi2, ts12, tm12a, tm12b,
    tM12, tM21, tS12a, tS12b /*, ta12*/;
  Math::real rlat1, rlon1, razi1, rlat2, rlon2, razi2, rm12;
  tgeod.Direct(lat1, lon1, azi1,  s12,
               tlat2, tlon2, tazi2, tm12a,
               tM12, tM21, tS12a);
  tS12a -= tgeod.EllipsoidArea() * (tazi2-azi2)/720;
  tgeod.Direct(lat2, lon2, azi2, -s12,
               tlat1, tlon1, tazi1, tm12b,
               tM12, tM21, tS12b);
  tS12b -= tgeod.EllipsoidArea() * (tazi1-azi1)/720;
  err[0] = max(dist(tgeod.EquatorialRadius(), tgeod.Flattening(),
                    lat2, lon2, tlat2, tlon2),
               dist(tgeod.EquatorialRadius(), tgeod.Flattening(),
                    lat1, lon1, tlat1, tlon1));
  err[1] = max(abs(azidiff(lat2, lon2, tlon2, azi2, tazi2)),
               abs(azidiff(lat1, lon1, tlon1, azi1, tazi1))) *
    tgeod.EquatorialRadius();
  err[2] = max(abs(tm12a - m12), abs(tm12b + m12));
  if (!isnan(S12))
    err[6] = max(abs(tS12a - S12), abs(tS12b + S12)) / tgeod.EquatorialRadius();

  /* ta12 = */ tgeod.Inverse(lat1, lon1, lat2, lon2,
                             ts12, tazi1, tazi2, tm12a,
                             tM12, tM21, tS12a);
  tS12a -= tgeod.EllipsoidArea() * ((tazi2-azi2)-(tazi1-azi1))/720;
  err[3] = abs(ts12 - s12);
  err[4] = max(abs(angdiff(azi1, tazi1)), abs(angdiff(azi2, tazi2))) *
    Math::degree() * abs(m12);
  if (lat1 + lat2 == 0)
    err[4] = min(err[4],
                 max(abs(angdiff(azi1, tazi2)), abs(angdiff(azi2, tazi1))) *
                 Math::degree() * abs(m12));
  // m12 and S12 are very sensitive with the inverse problem near conjugacy
  if (!(s12 > tgeod.EquatorialRadius() && m12 < 10e3)) {
    err[2] = max(err[2], abs(tm12a - m12));
    if (!isnan(S12))
      err[6] = max(err[6], abs(tS12a - S12) / tgeod.EquatorialRadius());
  }
  if (s12 > tgeod.EquatorialRadius()) {
    tgeod.Direct(lat1, lon1, tazi1,   ts12/2, rlat2, rlon2, razi2, rm12);
    tgeod.Direct(lat2, lon2, tazi2, - ts12/2, rlat1, rlon1, razi1, rm12);
    err[5] = dist(tgeod.EquatorialRadius(), tgeod.Flattening(),
                  rlat1, rlon1, rlat2, rlon2);
  } else {
    tgeod.Direct(lat1, lon1, tazi1,
                 ts12 + tgeod.EquatorialRadius(),
                 rlat2, rlon2, razi2, rm12);
    tgeod.Direct(lat2, lon2, tazi2, tgeod.EquatorialRadius(),
                 rlat1, rlon1, razi1, rm12);
    err[5] = dist(tgeod.EquatorialRadius(), tgeod.Flattening(),
                  rlat1, rlon1, rlat2, rlon2);
    tgeod.Direct(lat1, lon1, tazi1, - tgeod.EquatorialRadius(),
                 rlat2, rlon2, razi2, rm12);
    tgeod.Direct(lat2, lon2, tazi2,
                 - ts12 - tgeod.EquatorialRadius(),
                 rlat1, rlon1, razi1, rm12);
    err[5] = max(err[5], dist(tgeod.EquatorialRadius(), tgeod.Flattening(),
                              rlat1, rlon1, rlat2, rlon2));
  }
}

int main(int argc, char* argv[]) {
  Utility::set_digits();
  Math::real a = Constants::WGS84_a();
  Math::real f = Constants::WGS84_f();
  bool timing = false;
  int timecase = 0; // 0 = line, 1 = line ang, 2 = direct, 3 = inverse
  bool accuracytest = true;
  bool coverage = false;
  bool exact = false;
  if (argc == 2) {
    string arg = argv[1];
    if (arg == "-a") {
      accuracytest = true;
      coverage = false;
      timing = false;
      exact = false;
    } else if (arg == "-E") {
      accuracytest = true;
      coverage = false;
      timing = false;
      exact = true;
    } else if (arg == "-F") {
      accuracytest = true;
      coverage = false;
      timing = false;
      exact = true;
      string s;
      getline(cin, s);
      istringstream str(s);
      str >> a >> f;
    } else if (arg == "-c") {
      accuracytest = false;
      coverage = true;
      timing = false;
      exact = false;
    } else if (arg == "-t0") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 0;
      exact = false;
    } else if (arg == "-t1") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 1;
      exact = false;
    } else if (arg == "-t2") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 2;
      exact = false;
    } else if (arg == "-t3") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 3;
      exact = false;
    } else if (arg == "-T0") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 0;
      exact = true;
    } else if (arg == "-T1") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 1;
      exact = true;
    } else if (arg == "-T2") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 2;
      exact = true;
    } else if (arg == "-T3") {
      accuracytest = false;
      coverage = false;
      timing = true;
      timecase = 3;
      exact = true;
    } else
      return usage(arg == "-h" ? 0 : 1);
  } else if (argc > 2)
    return usage(1);

  if (timing) {
    if (!exact) {
      const Geodesic& geod = Geodesic::WGS84();
      unsigned cnt = 0;
      Math::real s = 0;
      Math::real dl;
      switch (timecase) {
      case 0:
        // Time Line
        dl = 2e7/1000;
        for (int i = 0; i <= 90; ++i) {
          Math::real lat1 = i;
          for (int j = 0; j <= 180; ++j) {
            Math::real azi1 = j;
            const GeodesicLine l(geod, lat1, 0.0, azi1);
            for (int k = 0; k <= 1000; ++k) {
              Math::real s12 = dl * k;
              Math::real lat2, lon2;
              l.Position(s12, lat2, lon2);
              ++cnt;
              s += lat2;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      case 1:
        // Time Line ang
        dl = Math::real(180)/1000;
        for (int i = 0; i <= 90; ++i) {
          Math::real lat1 = i;
          for (int j = 0; j <= 180; ++j) {
            Math::real azi1 = j;
            GeodesicLine l(geod, lat1, 0.0, azi1);
            for (int k = 0; k <= 1000; ++k) {
              Math::real s12 = dl * k;
              Math::real lat2, lon2;
              l.ArcPosition(s12, lat2, lon2);
              ++cnt;
              s += lat2;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      case 2:
        // Time Direct
        dl = 2e7/200;
        for (int i = 0; i <= 90; ++i) {
          Math::real lat1 = i;
          for (int j = 0; j <= 180; ++j) {
            Math::real azi1 = j;
            for (int k = 0; k <= 200; ++k) {
              Math::real s12 = dl * k;
              Math::real lat2, lon2;
              geod.Direct(lat1, 0.0, azi1, s12, lat2, lon2);
              ++cnt;
              s += lat2;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      case 3:
        // Time Inverse
        for (int i = 1; i <= 179; i += 2) {
          Math::real lat1 = i * Math::real(0.5);
          for (int j = -179; j <= 179; j += 2) {
            Math::real lat2 = j * Math::real(0.5);
            for (int k = 1; k <= 359; k += 2) {
              Math::real lon2 = k * Math::real(0.5);
              Math::real s12;
              geod.Inverse(lat1, 0.0, lat2, lon2, s12);
              ++cnt;
              s += s12;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      }
    } else {
      const GeodesicExact& geod = GeodesicExact::WGS84();
      unsigned cnt = 0;
      Math::real s = 0;
      Math::real dl;
      switch (timecase) {
      case 0:
        // Time Line
        dl = 2e7/1000;
        for (int i = 0; i <= 90; ++i) {
          Math::real lat1 = i;
          for (int j = 0; j <= 180; ++j) {
            Math::real azi1 = j;
            const GeodesicLineExact l(geod, lat1, 0.0, azi1);
            for (int k = 0; k <= 1000; ++k) {
              Math::real s12 = dl * k;
              Math::real lat2, lon2;
              l.Position(s12, lat2, lon2);
              ++cnt;
              s += lat2;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      case 1:
        // Time Line ang
        dl = Math::real(180)/1000;
        for (int i = 0; i <= 90; ++i) {
          Math::real lat1 = i;
          for (int j = 0; j <= 180; ++j) {
            Math::real azi1 = j;
            GeodesicLineExact l(geod, lat1, 0.0, azi1);
            for (int k = 0; k <= 1000; ++k) {
              Math::real s12 = dl * k;
              Math::real lat2, lon2;
              l.ArcPosition(s12, lat2, lon2);
              ++cnt;
              s += lat2;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      case 2:
        // Time Direct
        dl = 2e7/200;
        for (int i = 0; i <= 90; ++i) {
          Math::real lat1 = i;
          for (int j = 0; j <= 180; ++j) {
            Math::real azi1 = j;
            for (int k = 0; k <= 200; ++k) {
              Math::real s12 = dl * k;
              Math::real lat2, lon2;
              geod.Direct(lat1, 0.0, azi1, s12, lat2, lon2);
              ++cnt;
              s += lat2;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      case 3:
        // Time Inverse
        for (int i = 1; i <= 179; i += 2) {
          Math::real lat1 = i * Math::real(0.5);
          for (int j = -179; j <= 179; j += 2) {
            Math::real lat2 = j * Math::real(0.5);
            for (int k = 1; k <= 359; k += 2) {
              Math::real lon2 = k * Math::real(0.5);
              Math::real s12;
              geod.Inverse(lat1, 0.0, lat2, lon2, s12);
              ++cnt;
              s += s12;
            }
          }
        }
        cout << cnt << " " << s << "\n";
        break;
      }
    }
  }
  else if (accuracytest || coverage) {
    const Geodesic geod(a, f);
    const GeodesicExact geode(a, f);

    const unsigned NUMERR = 7;

    cout << fixed << setprecision(2);
    vector<Math::real> erra(NUMERR);
    vector<Math::real> err(NUMERR, 0.0);
    vector<unsigned> errind(NUMERR);
    unsigned cnt = 0;
    string s;
    while (getline(cin, s)) {
      istringstream str(s);
      Math::real lat1l, lon1l, azi1l, lat2l, lon2l, azi2l,
        s12l, a12l, m12l, S12l;
      if (!(str >> lat1l >> lon1l >> azi1l
                >> lat2l >> lon2l >> azi2l
                >> s12l >> a12l >> m12l))
        break;
      if (!(str >> S12l))
        S12l = Math::NaN();
      if (coverage) {
#if defined(GEOD_DIAG) && GEOD_DIAG
        Math::real
          lat1 = lat1l, lon1 = lon1l,
          lat2 = lat2l, lon2 = lon2l,
          azi1, azi2, s12, m12;
        geod.Inverse(lat1, lon1, lat2, lon2, s12, azi1, azi2, m12);
        cout << geod.coverage << " " << geod.niter << "\n";
#endif
      } else {
        exact ?
          GeodError< GeodesicExact >
          (geode, lat1l, lon1l, azi1l,
           lat2l, lon2l, azi2l,
           s12l, a12l, m12l, S12l,
           erra) :
          GeodError< Geodesic >
          (geod, lat1l, lon1l, azi1l,
           lat2l, lon2l, azi2l,
           s12l, a12l, m12l, S12l,
           erra);
        for (unsigned i = 0; i < NUMERR; ++i) {
          if (isfinite(err[i]) && !(erra[i] <= err[i])) {
            err[i] = erra[i];
            errind[i] = cnt;
          }
        }
        ++cnt;
      }
    }
    if (accuracytest) {
      Math::real mult = Math::real(Math::extra_digits() == 0 ? 1e9l :
                                   Math::extra_digits() <= 3 ? 1e12l : 1e15l);
      for (unsigned i = 0; i < NUMERR; ++i)
        cout << i << " " << mult * err[i]
             << " " << errind[i] << "\n";
    }
  }
}
