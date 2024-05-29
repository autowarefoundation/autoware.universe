// Determine time and position of closest approach for two vessels traveling
// along geodesics.  Thanks to
//    Jorge D. Taramona
//    Inspector de Navegación Aérea
//    Dirección General de Aviación Civil del Perú
// for suggesting this problem.  See discussion entry from 2014-08-19:
// https://sourceforge.net/p/geographiclib/discussion/1026620/thread/33ce09e0/

// Compile with
// g++ -O3  -std=c++11 -o ClosestApproach ClosestApproach.cpp -lGeographic -L/usr/local/lib -Wl,-rpath=/usr/local/lib

// Example: Planes leave

// (1) Istanbul 42N 29E bearing 51W traveling at 900 km/hr
// (2) Reyjkavik 64N 22W bearing 154E traveling at 800 km/hr

// at the same time; compute time of closest approach.  Guess that the time of
// closest approach is after 1 hr.  So run

// echo 42 29 -51 900e3 64 -22 154 800e3 1 | ./ClosestApproach

// 0 1 2696504 -3.532249e+12 46.74926 19.83775 57.4055 -16.17089
// 1 2.751696 1083726 6.786464e+10 52.80517 -0.05892146 45.43817 -9.817806
// 2 2.71894 1082700 -2657490 52.72565 0.3576737 45.66487 -9.90999
// 3 2.718941 1082700 -0.005293798 52.72565 0.3576574 45.66486 -9.909987
// 4 2.718941 1082700 0 52.72565 0.3576574 45.66486 -9.909987
// 5 2.718941 1082700 0 52.72565 0.3576574 45.66486 -9.909987
// 6 2.718941 1082700 0 52.72565 0.3576574 45.66486 -9.909987
// 7 2.718941 1082700 0 52.72565 0.3576574 45.66486 -9.909987
// 8 2.718941 1082700 0 52.72565 0.3576574 45.66486 -9.909987
// 9 2.718941 1082700 0 52.72565 0.3576574 45.66486 -9.909987

// At closest approach
//   time = 2.72 hr
//   dist = 1083 km
//   position of first plane = 52.7N 0.3E
//   position of second plane = 45.7N 9.9W

// CAVEAT: if the initial guess is bad, this program may return the time at
// which the distance is a maximum.

// Explanation:

// Consider f(t) = s12^2 / 2
// Find zeros of g(t) = f'(t) = s12' * s12 by Newton's method
// using g'(t) = (s12')^2 + s12'' * s12

// Using s12^2 / 2 as the governing function (as opposed, for example, to
// |s12|) means that in the planar limit f(t) is a quadratic function and g(t)
// is a linear function.  So Newton's method just needs a single iteration.

// Let gam{1,2} be angle between s12 and v{1,2}.

// s12' = v2 * cos(gam2) - v1 * cos(gam1)
// g(t) = s12 * (v2 * cos(gam2) - v1 * cos(gam1))
// s12'' =  - v2*sin(gam2) * dgam2/dt + v1*sin(gam1) * dgam1/dt
// dgam1/dt = (+ M12*v1*sin(gam1) - v2*sin(gam2)) / m12
// dgam2/dt = (- M21*v2*sin(gam2) + v1*sin(gam1)) / m12
// s12'' = (M12*v1^2*sin(gam1)^2 + M21*v2^2*sin(gam2)^2
//          - 2*v1*v2*sin(gam1)*sin(gam2)) / m12
// g'(t) = v1^2 * (cos(gam1)^2 + M12*s12/m12 * sin(gam1)^2)
//       + v2^2 * (cos(gam2)^2 + M21*s12/m12 * sin(gam2)^2)
//       - 2*v1*v2 * (cos(gam1)*cos(gam2) + s12/m12 * sin(gam1)*sin(gam2))

// Planar limit (m12 = s12, M12 = M21 = 1):
// g(t) = (v2-v1) . (r2-r1)
// g'(t) = v1^2 + v2^2 - 2*v1*v2 * cos(gam1-gam2)
//       = |v1-v2|^2 = const

// Special case when v2 = 0 (simple interception), v1 = 1

// s12' = - cos(gam1)
// g(t) = - s12 * cos(gam1)
// s12'' =  sin(gam1) * dgam1/dt
// dgam1/dt = M12*sin(gam1) / m12
// s12'' = M12*sin(gam1)^2 / m12
// g'(t) =  (cos(gam1)^2 + M12*s12/m12 * sin(gam1)^2)

// Planar limit (m12 = s12, M12 = M21 = 1):
// g(t) = -v1 . (r2-r1)
// g'(t) = 1 = const

#include <iostream>
#include <iomanip>
#include <cmath>

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/Utility.hpp>

using namespace std;
using namespace GeographicLib;

int main() {
  try {
    typedef Math::real real;
    Utility::set_digits();
    const Geodesic/*Exact*/ g = Geodesic/*Exact*/(Constants::WGS84_a(),
                                          Constants::WGS84_f());
    real lat1, lon1, azi1, v1;
    real lat2, lon2, azi2, v2;
    real t0;
    cout << setprecision(7);
    while (cin
           >> lat1 >> lon1 >> azi1 >> v1
           >> lat2 >> lon2 >> azi2 >> v2
           >> t0) {
      real t = t0;
      for (int i = 0; i < 10; ++i) {
        // Compute positions at time t
        real lat1a, lon1a, azi1a;
        real lat2a, lon2a, azi2a;
        g.Direct(lat1, lon1, azi1, t*v1, lat1a, lon1a, azi1a);
        g.Direct(lat2, lon2, azi2, t*v2, lat2a, lon2a, azi2a);
        // Compute distance at time t
        real s12, azi1c, azi2c, m12, M12, M21;
        g.Inverse(lat1a, lon1a, lat2a, lon2a,
                  s12, azi1c, azi2c, m12, M12, M21);
        real
          r12 = s12 / m12,
          gam1 = (azi1c - azi1a) * Math::degree(),
          gam2 = (azi2c - azi2a) * Math::degree(),
          x1 = v1 * cos(gam1), y1 = v1 * sin(gam1),
          x2 = v2 * cos(gam2), y2 = v2 * sin(gam2);
        // Find g = 0 using Newton's method where
        // g = d( (1/2)*s12^2 ) / dt = s12 * d(s12)/dt
        real g = s12 * (x2 - x1);
        real dg =             // dg = dg/dt
          (Math::sq(x1) + M12 * r12 * Math::sq(y1)) +
          (Math::sq(x2) + M21 * r12 * Math::sq(y2)) -
          2 * (x1 * x2 + r12 * y1 * y2);
        cout << i << " "
             << fixed << setprecision(12) << t << " " << s12 << " "
             << defaultfloat << setprecision(6) << g << " "
             << fixed << setprecision(20)
             << lat1a << " " << lon1a << " "
             << fixed << setprecision(6)
             << lat2a << " " << lon2a << "\n";
        t -= g/dg;              // Newton iteration
      }
    }
  }
  catch (const exception& e) {
    cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  catch (...) {
    cerr << "Caught unknown exception\n";
    return 1;
  }
  return 0;
}
