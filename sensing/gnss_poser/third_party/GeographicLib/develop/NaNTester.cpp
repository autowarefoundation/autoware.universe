/**
 * \file ProjTest.cpp
 * \brief Command line utility for testing transverse Mercator projections
 *
 * Copyright (c) Charles Karney (2008-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include "GeographicLib/EllipticFunction.hpp"
#include "GeographicLib/TransverseMercator.hpp"
#include "GeographicLib/TransverseMercatorExact.hpp"
#include "GeographicLib/PolarStereographic.hpp"
#include <iostream>
#include <cassert>

using namespace GeographicLib;
using namespace std;

int main() {
  {
    Math::real x, y, gamma, k;
    x = y = gamma = k = 0;
    PolarStereographic::UPS().Forward(true, Math::NaN<Math::real>(), 30.0,
                                    x, y, gamma, k);
    cout << x << " " << y << " " << k << "\n";
    x = y = gamma = k = 0;
    PolarStereographic::UPS().Forward(false, -80.0, Math::NaN<Math::real>(),
                                    x, y, gamma, k);
    cout << x << " " << y << " " << gamma << "\n";
  }
  {
    Math::real lat, lon, gamma, k;
    lat = lon = gamma = k = 0;
    PolarStereographic::UPS().Reverse(true, Math::NaN<Math::real>(), 0.0,
                                    lat, lon, gamma, k);
    cout << lat << " " << lon << " " << gamma << " " << k << "\n";
    lat = lon = gamma = k = 0;
    PolarStereographic::UPS().Reverse(false,0.0, Math::NaN<Math::real>(),
                                    lat, lon, gamma, k);
    cout << lat << " " << lon << " " << gamma << " " << k << "\n";
  }
  {
    Math::real x, y, gamma, k;
    x = y = gamma = k = 0;
    TransverseMercator::UTM().Forward(0.0, Math::NaN<Math::real>(), 0.0,
                                    x, y, gamma, k);
    cout << x << " " << y << " " << gamma << " " << k << "\n";
    x = y = gamma = k = 0;
    TransverseMercator::UTM().Forward(0.0, 0.0, Math::NaN<Math::real>(),
                                    x, y, gamma, k);
    cout << x << " " << y << " " << gamma << " " << k << "\n";
    x = y = gamma = k = 0;
    TransverseMercator::UTM().Forward(Math::NaN<Math::real>(), 0.0, 0.0,
                                    x, y, gamma, k);
    cout << x << " " << y << " " << gamma << " " << k << "\n";
  }
  {
    Math::real lat, lon, gamma, k;
    lat = lon = gamma = k = 0;
    TransverseMercator::UTM().Reverse(0.0, Math::NaN<Math::real>(), 0.0,
                                    lat, lon, gamma, k);
    cout << lat << " " << lon << " " << gamma << " " << k << "\n";
    lat = lon = gamma = k = 0;
    TransverseMercator::UTM().Reverse(0.0, 0.0, Math::NaN<Math::real>(),
                                    lat, lon, gamma, k);
    cout << lat << " " << lon << " " << gamma << " " << k << "\n";
    lat = lon = gamma = k = 0;
    TransverseMercator::UTM().Reverse(Math::NaN<Math::real>(), 0.0, 0.0,
                                    lat, lon, gamma, k);
    cout << lon << "\n";
  }
  {
    Math::real x, y, gamma, k;
    x = y = gamma = k = 0;
    TransverseMercatorExact::UTM().Forward(0.0, Math::NaN<Math::real>(), 0.0,
                                         x, y, gamma, k);
    cout << x << " " << y << " " << gamma << " " << k << "\n";
    x = y = gamma = k = 0;
    TransverseMercatorExact::UTM().Forward(0.0, 0.0, Math::NaN<Math::real>(),
                                         x, y, gamma, k);
    cout << x << " " << y << " " << gamma << " " << k << "\n";
    x = y = gamma = k = 0;
    TransverseMercatorExact::UTM().Forward(Math::NaN<Math::real>(), 0.0, 0.0,
                                         x, y, gamma, k);
    cout << x << " " << y << " " << gamma << " " << k << "\n";
  }
  {
    Math::real lat, lon, gamma, k;
    lat = lon = gamma = k = 0;
    TransverseMercatorExact::UTM().Reverse(0.0, Math::NaN<Math::real>(), 0.0,
                                         lat, lon, gamma, k);
    cout << lat << " " << lon << " " << gamma << " " << k << "\n";
    lat = lon = gamma = k = 0;
    TransverseMercatorExact::UTM().Reverse(0.0, 0.0, Math::NaN<Math::real>(),
                                         lat, lon, gamma, k);
    cout << lat << " " << lon << " " << gamma << " " << k << "\n";
    lat = lon = gamma = k = 0;
    TransverseMercatorExact::UTM().Reverse(Math::NaN<Math::real>(), 0.0, 0.0,
                                         lat, lon, gamma, k);
    cout << lon << "\n";
  }
  {
    EllipticFunction e(Math::NaN<Math::real>());
    Math::real sn, cn, dn;
    cout << " k2 " << e.k2()
         << " kp2 " << e.kp2()
         << " K " << e.K()
         << " E " << e.E()
         << " KE " << e.KE() << "\n";
    sn = cn = dn = 0;
    e.sncndn(Math::real(0.1), sn, cn, dn);
    cout << " sncndn " << sn << " " << cn << " " << dn
         << " E(phi) " << e.E(Math::real(0.1))
         << " E(sncndn) " << e.E(sn, cn, dn) << "\n";
  }
  {
    EllipticFunction e(Math::real(0.1));
    Math::real sn, cn, dn;
    sn = cn = dn = 0;
    e.sncndn(Math::NaN<Math::real>(), sn, cn, dn);
    cout << " sncndn " << sn << " " << cn << " " << dn
         << " E(phi) " << e.E(Math::NaN<Math::real>())
         << " E(sncndn) " << e.E(sn, cn, dn) << "\n";
  }
}
