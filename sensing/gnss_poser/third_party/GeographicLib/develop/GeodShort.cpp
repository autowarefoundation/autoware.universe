/**
 * \file GeodShort.cpp
 *
 * Test various solutions to the inverse problem in the limit of short lines
 * (following Bowring 1981).
 **********************************************************************/

#include <ctime>                // for std::time
#include <functional>           // for std::function and std::bind
#include <random>               // for C++11 random numbers
#include <limits>               // for digits
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/EllipticFunction.hpp>

using namespace GeographicLib;
using namespace std;

class GeodShort {
private:
  typedef Math::real real;
  real _a, _f, _f1, _b, _e2, _ep2, _e;
  EllipticFunction _E;
  inline real eatanhe(real x) const {
    return _f >= 0 ? _e * atanh(_e * x) : - _e * atan(_e * x);
  }
  static inline real psi0f(real phi) { return asinh(tan(phi)); }
  static inline real invpsi0f(real psi) { return atan(sinh(psi)); }
  inline real psif(real phi) { return psi0f(phi) - eatanhe(sin(phi)); }
  static inline void SinCosNorm(real& sinx, real& cosx) {
    real r = hypot(sinx, cosx);
    sinx /= r;
    cosx /= r;
  }
  static real GreatCircle(real sbet1, real cbet1,
                          real sbet2, real cbet2,
                          real omg12, real& azi1, real& azi2) {
    real
      // bet12 = bet2 - bet1 in [0, pi)
      sbet12 = sbet2 * cbet1 - cbet2 * sbet1,
      sbet12a = sbet2 * cbet1 + cbet2 * sbet1,
      somg12 = sin(omg12), comg12 = cos(omg12);
    real
      salp1 = cbet2 * somg12,
      calp1 = (comg12 >= 0 ?
               sbet12 + cbet2 * sbet1 * Math::sq(somg12) / (1 + comg12) :
               sbet12a - cbet2 * sbet1 * Math::sq(somg12) / (1 - comg12));
    real
      ssig12 = hypot(salp1, calp1),
      csig12 = sbet1 * sbet2 + cbet1 * cbet2 * comg12;

    real
      salp2 = cbet1 * somg12,
      calp2 = sbet12 - cbet1 * sbet2 * Math::sq(somg12) / (1 + comg12),
      sig12 = atan2(ssig12, csig12);

    azi1 = atan2(salp1, calp1) / Math::degree();
    azi2 = atan2(salp2, calp2) / Math::degree();
    return sig12;
  }
public:
  GeodShort(real a, real f)
    : _a(a)
    , _f(f)
    , _f1(1 - _f)
    , _b(_a * _f1)
    , _e2(_f * (2 - _f))
    , _ep2(_e2/Math::sq(_f1))
    , _e(sqrt(abs(_e2)))
    , _E(-_ep2) {}
  real Inverse(real lat1, real lon1, real lat2, real lon2,
               real& azi1, real& azi2) {
    int mode = 1;
    real
      phi1 = Math::degree() * lat1,
      phi2 = Math::degree() * lat2,
      lam12 = Math::degree() * Math::AngNormalize(lon2 - lon1),
      sbet1 = _f1 * sin(phi1), cbet1 = cos(phi1),
      sbet2 = _f1 * sin(phi2), cbet2 = cos(phi2);
    SinCosNorm(sbet1, cbet1); SinCosNorm(sbet2, cbet2);
    real
      dn1 = sqrt(1 + _ep2 * Math::sq(sbet1)),
      dn2 = sqrt(1 + _ep2 * Math::sq(sbet2)),
      dnm;
    if (mode == 0)
       dnm = (dn1 + dn2) / 2;
    else {
      real sbetm2 = Math::sq(sbet1 + sbet2);
      sbetm2 = sbetm2 / (sbetm2 + Math::sq(cbet1 + cbet2));
      dnm = sqrt(1 + _ep2 * sbetm2);
    }
    return _b * dnm * GreatCircle(sbet1, cbet1, sbet2, cbet2,
                                  lam12 / (_f1 * dnm),
                                  azi1, azi2);
  }
  real Inverse2(real lat1, real lon1, real lat2, real lon2,
               real& azi1, real& azi2) {
    real
      phi1 = Math::degree() * lat1,
      phi2 = Math::degree() * lat2,
      lam12 = Math::degree() * Math::AngNormalize(lon2 - lon1),
      sbet1 = _f1 * sin(phi1), cbet1 = cos(phi1),
      sbet2 = _f1 * sin(phi2), cbet2 = cos(phi2);
    SinCosNorm(sbet1, cbet1); SinCosNorm(sbet2, cbet2);
    real dnm;
    real sbetm2 = Math::sq(sbet1 + sbet2);
    sbetm2 = sbetm2 / (sbetm2 + Math::sq(cbet1 + cbet2));
    dnm = sqrt(1 + _ep2 * sbetm2);
    // Adjust bet1 and bet2 via conformal map
    real
      A = 1/(dnm * _f1),
      phim = atan((sbet1+sbet2)/(_f1*(cbet1+cbet2))),
      betm = atan((sbet1+sbet2)/(cbet1+cbet2)),
      psim = psif(phim),
      psipm = psi0f(betm),
      K = psipm - A * psim,
      bet1 = invpsi0f( A*psif(phi1) + K ),
      bet2 = invpsi0f( A*psif(phi2) + K );
    sbet1 = sin(bet1); cbet1 = cos(bet1);
    sbet2 = sin(bet2); cbet2 = cos(bet2);
    return _b * dnm * GreatCircle(sbet1, cbet1, sbet2, cbet2,
                                  lam12 / (_f1 * dnm),
                                  azi1, azi2);
  }
  real Bowring0(real lat1, real lon1, real lat2, real lon2,
                real& azi1, real& azi2) {
    int mode = 2;
    real
      phi1 = Math::degree() * lat1,
      phi2 = Math::degree() * lat2,
      lam12 = Math::degree() * Math::AngNormalize(lon2 - lon1),
      m = 0.5,
      phim = ( abs(phi1) >= abs(phi2) ?
               (1 - m) * phi1 + m * phi2 :
               (1 - m) * phi2 + m * phi1 ),
      dphi1 = phi1 - phim,
      dphi2 = phi2 - phim,
      cosm = cos(phim),
      sinm = sin(phim),
      A = sqrt(1 + _ep2 * Math::sq(Math::sq(cosm))),
      B = sqrt(1 + _ep2 * Math::sq(cosm)),
      C = sqrt(1 + _ep2),
      omg12 = A * lam12,
      phipm = atan(tan(phim)/B),
      R = _a * C/(B*B),
      phip1, phip2;
    if (mode == 0) {
      phip1 = phipm + (dphi1/B) * (1 + (3*_ep2*dphi1)/(4*B*B) *
                                   sin(2 * (phim + dphi1/3)));
      phip2 = phipm + (dphi2/B) * (1 + (3*_ep2*dphi2)/(4*B*B) *
                                   sin(2 * (phim + dphi2/3)));
    } else if (mode == 1) {
      real
        psi1 = psif(phi1), psi2 = psif(phi2),
        psim = psif(phim), psipm = psi0f(phipm);
      phip1 = invpsi0f(A * (psi1-psim) + psipm);
      phip2 = invpsi0f(A * (psi2-psim) + psipm);
    } else {
      phip1 = phipm + (dphi1/B) *
        ( 1 + _ep2*dphi1/(2*B*B) *
          (3*cosm*sinm + dphi1*(1-2*sinm*sinm +
                                _ep2 * (cosm*cosm * (1 + 3*sinm*sinm)))/B*B));
      phip2 = phipm + (dphi2/B) *
        ( 1 + _ep2*dphi2/(2*B*B) *
          (3*cosm*sinm + dphi2*(1-2*sinm*sinm +
                                _ep2 * (cosm*cosm * (1 + 3*sinm*sinm)))/B*B));
    }
    return R * GreatCircle(sin(phip1), cos(phip1), sin(phip2), cos(phip2),
                           omg12, azi1, azi2);
  }
  real Bowring1(real lat1, real lon1, real lat2, real lon2,
                real& azi1, real& azi2) {
    real
      phi1 = Math::degree() * lat1,
      phi2 = Math::degree() * lat2,
      lam12 = Math::degree() * Math::AngNormalize(lon2 - lon1),
      bet1 = atan(_f1 * tan(phi1)),
      bet2 = atan(_f1 * tan(phi2)),
      betm = (bet1 + bet2)/2,
      phim = atan(tan(betm) / _f1),
      cosm = cos(phim),
      A = sqrt(1 + _ep2 * Math::sq(Math::sq(cosm))),
      B = sqrt(1 + _ep2 * Math::sq(cosm)),
      C = sqrt(1 + _ep2),
      omg12 = A * lam12,
      phipm = atan(tan(phim)/B),
      R = _a * C/(B*B),
      m1 = _E.E(sin(bet1), cos(bet1), sqrt(1 + _ep2 * Math::sq(sin(bet1)))),
      m2 = _E.E(sin(bet2), cos(bet2), sqrt(1 + _ep2 * Math::sq(sin(bet2)))),
      mm = _E.E(sin(betm), cos(betm), sqrt(1 + _ep2 * Math::sq(sin(betm)))),
      phip1 = phipm + (m1 - mm) * _a * _f1 / R,
      phip2 = phipm + (m2 - mm) * _a * _f1 / R;
    return R * GreatCircle(sin(phip1), cos(phip1), sin(phip2), cos(phip2),
                           omg12, azi1, azi2);
  }
  real Bowring2(real lat1, real lon1, real lat2, real lon2,
                real& azi1, real& azi2) {
    real highfact = 1;
    real
      phi1 = Math::degree() * lat1,
      phi2 = Math::degree() * lat2,
      lam12 = Math::degree() * Math::AngNormalize(lon2 - lon1),
      bet1 = atan(_f1 * tan(phi1)),
      bet2 = atan(_f1 * tan(phi2)),
      betm = (bet1 + bet2)/2,
      sbetm = sin(betm),
      cbetm = cos(betm),
      phim = atan(tan(betm) / _f1),
      cosm = cos(phim),
      A = sqrt(1 + _ep2 * Math::sq(Math::sq(cosm))),
      B = sqrt(1 + _ep2 * Math::sq(cosm)),
      C = sqrt(1 + _ep2),
      omg12 = A * lam12,
      phipm = atan(tan(phim)/B),
      R = _a * C/(B*B),
      dbet1 = bet1-betm,
      dbet2 = bet2-betm,
      dnm2 = 1+_ep2*Math::sq(sbetm),
      dnm = sqrt(dnm2),
      phip1 = phipm + dbet1/dnm *
      (1 + dbet1 * _ep2/(2*dnm2) *
       ( cbetm*sbetm + highfact * dbet1 * ( Math::sq(cbetm) -
                                            Math::sq(sbetm)*dnm2)/(3*dnm2) )),
      phip2 = phipm + dbet2/dnm *
      (1 + dbet2 * _ep2/(2*dnm2) *
       ( cbetm*sbetm + highfact * dbet2 * ( Math::sq(cbetm) -
                                            Math::sq(sbetm)*dnm2)/(3*dnm2) ));
    return R * GreatCircle(sin(phip1), cos(phip1), sin(phip2), cos(phip2),
                           omg12, azi1, azi2);
  }
};

int main(int argc, char* argv[]) {
  try {
    // Args are f and sig
    if (argc != 3) {
      cerr << "Usage: GeodShort f sig\n";
      return 1;
    }
    typedef Math::real real;
    real
      f = Utility::fract<real>(string(argv[1])),
      sig = Utility::val<real>(string(argv[2]));
    Geodesic g(1, f);
    GeodesicExact ge(1, f);
    GeodShort s(1, f);
    bool exact = abs(f) > 0.02;
    real norm, consist;
    {
      real m;
      if (exact)
        ge.Inverse(0, 0, 90, 0, m);
      else
        g.Inverse(0, 0, 90, 0, m);
      norm = max(m, Math::pi()/2 * g.EquatorialRadius());
      consist = min(m, Math::pi()/2 * g.EquatorialRadius()) /
        (Math::pi()/2);
    }
    unsigned seed = random_device()(); // Set seed from random_device
    mt19937 r(seed);                   // Initialize URNG
    uniform_real_distribution<double> U;
    cout << norm << " " << consist << " "
         << f << " " << sig << " " << seed << endl;
    real maxerr1 = -1, maxerr2 = -1, maxerr3 = -1;
    for (unsigned k = 0; k < 10000000; ++k) {
      real
        lat1 = 90*real(U(r)),
        lon1 = 0,
        azi1 = 180*real(U(r)),
        lat2, lon2, s12, azi2;
      if (exact)
        ge.ArcDirect(lat1, lon1, azi1, sig, lat2, lon2, azi2, s12);
      else
        g.ArcDirect(lat1, lon1, azi1, sig, lat2, lon2, azi2, s12);
      real
        s12a = s.Bowring2(lat1, lon1, lat2, lon2, azi1, azi2),
        err1 = abs(s12a - s12) / norm;
      if (err1 > maxerr1) {
        maxerr1 = err1;
        cout << "A " << k << " "
                  << lat1 << " " << azi1 << " " << maxerr1 << endl;
      }
      real lat, lon, azi1a, azi2a;
      if (exact)
        ge.Direct(lat1, lon1, azi1, s12a, lat, lon);
      else
        g.Direct(lat1, lon1, azi1, s12a, lat, lon);
      real err2 = s.Inverse(lat2, lon2, lat, lon, azi1a, azi2a);
      if (exact)
        ge.Direct(lat2, lon2, azi2, -s12a, lat, lon);
      else
        g.Direct(lat2, lon2, azi2, -s12a, lat, lon);
      err2 = max(err2, s.Inverse(lat1, lon1, lat, lon, azi1a, azi2a)) / norm;
      if (err2 > maxerr2) {
        maxerr2 = err2;
        cout << "B " << k << " "
                  << lat1 << " " << azi1 << " " << maxerr2 << endl;
      }
      real latx, lonx;
      if (exact) {
        ge.Direct(lat1, lon1, azi1, s12a + consist, lat, lon);
        ge.Direct(lat2, lon2, azi2, consist, latx, lonx);
      } else {
        g.Direct(lat1, lon1, azi1, s12a + consist, lat, lon);
        g.Direct(lat2, lon2, azi2, consist, latx, lonx);
      }
      real err3 = s.Inverse(lat, lon, latx, lonx, azi1a, azi2a);
      if (exact) {
        ge.Direct(lat1, lon1, azi1, -consist, lat, lon);
        ge.Direct(lat2, lon2, azi2, -s12a - consist, latx, lonx);
      } else {
        g.Direct(lat1, lon1, azi1, -consist, lat, lon);
        g.Direct(lat2, lon2, azi2, -s12a - consist, latx, lonx);
      }
      err3 = max(err3, s.Inverse(lat, lon, latx, lonx, azi1a, azi2a)) / norm;
      if (err3 > maxerr3) {
        maxerr3 = err3;
        cout << "C " << k << " "
                  << lat1 << " " << azi1 << " " << maxerr3 << endl;
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
  cout << "DONE\n";
  return 0;
}
