/**
 * \file GeodesicLine30.cpp
 * \brief Implementation for GeographicLib::GeodesicLine30 class
 *
 * Copyright (c) Charles Karney (2009-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 *
 * This is a reformulation of the geodesic problem.  The notation is as
 * follows:
 * - at a general point (no suffix or 1 or 2 as suffix)
 *   - phi = latitude
 *   - beta = latitude on auxiliary sphere
 *   - omega = longitude on auxiliary sphere
 *   - lambda = longitude
 *   - alpha = azimuth of great circle
 *   - sigma = arc length along great circle
 *   - s = distance
 *   - tau = scaled distance (= sigma at multiples of pi/2)
 * - at northwards equator crossing
 *   - beta = phi = 0
 *   - omega = lambda = 0
 *   - alpha = alpha0
 *   - sigma = s = 0
 * - a 12 suffix means a difference, e.g., s12 = s2 - s1.
 * - s and c prefixes mean sin and cos
 **********************************************************************/

#include "GeodesicLine30.hpp"

namespace GeographicLib {

  using namespace std;

  template<typename real>
  GeodesicLine30<real>::GeodesicLine30(const Geodesic30<real>& g,
                                       real lat1, real lon1, real azi1,
                                       unsigned caps)
    : _a(g._a)
    , _f(g._f)
    , _b(g._b)
    , _c2(g._c2)
    , _f1(g._f1)
      // Always allow latitude and azimuth
    , _caps(caps | LATITUDE | AZIMUTH)
  {
    azi1 = Math::AngNormalize(azi1);
    // Guard against underflow in salp0
    azi1 = Geodesic30<real>::AngRound(azi1);
    lon1 = Math::AngNormalize(lon1);
    _lat1 = lat1;
    _lon1 = lon1;
    _azi1 = azi1;
    // alp1 is in [0, pi]
    real alp1 = azi1 * Math::degree<real>();
    // Enforce sin(pi) == 0 and cos(pi/2) == 0.  Better to face the ensuing
    // problems directly than to skirt them.
    _salp1 =     azi1  == -180 ? 0 : sin(alp1);
    _calp1 = abs(azi1) ==   90 ? 0 : cos(alp1);
    real cbet1, sbet1, phi;
    phi = lat1 * Math::degree<real>();
    // Ensure cbet1 = +epsilon at poles
    sbet1 = _f1 * sin(phi);
    cbet1 = abs(lat1) == 90 ? Geodesic30<real>::tiny_ : cos(phi);
    Geodesic30<real>::SinCosNorm(sbet1, cbet1);

    // Evaluate alp0 from sin(alp1) * cos(bet1) = sin(alp0),
    _salp0 = _salp1 * cbet1; // alp0 in [0, pi/2 - |bet1|]
    // Alt: calp0 = hypot(sbet1, calp1 * cbet1).  The following
    // is slightly better (consider the case salp1 = 0).
    _calp0 = hypot(_calp1, _salp1 * sbet1);
    // Evaluate sig with tan(bet1) = tan(sig1) * cos(alp1).
    // sig = 0 is nearest northward crossing of equator.
    // With bet1 = 0, alp1 = pi/2, we have sig1 = 0 (equatorial line).
    // With bet1 =  pi/2, alp1 = -pi, sig1 =  pi/2
    // With bet1 = -pi/2, alp1 =  0 , sig1 = -pi/2
    // Evaluate omg1 with tan(omg1) = sin(alp0) * tan(sig1).
    // With alp0 in (0, pi/2], quadrants for sig and omg coincide.
    // No atan2(0,0) ambiguity at poles since cbet1 = +epsilon.
    // With alp0 = 0, omg1 = 0 for alp1 = 0, omg1 = pi for alp1 = pi.
    _ssig1 = sbet1; _somg1 = _salp0 * sbet1;
    _csig1 = _comg1 = sbet1 != 0 || _calp1 != 0 ? cbet1 * _calp1 : 1;
    Geodesic30<real>::SinCosNorm(_ssig1, _csig1); // sig1 in (-pi, pi]
    Geodesic30<real>::SinCosNorm(_somg1, _comg1);

    _k2 = Math::sq(_calp0) * g._ep2;
    real eps = _k2 / (2 * (1 + sqrt(1 + _k2)) + _k2);

    if (_caps & CAP_C1) {
      _A1m1 = Geodesic30<real>::A1m1f(eps);
      Geodesic30<real>::C1f(eps, _C1a);
      _B11 = Geodesic30<real>::SinCosSeries(true, _ssig1, _csig1, _C1a, nC1_);
      real s = sin(_B11), c = cos(_B11);
      // tau1 = sig1 + B11
      _stau1 = _ssig1 * c + _csig1 * s;
      _ctau1 = _csig1 * c - _ssig1 * s;
      // Not necessary because C1pa reverts C1a
      //    _B11 = -SinCosSeries(true, _stau1, _ctau1, _C1pa, nC1p_);
    }

    if (_caps & CAP_C1p)
      Geodesic30<real>::C1pf(eps, _C1pa);

    if (_caps & CAP_C2) {
      _A2m1 = Geodesic30<real>::A2m1f(eps);
      Geodesic30<real>::C2f(eps, _C2a);
      _B21 = Geodesic30<real>::SinCosSeries(true, _ssig1, _csig1, _C2a, nC2_);
    }

    if (_caps & CAP_C3) {
      g.C3f(eps, _C3a);
      _A3c = -_f * _salp0 * g.A3f(eps);
      _B31 = Geodesic30<real>::SinCosSeries(true, _ssig1, _csig1,
                                            _C3a, nC3_-1);
    }

    if (_caps & CAP_C4) {
      g.C4f(_k2, _C4a);
      // Multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0)
      _A4 = Math::sq(_a) * _calp0 * _salp0 * g._e2;
      _B41 = Geodesic30<real>::SinCosSeries(false, _ssig1, _csig1, _C4a, nC4_);
    }
  }

  template<typename real>
  real GeodesicLine30<real>::GenPosition(bool arcmode, real s12_a12,
                                            unsigned outmask,
                                            real& lat2, real& lon2, real& azi2,
                                            real& s12, real& m12,
                                            real& M12, real& M21,
                                            real& S12)
  const {
    outmask &= _caps & OUT_ALL;
    if (!( Init() && (arcmode || (_caps & DISTANCE_IN & OUT_ALL)) ))
      // Uninitialized or impossible distance calculation requested
      return Math::NaN<real>();

    // Avoid warning about uninitialized B12.
    real sig12, ssig12, csig12, B12 = 0, AB1 = 0;
    if (arcmode) {
      // Interpret s12_a12 as spherical arc length
      sig12 = s12_a12 * Math::degree<real>();
      real s12a = abs(s12_a12);
      s12a -= 180 * floor(s12a / 180);
      ssig12 = s12a ==  0 ? 0 : sin(sig12);
      csig12 = s12a == 90 ? 0 : cos(sig12);
    } else {
      // Interpret s12_a12 as distance
      real
        tau12 = s12_a12 / (_b * (1 + _A1m1)),
        s = sin(tau12),
        c = cos(tau12);
      // tau2 = tau1 + tau12
      B12 = - Geodesic30<real>::SinCosSeries(true, _stau1 * c + _ctau1 * s,
                                     _ctau1 * c - _stau1 * s,
                                     _C1pa, nC1p_);
      sig12 = tau12 - (B12 - _B11);
      ssig12 = sin(sig12);
      csig12 = cos(sig12);
    }

    real omg12, lam12, lon12;
    real ssig2, csig2, sbet2, cbet2, somg2, comg2, salp2, calp2;
    // sig2 = sig1 + sig12
    ssig2 = _ssig1 * csig12 + _csig1 * ssig12;
    csig2 = _csig1 * csig12 - _ssig1 * ssig12;
    if (outmask & (DISTANCE | REDUCEDLENGTH | GEODESICSCALE)) {
      if (arcmode)
        B12 = Geodesic30<real>::SinCosSeries(true, ssig2, csig2, _C1a, nC1_);
      AB1 = (1 + _A1m1) * (B12 - _B11);
    }
    // sin(bet2) = cos(alp0) * sin(sig2)
    sbet2 = _calp0 * ssig2;
    // Alt: cbet2 = hypot(csig2, salp0 * ssig2);
    cbet2 = hypot(_salp0, _calp0 * csig2);
    if (cbet2 == 0)
      // I.e., salp0 = 0, csig2 = 0.  Break the degeneracy in this case
      cbet2 = csig2 = Geodesic30<real>::tiny_;
    // tan(omg2) = sin(alp0) * tan(sig2)
    somg2 = _salp0 * ssig2; comg2 = csig2;  // No need to normalize
    // tan(alp0) = cos(sig2)*tan(alp2)
    salp2 = _salp0; calp2 = _calp0 * csig2; // No need to normalize
    // omg12 = omg2 - omg1
    omg12 = atan2(somg2 * _comg1 - comg2 * _somg1,
                  comg2 * _comg1 + somg2 * _somg1);

    if (outmask & DISTANCE)
      s12 = arcmode ? _b * ((1 + _A1m1) * sig12 + AB1) : s12_a12;

    if (outmask & LONGITUDE) {
      lam12 = omg12 + _A3c *
        ( sig12 +
          (Geodesic30<real>::SinCosSeries(true, ssig2, csig2, _C3a, nC3_-1)
           - _B31));
      lon12 = lam12 / Math::degree<real>();
      lon12 = Math::AngNormalize(lon12);
      lon2 = Math::AngNormalize(_lon1 + lon12);
    }

    if (outmask & LATITUDE)
      lat2 = atan2(sbet2, _f1 * cbet2) / Math::degree<real>();

    if (outmask & AZIMUTH)
      // minus signs give range [-180, 180). 0- converts -0 to +0.
      azi2 = 0 - atan2(-salp2, calp2) / Math::degree<real>();

    if (outmask & (REDUCEDLENGTH | GEODESICSCALE)) {
      real
        ssig1sq = Math::sq(_ssig1),
        ssig2sq = Math::sq( ssig2),
        w1 = sqrt(1 + _k2 * ssig1sq),
        w2 = sqrt(1 + _k2 * ssig2sq),
        B22 = Geodesic30<real>::SinCosSeries(true, ssig2, csig2, _C2a, nC2_),
        AB2 = (1 + _A2m1) * (B22 - _B21),
        J12 = (_A1m1 - _A2m1) * sig12 + (AB1 - AB2);
      if (outmask & REDUCEDLENGTH)
        // Add parens around (_csig1 * ssig2) and (_ssig1 * csig2) to ensure
        // accurate cancellation in the case of coincident points.
        m12 = _b * ((w2 * (_csig1 * ssig2) - w1 * (_ssig1 * csig2))
                  - _csig1 * csig2 * J12);
      if (outmask & GEODESICSCALE) {
        M12 = csig12 + (_k2 * (ssig2sq - ssig1sq) *  ssig2 / (w1 + w2)
                        - csig2 * J12) * _ssig1 / w1;
        M21 = csig12 - (_k2 * (ssig2sq - ssig1sq) * _ssig1 / (w1 + w2)
                        - _csig1 * J12) * ssig2 / w2;
      }
    }

    if (outmask & AREA) {
      real
        B42 = Geodesic30<real>::SinCosSeries(false, ssig2, csig2, _C4a, nC4_);
      real salp12, calp12;
      if (_calp0 == 0 || _salp0 == 0) {
        // alp12 = alp2 - alp1, used in atan2 so no need to normalize
        salp12 = salp2 * _calp1 - calp2 * _salp1;
        calp12 = calp2 * _calp1 + salp2 * _salp1;
        // The right thing appears to happen if alp1 = +/-180 and alp2 = 0, viz
        // salp12 = -0 and alp12 = -180.  However this depends on the sign
        // being attached to 0 correctly.  The following ensures the correct
        // behavior.
        if (salp12 == 0 && calp12 < 0) {
          salp12 = Geodesic30<real>::tiny_ * _calp1;
          calp12 = -1;
        }
      } else {
        // tan(alp) = tan(alp0) * sec(sig)
        // tan(alp2-alp1) = (tan(alp2) -tan(alp1)) / (tan(alp2)*tan(alp1)+1)
        // = calp0 * salp0 * (csig1-csig2) / (salp0^2 + calp0^2 * csig1*csig2)
        // If csig12 > 0, write
        //   csig1 - csig2 = ssig12 * (csig1 * ssig12 / (1 + csig12) + ssig1)
        // else
        //   csig1 - csig2 = csig1 * (1 - csig12) + ssig12 * ssig1
        // No need to normalize
        salp12 = _calp0 * _salp0 *
          (csig12 <= 0 ? _csig1 * (1 - csig12) + ssig12 * _ssig1 :
           ssig12 * (_csig1 * ssig12 / (1 + csig12) + _ssig1));
        calp12 = Math::sq(_salp0) + Math::sq(_calp0) * _csig1 * csig2;
      }
      S12 = _c2 * atan2(salp12, calp12) + _A4 * (B42 - _B41);
    }

    return arcmode ? s12_a12 : sig12 / Math::degree<real>();
  }

  template class GeodesicLine30<double>;
#if GEOGRAPHICLIB_HAVE_LONG_DOUBLE
  template class GeodesicLine30<long double>;
#endif

} // namespace GeographicLib
