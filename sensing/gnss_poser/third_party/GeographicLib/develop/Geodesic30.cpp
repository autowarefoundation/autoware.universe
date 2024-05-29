/**
 * \file Geodesic30.cpp
 * \brief Implementation for GeographicLib::Geodesic30 class
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

#include "Geodesic30.hpp"
#include "GeodesicLine30.hpp"

#if defined(_MSC_VER)
// Squelch warnings about potentially uninitialized local variables
#  pragma warning (disable: 4701)
#endif

namespace GeographicLib {

  using namespace std;

  // Underflow guard.  We require
  //   tiny_ * epsilon() > 0
  //   tiny_ + epsilon() == epsilon()
  template<typename real>
  const real Geodesic30<real>::tiny_ = sqrt(numeric_limits<real>::min());
  template<typename real>
  const real Geodesic30<real>::tol0_ = numeric_limits<real>::epsilon();
  // Increase multiplier in defn of tol1_ from 100 to 200 to fix inverse case
  // 52.784459512564 0 -52.784459512563990912 179.634407464943777557
  // which otherwise failed for Visual Studio 10 (Release and Debug)
  template<typename real>
  const real Geodesic30<real>::tol1_ = 200 * tol0_;
  template<typename real>
  const real Geodesic30<real>::tol2_ = sqrt(numeric_limits<real>::epsilon());
  template<typename real>
  const real Geodesic30<real>::xthresh_ = 1000 * tol2_;

  template<typename real>
  Geodesic30<real>::Geodesic30(real a, real f)
    : _a(a)
    , _f(f <= 1 ? f : 1/f)
    , _f1(1 - _f)
    , _e2(_f * (2 - _f))
    , _ep2(_e2 / Math::sq(_f1))       // e2 / (1 - e2)
    , _n(_f / ( 2 - _f))
    , _b(_a * _f1)
    , _c2((Math::sq(_a) + Math::sq(_b) *
           (_e2 == 0 ? 1 :
            (_e2 > 0 ? atanh(sqrt(_e2)) : atan(sqrt(-_e2))) /
            sqrt(abs(_e2))))/2) // authalic radius squared
      // The sig12 threshold for "really short"
    , _etol2(10 * tol2_ / max(real(0.1), sqrt(abs(_e2))))
  {
    if (!(isfinite(_a) && _a > 0))
      throw GeographicErr("Major radius is not positive");
    if (!(isfinite(_b) && _b > 0))
      throw GeographicErr("Minor radius is not positive");
    A3coeff();
    C3coeff();
    C4coeff();
  }

  template<typename real>
  const Geodesic30<real> Geodesic30<real>::WGS84(Constants::WGS84_a<real>(),
                                                 Constants::WGS84_f<real>());

  template<typename real>
  real Geodesic30<real>::SinCosSeries(bool sinp,
                                      real sinx, real cosx,
                                      const real c[], int n) {
    // Evaluate
    // y = sinp ? sum(c[i] * sin( 2*i    * x), i, 1, n) :
    //            sum(c[i] * cos((2*i+1) * x), i, 0, n-1) :
    // using Clenshaw summation.  N.B. c[0] is unused for sin series
    // Approx operation count = (n + 5) mult and (2 * n + 2) add
    c += (n + sinp);            // Point to one beyond last element
    real
      ar = 2 * (cosx - sinx) * (cosx + sinx), // 2 * cos(2 * x)
      y0 = n & 1 ? *--c : 0, y1 = 0;          // accumulators for sum
    // Now n is even
    n /= 2;
    while (n--) {
      // Unroll loop x 2, so accumulators return to their original role
      y1 = ar * y0 - y1 + *--c;
      y0 = ar * y1 - y0 + *--c;
    }
    return sinp
      ? 2 * sinx * cosx * y0    // sin(2 * x) * y0
      : cosx * (y0 - y1);       // cos(x) * (y0 - y1)
  }

  template<typename real>
  GeodesicLine30<real> Geodesic30<real>::Line(real lat1, real lon1, real azi1,
                                              unsigned caps) const {
    return GeodesicLine30<real>(*this, lat1, lon1, azi1, caps);
  }

  template<typename real>
  real Geodesic30<real>::GenDirect(real lat1, real lon1, real azi1,
                                   bool arcmode, real s12_a12,
                                   unsigned outmask,
                                   real& lat2, real& lon2, real& azi2,
                                   real& s12, real& m12,
                                   real& M12, real& M21,
                                   real& S12) const {
    return GeodesicLine30<real>(*this, lat1, lon1, azi1,
                                // supply DISTANCE_IN if necessary
                                outmask | (arcmode ? NONE : DISTANCE_IN))
      .                         // Note the dot!
      GenPosition(arcmode, s12_a12, outmask,
                  lat2, lon2, azi2, s12, m12, M12, M21, S12);
  }

  template<typename real>
  real Geodesic30<real>::GenInverse(real lat1, real lon1,
                                    real lat2, real lon2,
                                    unsigned outmask,
                                    real& s12, real& azi1, real& azi2,
                                    real& m12, real& M12, real& M21,
                                    real& S12) const {
    outmask &= OUT_ALL;
    lon1 = Math::AngNormalize(lon1);
    real lon12 = Math::AngNormalize(Math::AngNormalize(lon2) - lon1);
    // If very close to being on the same meridian, then make it so.
    // Not sure this is necessary...
    lon12 = AngRound(lon12);
    // Make longitude difference positive.
    int lonsign = lon12 >= 0 ? 1 : -1;
    lon12 *= lonsign;
    if (lon12 == 180)
      lonsign = 1;
    // If really close to the equator, treat as on equator.
    lat1 = AngRound(lat1);
    lat2 = AngRound(lat2);
    // Swap points so that point with higher (abs) latitude is point 1
    int swapp = abs(lat1) >= abs(lat2) ? 1 : -1;
    if (swapp < 0) {
      lonsign *= -1;
      swap(lat1, lat2);
    }
    // Make lat1 <= 0
    int latsign = lat1 < 0 ? 1 : -1;
    lat1 *= latsign;
    lat2 *= latsign;
    // Now we have
    //
    //     0 <= lon12 <= 180
    //     -90 <= lat1 <= 0
    //     lat1 <= lat2 <= -lat1
    //
    // longsign, swapp, latsign register the transformation to bring the
    // coordinates to this canonical form.  In all cases, 1 means no change was
    // made.  We make these transformations so that there are few cases to
    // check, e.g., on verifying quadrants in atan2.  In addition, this
    // enforces some symmetries in the results returned.

    real phi, sbet1, cbet1, sbet2, cbet2, s12x, m12x;

    phi = lat1 * Math::degree<real>();
    // Ensure cbet1 = +epsilon at poles
    sbet1 = _f1 * sin(phi);
    cbet1 = lat1 == -90 ? tiny_ : cos(phi);
    SinCosNorm(sbet1, cbet1);

    phi = lat2 * Math::degree<real>();
    // Ensure cbet2 = +epsilon at poles
    sbet2 = _f1 * sin(phi);
    cbet2 = abs(lat2) == 90 ? tiny_ : cos(phi);
    SinCosNorm(sbet2, cbet2);

    // If cbet1 < -sbet1, then cbet2 - cbet1 is a sensitive measure of the
    // |bet1| - |bet2|.  Alternatively (cbet1 >= -sbet1), abs(sbet2) + sbet1 is
    // a better measure.  This logic is used in assigning calp2 in Lambda12.
    // Sometimes these quantities vanish and in that case we force bet2 = +/-
    // bet1 exactly.  An example where is is necessary is the inverse problem
    // 48.522876735459 0 -48.52287673545898293 179.599720456223079643
    // which failed with Visual Studio 10 (Release and Debug)

    if (cbet1 < -sbet1) {
      if (cbet2 == cbet1)
        sbet2 = sbet2 < 0 ? sbet1 : -sbet1;
    } else {
      if (abs(sbet2) == -sbet1)
        cbet2 = cbet1;
    }

    real
      lam12 = lon12 * Math::degree<real>(),
      slam12 = lon12 == 180 ? 0 : sin(lam12),
      clam12 = cos(lam12);      // lon12 == 90 isn't interesting

    real a12, sig12, calp1, salp1, calp2, salp2;
    // index zero elements of these arrays are unused
    real C1a[nC1_ + 1], C2a[nC2_ + 1], C3a[nC3_];

    bool meridian = lat1 == -90 || slam12 == 0;

    if (meridian) {

      // Endpoints are on a single full meridian, so the geodesic might lie on
      // a meridian.

      calp1 = clam12; salp1 = slam12; // Head to the target longitude
      calp2 = 1; salp2 = 0;           // At the target we're heading north

      real
        // tan(bet) = tan(sig) * cos(alp)
        ssig1 = sbet1, csig1 = calp1 * cbet1,
        ssig2 = sbet2, csig2 = calp2 * cbet2;

      // sig12 = sig2 - sig1
      sig12 = atan2(max(csig1 * ssig2 - ssig1 * csig2, real(0)),
                    csig1 * csig2 + ssig1 * ssig2);
      {
        real dummy;
        Lengths(_n, sig12, ssig1, csig1, ssig2, csig2,
                cbet1, cbet2, s12x, m12x, dummy,
                (outmask & GEODESICSCALE) != 0U, M12, M21, C1a, C2a);
      }
      // Add the check for sig12 since zero length geodesics might yield m12 <
      // 0.  Test case was
      //
      //    echo 20.001 0 20.001 0 | GeodSolve -i
      //
      // In fact, we will have sig12 > pi/2 for meridional geodesic which is
      // not a shortest path.
      if (sig12 < 1 || m12x >= 0) {
        m12x *= _a;
        s12x *= _b;
        a12 = sig12 / Math::degree<real>();
      } else
        // m12 < 0, i.e., prolate and too close to anti-podal
        meridian = false;
    }

    real omg12;
    if (!meridian &&
        sbet1 == 0 &&   // and sbet2 == 0
        // Mimic the way Lambda12 works with calp1 = 0
        (_f <= 0 || lam12 <= Math::pi<real>() - _f * Math::pi<real>())) {

      // Geodesic runs along equator
      calp1 = calp2 = 0; salp1 = salp2 = 1;
      s12x = _a * lam12;
      m12x = _b * sin(lam12 / _f1);
      if (outmask & GEODESICSCALE)
        M12 = M21 = cos(lam12 / _f1);
      a12 = lon12 / _f1;
      sig12 = omg12 = lam12 / _f1;

    } else if (!meridian) {

      // Now point1 and point2 belong within a hemisphere bounded by a
      // meridian and geodesic is neither meridional or equatorial.

      // Figure a starting point for Newton's method
      sig12 = InverseStart(sbet1, cbet1, sbet2, cbet2,
                           lam12,
                           salp1, calp1, salp2, calp2,
                           C1a, C2a);

      if (sig12 >= 0) {
        // Short lines (InverseStart sets salp2, calp2)
        real wm = sqrt(1 - _e2 * Math::sq((cbet1 + cbet2) / 2));
        s12x = sig12 * _a * wm;
        m12x = Math::sq(wm) * _a / _f1 * sin(sig12 * _f1 / wm);
        if (outmask & GEODESICSCALE)
          M12 = M21 = cos(sig12 * _f1 / wm);
        a12 = sig12 / Math::degree<real>();
        omg12 = lam12 / wm;
      } else {

        // Newton's method
        real ssig1, csig1, ssig2, csig2, eps;
        real ov = 0;
        unsigned numit = 0;
        for (unsigned trip = 0; numit < maxit_; ++numit) {
          real dv;
          real v = Lambda12(sbet1, cbet1, sbet2, cbet2, salp1, calp1,
                            salp2, calp2, sig12, ssig1, csig1, ssig2, csig2,
                            eps, omg12, trip < 1, dv, C1a, C2a, C3a) - lam12;
          if (!(abs(v) > tiny_) || !(trip < 1)) {
            if (!(abs(v) <= max(tol1_, ov)))
              numit = maxit_;
            break;
          }
          real
            dalp1 = -v/dv;
          real
            sdalp1 = sin(dalp1), cdalp1 = cos(dalp1),
            nsalp1 = salp1 * cdalp1 + calp1 * sdalp1;
          calp1 = calp1 * cdalp1 - salp1 * sdalp1;
          salp1 = max(real(0), nsalp1);
          SinCosNorm(salp1, calp1);
          // In some regimes we don't get quadratic convergence because slope
          // -> 0.  So use convergence conditions based on epsilon instead of
          // sqrt(epsilon).  The first criterion is a test on abs(v) against
          // 200 * epsilon.  The second takes credit for an anticipated
          // reduction in abs(v) by v/ov (due to the latest update in alp1) and
          // checks this against epsilon.
          if (!(abs(v) >= tol1_ && Math::sq(v) >= ov * tol0_)) ++trip;
          ov = abs(v);
        }

        if (numit >= maxit_) {
          // Signal failure.
          if (outmask & DISTANCE)
            s12 = Math::NaN<real>();
          if (outmask & AZIMUTH)
            azi1 = azi2 = Math::NaN<real>();
          if (outmask & REDUCEDLENGTH)
            m12 = Math::NaN<real>();
          if (outmask & GEODESICSCALE)
            M12 = M21 = Math::NaN<real>();
          if (outmask & AREA)
            S12 = Math::NaN<real>();
          return Math::NaN<real>();
        }

        {
          real dummy;
          Lengths(eps, sig12, ssig1, csig1, ssig2, csig2,
                  cbet1, cbet2, s12x, m12x, dummy,
                  (outmask & GEODESICSCALE) != 0U, M12, M21, C1a, C2a);
        }
        m12x *= _a;
        s12x *= _b;
        a12 = sig12 / Math::degree<real>();
        omg12 = lam12 - omg12;
      }
    }

    if (outmask & DISTANCE)
      s12 = 0 + s12x;           // Convert -0 to 0

    if (outmask & REDUCEDLENGTH)
      m12 = 0 + m12x;           // Convert -0 to 0

    if (outmask & AREA) {
      real
        // From Lambda12: sin(alp1) * cos(bet1) = sin(alp0)
        salp0 = salp1 * cbet1,
        calp0 = hypot(calp1, salp1 * sbet1); // calp0 > 0
      real alp12;
      if (calp0 != 0 && salp0 != 0) {
        real
          // From Lambda12: tan(bet) = tan(sig) * cos(alp)
          ssig1 = sbet1, csig1 = calp1 * cbet1,
          ssig2 = sbet2, csig2 = calp2 * cbet2,
          k2 = Math::sq(calp0) * _ep2,
          // Multiplier = a^2 * e^2 * cos(alpha0) * sin(alpha0).
          A4 = Math::sq(_a) * calp0 * salp0 * _e2;
        SinCosNorm(ssig1, csig1);
        SinCosNorm(ssig2, csig2);
        real C4a[nC4_];
        C4f(k2, C4a);
        real
          B41 = SinCosSeries(false, ssig1, csig1, C4a, nC4_),
          B42 = SinCosSeries(false, ssig2, csig2, C4a, nC4_);
        S12 = A4 * (B42 - B41);
      } else
        // Avoid problems with indeterminate sig1, sig2 on equator
        S12 = 0;

      if (!meridian &&
          omg12 < real(0.75) * Math::pi<real>() && // Long difference too big
          sbet2 - sbet1 < real(1.75)) {            // Lat difference too big
        // Use tan(Gamma/2) = tan(omg12/2)
        // * (tan(bet1/2)+tan(bet2/2))/(1+tan(bet1/2)*tan(bet2/2))
        // with tan(x/2) = sin(x)/(1+cos(x))
        real
          somg12 = sin(omg12), domg12 = 1 + cos(omg12),
          dbet1 = 1 + cbet1, dbet2 = 1 + cbet2;
        alp12 = 2 * atan2( somg12 * ( sbet1 * dbet2 + sbet2 * dbet1 ),
                           domg12 * ( sbet1 * sbet2 + dbet1 * dbet2 ) );
      } else {
        // alp12 = alp2 - alp1, used in atan2 so no need to normalize
        real
          salp12 = salp2 * calp1 - calp2 * salp1,
          calp12 = calp2 * calp1 + salp2 * salp1;
        // The right thing appears to happen if alp1 = +/-180 and alp2 = 0, viz
        // salp12 = -0 and alp12 = -180.  However this depends on the sign
        // being attached to 0 correctly.  The following ensures the correct
        // behavior.
        if (salp12 == 0 && calp12 < 0) {
          salp12 = tiny_ * calp1;
          calp12 = -1;
        }
        alp12 = atan2(salp12, calp12);
      }
      S12 += _c2 * alp12;
      S12 *= swapp * lonsign * latsign;
      // Convert -0 to 0
      S12 += 0;
    }

    // Convert calp, salp to azimuth accounting for lonsign, swapp, latsign.
    if (swapp < 0) {
      swap(salp1, salp2);
      swap(calp1, calp2);
      if (outmask & GEODESICSCALE)
        swap(M12, M21);
    }

    salp1 *= swapp * lonsign; calp1 *= swapp * latsign;
    salp2 *= swapp * lonsign; calp2 *= swapp * latsign;

    if (outmask & AZIMUTH) {
      // minus signs give range [-180, 180). 0- converts -0 to +0.
      azi1 = 0 - atan2(-salp1, calp1) / Math::degree<real>();
      azi2 = 0 - atan2(-salp2, calp2) / Math::degree<real>();
    }

    // Returned value in [0, 180]
    return a12;
  }

  template<typename real>
  void Geodesic30<real>::Lengths(real eps, real sig12,
                                 real ssig1, real csig1,
                                 real ssig2, real csig2,
                                 real cbet1, real cbet2,
                                 real& s12b, real& m12a, real& m0,
                                 bool scalep, real& M12, real& M21,
                                 // Scratch areas of the right size
                                 real C1a[], real C2a[]) const {
    // Return m12a = (reduced length)/_a; also calculate s12b = distance/_b,
    // and m0 = coefficient of secular term in expression for reduced length.
    C1f(eps, C1a);
    C2f(eps, C2a);
    real
      A1m1 = A1m1f(eps),
      AB1 = (1 + A1m1) * (SinCosSeries(true, ssig2, csig2, C1a, nC1_) -
                          SinCosSeries(true, ssig1, csig1, C1a, nC1_)),
      A2m1 = A2m1f(eps),
      AB2 = (1 + A2m1) * (SinCosSeries(true, ssig2, csig2, C2a, nC2_) -
                          SinCosSeries(true, ssig1, csig1, C2a, nC2_)),
      cbet1sq = Math::sq(cbet1),
      cbet2sq = Math::sq(cbet2),
      w1 = sqrt(1 - _e2 * cbet1sq),
      w2 = sqrt(1 - _e2 * cbet2sq),
      // Make sure it's OK to have repeated dummy arguments
      m0x = A1m1 - A2m1,
      J12 = m0x * sig12 + (AB1 - AB2);
    m0 = m0x;
    // Missing a factor of _a.
    // Add parens around (csig1 * ssig2) and (ssig1 * csig2) to ensure accurate
    // cancellation in the case of coincident points.
    m12a = (w2 * (csig1 * ssig2) - w1 * (ssig1 * csig2))
      - _f1 * csig1 * csig2 * J12;
    // Missing a factor of _b
    s12b = (1 + A1m1) * sig12 + AB1;
    if (scalep) {
      real csig12 = csig1 * csig2 + ssig1 * ssig2;
      J12 *= _f1;
      M12 = csig12 + (_e2 * (cbet1sq - cbet2sq) * ssig2 / (w1 + w2)
                      - csig2 * J12) * ssig1 / w1;
      M21 = csig12 - (_e2 * (cbet1sq - cbet2sq) * ssig1 / (w1 + w2)
                      - csig1 * J12) * ssig2 / w2;
    }
  }

  template<typename real>
  real Geodesic30<real>::Astroid(real x, real y) {
    // Solve k^4+2*k^3-(x^2+y^2-1)*k^2-2*y^2*k-y^2 = 0 for positive root k.
    // This solution is adapted from Geocentric::Reverse.
    real k;
    real
      p = Math::sq(x),
      q = Math::sq(y),
      r = (p + q - 1) / 6;
    if ( !(q == 0 && r <= 0) ) {
      real
        // Avoid possible division by zero when r = 0 by multiplying equations
        // for s and t by r^3 and r, resp.
        S = p * q / 4,            // S = r^3 * s
        r2 = Math::sq(r),
        r3 = r * r2,
        // The discrimant of the quadratic equation for T3.  This is zero on
        // the evolute curve p^(1/3)+q^(1/3) = 1
        disc = S * (S + 2 * r3);
      real u = r;
      if (disc >= 0) {
        real T3 = S + r3;
        // Pick the sign on the sqrt to maximize abs(T3).  This minimizes loss
        // of precision due to cancellation.  The result is unchanged because
        // of the way the T is used in definition of u.
        T3 += T3 < 0 ? -sqrt(disc) : sqrt(disc); // T3 = (r * t)^3
        // N.B. cbrt always returns the real root.  cbrt(-8) = -2.
        real T = cbrt(T3); // T = r * t
        // T can be zero; but then r2 / T -> 0.
        u += T + (T != 0 ? r2 / T : 0);
      } else {
        // T is complex, but the way u is defined the result is real.
        real ang = atan2(sqrt(-disc), -(S + r3));
        // There are three possible cube roots.  We choose the root which
        // avoids cancellation.  Note that disc < 0 implies that r < 0.
        u += 2 * r * cos(ang / 3);
      }
      real
        v = sqrt(Math::sq(u) + q),    // guaranteed positive
        // Avoid loss of accuracy when u < 0.
        uv = u < 0 ? q / (v - u) : u + v, // u+v, guaranteed positive
        w = (uv - q) / (2 * v);           // positive?
      // Rearrange expression for k to avoid loss of accuracy due to
      // subtraction.  Division by 0 not possible because uv > 0, w >= 0.
      k = uv / (sqrt(uv + Math::sq(w)) + w);   // guaranteed positive
    } else {               // q == 0 && r <= 0
      // y = 0 with |x| <= 1.  Handle this case directly.
      // for y small, positive root is k = abs(y)/sqrt(1-x^2)
      k = 0;
    }
    return k;
  }

  template<typename real>
  real Geodesic30<real>::InverseStart(real sbet1, real cbet1,
                                      real sbet2, real cbet2,
                                      real lam12,
                                      real& salp1, real& calp1,
                                      // Only updated if return val >= 0
                                      real& salp2, real& calp2,
                                      // Scratch areas of the right size
                                      real C1a[], real C2a[]) const {
    // Return a starting point for Newton's method in salp1 and calp1 (function
    // value is -1).  If Newton's method doesn't need to be used, return also
    // salp2 and calp2 and function value is sig12.
    real
      sig12 = -1,               // Return value
      // bet12 = bet2 - bet1 in [0, pi); bet12a = bet2 + bet1 in (-pi, 0]
      sbet12 = sbet2 * cbet1 - cbet2 * sbet1,
      cbet12 = cbet2 * cbet1 + sbet2 * sbet1;
#if defined(__GNUC__) && __GNUC__ == 4 && \
  (__GNUC_MINOR__ < 6 || defined(__MINGW32__))
    // Volatile declaration needed to fix inverse cases
    // 88.202499451857 0 -88.202499451857 179.981022032992859592
    // 89.262080389218 0 -89.262080389218 179.992207982775375662
    // 89.333123580033 0 -89.333123580032997687 179.99295812360148422
    // which otherwise fail with g++ 4.4.4 x86 -O3 (Linux)
    // and g++ 4.4.0 (mingw) and g++ 4.6.1 (tdm mingw).
    real sbet12a;
    {
      volatile real xx1 = sbet2 * cbet1;
      volatile real xx2 = cbet2 * sbet1;
      sbet12a = xx1 + xx2;
    }
#else
    real sbet12a = sbet2 * cbet1 + cbet2 * sbet1;
#endif
    bool shortline = cbet12 >= 0 && sbet12 < real(0.5) &&
      lam12 <= Math::pi<real>() / 6;
    real
      omg12 = (!shortline ? lam12 :
               lam12 / sqrt(1 - _e2 * Math::sq((cbet1 + cbet2) / 2))),
      somg12 = sin(omg12), comg12 = cos(omg12);

    salp1 = cbet2 * somg12;
    calp1 = comg12 >= 0 ?
      sbet12 + cbet2 * sbet1 * Math::sq(somg12) / (1 + comg12) :
      sbet12a - cbet2 * sbet1 * Math::sq(somg12) / (1 - comg12);

    real
      ssig12 = hypot(salp1, calp1),
      csig12 = sbet1 * sbet2 + cbet1 * cbet2 * comg12;

    if (shortline && ssig12 < _etol2) {
      // really short lines
      salp2 = cbet1 * somg12;
      calp2 = sbet12 - cbet1 * sbet2 * Math::sq(somg12) / (1 + comg12);
      SinCosNorm(salp2, calp2);
      // Set return value
      sig12 = atan2(ssig12, csig12);
    } else if (csig12 >= 0 ||
               ssig12 >= 3 * abs(_f) * Math::pi<real>() * Math::sq(cbet1)) {
      // Nothing to do, zeroth order spherical approximation is OK
    } else {
      // Scale lam12 and bet2 to x, y coordinate system where antipodal point
      // is at origin and singular point is at y = 0, x = -1.
      real y, lamscale, betscale;
      // Volatile declaration needed to fix inverse case
      // 56.320923501171 0 -56.320923501171 179.664747671772880215
      // which otherwise fails with g++ 4.4.4 x86 -O3
      volatile real x;
      if (_f >= 0) {            // In fact f == 0 does not get here
        // x = dlong, y = dlat
        {
          real
            k2 = Math::sq(sbet1) * _ep2,
            eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2);
          lamscale = _f * cbet1 * A3f(eps) * Math::pi<real>();
        }
        betscale = lamscale * cbet1;

        x = (lam12 - Math::pi<real>()) / lamscale;
        y = sbet12a / betscale;
      } else {                  // _f < 0
        // x = dlat, y = dlong
        real
          cbet12a = cbet2 * cbet1 - sbet2 * sbet1,
          bet12a = atan2(sbet12a, cbet12a);
        real m12a, m0, dummy;
        // In the case of lon12 = 180, this repeats a calculation made in
        // Inverse.
        Lengths(_n, Math::pi<real>() + bet12a, sbet1, -cbet1, sbet2, cbet2,
                cbet1, cbet2, dummy, m12a, m0, false,
                dummy, dummy, C1a, C2a);
        x = -1 + m12a/(_f1 * cbet1 * cbet2 * m0 * Math::pi<real>());
        betscale = x < -real(0.01) ? sbet12a / x :
          -_f * Math::sq(cbet1) * Math::pi<real>();
        lamscale = betscale / cbet1;
        y = (lam12 - Math::pi<real>()) / lamscale;
      }

      if (y > -tol1_ && x > -1 - xthresh_) {
        // strip near cut
        if (_f >= 0) {
          salp1 = min(real(1), -real(x)); calp1 = - sqrt(1 - Math::sq(salp1));
        } else {
          calp1 = max(real(x > -tol1_ ? 0 : -1), real(x));
          salp1 = sqrt(1 - Math::sq(calp1));
        }
      } else {
        // Estimate alp1, by solving the astroid problem.
        //
        // Could estimate alpha1 = theta + pi/2, directly, i.e.,
        //   calp1 = y/k; salp1 = -x/(1+k);  for _f >= 0
        //   calp1 = x/(1+k); salp1 = -y/k;  for _f < 0 (need to check)
        //
        // However, it's better to estimate omg12 from astroid and use
        // spherical formula to compute alp1.  This reduces the mean number of
        // Newton iterations for astroid cases from 2.24 (min 0, max 6) to 2.12
        // (min 0 max 5).  The changes in the number of iterations are as
        // follows:
        //
        // change percent
        //    1       5
        //    0      78
        //   -1      16
        //   -2       0.6
        //   -3       0.04
        //   -4       0.002
        //
        // The histogram of iterations is (m = number of iterations estimating
        // alp1 directly, n = number of iterations estimating via omg12, total
        // number of trials = 148605):
        //
        //  iter    m      n
        //    0   148    186
        //    1 13046  13845
        //    2 93315 102225
        //    3 36189  32341
        //    4  5396      7
        //    5   455      1
        //    6    56      0
        //
        // Because omg12 is near pi, estimate work with omg12a = pi - omg12
        real k = Astroid(x, y);
        real
          omg12a = lamscale * ( _f >= 0 ? -x * k/(1 + k) : -y * (1 + k)/k );
        somg12 = sin(omg12a), comg12 = -cos(omg12a);
        // Update spherical estimate of alp1 using omg12 instead of lam12
        salp1 = cbet2 * somg12;
        calp1 = sbet12a - cbet2 * sbet1 * Math::sq(somg12) / (1 - comg12);
      }
    }
    SinCosNorm(salp1, calp1);
    return sig12;
  }

  template<typename real>
  real Geodesic30<real>::Lambda12(real sbet1, real cbet1,
                                  real sbet2, real cbet2,
                                  real salp1, real calp1,
                                  real& salp2, real& calp2,
                                  real& sig12,
                                  real& ssig1, real& csig1,
                                  real& ssig2, real& csig2,
                                  real& eps, real& domg12,
                                  bool diffp, real& dlam12,
                                  // Scratch areas of the right size
                                  real C1a[], real C2a[], real C3a[]) const
    {

    if (sbet1 == 0 && calp1 == 0)
      // Break degeneracy of equatorial line.  This case has already been
      // handled.
      calp1 = -tiny_;

    real
      // sin(alp1) * cos(bet1) = sin(alp0)
      salp0 = salp1 * cbet1,
      calp0 = hypot(calp1, salp1 * sbet1); // calp0 > 0

    real somg1, comg1, somg2, comg2, omg12, lam12;
    // tan(bet1) = tan(sig1) * cos(alp1)
    // tan(omg1) = sin(alp0) * tan(sig1) = tan(omg1)=tan(alp1)*sin(bet1)
    ssig1 = sbet1; somg1 = salp0 * sbet1;
    csig1 = comg1 = calp1 * cbet1;
    SinCosNorm(ssig1, csig1);
    // SinCosNorm(somg1, comg1); -- don't need to normalize!

    // Enforce symmetries in the case abs(bet2) = -bet1.  Need to be careful
    // about this case, since this can yield singularities in the Newton
    // iteration.
    // sin(alp2) * cos(bet2) = sin(alp0)
    salp2 = cbet2 != cbet1 ? salp0 / cbet2 : salp1;
    // calp2 = sqrt(1 - sq(salp2))
    //       = sqrt(sq(calp0) - sq(sbet2)) / cbet2
    // and subst for calp0 and rearrange to give (choose positive sqrt
    // to give alp2 in [0, pi/2]).
    calp2 = cbet2 != cbet1 || abs(sbet2) != -sbet1 ?
      sqrt(Math::sq(calp1 * cbet1) +
           (cbet1 < -sbet1 ?
            (cbet2 - cbet1) * (cbet1 + cbet2) :
            (sbet1 - sbet2) * (sbet1 + sbet2))) / cbet2 :
      abs(calp1);
    // tan(bet2) = tan(sig2) * cos(alp2)
    // tan(omg2) = sin(alp0) * tan(sig2).
    ssig2 = sbet2; somg2 = salp0 * sbet2;
    csig2 = comg2 = calp2 * cbet2;
    SinCosNorm(ssig2, csig2);
    // SinCosNorm(somg2, comg2); -- don't need to normalize!

    // sig12 = sig2 - sig1, limit to [0, pi]
    sig12 = atan2(max(csig1 * ssig2 - ssig1 * csig2, real(0)),
                  csig1 * csig2 + ssig1 * ssig2);

    // omg12 = omg2 - omg1, limit to [0, pi]
    omg12 = atan2(max(comg1 * somg2 - somg1 * comg2, real(0)),
                  comg1 * comg2 + somg1 * somg2);
    real B312, h0;
    real k2 = Math::sq(calp0) * _ep2;
    eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2);
    C3f(eps, C3a);
    B312 = (SinCosSeries(true, ssig2, csig2, C3a, nC3_-1) -
            SinCosSeries(true, ssig1, csig1, C3a, nC3_-1));
    h0 = -_f * A3f(eps);
    domg12 = salp0 * h0 * (sig12 + B312);
    lam12 = omg12 + domg12;

    if (diffp) {
      if (calp2 == 0)
        dlam12 = - 2 * sqrt(1 - _e2 * Math::sq(cbet1)) / sbet1;
      else {
        real dummy;
        Lengths(eps, sig12, ssig1, csig1, ssig2, csig2,
                cbet1, cbet2, dummy, dlam12, dummy,
                false, dummy, dummy, C1a, C2a);
        dlam12 /= calp2 * cbet2;
      }
    }
    return lam12;
  }

  template<typename real>
  real Geodesic30<real>::A3f(real eps) const {
    // Evaluation sum(_A3c[k] * eps^k, k, 0, nA3x_-1) by Horner's method
    real v = 0;
    for (int i = nA3x_; i; )
      v = eps * v + _A3x[--i];
    return v;
  }

  template<typename real>
  void Geodesic30<real>::C3f(real eps, real c[]) const {
    // Evaluation C3 coeffs by Horner's method
    // Elements c[1] thru c[nC3_ - 1] are set
    for (int j = nC3x_, k = nC3_ - 1; k; ) {
      real t = 0;
      for (int i = nC3_ - k; i; --i)
        t = eps * t + _C3x[--j];
      c[k--] = t;
    }

    real mult = 1;
    for (int k = 1; k < nC3_; ) {
      mult *= eps;
      c[k++] *= mult;
    }
  }

  template<typename real>
  void Geodesic30<real>::C4f(real k2, real c[]) const {
    // Evaluation C4 coeffs by Horner's method
    // Elements c[0] thru c[nC4_ - 1] are set
    for (int j = nC4x_, k = nC4_; k; ) {
      real t = 0;
      for (int i = nC4_ - k + 1; i; --i)
        t = k2 * t + _C4x[--j];
      c[--k] = t;
    }

    real mult = 1;
    for (int k = 1; k < nC4_; ) {
      mult *= k2;
      c[k++] *= mult;
    }
  }

  // Generated by Maxima on 2012-09-23 10:50:07-04:00

  // The scale factor A1-1 = mean value of (d/dsigma)I1 - 1
  template<typename real>
  real Geodesic30<real>::A1m1f(real eps) {
    real
      eps2 = Math::sq(eps),
      t;
    switch (nA1_/2) {
    case 15:
      t = eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(111759833025.L)*eps2+real(137975102500.L))+
        real(173075968576.L))+real(221170802944.L))+real(288876150784.L))+
        real(387302096896.L))+real(536058265600.L))+real(771923902464.L))+
        real(1169304846336.L))+real(1894080577536.L))+real(3367254360064.L))+
        real(6871947673600.L))+real(17592186044416.L))+real(70368744177664.L))+
        real(1125899906842624.L))/real(4503599627370496.L);
      break;
    default:
      static_assert(nA1_ == 30, "Bad value of nA1_");
      t = 0;
    }
    return (t + eps) / (1 - eps);
  }

  // The coefficients C1[l] in the Fourier expansion of B1
  template<typename real>
  void Geodesic30<real>::C1f(real eps, real c[]) {
    real
      eps2 = Math::sq(eps),
      d = eps;
    switch (nC1_) {
    case 30:
      c[1] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(951224759.L)*eps2+real(1177072206.L))+
        real(1480003424.L))+real(1897996096.L))+real(2480922624.L))+
        real(3368206336.L))+real(4540596224.L))+real(7249330176.L))+
        real(7784628224.L))+real(28454158336.L))-real(25769803776.L))+
        real(326417514496.L))-real(1099511627776.L))+real(6597069766656.L))-
        real(17592186044416.L))/real(35184372088832.L);
      d *= eps;
      c[2] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(1417935787335.L)*eps2+real(1743500873728.L))+
        real(2176676356096.L))+real(2767257206784.L))+real(3587210215424.L))+
        real(4791606444032.L))+real(6478891057152.L))+real(9658307706880.L))+
        real(12315818721280.L))+real(29686813949952.L))+real(2199023255552.L))+
        real(246290604621824.L))-real(633318697598976.L))+
        real(4503599627370496.L))-real(9007199254740992.L))/
        real(144115188075855872.L);
      d *= eps;
      c[3] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(real(382058512065.L)*eps2+real(473930894336.L))+
        real(598024544256.L))+real(768956203008.L))+real(1015512039424.L))+
        real(1363710836736.L))+real(1973201928192.L))+real(2608924196864.L))+
        real(5430986145792.L))+real(2473901162496.L))+real(37383395344384.L))-
        real(79164837199872.L))+real(633318697598976.L))-
        real(1125899906842624.L))/real(54043195528445952.L);
      d *= eps;
      c[4] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(real(172435304205.L)*eps2+real(212683932395.L))+
        real(266623631360.L))+real(340357865472.L))+real(445355655168.L))+
        real(593424023552.L))+real(842350460928.L))+real(1124945887232.L))+
        real(2166811000832.L))+real(1398011854848.L))+real(13194139533312.L))-
        real(24189255811072.L))+real(211106232532992.L))-
        real(351843720888320.L))/real(36028797018963968.L);
      d *= eps;
      c[5] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(real(386510475025.L)*eps2+real(481751504400.L))+
        real(611052838912.L))+real(793330483200.L))+real(1049355878400.L))+
        real(1468530688000.L))+real(1964611993600.L))+real(3602135384064.L))+
        real(2716566814720.L))+real(20100446945280.L))-real(32985348833280.L))+
        real(307863255777280.L))-real(492581209243648.L))/
        real(90071992547409920.L);
      d *= eps;
      c[6] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(real(1415580640915.L)*eps2+real(1755398381760.L))+
        real(2213942336640.L))+real(2855022493696.L))+real(3751349059584.L))+
        real(5191055179776.L))+real(6934929342464.L))+real(12282532724736.L))+
        real(10108205531136.L))+real(64183991271424.L))-real(96482145337344.L))+
        real(949978046398464.L))-real(1477743627730944.L))/
        real(432345564227567616.L);
      d *= eps;
      c[7] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(25143094970.L)*eps2+real(31551077565.L))+real(40447578920.L))+
        real(52829751296.L))+real(72429207552.L))+real(96504905728.L))+
        real(166545326080.L))+real(144804151296.L))+real(826781204480.L))-
        real(1157493686272.L))+real(11905649344512.L))-real(18141941858304.L))/
        real(7881299347898368.L);
      d *= eps;
      c[8] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(23168297355.L)*eps2+real(28941724625.L))+real(36909403040.L))+
        real(47951507760.L))+real(65227194368.L))+real(86639116288.L))+
        real(146515427328.L))+real(132225433600.L))+real(697932185600.L))-
        real(921270484992.L))+real(9826885173248.L))-real(14740327759872.L))/
        real(9007199254740992.L);
      d *= eps;
      c[9] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(26678953530.L)*eps2+real(33866001805.L))+real(43787160360.L))+
        real(59161708800.L))+real(78330593280.L))+real(130310995968.L))+
        real(120777080832.L))+real(600003575808.L))-real(753766760448.L))+
        real(8291434364928.L))-real(12283606466560.L))/
        real(10133099161583616.L);
      d *= eps;
      c[10] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(1580768752695.L)*eps2+real(1998272995520.L))+
        real(2572543614080.L))+real(3455411896320.L))+real(4560620257280.L))+
        real(7484836151296.L))+real(7076126392320.L))+real(33495376199680.L))-
        real(40340480327680.L))+real(455610130759680.L))-
        real(668228191780864.L))/real(720575940379279360.L);
      d *= eps;
      c[11] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(462544929605.L)*eps2+real(593153502480.L))+real(792594344960.L))+
        real(1042964172800.L))+real(1692233072640.L))+real(1624357470208.L))+
        real(7391571607552.L))-real(8583760576512.L))+real(99190122217472.L))-
        real(144276541407232.L))/real(198158383604301824.L);
      d *= eps;
      c[12] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(214950398465.L)*eps2+real(274671375543.L))+real(365337409536.L))+
        real(479381617664.L))+real(770227863552.L))+real(748240404480.L))+
        real(3294776786944.L))-real(3706623885312.L))+real(43699144753152.L))-
        real(63120986865664.L))/real(108086391056891904.L);
      d *= eps;
      c[13] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(real(510681541877.L)*
        eps2+real(676451782656.L))+real(885260446720.L))+real(1410333609984.L))+
        real(1383317864448.L))+real(5924196974592.L))-real(6481172758528.L))+
        real(77774073102336.L))-real(111675592146944.L))/
        real(234187180623265792.L);
      d *= eps;
      c[14] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(1905429911595.L)*eps2+real(2514563194880.L))+
        real(3282620293120.L))+real(5190881525760.L))+real(5131649187840.L))+
        real(21458803752960.L))-real(22902221045760.L))+
        real(279188980367360.L))-real(398841400524800.L))/
        real(1008806316530991104.L);
      d *= eps;
      c[15] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(real(610648965.L)*eps2+
        real(795328960.L))+real(1249428816.L))+real(1243258560.L))+
        real(5092413440.L))-real(5316157440.L))+real(65727037440.L))-
        real(93478453248.L))/real(281474976710656.L);
      d *= eps;
      c[16] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(real(4284219735.L)*eps2+
        real(5567912565.L))+real(8695865760.L))+real(8700508080.L))+
        real(34997329920.L))-real(35816939520.L))+real(448490373120.L))-
        real(635361361920.L))/real(2251799813685248.L);
      d *= eps;
      c[17] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(real(10425833355.L)*eps2+
        real(16197534720.L))+real(16281896880.L))+real(64452690240.L))-
        real(64790138880.L))+real(820675092480.L))-real(1158600130560.L))/
        real(4785074604081152.L);
      d *= eps;
      c[18] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(real(835245163005.L)*eps2+
        real(1291494481920.L))+real(1303425146880.L))+real(5086661099520.L))-
        real(5030763724800.L))+real(64393775677440.L))-real(90628276879360.L))/
        real(432345564227567616.L);
      d *= eps;
      c[19] = d*(eps2*(eps2*(eps2*(eps2*(real(907759240185.L)*eps2+
        real(919324016640.L))+real(3542178524160.L))-real(3451662888960.L))+
        real(44606105026560.L))-real(62605059686400.L))/
        real(342273571680157696.L);
      d *= eps;
      c[20] = d*(eps2*(eps2*(eps2*(eps2*(real(426519840213.L)*eps2+
        real(433261385475.L))+real(1650270566400.L))-real(1586405606400.L))+
        real(20682028646400.L))-real(28954840104960.L))/
        real(180143985094819840.L);
      d *= eps;
      c[21] = d*(eps2*(eps2*(eps2*(real(272832605955.L)*eps2+
        real(1028430756080.L))-real(976367508480.L))+real(12832258682880.L))-
        real(17924424826880.L))/real(126100789566373888.L);
      d *= eps;
      c[22] = d*(eps2*(eps2*(eps2*(real(3098718179667.L)*eps2+
        real(11570204048832.L))-real(10858767553920.L))+
        real(143785060024320.L))-real(200427659427840.L))/
        real(1585267068834414592.L);
      d *= eps;
      c[23] = d*(eps2*(eps2*(real(169946454762.L)*eps2-real(157807422279.L))+
        real(2104098963720.L))-real(2927442036480.L))/
        real(25895697857380352.L);
      d *= eps;
      c[24] = d*(eps2*(eps2*(real(160153129039.L)*eps2-real(147251741859.L))+
        real(1976023374624.L))-real(2744476909200.L))/
        real(27021597764222976.L);
      d *= eps;
      c[25] = d*((real(1860438674025.L)-real(137810272150.L)*eps2)*eps2-
        real(2579808294648.L))/real(28147497671065600.L);
      d *= eps;
      c[26] = d*((real(112364983500224.L)-real(8276884945329.L)*eps2)*eps2-
        real(155582284846464.L))/real(1873497444986126336.L);
      d *= eps;
      c[27] = d*(real(26567064265077.L)*eps2-real(36734706144304.L))/
        real(486388759756013568.L);
      d *= eps;
      c[28] = d*(real(1798281489207.L)*eps2-real(2483341104143.L))/
        real(36028797018963968.L);
      d *= eps;
      c[29] = -real(32968493968795.L)*d/real(522417556774977536.L);
      d *= eps;
      c[30] = -real(125280277081421.L)*d/real(2161727821137838080.L);
      break;
    default:
      static_assert(nC1_ == 30, "Bad value of nC1_");
    }
  }

  // The coefficients C1p[l] in the Fourier expansion of B1p
  template<typename real>
  void Geodesic30<real>::C1pf(real eps, real c[]) {
    real
      eps2 = Math::sq(eps),
      d = eps;
    switch (nC1p_) {
    case 30:
      c[1] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*((-real(68324470839108426239947872773.L)*eps2-
        real(100963451984547929658324705360.L))*eps2-
        real(77643769452916184464241568000.L))-
        real(248676038325957661489987338240.L))+
        real(113684352518262362713717800960.L))-
        real(1127410089522103286510439628800.L))+
        real(2081152960718607813376475136000.L))-
        real(7186894554473028456944762880000.L))+
        real(16867131550861194884152295424000.L))-
        real(46507878164060538112453902336000.L))+
        real(108050868704809065948370698240000.L))-
        real(259212640734969911992280678400000.L))+
        real(522781796440275452757540864000000.L))-
        real(1101667005181458515079305625600000.L))+
        real(1958519120322592915696543334400000.L))/
        real(3917038240645185831393086668800000.L);
      d *= eps;
      c[2] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(814580642623039771441079625525.L)*eps2-
        real(2758054021727848902441714037504.L))+
        real(5841099452088566757436314278400.L))-
        real(15015410677768696577330424053760.L))+
        real(33449812442363563514538452582400.L))-
        real(77082790785078302886337708032000.L))+
        real(165299057361096899617624581734400.L))-
        real(348610606399138130253462896640000.L))+
        real(690314061309673190606896103424000.L))-
        real(1306232492667478773033452175360000.L))+
        real(2279573575762867049898757324800000.L))-
        real(3662490258622355775574801121280000.L))+
        real(5106685596934885825107197952000000.L))-
        real(6038767287661328156731008614400000.L))+
        real(4896297800806482289241358336000000.L))/
        real(15668152962580743325572346675200000.L);
      d *= eps;
      c[3] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*((real(66983330039935626181978880.L)-
        real(32351107913549549438520051.L)*eps2)*eps2-
        real(135828898775373995016134400.L))+
        real(265038269013112292979240960.L))-
        real(499894271293606938484736000.L))+
        real(901099857397550682577305600.L))-
        real(1548182789749135028256768000.L))+
        real(2504243808857805742407680000.L))-
        real(3777470861965157134761984000.L))+
        real(5207879022294655932825600000.L))-
        real(6410493668683863419781120000.L))+
        real(6711738155484105513369600000.L))-
        real(5552627222737626071040000000.L))+
        real(2862687812611398329958400000.L))/
        real(9476483793472215161241600000.L);
      d *= eps;
      c[4] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*((real(762137556504201019469508411916621.L)-
        real(417028145574541060069227783167265.L)*eps2)*eps2-
        real(1347037556014519000812637339643904.L))+
        real(2290865869627361243034148720097280.L))-
        real(3732887799512153374731844991385600.L))+
        real(5788766810661907094463171354624000.L))-
        real(8480027281571610664838334146150400.L))+
        real(11609853223355956484304056981913600.L))-
        real(14655130647665604565452988612608000.L))+
        real(16714903616138937969339248148480000.L))-
        real(16729663337445375776495606169600000.L))+
        real(13971638751037696067208814264320000.L))-
        real(8884813245954512750481852334080000.L))+
        real(3338153031904002769071321907200000.L))/
        real(9512807155852594161954639052800000.L);
      d *= eps;
      c[5] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(real(197007042130783194846374402645.L)*eps2-
        real(310335025974540114235834633320.L))+
        real(468663841101668757430246139328.L))-
        real(674902579857564785639108613120.L))+
        real(920446562447687105035214929920.L))-
        real(1178763926539972491250402590720.L))+
        real(1401601783007433886301442539520.L))-
        real(1524206500940894902370835628032.L))+
        real(1483711568246472584132831477760.L))-
        real(1252110213364963963086789672960.L))+
        real(868590197349453641063501660160.L))-
        real(448183687833246135538270863360.L))+
        real(132135025678449294709522169856.L))/
        real(292701758641618281906296586240.L);
      d *= eps;
      c[6] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(real(454884237151609068934000398965.L)*eps2-
        real(647709749710146577129911572736.L))+
        real(881252423333079185890177150464.L))-
        real(1139031149446319255026067537920.L))+
        real(1388495367928973101254244761600.L))-
        real(1581814879398842374510608384000.L))+
        real(1664133654311826048422405734400.L))-
        real(1591031096001163615754885529600.L))+
        real(1351459277299922197612068864000.L))-
        real(986025568226589090387066880000.L))+
        real(584568285559019617753497600000.L))-
        real(253599041355938741008465920000.L))+
        real(61446891514089165008404480000.L))/
        real(99138599685555481686835200000.L);
      d *= eps;
      c[7] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        ((real(1933592507571132504843050837332800.L)-
        real(1560093808636899799908748023402307.L)*eps2)*eps2-
        real(2269463677375700479180225987699200.L))+
        real(2503402603028242843974312568135680.L))-
        real(2570523109811007986598049456128000.L))+
        real(2426509695624250009060927025971200.L))-
        real(2070822079813869223206274990080000.L))+
        real(1560603932132966084124113633280000.L))-
        real(1002786629686644065633770143744000.L))+
        real(518849381842413445280327270400000.L))-
        real(194075937964220398187462000640000.L))+
        real(39886998826465358681997312000000.L))/
        real(44801289588002798250963763200000.L);
      d *= eps;
      c[8] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        ((real(984696987290818372019177477029535177.L)-
        real(864547928795698718505279406150164525.L)*eps2)*eps2-
        real(1059212781301837076844867522945439200.L))+
        real(1067600725597368985187252235977777520.L))-
        real(998294893169085316134100120397414400.L))+
        real(854931331289334181570726114492416000.L))-
        real(659049519797922969370295979422515200.L))+
        real(446379390529318486159003751546880000.L))-
        real(256244053370973109509731150462976000.L))+
        real(117585126621112641831768183275520000.L))-
        real(38643751342937610938728474214400000.L))+
        real(6892781229492559807214612643840000.L))/
        real(5213750075803825646455907942400000.L);
      d *= eps;
      c[9] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(24418246169693520510978698857611.L)*eps2-
        real(24253326300244709781611959454520.L))+
        real(22513514221874042377178411088000.L))-
        real(19332277974343973213971609927680.L))+
        real(15154270228073449992955864350720.L))-
        real(10653698646765737098261718630400.L))+
        real(6552730956974773631041142784000.L))-
        real(3398961382621904455933624320000.L))+
        real(1400957911881999850936467456000.L))-
        real(410480624651529181603037184000.L))+
        real(64654877575574359396843520000.L))/
        real(32107387398162854864486400000.L);
      d *= eps;
      c[10] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(138839599113992807793637100518615941.L)*eps2-
        real(128131167792553101962904293046742400.L))+
        real(110260958710051776687312636890092800.L))-
        real(87551101512248381287089934742323200.L))+
        real(63283736067508396218919938726297600.L))-
        real(40893927907591124875876883805241344.L))+
        real(23036160394195362211714095843901440.L))-
        real(10897812315679318610603987215319040.L))+
        real(4076274135466310279303908703600640.L))-
        real(1077299162488871713814008289034240.L))+
        real(151894970117062258998539842486272.L))/
        real(48535273432937431472462270300160.L);
      d *= eps;
      c[11] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        ((real(1665625834987958256879116059014151260.L)-
        real(2076374429540700401175830042384968739.L)*eps2)*eps2-
        real(1230152129036161473668382101876400000.L))+
        real(825014868343001262012438817283427840.L))-
        real(493291190504999836140641758153359360.L))+
        real(256314215252126631526441791587942400.L))-
        real(111444991601324978897209783222272000.L))+
        real(38152067424250128481246469160960000.L))-
        real(9181902006121023847950396162048000.L))+
        real(1171620224064195871738033078272000.L))/
        real(236988639809264802111632179200000.L);
      d *= eps;
      c[12] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        ((real(1566579885197209278191513754936291.L)-
        real(2085007539582231956105868055871375.L)*eps2)*eps2-
        real(1081564837193007139085006500454400.L))+
        real(676597519479721211941474550149120.L))-
        real(376439934683190310573826536243200.L))+
        real(181512206576201286255575089152000.L))-
        real(73010665125800598925415442022400.L))+
        real(23039596989730815168730890240000.L))-
        real(5089466730455646806502014976000.L))+
        real(593026415839762145723023360000.L))/
        real(74917237262379994683801600000.L);
      d *= eps;
      c[13] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(532307461025379280326677331995497934939.L)*eps2-
        real(345039671067654221891606656570321837440.L))+
        real(202261809397838430653182447934280460800.L))-
        real(105224293603866564967604498396119941120.L))+
        real(47328494509066815223141356725280768000.L))-
        real(17710172422980886112449002486733209600.L))+
        real(5182990652328470877142441468624896000.L))-
        real(1057956832086234653846310664273920000.L))+
        real(113416761102230302535612708683776000.L))/
        real(8847575886212552612167601356800000.L);
      d *= eps;
      c[14] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(381673703796454946376866854636258900545.L)*eps2-
        real(233162687162399658075446212907744119808.L))+
        real(128591597036868709065146782487859259392.L))-
        real(62819495885335605812729108180172472320.L))+
        real(26476396185336335227480186309636915200.L))-
        real(9261485971310904757896459287789568000.L))+
        real(2526857900860600021630197366049996800.L))-
        real(479343484709923171245446105609011200.L))+
        real(47580438597320556305728703299584000.L))/
        real(2269932005792141778048830668800000.L);
      d *= eps;
      c[15] = d*(eps2*(eps2*(eps2*(eps2*(eps2*
        ((real(45423579832955913951377090935860.L)-
        real(87233394546134174610692634458385.L)*eps2)*eps2-
        real(20915336097159337061395196109696.L))+
        real(8292934767856858229681031605760.L))-
        real(2723266546205253938500821647360.L))+
        real(695844551801791388795264040960.L))-
        real(123285037044637818756330946560.L))+
        real(11393008072733014574426488832.L))/
        real(329635843954471976608727040.L);
      d *= eps;
      c[16] = d*(eps2*(eps2*(eps2*(eps2*(eps2*
        ((real(11533333831474595735804817621789252563149.L)-
        real(23385431753887353457020778401687008559585.L)*eps2)*eps2-
        real(5022023966106875551755963911088924814176.L))+
        real(1879853975607475524788413072716518817840.L))-
        real(581684618230578562286841222043631001600.L))+
        real(139755997142047743996425143710587904000.L))-
        real(23226444168040344046247956338632294400.L))+
        real(2007787334760574615337152884439449600.L))/
        real(34975573425183997044975049113600000.L);
      d *= eps;
      c[17] = d*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(124349170388985303664894222783437945963763.L)*eps2-
        real(51356157953416379240484749550601957961760.L))+
        real(18205265484674873030559021199395296150400.L))-
        real(5325775544578042828825884040410764328960.L))+
        real(1207441436474781830211735618414157824000.L))-
        real(188951805799862347867752206201467699200.L))+
        real(15342567243414777653748406712008704000.L))/
        real(159888335657983986491314510233600000.L);
      d *= eps;
      c[18] = d*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(29415792271000759541355178757686976175.L)*eps2-
        real(11553436356896028861696238148248950816.L))+
        real(3889484699196173251397739824940350400.L))-
        real(1078919550719440749254739026503188480.L))+
        real(231550222629276503187643387736064000.L))-
        real(34235402664630988684635673853952000.L))+
        real(2620768370186874435326977402470400.L))/
        real(16246338023470404561430118400000.L);
      d *= eps;
      c[19] = d*(eps2*(eps2*(eps2*
        ((real(39971741300697512082489191783652938894460.L)-
        real(124709016547035761740817690707989868083971.L)*eps2)*eps2-
        real(10541946622005479349026662231862999616000.L))+
        real(2147737484972663027856567855897083504640.L))-
        real(300935078555177912914872771533919682560.L))+
        real(21789656964420596148926545790179737600.L))/
        real(79944167828991993245657255116800000.L);
      d *= eps;
      c[20] = d*(eps2*(eps2*(eps2*
        ((real(132759100007370735641836237776193892645.L)-
        real(434049634134064433931958935068903736057.L)*eps2)*eps2-
        real(33369654052209003816833461479580677120.L))+
        real(6470339796376061376469829143873374720.L))-
        real(861516011024441885398386150250168320.L))+
        real(59174556981142617473106342842155008.L))/
        real(127910668526387189193051608186880.L);
      d *= eps;
      c[21] = d*(eps2*(eps2*(eps2*
        (real(1184039565075904265080239551544199179.L)*eps2-
        real(284267116555971547780305491131427200.L))+
        real(52580419360935590930783108765760000.L))-
        real(6669211230344444283882571187404800.L))+
        real(435695986025198666213623911219200.L))/
        real(552596531410557978279936000000.L);
      d *= eps;
      c[22] = d*(eps2*(eps2*(eps2*
        (real(33266706117403657665760273846889869243353975.L)*eps2-
        real(7643873319369712225472473426041843527605504.L))+
        real(1351605986324662737962699088554522988480000.L))-
        real(163676402135723425310884271475634725273600.L))+
        real(10194536630734666447855952558032293888000.L))/
        real(7558357685650152088680322301952000000.L);
      d *= eps;
      c[23] = d*(eps2*((real(872462305125855376882843663898045102533140.L)-
        real(5151480396283925610995816981891860586194601.L)*eps2)*eps2-
        real(101077452067939359628850379641405695344000.L))+
        real(6015188191188926310879078363735341913600.L))/
        real(2598185454442239780483860791296000000.L);
      d *= eps;
      c[24] = d*(eps2*((real(27724661943728900581318217098970206053.L)-
        real(170609658731643250810604763580488287545.L)*eps2)*eps2-
        real(3078627217489758413403668512964568672.L))+
        real(175398209678212825304841268332564400.L))/
        real(44000498813565679020539904000000.L);
      d *= eps;
      c[25] = d*(eps2*(real(176823091830610438772384180302418617495025.L)*eps2-
        real(18852072824833509228802243728817139474400.L))+
        real(1030121515251918847566651624253785226368.L))/
        real(149655482175873011355870381578649600.L);
      d *= eps;
      c[26] = d*(eps2*(real(446067840719735024384293885817337822131402595.L)*
        eps2-real(45733923317696663408149648125746073883353248.L))+
        real(2400791212899461266985404924139079085724352.L))/
        real(201459302929059822979056282894336000000.L);
      d *= eps;
      c[27] = d*(real(114663057448096171172679698534568324260.L)-
        real(2270133399797625494039372220001669690149.L)*eps2)/
        real(5544062850509275556588027904000000.L);
      d *= eps;
      c[28] = d*(real(3982985710570553680540246736094623237083171.L)-
        real(81838994378218625952396490652351249877557775.L)*eps2)/
        real(110714514875008095135720435351552000000.L);
      d *= eps;
      c[29] = real(2381352350093111938327626556685002210278872879.L)*d/
        real(37975078602127776631552109325582336000000.L);
      d *= eps;
      c[30] = real(20034557328168749612075941075238883149.L)*d/
        real(182929433787470496587153418485760.L);
      break;
    default:
      static_assert(nC1p_ == 30, "Bad value of nC1p_");
    }
  }

  // The scale factor A2-1 = mean value of (d/dsigma)I2 - 1
  template<typename real>
  real Geodesic30<real>::A2m1f(real eps) {
    real
      eps2 = Math::sq(eps),
      t;
    switch (nA2_/2) {
    case 15:
      t = eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(93990019574025.L)*eps2+real(100583849722500.L))+
        real(108172480360000.L))+real(116999354757376.L))+
        real(127394382495744.L))+real(139816056979456.L))+
        real(154920838758400.L))+real(173682878054400.L))+
        real(197612519030784.L))+real(229183749881856.L))+
        real(272747603165184.L))+real(336725436006400.L))+
        real(439804651110400.L))+real(633318697598976.L))+
        real(1125899906842624.L))/real(4503599627370496.L);
      break;
    default:
      static_assert(nA2_ == 30, "Bad value of nA2_");
      t = 0;
    }
    return t * (1 - eps) - eps;
  }

  // The coefficients C2[l] in the Fourier expansion of B2
  template<typename real>
  void Geodesic30<real>::C2f(real eps, real c[]) {
    real
      eps2 = Math::sq(eps),
      d = eps;
    switch (nC2_) {
    case 30:
      c[1] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(112374407161.L)*eps2+real(123069496442.L))+
        real(135749473696.L))+real(150992398784.L))+real(169617399808.L))+
        real(192822001664.L))+real(222423678976.L))+real(261309857792.L))+
        real(314337918976.L))+real(390305153024.L))+real(506806140928.L))+
        real(704374636544.L))+real(1099511627776.L))+real(2199023255552.L))+
        real(17592186044416.L))/real(35184372088832.L);
      d *= eps;
      c[2] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(eps2*(real(294392827406755.L)*eps2+
        real(321459409719296.L))+real(353374167228416.L))+
        real(391498236559360.L))+real(437739245010944.L))+
        real(494850596143104.L))+real(566948769300480.L))+
        real(660457522200576.L))+real(785951097880576.L))+
        real(962072674304000.L))+real(1224855953342464.L))+
        real(1653665488175104.L))+real(2462906046218240.L))+
        real(4503599627370496.L))+real(27021597764222976.L))/
        real(144115188075855872.L);
      d *= eps;
      c[3] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(real(88649029545351.L)*eps2+real(97159766082560.L))+
        real(107272242438144.L))+real(119462631407616.L))+
        real(134411503009792.L))+real(153124954177536.L))+
        real(177152704118784.L))+real(209007335702528.L))+
        real(253040145727488.L))+real(317483982520320.L))+
        real(420013441810432.L))+real(606930418532352.L))+
        real(1055531162664960.L))+real(5629499534213120.L))/
        real(54043195528445952.L);
      d *= eps;
      c[4] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(eps2*(real(46591933629593.L)*eps2+real(50930607972739.L))+
        real(56062453006336.L))+real(62216889098240.L))+real(69719339302912.L))+
        real(79046821281792.L))+real(90927142010880.L))+
        real(106527738298368.L))+real(127846144016384.L))+
        real(158610994757632.L))+real(206708186021888.L))+
        real(292470092988416.L))+real(492581209243648.L))+
        real(2462906046218240.L))/real(36028797018963968.L);
      d *= eps;
      c[5] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(real(104424162091095.L)*eps2+real(114641179231920.L))+
        real(126840419622912.L))+real(141637914951680.L))+
        real(159930894254080.L))+real(183078748160000.L))+
        real(213245461790720.L))+real(254103418568704.L))+
        real(312448133365760.L))+real(402524334981120.L))+
        real(560750930165760.L))+real(923589767331840.L))+
        real(4433230883192832.L))/real(90071992547409920.L);
      d *= eps;
      c[6] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (eps2*(real(422949801695839.L)*eps2+real(463237261716672.L))+
        real(511160667067008.L))+real(569047033446400.L))+
        real(640269085310976.L))+real(729910414934016.L))+
        real(846019470295040.L))+real(1002178005172224.L))+
        real(1223376337108992.L))+real(1561718828302336.L))+
        real(2149820110209024.L))+real(3483252836794368.L))+
        real(16255179905040384.L))/real(432345564227567616.L);
      d *= eps;
      c[7] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(7259112534786.L)*eps2+real(7991061609615.L))+
        real(8872011950424.L))+real(9951592749056.L))+real(11304320499712.L))+
        real(13047708778496.L))+real(15379290128384.L))+real(18661179916288.L))+
        real(23645942448128.L))+real(32244466974720.L))+real(51591147159552.L))+
        real(235845244157952.L))/real(7881299347898368.L);
      d *= eps;
      c[8] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(7246735300607.L)*eps2+real(7960491282361.L))+
        real(8816901289504.L))+real(9862852474928.L))+real(11168541835264.L))+
        real(12844357124096.L))+real(15075351986176.L))+real(18199944888320.L))+
        real(22920092975104.L))+real(31016106328064.L))+real(49134425866240.L))+
        real(221104916398080.L))/real(9007199254740992.L);
      d *= eps;
      c[9] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(7907584547394.L)*eps2+real(8739550412095.L))+
        real(9752698880856.L))+real(11013436336896.L))+real(12625975050240.L))+
        real(14764689063936.L))+real(17747964592128.L))+real(22235465121792.L))+
        real(29899414831104.L))+real(46984794734592.L))+
        real(208821309931520.L))/real(10133099161583616.L);
      d *= eps;
      c[10] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(501774575662355.L)*eps2+real(553499208322240.L))+
        real(616330662236800.L))+real(694305488322560.L))+
        real(793748863221760.L))+real(925230763606016.L))+
        real(1108031265832960.L))+real(1382072157470720.L))+
        real(1848543186780160.L))+real(2885530828144640.L))+
        real(12696335643836416.L))/real(720575940379279360.L);
      d *= eps;
      c[11] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(136783962738819.L)*eps2+real(152015525119920.L))+
        real(170874299667456.L))+real(194866052919296.L))+
        real(226504843755520.L))+real(270373555798016.L))+
        real(335958851452928.L))+real(447309301153792.L))+
        real(694330855522304.L))+real(3029807369551872.L))/
        real(198158383604301824.L);
      d *= eps;
      c[12] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(67559103579581.L)*eps2+real(74950388895663.L))+
        real(84083481925632.L))+real(95677911809024.L))+
        real(110934308585472.L))+real(132040506114048.L))+
        real(163524974215168.L))+real(216870011535360.L))+
        real(335026776440832.L))+real(1451782697910272.L))/
        real(108086391056891904.L);
      d *= eps;
      c[13] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(147776979832755.L)*eps2+real(165490748866560.L))+
        real(187937520276480.L))+real(217418694471680.L))+
        real(258127187968000.L))+real(318742050897920.L))+
        real(421276229304320.L))+real(648117275852800.L))+
        real(2791889803673600.L))/real(234187180623265792.L);
      d *= eps;
      c[14] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*
        (real(582677710836375.L)*eps2+real(651472815022080.L))+
        real(738512726138880.L))+real(852646813777920.L))+
        real(1009996668764160.L))+real(1243937022935040.L))+
        real(1639144677703680.L))+real(2512700823306240.L))+
        real(10768717814169600.L))/real(1008806316530991104.L);
      d *= eps;
      c[15] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(real(166992644595.L)*eps2+
        real(188995997440.L))+real(217809042096.L))+real(257478434880.L))+
        real(316382965760.L))+real(415787950080.L))+real(635361361920.L))+
        real(2710875144192.L))/real(281474976710656.L);
      d *= eps;
      c[16] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(eps2*(real(1233068062875.L)*eps2+
        real(1393471517565.L))+real(1603268113440.L))+real(1891781738160.L))+
        real(2319741050880.L))+real(3041325342720.L))+real(4634400522240.L))+
        real(19696202219520.L))/real(2251799813685248.L);
      d *= eps;
      c[17] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(real(2740816770045.L)*eps2+
        real(3148733259840.L))+real(3709151088720.L))+real(4539696553920.L))+
        real(5939096064000.L))+real(9027426017280.L))+real(38233804308480.L))/
        real(4785074604081152.L);
      d *= eps;
      c[18] = d*(eps2*(eps2*(eps2*(eps2*(eps2*(real(230103086433585.L)*eps2+
        real(263986178088960.L))+real(310497212620800.L))+
        real(379375482224640.L))+real(495362534768640.L))+
        real(751260716236800.L))+real(3171989690777600.L))/
        real(432345564227567616.L);
      d *= eps;
      c[19] = d*(eps2*(eps2*(eps2*(eps2*(real(194621096681295.L)*eps2+
        real(228591595868160.L))+real(278866603299840.L))+
        real(363486653460480.L))+real(550141961994240.L))+
        real(2316387208396800.L))/real(342273571680157696.L);
      d *= eps;
      c[20] = d*(eps2*(eps2*(eps2*(eps2*(real(95701981543233.L)*eps2+
        real(112262383011675.L))+real(136757870745600.L))+
        real(177971206732800.L))+real(268866372403200.L))+
        real(1129238764093440.L))/real(180143985094819840.L);
      d *= eps;
      c[21] = d*(eps2*(eps2*(eps2*(real(73552247104245.L)*eps2+
        real(89484130583120.L))+real(116280720890880.L))+
        real(175374201999360.L))+real(734901417902080.L))/
        real(126100789566373888.L);
      d *= eps;
      c[22] = d*(eps2*(eps2*(eps2*(real(867892442332575.L)*eps2+
        real(1054610993641920.L))+real(1368579152054400.L))+
        real(2060919193681920.L))+real(8618389355397120.L))/
        real(1585267068834414592.L);
      d *= eps;
      c[23] = d*(eps2*(eps2*(real(16193469332322.L)*eps2+
        real(20988387163107.L))+real(31561484455800.L))+
        real(131734891641600.L))/real(25895697857380352.L);
      d *= eps;
      c[24] = d*(eps2*(eps2*(real(15922598844211.L)*eps2+
        real(20613660508197.L))+real(30957699535776.L))+
        real(128990414732400.L))/real(27021597764222976.L);
      d *= eps;
      c[25] = d*(eps2*(real(20258110006050.L)*eps2+real(30387165009075.L))+
        real(126410606437752.L))/real(28147497671065600.L);
      d *= eps;
      c[26] = d*(eps2*(real(1274891096275979.L)*eps2+real(1910204719503808.L))+
        real(7934696527169664.L))/real(1873497444986126336.L);
      d *= eps;
      c[27] = d*(real(469351468683027.L)*eps2+real(1946939425648112.L))/
        real(486388759756013568.L);
      d *= eps;
      c[28] = d*(real(32968493968795.L)*eps2+real(136583760727865.L))/
        real(36028797018963968.L);
      d *= eps;
      c[29] = real(1879204156221315.L)*d/real(522417556774977536.L);
      d *= eps;
      c[30] = real(7391536347803839.L)*d/real(2161727821137838080.L);
      break;
    default:
      static_assert(nC2_ == 30, "Bad value of nC2_");
    }
  }

  // The scale factor A3 = mean value of (d/dsigma)I3
  template<typename real>
  void Geodesic30<real>::A3coeff() {
    switch (nA3_) {
    case 30:
      _A3x[0] = 1;
      _A3x[1] = (_n-1)/real(2.L);
      _A3x[2] = (_n*(real(3.L)*_n-1)-real(2.L))/real(8.L);
      _A3x[3] = (_n*(_n*(real(5.L)*_n-1)-real(3.L))-1)/real(16.L);
      _A3x[4] = (_n*(_n*(_n*(real(35.L)*_n-real(5.L))-real(20.L))-real(4.L))-
        real(6.L))/real(128.L);
      _A3x[5] = (_n*(_n*(_n*(_n*(real(63.L)*_n-real(7.L))-real(35.L))-
        real(5.L))-real(10.L))-real(6.L))/real(256.L);
      _A3x[6] = (_n*(_n*(_n*(_n*(_n*(real(231.L)*_n-real(21.L))-real(126.L))-
        real(14.L))-real(35.L))-real(15.L))-real(20.L))/real(1024.L);
      _A3x[7] = (_n*(_n*(_n*(_n*(_n*(_n*(real(429.L)*_n-real(33.L))-
        real(231.L))-real(21.L))-real(63.L))-real(21.L))-real(35.L))-
        real(25.L))/real(2048.L);
      _A3x[8] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(6435.L)*_n-real(429.L))-
        real(3432.L))-real(264.L))-real(924.L))-real(252.L))-real(504.L))-
        real(280.L))-real(350.L))/real(32768.L);
      _A3x[9] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(12155.L)*_n-real(715.L))-
        real(6435.L))-real(429.L))-real(1716.L))-real(396.L))-real(924.L))-
        real(420.L))-real(630.L))-real(490.L))/real(65536.L);
      _A3x[10] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(46189.L)*_n-
        real(2431.L))-real(24310.L))-real(1430.L))-real(6435.L))-real(1287.L))-
        real(3432.L))-real(1320.L))-real(2310.L))-real(1470.L))-real(1764.L))/
        real(262144.L);
      _A3x[11] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(88179.L)*_n-
        real(4199.L))-real(46189.L))-real(2431.L))-real(12155.L))-real(2145.L))-
        real(6435.L))-real(2145.L))-real(4290.L))-real(2310.L))-real(3234.L))-
        real(2646.L))/real(524288.L);
      _A3x[12] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(676039.L)*_n-
        real(29393.L))-real(352716.L))-real(16796.L))-real(92378.L))-
        real(14586.L))-real(48620.L))-real(14300.L))-real(32175.L))-
        real(15015.L))-real(24024.L))-real(16632.L))-real(19404.L))/
        real(4194304.L);
      _A3x[13] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(1300075.L)*_n-real(52003.L))-real(676039.L))-real(29393.L))-
        real(176358.L))-real(25194.L))-real(92378.L))-real(24310.L))-
        real(60775.L))-real(25025.L))-real(45045.L))-real(27027.L))-
        real(36036.L))-real(30492.L))/real(8388608.L);
      _A3x[14] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(5014575.L)*_n-real(185725.L))-real(2600150.L))-real(104006.L))-
        real(676039.L))-real(88179.L))-real(352716.L))-real(83980.L))-
        real(230945.L))-real(85085.L))-real(170170.L))-real(90090.L))-
        real(135135.L))-real(99099.L))-real(113256.L))/real(33554432.L);
      _A3x[15] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(334305.L)*_n-real(5014575.L))*_n-real(185725.L))-real(1300075.L))-
        real(156009.L))-real(676039.L))-real(146965.L))-real(440895.L))-
        real(146965.L))-real(323323.L))-real(153153.L))-real(255255.L))-
        real(165165.L))-real(212355.L))-real(184041.L))/real(67108864.L);
      _A3x[16] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(2674440.L)*
        _n-real(20058300.L))*_n-real(2228700.L))-real(10400600.L))-
        real(2080120.L))-real(6760390.L))-real(2057510.L))-real(4938024.L))-
        real(2116296.L))-real(3879876.L))-real(2246244.L))-real(3208920.L))-
        real(2453880.L))-real(2760615.L))/real(1073741824.L);
      _A3x[17] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(4011660.L)*_n-
        real(20058300.L))*_n-real(3714500.L))-real(13000750.L))-
        real(3640210.L))-real(9464546.L))-real(3703518.L))-real(7407036.L))-
        real(3879876.L))-real(6096948.L))-real(4171596.L))-real(5214495.L))-
        real(4601025.L))/real(2147483648.L);
      _A3x[18] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(13372200.L)*_n-
        real(50145750.L))*_n-real(13000750.L))-real(36402100.L))-
        real(13104756.L))-real(28393638.L))-real(13579566.L))-real(23279256.L))-
        real(14410968.L))-real(19815081.L))-real(15643485.L))-real(17381650.L))/
        real(8589934592.L);
      _A3x[19] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(23401350.L)*_n-
        real(70204050.L))*_n-real(23401350.L))-real(54603150.L))-
        real(24025386.L))-real(44618574.L))-real(25219194.L))-real(37828791.L))-
        real(27020565.L))-real(33025135.L))-real(29548805.L))/
        real(17179869184.L);
      _A3x[20] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(168489720.L)*_n-
        real(421224300.L))*_n-real(171609900.L))-real(343219800.L))-
        real(178474296.L))-real(290020731.L))-real(189143955.L))-
        real(252191940.L))-real(204155380.L))-real(224570918.L))/
        real(137438953472.L);
      _A3x[21] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(308897820.L)*_n-
        real(661923900.L))*_n-real(318704100.L))-real(557732175.L))-
        real(334639305.L))-real(483367885.L))-real(357271915.L))-
        real(428726298.L))-real(387895222.L))/real(274877906944.L);
      _A3x[22] = (_n*(_n*(_n*(_n*(_n*((-real(1147334760.L)*_n-
        real(2151252675.L))*_n-real(1195140375.L))-real(1859107250.L))-
        real(1264192930.L))-real(1643450809.L))-real(1357633277.L))-
        real(1481054484.L))/real(1099511627776.L);
      _A3x[23] = (_n*(_n*(_n*(_n*((-real(2151252675.L)*_n-real(3585421125.L))*
        _n-real(2257487375.L))-real(3160482325.L))-real(2401966567.L))-
        real(2838687761.L))-real(2591845347.L))/real(2199023255552.L);
      _A3x[24] = (_n*(_n*(_n*((-real(16253909100.L)*_n-real(24380863650.L))*_n-
        real(17156904050.L))-real(21836059700.L))-real(18342290148.L))-
        real(19870814327.L))/real(17592186044416.L);
      _A3x[25] = (_n*(_n*((-real(30882427290.L)*_n-real(42112400850.L))*_n-
        real(32754089550.L))-real(38213104475.L))-real(35156056117.L))/
        real(35184372088832.L);
      _A3x[26] = (_n*((-real(117914722380.L)*_n-real(147393402975.L))*_n-
        real(125557343275.L))-real(135215600450.L))/real(140737488355328.L);
      _A3x[27] = ((-real(226003217895.L)*_n-real(260772943725.L))*_n-
        real(241456429375.L))/real(281474976710656.L);
      _A3x[28] = (-real(869243145750.L)*_n-real(931331941875.L))/
        real(1125899906842624.L);
      _A3x[29] = -real(1676397495375.L)/real(2251799813685248.L);
      break;
    default:
      static_assert(nA3_ == 30, "Bad value of nA3_");
    }
  }

  // The coefficients C3[l] in the Fourier expansion of B3
  template<typename real>
  void Geodesic30<real>::C3coeff() {
    switch (nC3_) {
    case 30:
      _C3x[0] = (1-_n)/real(4.L);
      _C3x[1] = (1-_n*_n)/real(8.L);
      _C3x[2] = (_n*((-real(5.L)*_n-1)*_n+real(3.L))+real(3.L))/real(64.L);
      _C3x[3] = (_n*(_n*((-real(7.L)*_n-real(2.L))*_n+real(2.L))+real(2.L))+
        real(5.L))/real(128.L);
      _C3x[4] = (_n*(_n*(_n*((-real(21.L)*_n-real(7.L))*_n+real(2.L))+
        real(3.L))+real(11.L))+real(12.L))/real(512.L);
      _C3x[5] = (_n*(_n*(_n*(_n*((-real(33.L)*_n-real(12.L))*_n-1)+real(2.L))+
        real(13.L))+real(10.L))+real(21.L))/real(1024.L);
      _C3x[6] = (_n*(_n*(_n*(_n*(_n*((-real(429.L)*_n-real(165.L))*_n-
        real(51.L))+real(3.L))+real(127.L))+real(83.L))+real(189.L))+
        real(243.L))/real(16384.L);
      _C3x[7] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(715.L)*_n-real(286.L))*_n-
        real(132.L))-real(22.L))+real(156.L))+real(90.L))+real(256.L))+
        real(218.L))+real(435.L))/real(32768.L);
      _C3x[8] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(2431.L)*_n-real(1001.L))*_n-
        real(572.L))-real(143.L))+real(376.L))+real(190.L))+real(730.L))+
        real(518.L))+real(953.L))+real(1380.L))/real(131072.L);
      _C3x[9] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(4199.L)*_n-
        real(1768.L))*_n-real(1157.L))-real(338.L))+real(428.L))+real(178.L))+
        real(1062.L))+real(660.L))+real(1355.L))+real(1268.L))+real(2511.L))/
        real(262144.L);
      _C3x[10] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(29393.L)*_n-
        real(12597.L))*_n-real(9061.L))-real(2873.L))+real(1690.L))+
        real(429.L))+real(6235.L))+real(3415.L))+real(8243.L))+real(6456.L))+
        real(10654.L))+real(16802.L))/real(2097152.L);
      _C3x[11] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(52003.L)*_n-
        real(22610.L))*_n-real(17442.L))-real(5814.L))+real(1019.L))-
        real(394.L))+real(9169.L))+real(4366.L))+real(12858.L))+real(8948.L))+
        real(15445.L))+real(15504.L))+real(30954.L))/real(4194304.L);
      _C3x[12] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(185725.L)*
        _n-real(81719.L))*_n-real(66538.L))-real(22933.L))-real(2431.L))-
        real(4760.L))+real(26848.L))+real(10730.L))+real(40625.L))+
        real(25475.L))+real(48544.L))+real(40995.L))+real(63733.L))+
        real(107156.L))/real(16777216.L);
      _C3x[13] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(334305.L)*_n-real(148580.L))*_n-real(126293.L))-real(44574.L))-
        real(13889.L))-real(13566.L))+real(38843.L))+real(12060.L))+
        real(64637.L))+real(36480.L))+real(78579.L))+real(59706.L))+
        real(93101.L))+real(98474.L))+real(199327.L))/real(33554432.L);
      _C3x[14] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(4345965.L)*_n-real(3825935.L))*_n-real(1374365.L))-real(645031.L))-
        real(515185.L))+real(879393.L))+real(172907.L))+real(1651687.L))+
        real(830627.L))+real(2065081.L))+real(1438067.L))+real(2385073.L))+
        real(2129255.L))+real(3197305.L))+real(5651931.L))/real(1073741824.L);
      _C3x[15] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(2635110.L)*
        _n-real(1567082.L))*_n-real(1128334.L))+real(1196886.L))+real(41154.L))+
        real(2640872.L))+real(1160954.L))+real(3421388.L))+real(2191610.L))+
        real(3948278.L))+real(3199618.L))+real(4680678.L))+real(5154098.L))+
        real(10594535.L))/real(2147483648.L);
      _C3x[16] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(3036238.L)-
        real(4732710.L)*_n)*_n-real(711930.L))+real(8433504.L))+
        real(3135890.L))+real(11400458.L))+real(6698326.L))+real(13284500.L))+
        real(9985014.L))+real(15160510.L))+real(14113798.L))+real(20752127.L))+
        real(38202700.L))/real(8589934592.L);
      _C3x[17] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(13411750.L)-
        real(2687228.L)*_n)*_n+real(3975956.L))+real(19062830.L))+
        real(10199028.L))+real(22561454.L))+real(15828372.L))+real(25469662.L))+
        real(21652916.L))+real(30360645.L))+real(34518196.L))+real(72058593.L))/
        real(17179869184.L);
      _C3x[18] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(17720134.L)*_n+
        real(127773982.L))+real(61522230.L))+real(154231298.L))+
        real(101028014.L))+real(174177686.L))+real(138481974.L))+
        real(198093717.L))+real(190535320.L))+real(276517950.L))+
        real(526506770.L))/real(137438953472.L);
      _C3x[19] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(91225460.L)*_n+
        real(264750400.L))+real(161444708.L))+real(300813408.L))+
        real(225494980.L))+real(336216217.L))+real(296506456.L))+
        real(403691887.L))+real(471128272.L))+real(998233018.L))/
        real(274877906944.L);
      _C3x[20] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(514826968.L)*_n+
        real(1045980884.L))+real(740839452.L))+real(1162954687.L))+
        real(964708022.L))+real(1323033446.L))+real(1306299508.L))+
        real(1880398291.L))+real(3684424788.L))/real(1099511627776.L);
      _C3x[21] = (_n*(_n*(_n*(_n*(_n*(_n*(real(1221777256.L)*_n+
        real(2032983351.L))+real(1601333264.L))+real(2262026963.L))+
        real(2053373904.L))+real(2737209361.L))+real(3264991480.L))+
        real(7015755337.L))/real(2199023255552.L);
      _C3x[22] = (_n*(_n*(_n*(_n*(_n*(real(10742315896.L)*_n+
        real(15783055573.L))+real(13531950573.L))+real(17993187773.L))+
        real(18152051013.L))+real(25998927035.L))+real(52217815849.L))/
        real(17592186044416.L);
      _C3x[23] = (_n*(_n*(_n*(_n*(real(22784142500.L)*_n+real(30928449214.L))+
        real(28740438038.L))+real(37719439012.L))+real(45836220854.L))+
        real(99797592861.L))/real(35184372088832.L);
      _C3x[24] = (_n*(_n*(_n*(real(95605222558.L)*_n+real(124188653752.L))+
        real(127568225626.L))+real(182178015931.L))+real(373911064852.L))/
        real(140737488355328.L);
      _C3x[25] = (_n*(_n*(real(203049255084.L)*_n+real(263375014467.L))+
        real(325226738124.L))+real(716876091533.L))/real(281474976710656.L);
      _C3x[26] = (_n*(real(905405918351.L)*_n+real(1291014428313.L))+
        real(2701098070323.L))/real(1125899906842624.L);
      _C3x[27] = (real(2328931996322.L)*_n+real(5192918413211.L))/
        real(2251799813685248.L);
      _C3x[28] = real(4914956648311.L)/real(2251799813685248.L);
      _C3x[29] = ((_n-real(3.L))*_n+real(2.L))/real(32.L);
      _C3x[30] = (_n*(_n*(real(2.L)*_n-real(3.L))-real(2.L))+real(3.L))/
        real(64.L);
      _C3x[31] = (_n*(_n*(_n*(real(7.L)*_n-real(6.L))-real(9.L))+real(2.L))+
        real(6.L))/real(256.L);
      _C3x[32] = (_n*(_n*(_n*(_n*(real(6.L)*_n-real(3.L))-real(7.L))-real(2.L))+
        1)+real(5.L))/real(256.L);
      _C3x[33] = (_n*(_n*(_n*(_n*(_n*(real(165.L)*_n-real(45.L))-real(164.L))-
        real(94.L))-real(39.L))+real(69.L))+real(108.L))/real(8192.L);
      _C3x[34] = (_n*(_n*(_n*(_n*(_n*(_n*(real(286.L)*_n-real(33.L))-
        real(238.L))-real(185.L))-real(126.L))+real(31.L))+real(78.L))+
        real(187.L))/real(16384.L);
      _C3x[35] = (_n*(_n*(_n*(_n*(_n*(_n*(real(1001.L)*_n*_n-real(693.L))-
        real(656.L))-real(537.L))-real(96.L))+real(31.L))+real(376.L))+
        real(574.L))/real(65536.L);
      _C3x[36] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(884.L)*_n+real(78.L))-
        real(507.L))-real(560.L))-real(507.L))-real(200.L))-real(119.L))+
        real(172.L))+real(249.L))+real(510.L))/real(65536.L);
      _C3x[37] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(12597.L)*_n+
        real(1989.L))-real(5954.L))-real(7540.L))-real(7284.L))-real(3873.L))-
        real(3109.L))+real(714.L))+real(1506.L))+real(4254.L))+real(6700.L))/
        real(1048576.L);
      _C3x[38] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(22610.L)*_n+
        real(4845.L))-real(8738.L))-real(12633.L))-real(12776.L))-real(8057.L))-
        real(7260.L))-real(1085.L))-real(40.L))+real(4800.L))+real(6204.L))+
        real(12130.L))/real(2097152.L);
      _C3x[39] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(81719.L)*_n+
        real(21318.L))-real(25517.L))-real(42330.L))-real(44304.L))-
        real(31372.L))-real(30128.L))-real(10320.L))-real(7799.L))+
        real(9344.L))+real(12521.L))+real(25360.L))+real(41508.L))/
        real(8388608.L);
      _C3x[40] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(74290.L)*
        _n+real(22287.L))-real(18411.L))-real(35530.L))-real(38199.L))-
        real(29513.L))-real(29528.L))-real(13678.L))-real(12422.L))+
        real(2534.L))+real(4591.L))+real(15804.L))+real(19679.L))+
        real(38096.L))/real(8388608.L);
      _C3x[41] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(4345965.L)*_n+real(1448655.L))-real(832048.L))-real(1914098.L))-
        real(2103053.L))-real(1743367.L))-real(1794994.L))-real(983384.L))-
        real(962677.L))-real(136037.L))-real(56814.L))+real(601990.L))+
        real(723929.L))+real(1262293.L))+real(2143640.L))/real(536870912.L);
      _C3x[42] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(2890755.L)*_n-real(1131830.L))-real(3232489.L))-real(3616194.L))-
        real(3182937.L))-real(3347214.L))-real(2047913.L))-real(2091694.L))-
        real(671633.L))-real(596986.L))+real(604683.L))+real(754922.L))+
        real(1663043.L))+real(2045006.L))+real(3976491.L))/real(1073741824.L);
      _C3x[43] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(10951220.L)*_n-real(12440079.L))*_n-real(11548200.L))-
        real(12341583.L))-real(8180980.L))-real(8591613.L))-real(3724560.L))-
        real(3691453.L))+real(650048.L))+real(1006901.L))+real(4387428.L))+
        real(5052845.L))+real(8120200.L))+real(14259486.L))/real(4294967296.L);
      _C3x[44] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(10436802.L)*_n-
        real(11294905.L))*_n-real(7966766.L))-real(8533921.L))-real(4366622.L))-
        real(4538443.L))-real(643704.L))-real(481047.L))+real(2682654.L))+
        real(3065499.L))+real(5531406.L))+real(6786587.L))+real(13335454.L))/
        real(4294967296.L);
      _C3x[45] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(122143714.L)*_n-
        real(132790935.L))*_n-real(75688034.L))-real(80897803.L))-
        real(25224538.L))-real(25183979.L))+real(21864198.L))+real(25399201.L))+
        real(62111350.L))+real(70086290.L))+real(107249998.L))+
        real(194077092.L))/real(68719476736.L);
      _C3x[46] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(78404126.L)*_n-
        real(85404217.L))*_n-real(35776594.L))-real(37828469.L))+
        real(5609194.L))+real(7175491.L))+real(42027410.L))+real(46289200.L))+
        real(74618230.L))+real(91786606.L))+real(182709161.L))/
        real(68719476736.L);
      _C3x[47] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(344253558.L)*_n-
        real(374065652.L))*_n-real(54776062.L))-real(55349724.L))+
        real(208666266.L))+real(227499107.L))+real(436486330.L))+
        real(487608355.L))+real(723295836.L))+real(1344523348.L))/
        real(549755813888.L);
      _C3x[48] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(130335398.L)*_n-
        real(141976804.L))*_n+real(106639482.L))+real(114645394.L))+
        real(314363814.L))+real(339882125.L))+real(510119863.L))+
        real(630601557.L))+real(1272679074.L))/real(549755813888.L);
      _C3x[49] = (_n*(_n*(_n*(_n*(_n*(_n*(real(403726013.L)*_n+
        real(390195853.L))+real(3452592737.L))+real(3666442512.L))+
        real(6141323658.L))+real(6828890391.L))+real(9924101863.L))+
        real(18898590996.L))/real(8796093022208.L);
      _C3x[50] = (_n*(_n*(_n*(_n*(_n*(real(4320090170.L)*_n+real(4491433394.L))+
        real(9227803497.L))+real(9876835726.L))+real(14121810537.L))+
        real(17563998162.L))+real(35940468233.L))/real(17592186044416.L);
      _C3x[51] = (_n*(_n*(_n*(_n*(real(26959223616.L)*_n+real(28247205103.L))+
        real(43392901448.L))+real(48175476253.L))+real(69053142104.L))+
        real(134394903642.L))/real(70368744177664.L);
      _C3x[52] = (_n*(_n*(_n*(real(33603857506.L)*_n+real(35768834103.L))+
        real(49404986752.L))+real(61855152171.L))+real(128285952730.L))/
        real(70368744177664.L);
      _C3x[53] = (_n*(_n*(real(308327089280.L)*_n+real(342398306855.L))+
        real(486224128685.L))+real(965196991034.L))/real(562949953421312.L);
      _C3x[54] = (_n*(real(698099481901.L)*_n+real(879958500946.L))+
        real(1848743160243.L))/real(1125899906842624.L);
      _C3x[55] = (real(1729207092437.L)*_n+real(3494999262339.L))/
        real(2251799813685248.L);
      _C3x[56] = real(6713540223121.L)/real(4503599627370496.L);
      _C3x[57] = (_n*((real(5.L)-_n)*_n-real(9.L))+real(5.L))/real(192.L);
      _C3x[58] = (_n*(_n*((real(10.L)-real(3.L)*_n)*_n-real(6.L))-real(10.L))+
        real(9.L))/real(384.L);
      _C3x[59] = (_n*(_n*(_n*((real(65.L)-real(27.L)*_n)*_n+real(5.L))-
        real(77.L))-real(8.L))+real(42.L))/real(3072.L);
      _C3x[60] = (_n*(_n*(_n*(_n*((real(100.L)-real(55.L)*_n)*_n+real(54.L))-
        real(94.L))-real(71.L))-real(6.L))+real(72.L))/real(6144.L);
      _C3x[61] = (_n*(_n*(_n*(_n*(_n*((real(605.L)-real(429.L)*_n)*_n+
        real(547.L))-real(363.L))-real(537.L))-real(383.L))+real(143.L))+
        real(417.L))/real(49152.L);
      _C3x[62] = (_n*(_n*(_n*(_n*(_n*(_n*((real(910.L)-real(819.L)*_n)*_n+
        real(1114.L))-real(218.L))-real(778.L))-real(882.L))-real(246.L))+
        real(190.L))+real(729.L))/real(98304.L);
      _C3x[63] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(2730.L)-real(3094.L)*_n)*_n+
        real(4186.L))+real(356.L))-real(1937.L))-real(3090.L))-real(1814.L))-
        real(792.L))+real(1131.L))+real(2324.L))/real(393216.L);
      _C3x[64] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(4080.L)-real(5814.L)*
        _n)*_n+real(7580.L))+real(2120.L))-real(2011.L))-real(4800.L))-
        real(3853.L))-real(3048.L))-real(51.L))+real(1648.L))+real(4149.L))/
        real(786432.L);
      _C3x[65] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(24225.L)-
        real(43605.L)*_n)*_n+real(53907.L))+real(22993.L))-real(5254.L))-
        real(27545.L))-real(27433.L))-real(27855.L))-real(10989.L))-
        real(1232.L))+real(14894.L))+real(27894.L))/real(6291456.L);
      _C3x[66] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(35530.L)-
        real(81719.L)*_n)*_n+real(94962.L))+real(51646.L))+real(4515.L))-
        real(36574.L))-real(44451.L))-real(53734.L))-real(31770.L))-
        real(20004.L))+real(7829.L))+real(23136.L))+real(50634.L))/
        real(12582912.L);
      _C3x[67] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(408595.L)-
        real(1225785.L)*_n)*_n+real(1332375.L))+real(854981.L))+real(232968.L))-
        real(348362.L))-real(538250.L))-real(761266.L))-real(550477.L))-
        real(467735.L))-real(108067.L))+real(81155.L))+real(385912.L))+
        real(703956.L))/real(201326592.L);
      _C3x[68] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(568100.L)-real(2300805.L)*_n)*_n+real(2332706.L))+
        real(1693774.L))+real(672619.L))-real(331458.L))-real(762238.L))-
        real(1277532.L))-real(1055003.L))-real(1050096.L))-real(498952.L))-
        real(229022.L))+real(317377.L))+real(626234.L))+real(1294296.L))/
        real(402653184.L);
      _C3x[69] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(2982525.L)*_n+real(16332875.L))+real(13082469.L))+
        real(6386185.L))-real(450699.L))-real(3957669.L))-real(8235207.L))-
        real(7543081.L))-real(8411347.L))-real(5167605.L))-real(3838275.L))-
        real(37047.L))+real(1893741.L))+real(5056083.L))+real(9205697.L))/
        real(3221225472.L);
      _C3x[70] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(24862310.L)*_n+real(13879396.L))+real(2352502.L))-
        real(4406176.L))-real(12794634.L))-real(12854322.L))-real(15794010.L))-
        real(11195418.L))-real(9970586.L))-real(3625900.L))-real(624250.L))+
        real(5065480.L))+real(8466918.L))+real(17095125.L))/real(6442450944.L);
      _C3x[71] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(18855814.L)*
        _n-real(6400580.L))-real(38210704.L))-real(42159819.L))-
        real(56880180.L))-real(44376336.L))-real(44090670.L))-real(23563667.L))-
        real(15009754.L))+real(5634896.L))+real(16018540.L))+real(33665445.L))+
        real(61816020.L))/real(25769803776.L);
      _C3x[72] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(54213352.L)*_n-
        real(66671799.L))*_n-real(99351680.L))-real(83302053.L))-
        real(89782004.L))-real(57369983.L))-real(46741460.L))-real(10420713.L))+
        real(6465344.L))+real(37679829.L))+real(57727272.L))+real(115705971.L))/
        real(51539607552.L);
      _C3x[73] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(677301442.L)*_n-
        real(602412942.L))*_n-real(695521790.L))-real(494978754.L))-
        real(455400986.L))-real(206502966.L))-real(102045774.L))+
        real(129846877.L))+real(247039696.L))+real(455562458.L))+
        real(847432470.L))/real(412316860416.L);
      _C3x[74] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(1302020508.L)*_n-
        real(997905328.L))*_n-real(994612524.L))-real(576720928.L))-
        real(428277468.L))-real(5966043.L))+real(188513240.L))+
        real(546610979.L))+real(795711904.L))+real(1596424446.L))/
        real(824633720832.L);
      _C3x[75] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(4082983123.L)*_n-
        real(2701446150.L))*_n-real(2333123246.L))-real(829177568.L))-
        real(196806795.L))+real(1148287217.L))+real(1838348957.L))+
        real(3127267353.L))+real(5904336436.L))/real(3298534883328.L);
      _C3x[76] = (_n*(_n*(_n*(_n*(_n*((-real(5459782588.L)*_n-
        real(2829585614.L))*_n-real(1858269176.L))+real(648655007.L))+
        real(1798151526.L))+real(3931237174.L))+real(5543454950.L))+
        real(11181923379.L))/real(6597069766656.L);
      _C3x[77] = (_n*(_n*(_n*(_n*((-real(23595042916.L)*_n-real(5244741905.L))*
        _n+real(2464941795.L))+real(18531587253.L))+real(26965095639.L))+
        real(43484270377.L))+real(83373729243.L))/real(52776558133248.L);
      _C3x[78] = (_n*(_n*(_n*((real(16288795140.L)-real(14117384220.L)*_n)*_n+
        real(30230987842.L))+real(56471603386.L))+real(78015773010.L))+
        real(158600795607.L))/real(105553116266496.L);
      _C3x[79] = (_n*(_n*(_n*(real(45025206620.L)*_n+real(143498036452.L))+
        real(196715312556.L))+real(305691627561.L))+real(595152058524.L))/
        real(422212465065984.L);
      _C3x[80] = (_n*(_n*(real(240498860600.L)*_n+real(406385867693.L))+
        real(553968586344.L))+real(1136437368711.L))/real(844424930131968.L);
      _C3x[81] = (_n*(real(1433809308597.L)*_n+real(2169875044125.L))+
        real(4287844495145.L))/real(3377699720527872.L);
      _C3x[82] = (real(3965575037462.L)*_n+real(8214264531663.L))/
        real(6755399441055744.L);
      _C3x[83] = real(41515375436483.L)/real(36028797018963968.L);
      _C3x[84] = (_n*(_n*((_n-real(7.L))*_n+real(20.L))-real(28.L))+real(14.L))/
        real(1024.L);
      _C3x[85] = (_n*(_n*(_n*(_n*(real(4.L)*_n-real(21.L))+real(36.L))-
        real(7.L))-real(40.L))+real(28.L))/real(2048.L);
      _C3x[86] = (_n*(_n*(_n*(_n*(_n*(real(22.L)*_n-real(91.L))+real(91.L))+
        real(78.L))-real(129.L))-real(43.L))+real(72.L))/real(8192.L);
      _C3x[87] = (_n*(_n*(_n*(_n*(_n*(_n*(real(52.L)*_n-real(175.L))+
        real(88.L))+real(213.L))-real(94.L))-real(165.L))-real(46.L))+
        real(127.L))/real(16384.L);
      _C3x[88] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(455.L)*_n-real(1274.L))+
        real(165.L))+real(1602.L))+real(150.L))-real(908.L))-real(1010.L))+
        real(48.L))+real(772.L))/real(131072.L);
      _C3x[89] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(476.L)*_n-real(1127.L))-
        real(189.L))+real(1312.L))+real(605.L))-real(351.L))-real(942.L))-
        real(518.L))+real(50.L))+real(684.L))/real(131072.L);
      _C3x[90] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1938.L)*_n-
        real(3927.L))-real(1633.L))+real(3978.L))+real(3078.L))+real(292.L))-
        real(2519.L))-real(2525.L))-real(1602.L))+real(678.L))+real(2242.L))/
        real(524288.L);
      _C3x[91] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(3876.L)*_n-
        real(6783.L))-real(4284.L))+real(5677.L))+real(6196.L))+real(2662.L))-
        real(2448.L))-real(4123.L))-real(4432.L))-real(1467.L))+real(1092.L))+
        real(4034.L))/real(1048576.L);
      _C3x[92] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(245157.L)*_n-
        real(373065.L))-real(308142.L))+real(242760.L))+real(359570.L))+
        real(237602.L))-real(25612.L))-real(167628.L))-real(268499.L))-
        real(177843.L))-real(75616.L))+real(90456.L))+real(220860.L))/
        real(67108864.L);
      _C3x[93] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(480700.L)*_n-real(639331.L))-real(643720.L))+real(294443.L))+
        real(613724.L))+real(521638.L))+real(129576.L))-real(148074.L))-
        real(421892.L))-real(383837.L))-real(307316.L))-real(47759.L))+
        real(148928.L))+real(402920.L))/real(134217728.L);
      _C3x[94] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(1874730.L)*_n-real(2187185.L))-real(2581359.L))+real(587290.L))+
        real(2007901.L))+real(2057345.L))+real(964898.L))+real(15792.L))-
        real(1137956.L))-real(1335009.L))-real(1430285.L))-real(745290.L))-
        real(163129.L))+real(654529.L))+real(1417728.L))/real(536870912.L);
      _C3x[95] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(3736005.L)*_n-real(5047120.L))*_n+real(278231.L))+real(3169734.L))+
        real(3815723.L))+real(2408190.L))+real(920027.L))-real(1248824.L))-
        real(2001037.L))-real(2723500.L))-real(1983405.L))-real(1305006.L))+
        real(90895.L))+real(1104706.L))+real(2615571.L))/real(1073741824.L);
      _C3x[96] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(19312686.L)-
        real(3136510.L)*_n)*_n+real(27186182.L))+real(20776832.L))+
        real(12110524.L))-real(3156661.L))-real(10225072.L))-real(18409849.L))-
        real(16324854.L))-real(14503236.L))-real(5969538.L))+real(520358.L))+
        real(9152912.L))+real(18770476.L))/real(8589934592.L);
      _C3x[97] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(23499284.L)*
        _n+real(20757564.L))+real(14965037.L))+real(2117612.L))-
        real(4900342.L))-real(14129447.L))-real(14665951.L))-real(15712829.L))-
        real(9895596.L))-real(5358338.L))+real(2323595.L))+real(7885978.L))+
        real(17471468.L))/real(8589934592.L);
      _C3x[98] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(65561705.L)*_n+
        real(23786693.L))-real(1657343.L))-real(39101036.L))-real(47712508.L))-
        real(60088211.L))-real(46312722.L))-real(36260374.L))-real(10716843.L))+
        real(7655783.L))+real(31805471.L))+real(63607578.L))/
        real(34359738368.L);
      _C3x[99] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(25102510.L)*_n-
        real(46318370.L))-real(70453029.L))-real(105735978.L))-
        real(93188846.L))-real(87564902.L))-real(48412679.L))-real(20042882.L))+
        real(23559768.L))+real(55739490.L))+real(119278493.L))/
        real(68719476736.L);
      _C3x[100] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(732828227.L)*_n-
        real(1391328708.L))*_n-real(1366640704.L))-real(1464442582.L))-
        real(1022676160.L))-real(717924527.L))-real(106745493.L))+
        real(319963578.L))+real(887074472.L))+real(1756525392.L))/
        real(1099511627776.L);
      _C3x[101] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(2332298309.L)*_n-
        real(2809920676.L))*_n-real(2248335200.L))-real(1928941304.L))-
        real(922185663.L))-real(223023504.L))+real(797767751.L))+
        real(1576265804.L))+real(3313698360.L))/real(2199023255552.L);
      _C3x[102] = (_n*(_n*(_n*(_n*(_n*((-real(8944055483.L)*_n-
        real(8691989170.L))*_n-real(5569520181.L))-real(3483012103.L))+
        real(215987194.L))+real(2760680993.L))+real(6225526419.L))+
        real(12308219876.L))/real(8796093022208.L);
      _C3x[103] = (_n*(_n*(_n*(_n*((-real(13263280532.L)*_n-
        real(10560083040.L))*_n-real(4164504717.L))+real(157999950.L))+
        real(6303559381.L))+real(11189892910.L))+real(23336011385.L))/
        real(17592186044416.L);
      _C3x[104] = (_n*(_n*(_n*((-real(29964401456.L)*_n-real(16179701905.L))*_n+
        real(6562138344.L))+real(22131275675.L))+real(44015198152.L))+
        real(87304283850.L))/real(70368744177664.L);
      _C3x[105] = (_n*(_n*((real(5148539461.L)-real(8353615866.L)*_n)*_n+
        real(24122697312.L))+real(39939504101.L))+real(83113869946.L))/
        real(70368744177664.L);
      _C3x[106] = (_n*(_n*(real(36921324958.L)*_n+real(85690129986.L))+
        real(156748055238.L))+real(312802619769.L))/real(281474976710656.L);
      _C3x[107] = (_n*(real(90862776950.L)*_n+real(143400783150.L))+
        real(298867963845.L))/real(281474976710656.L);
      _C3x[108] = (real(17988171936502.L)*_n+real(36174484487935.L))/
        real(36028797018963968.L);
      _C3x[109] = real(34671257326469.L)/real(36028797018963968.L);
      _C3x[110] = (_n*(_n*(_n*((real(9.L)-_n)*_n-real(35.L))+real(75.L))-
        real(90.L))+real(42.L))/real(5120.L);
      _C3x[111] = (_n*(_n*(_n*(_n*((real(36.L)-real(5.L)*_n)*_n-real(100.L))+
        real(114.L))+real(15.L))-real(150.L))+real(90.L))/real(10240.L);
      _C3x[112] = (_n*(_n*(_n*(_n*(_n*((real(387.L)-real(65.L)*_n)*_n-
        real(783.L))+real(319.L))+real(883.L))-real(781.L))-real(455.L))+
        real(495.L))/real(81920.L);
      _C3x[113] = (_n*(_n*(_n*(_n*(_n*(_n*((real(882.L)-real(175.L)*_n)*_n-
        real(1300.L))-real(278.L))+real(1844.L))-real(54.L))-real(1264.L))-
        real(550.L))+real(895.L))/real(163840.L);
      _C3x[114] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(3690.L)-real(850.L)*_n)*_n-
        real(3886.L))-real(3248.L))+real(5431.L))+real(3118.L))-real(2190.L))-
        real(4340.L))-real(545.L))+real(2820.L))/real(655360.L);
      _C3x[115] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(7344.L)-real(1938.L)*
        _n)*_n-real(5272.L))-real(8432.L))+real(6277.L))+real(8056.L))+
        real(1267.L))-real(6368.L))-real(5399.L))-real(600.L))+real(5065.L))/
        real(1310720.L);
      _C3x[116] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(113373.L)-
        real(33915.L)*_n)*_n-real(50133.L))-real(142119.L))+real(38817.L))+
        real(115867.L))+real(72641.L))-real(38765.L))-real(82240.L))-
        real(69366.L))+real(7970.L))+real(67870.L))/real(20971520.L);
      _C3x[117] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(214434.L)-
        real(72105.L)*_n)*_n-real(44916.L))-real(271578.L))-real(11884.L))+
        real(172634.L))+real(182110.L))+real(25898.L))-real(93317.L))-
        real(157368.L))-real(83038.L))+real(15980.L))+real(123150.L))/
        real(41943040.L);
      _C3x[118] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(1600731.L)-
        real(600875.L)*_n)*_n-real(13585.L))-real(1961579.L))-real(566304.L))+
        real(866314.L))+real(1406914.L))+real(709178.L))-real(128591.L))-
        real(953267.L))-real(914519.L))-real(534457.L))+real(234980.L))+
        real(855060.L))/real(335544320.L);
      _C3x[119] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(2960100.L)-real(1233375.L)*_n)*_n+real(498916.L))-
        real(3411122.L))-real(1692661.L))+real(819102.L))+real(2362622.L))+
        real(1855500.L))+real(693943.L))-real(1011848.L))-real(1591338.L))-
        real(1627622.L))-real(606887.L))+real(415890.L))+real(1568780.L))/
        real(671088640.L);
      _C3x[120] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(21756735.L)*_n+real(7120685.L))-real(23067781.L))-
        real(15811785.L))+real(710823.L))+real(14255529.L))+real(15202259.L))+
        real(10250183.L))-real(1456765.L))-real(8161927.L))-real(12780537.L))-
        real(9244317.L))-real(3945181.L))+real(4033765.L))+real(11153319.L))/
        real(5368709120.L);
      _C3x[121] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(38083630.L)*_n-real(33134444.L))*_n-real(7271838.L))+
        real(19170884.L))+real(27391698.L))+real(24863984.L))+real(7123418.L))-
        real(6259204.L))-real(19860598.L))-real(20046964.L))-real(16199990.L))-
        real(3877396.L))+real(7101050.L))+real(20660035.L))/
        real(10737418240.L);
      _C3x[122] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(43151928.L)-
        real(53355262.L)*_n)*_n+real(90086284.L))+real(102204983.L))+
        real(54339236.L))+real(9788240.L))-real(49106330.L))-real(67981809.L))-
        real(75409366.L))-real(45422452.L))-real(12628752.L))+real(30777355.L))+
        real(74697180.L))/real(42949672960.L);
      _C3x[123] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(136154144.L)*_n+
        real(190838215.L))+real(135658088.L))+real(71547201.L))-
        real(38897276.L))-real(93895333.L))-real(140097532.L))-
        real(116140263.L))-real(79697544.L))-real(6919577.L))+real(54581600.L))+
        real(139472685.L))/real(85899345920.L);
      _C3x[124] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1150583902.L)*_n+
        real(843007180.L))+real(87120772.L))-real(381299022.L))-
        real(890514168.L))-real(919543084.L))-real(854983080.L))-
        real(438238873.L))-real(42146824.L))+real(451497470.L))+
        real(1021490770.L))/real(687194767360.L);
      _C3x[125] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(773183540.L)*_n-
        real(83029198.L))-real(1204494192.L))-real(1551156122.L))-
        real(1789274128.L))-real(1306192047.L))-real(774506096.L))+
        real(91038377.L))+real(808211280.L))+real(1919928970.L))/
        real(1374389534720.L);
      _C3x[126] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(2504691295.L)*_n-
        real(4510599178.L))*_n-real(6493267834.L))-real(5783852602.L))-
        real(4780409067.L))-real(2051760759.L))+real(347941049.L))+
        real(3265965655.L))+real(7101103260.L))/real(5497558138880.L);
      _C3x[127] = (_n*(_n*(_n*(_n*(_n*((-real(10518977828.L)*_n-
        real(10971830148.L))*_n-real(10992488016.L))-real(7250179399.L))-
        real(3662192098.L))+real(1566413948.L))+real(5899261070.L))+
        real(13420534285.L))/real(10995116277760.L);
      _C3x[128] = (_n*(_n*(_n*(_n*((-real(87342708312.L)*_n-
        real(70583370027.L))*_n-real(53136976015.L))-real(18044662719.L))+
        real(11403394345.L))+real(47077041855.L))+real(100071330445.L))/
        real(87960930222080.L);
      _C3x[129] = (_n*(_n*(_n*((-real(132890278572.L)*_n-real(79742300086.L))*
        _n-real(32604144594.L))+real(31724468588.L))+real(85731483950.L))+
        real(190008619665.L))/real(175921860444160.L);
      _C3x[130] = (_n*(_n*((-real(293633984252.L)*_n-real(68431954348.L))*_n+
        real(115135641616.L))+real(339587063745.L))+real(713060618004.L))/
        real(703687441776640.L);
      _C3x[131] = (_n*((real(277112865109.L)-real(125873779648.L)*_n)*_n+
        real(622912862560.L))+real(1359292093255.L))/real(1407374883553280.L);
      _C3x[132] = (_n*(real(4035517048233.L)*_n+real(9826950043155.L))+
        real(20516231418705.L))/real(22517998136852480.L);
      _C3x[133] = (real(3628272594518.L)*_n+real(7848710341433.L))/
        real(9007199254740992.L);
      _C3x[134] = real(29752988542989.L)/real(36028797018963968.L);
      _C3x[135] = (_n*(_n*(_n*(_n*((_n-real(11.L))*_n+real(54.L))-real(154.L))+
        real(275.L))-real(297.L))+real(132.L))/real(24576.L);
      _C3x[136] = (_n*(_n*(_n*(_n*(_n*(_n*(real(6.L)*_n-real(55.L))+
        real(210.L))-real(407.L))+real(334.L))+real(165.L))-real(550.L))+
        real(297.L))/real(49152.L);
      _C3x[137] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(45.L)*_n-real(352.L))+
        real(1067.L))-real(1352.L))-real(73.L))+real(1924.L))-real(1105.L))-
        real(1012.L))+real(858.L))/real(196608.L);
      _C3x[138] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(68.L)*_n-real(462.L))+
        real(1125.L))-real(828.L))-real(987.L))+real(1602.L))+real(457.L))-
        real(1104.L))-real(663.L))+real(792.L))/real(196608.L);
      _C3x[139] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(2907.L)*_n-
        real(17391.L))+real(34196.L))-real(9770.L))-real(42985.L))+
        real(25557.L))+real(34762.L))-real(5372.L))-real(33770.L))-
        real(8666.L))+real(20532.L))/real(6291456.L);
      _C3x[140] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(7182.L)*_n-
        real(38247.L))+real(60690.L))+real(7489.L))-real(85142.L))+
        real(5071.L))+real(64414.L))+real(35811.L))-real(35836.L))-
        real(47462.L))-real(11308.L))+real(37338.L))/real(12582912.L);
      _C3x[141] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(33649.L)*_n-
        real(160930.L))+real(204725.L))+real(107244.L))-real(288887.L))-
        real(101252.L))+real(161989.L))+real(212012.L))+real(11772.L))-
        real(140214.L))-real(159660.L))-real(7764.L))+real(127316.L))/
        real(50331648.L);
      _C3x[142] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(37950.L)*_n-real(164197.L))+real(165187.L))+real(156973.L))-
        real(219453.L))-real(163464.L))+real(57699.L))+real(196412.L))+
        real(110569.L))-real(33957.L))-real(150236.L))-real(108221.L))-
        real(1716.L))+real(116454.L))/real(50331648.L);
      _C3x[143] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(1332045.L)*_n-real(5245955.L))+real(4076842.L))+real(5917318.L))-
        real(4780941.L))-real(5835657.L))-real(840868.L))+real(4499436.L))+
        real(4654573.L))+real(1817813.L))-real(2581642.L))-real(3881158.L))-
        real(2854965.L))+real(449487.L))+real(3273672.L))/real(1610612736.L);
      _C3x[144] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(5953090.L)-real(10311015.L)*_n)*_n+real(12596617.L))-
        real(5567034.L))-real(11135251.L))-real(5289630.L))+real(4820965.L))+
        real(8679918.L))+real(6963957.L))-real(310838.L))-real(5162907.L))-
        real(7247618.L))-real(3812959.L))+real(920682.L))+real(6040593.L))/
        real(3221225472.L);
      _C3x[145] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(50566880.L)*_n-real(9027109.L))-real(38573596.L))-
        real(29155365.L))+real(4319952.L))+real(26216619.L))+real(31891860.L))+
        real(13148195.L))-real(6093792.L))-real(23482225.L))-real(22504028.L))-
        real(12576273.L))+real(5519128.L))+real(21661914.L))/
        real(12884901888.L);
      _C3x[146] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(30931858.L)*
        _n-real(32175059.L))*_n-real(7242830.L))+real(15741021.L))+
        real(29478542.L))+real(20985541.L))+real(6753674.L))-real(12859795.L))-
        real(20330866.L))-real(20205383.L))-real(8147322.L))+real(5140585.L))+
        real(20146890.L))/real(12884901888.L);
      _C3x[147] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(91139599.L)-
        real(242567240.L)*_n)*_n+real(375851872.L))+real(371070857.L))+
        real(248291350.L))-real(30685755.L))-real(208795576.L))-
        real(323074749.L))-real(248000062.L))-real(107729106.L))+
        real(96179106.L))+real(293300764.L))/real(206158430208.L);
      _C3x[148] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(257879164.L)*_n+
        real(343367001.L))+real(319534237.L))+real(109205227.L))-
        real(69647705.L))-real(246369225.L))-real(268855985.L))-
        real(220137880.L))-real(64982994.L))+real(88183634.L))+
        real(274690815.L))/real(206158430208.L);
      _C3x[149] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(2671982082.L)*_n+
        real(1604379736.L))+real(442573818.L))-real(1103405692.L))-
        real(1777876366.L))-real(2036680639.L))-real(1342013078.L))-
        real(427562847.L))+real(758331652.L))+real(2022220812.L))/
        real(1649267441664.L);
      _C3x[150] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1191646598.L)*_n-
        real(235840100.L))-real(1128612206.L))-real(1840135234.L))-
        real(1670440794.L))-real(1189576929.L))-real(214107557.L))+
        real(694527495.L))+real(1905124158.L))/real(1649267441664.L);
      _C3x[151] = (_n*(_n*(_n*(_n*(_n*((-real(6581283849.L)*_n-
        real(21849837611.L))*_n-real(25566555333.L))-real(24823268906.L))-
        real(14379685422.L))-real(2726443515.L))+real(11489787637.L))+
        real(28301872956.L))/real(26388279066624.L);
      _C3x[152] = (_n*(_n*(_n*(_n*((-real(40456267930.L)*_n-
        real(50251646566.L))*_n-real(40456260929.L))-real(25556761710.L))-
        real(1055263841.L))+real(21111671798.L))+real(53594155323.L))/
        real(52776558133248.L);
      _C3x[153] = (_n*(_n*(_n*((-real(171788072360.L)*_n-real(149171287893.L))*
        _n-real(76119405724.L))-real(2047769071.L))+real(85526267172.L))+
        real(200488999982.L))/real(211106232532992.L);
      _C3x[154] = (_n*(_n*((-real(121054296908.L)*_n-real(67906408573.L))*_n+
        real(9584810812.L))+real(78919576567.L))+real(190644244200.L))/
        real(211106232532992.L);
      _C3x[155] = (_n*((real(316127348097.L)-real(1578353856668.L)*_n)*_n+
        real(2527332335909.L))+real(5739619727334.L))/real(6755399441055744.L);
      _C3x[156] = (_n*(real(378631414483.L)*_n+real(1561962329898.L))+
        real(3651975781065.L))/real(4503599627370496.L);
      _C3x[157] = (real(9311073056275.L)*_n+real(20719053535587.L))/
        real(27021597764222976.L);
      _C3x[158] = real(13225333818489.L)/real(18014398509481984.L);
      _C3x[159] = (_n*(_n*(_n*(_n*(_n*((real(13.L)-_n)*_n-real(77.L))+
        real(273.L))-real(637.L))+real(1001.L))-real(1001.L))+real(429.L))/
        real(114688.L);
      _C3x[160] = (_n*(_n*(_n*(_n*(_n*(_n*((real(78.L)-real(7.L)*_n)*_n-
        real(378.L))+real(1014.L))-real(1526.L))+real(910.L))+real(910.L))-
        real(2002.L))+real(1001.L))/real(229376.L);
      _C3x[161] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(1157.L)-real(119.L)*_n)*_n-
        real(4667.L))+real(9475.L))-real(7912.L))-real(4808.L))+real(15064.L))-
        real(5824.L))-real(8372.L))+real(6006.L))/real(1835008.L);
      _C3x[162] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(3432.L)-real(399.L)*
        _n)*_n-real(11662.L))+real(17642.L))-real(3979.L))-real(22002.L))+
        real(19344.L))+real(12576.L))-real(14588.L))-real(11648.L))+
        real(11284.L))/real(3670016.L);
      _C3x[163] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(35815.L)-
        real(4655.L)*_n)*_n-real(103371.L))+real(112179.L))+real(47549.L))-
        real(185267.L))+real(27159.L))+real(146613.L))+real(20088.L))-
        real(124082.L))-real(46830.L))+real(74802.L))/real(29360128.L);
      _C3x[164] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(86086.L)-
        real(12397.L)*_n)*_n-real(212174.L))+real(152466.L))+real(198174.L))-
        real(290474.L))-real(125864.L))+real(196038.L))+real(205965.L))-
        real(78680.L))-real(191254.L))-real(65436.L))+real(137550.L))/
        real(58720256.L);
      _C3x[165] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(391391.L)-
        real(61985.L)*_n)*_n-real(826133.L))+real(324603.L))+real(996015.L))-
        real(712496.L))-real(823810.L))+real(205090.L))+real(883334.L))+
        real(341577.L))-real(410489.L))-real(673813.L))-real(109284.L))+
        real(476000.L))/real(234881024.L);
      _C3x[166] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(855140.L)-real(148005.L)*_n)*_n-real(1546842.L))+real(144122.L))+
        real(2083638.L))-real(569118.L))-real(1686973.L))-real(528180.L))+
        real(1196404.L))+real(1265672.L))+real(213351.L))-real(1029806.L))-
        real(989009.L))-real(137830.L))+real(877436.L))/real(469762048.L);
      _C3x[167] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(14517945.L)*_n-real(22471345.L))-real(4339571.L))+
        real(31489647.L))+real(1831549.L))-real(22143117.L))-real(17180199.L))+
        real(7109039.L))+real(19266365.L))+real(14479171.L))-real(4244023.L))-
        real(14648021.L))-real(13415143.L))+real(17647.L))+real(12461421.L))/
        real(7516192768.L);
      _C3x[168] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(55462386.L)-real(18994690.L)*_n)*_n+real(19065734.L))-
        real(30092154.L))-real(40432410.L))-real(6438274.L))+real(25471590.L))+
        real(35942182.L))+real(12914102.L))-real(12597854.L))-real(29019826.L))-
        real(19492242.L))+real(854350.L))+real(23120321.L))/
        real(15032385536.L);
      _C3x[169] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(114271968.L)*_n-real(61892470.L))-real(153875770.L))-
        real(81163880.L))+real(37719552.L))+real(126465718.L))+
        real(98351552.L))+real(22111476.L))-real(72519030.L))-real(96403730.L))-
        real(65595642.L))+real(12136985.L))+real(83547436.L))/
        real(60129542144.L);
      _C3x[170] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(254627588.L)*_n-
        real(218231222.L))*_n-real(38185276.L))+real(171451874.L))+
        real(214174016.L))+real(145938602.L))-real(27770712.L))-
        real(140240378.L))-real(178803164.L))-real(94448081.L))+
        real(25181940.L))+real(156047703.L))/real(120259084288.L);
      _C3x[171] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(623528480.L)-
        real(965055826.L)*_n)*_n+real(1434944084.L))+real(1528801190.L))+
        real(566356676.L))-real(401212432.L))-real(1207886864.L))-
        real(1163707393.L))-real(635678176.L))+real(273117306.L))+
        real(1142557542.L))/real(962072674304.L);
      _C3x[172] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1889155476.L)*_n+
        real(3018421226.L))+real(2093307448.L))+real(594189030.L))-
        real(1339430872.L))-real(2152615067.L))-real(2107960000.L))-
        real(897778931.L))+real(524636224.L))+real(2146919726.L))/
        real(1924145348608.L);
      _C3x[173] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(9713659090.L)*_n+
        real(6430254860.L))-real(588478448.L))-real(5503162739.L))-
        real(8503273468.L))-real(6807044438.L))-real(3021327022.L))+
        real(2399508349.L))+real(7939231980.L))/real(7696581394432.L);
      _C3x[174] = (_n*(_n*(_n*(_n*(_n*(_n*(real(6723580112.L)*_n-
        real(3165663579.L))-real(12702420944.L))-real(14718114487.L))-
        real(12227922220.L))-real(4098421761.L))+real(4520065116.L))+
        real(14996625623.L))/real(15393162788864.L);
      _C3x[175] = (_n*(_n*(_n*(_n*((-real(51184976260.L)*_n-
        real(94959230613.L))*_n-real(112176079097.L))-real(78636170271.L))-
        real(27452041365.L))+real(38672730649.L))+real(111809128095.L))/
        real(123145302310912.L);
      _C3x[176] = (_n*(_n*(_n*((-real(197145724940.L)*_n-real(191745972804.L))*
        _n-real(140982075974.L))-real(34174146538.L))+real(72451525482.L))+
        real(212158972431.L))/real(246290604621824.L);
      _C3x[177] = (_n*(_n*((-real(1437329346694.L)*_n-real(902518571658.L))*_n-
        real(224701698186.L))+real(599572110655.L))+real(1592207279551.L))/
        real(1970324836974592.L);
      _C3x[178] = (_n*((-real(231337075012.L)*_n-real(31913675941.L))*_n+
        real(160383973836.L))+real(433302064951.L))/real(562949953421312.L);
      _C3x[179] = ((real(1302630681669.L)-real(195127280595.L)*_n)*_n+
        real(3269693185065.L))/real(4503599627370496.L);
      _C3x[180] = (real(2442794262822.L)*_n+real(6249998138435.L))/
        real(9007199254740992.L);
      _C3x[181] = real(5922672166091.L)/real(9007199254740992.L);
      _C3x[182] = (_n*(_n*(_n*(_n*(_n*(_n*((_n-real(15.L))*_n+real(104.L))-
        real(440.L))+real(1260.L))-real(2548.L))+real(3640.L))-real(3432.L))+
        real(1430.L))/real(524288.L);
      _C3x[183] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(8.L)*_n-real(105.L))+
        real(616.L))-real(2095.L))+real(4416.L))-real(5444.L))+real(2240.L))+
        real(4212.L))-real(7280.L))+real(3432.L))/real(1048576.L);
      _C3x[184] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(76.L)*_n-
        real(885.L))+real(4455.L))-real(12274.L))+real(18469.L))-real(9469.L))-
        real(15300.L))+real(27880.L))-real(6868.L))-real(16692.L))+
        real(10608.L))/real(4194304.L);
      _C3x[185] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(280.L)*_n-
        real(2925.L))+real(12768.L))-real(28533.L))+real(28358.L))+
        real(8933.L))-real(48206.L))+real(25381.L))+real(31144.L))-
        real(23084.L))-real(24344.L))+real(20228.L))/real(8388608.L);
      _C3x[186] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(3542.L)*_n-
        real(33495.L))+real(127809.L))-real(230466.L))+real(122726.L))+
        real(223896.L))-real(324398.L))-real(69212.L))+real(268897.L))+
        real(104101.L))-real(219640.L))-real(110368.L))+real(136608.L))/
        real(67108864.L);
      _C3x[187] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(10120.L)*_n-real(87285.L))+real(292930.L))-real(420741.L))+
        real(34854.L))+real(579146.L))-real(358116.L))-real(424820.L))+
        real(220376.L))+real(465795.L))-real(38868.L))-real(365791.L))-
        real(161296.L))+real(253696.L))/real(134217728.L);
      _C3x[188] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(53820.L)*_n-real(426075.L))+real(1263413.L))-real(1411598.L))-
        real(565193.L))+real(2300175.L))-real(262316.L))-real(1830342.L))-
        real(439342.L))+real(1517573.L))+real(1091783.L))-real(488554.L))-
        real(1348101.L))-real(344251.L))+real(889008.L))/real(536870912.L);
      _C3x[189] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(2606500.L)-real(991575.L)*_n)*_n-real(2173807.L))-
        real(2158390.L))+real(3906577.L))+real(1224426.L))-real(2724453.L))-
        real(2448068.L))+real(1183963.L))+real(2722720.L))+real(1248445.L))-
        real(1597174.L))-real(2099405.L))-real(485734.L))+real(1650255.L))/
        real(1073741824.L);
      _C3x[190] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-
        real(12008165.L)*_n-real(22469572.L))*_n+real(23263386.L))+
        real(18003332.L))-real(10903668.L))-real(22903967.L))-real(4557546.L))+
        real(15031227.L))+real(18603786.L))+real(1717308.L))-real(12627706.L))-
        real(14639770.L))-real(1541872.L))+real(11824512.L))/
        real(8589934592.L);
      _C3x[191] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(29579711.L)*
        _n+real(42402232.L))-real(1039922.L))-real(39561952.L))-
        real(26863970.L))+real(8845690.L))+real(35366520.L))+real(23602382.L))-
        real(3401152.L))-real(26898756.L))-real(22490338.L))-real(1720908.L))+
        real(22052152.L))/real(17179869184.L);
      _C3x[192] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(60038145.L)*_n-
        real(107353530.L))-real(133553528.L))-real(40575238.L))+
        real(89742068.L))+real(118191232.L))+real(63507638.L))-
        real(44205580.L))-real(94917430.L))-real(77239730.L))+real(2874382.L))+
        real(80228148.L))/real(68719476736.L);
      _C3x[193] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(250602523.L)*_n-
        real(180325740.L))*_n+real(51990092.L))+real(195016704.L))+
        real(204959298.L))+real(48236816.L))-real(101853960.L))-
        real(182774340.L))-real(118089824.L))+real(9842628.L))+
        real(150439306.L))/real(137438953472.L);
      _C3x[194] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(867626658.L)-
        real(463032277.L)*_n)*_n+real(1654038312.L))+real(1112988436.L))+
        real(98397140.L))-real(998545940.L))-real(1246247706.L))-
        real(808497650.L))+real(163832248.L))+real(1107294472.L))/
        real(1099511627776.L);
      _C3x[195] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(2517056589.L)*_n+
        real(2729426664.L))+real(1648670380.L))-real(520146568.L))-
        real(1919041474.L))-real(2311774100.L))-real(1228090618.L))+
        real(342460324.L))+real(2087093160.L))/real(2199023255552.L);
      _C3x[196] = (_n*(_n*(_n*(_n*(_n*(_n*(real(9631735815.L)*_n+
        real(3342091072.L))-real(3029285298.L))-real(8027650086.L))-
        real(7779350468.L))-real(4203816666.L))+real(1751654138.L))+
        real(7750396492.L))/real(8796093022208.L);
      _C3x[197] = (_n*(_n*(_n*(_n*(_n*(real(3576385429.L)*_n-
        real(8937940728.L))-real(14609523466.L))-real(14208255580.L))-
        real(6302714618.L))+real(3432444820.L))+real(14676225214.L))/
        real(17592186044416.L);
      _C3x[198] = (_n*(_n*(_n*((-real(296563545564.L)*_n-real(460504627654.L))*
        _n-real(380362677526.L))-real(172492566812.L))+real(123705035524.L))+
        real(439177896603.L))/real(562949953421312.L);
      _C3x[199] = (_n*(_n*((-real(409510620075.L)*_n-real(345172315152.L))*_n-
        real(126131044829.L))+real(118724470300.L))+real(417516380062.L))/
        real(562949953421312.L);
      _C3x[200] = (_n*((-real(1151485441526.L)*_n-real(429816687963.L))*_n+
        real(506107726005.L))+real(1571125991940.L))/real(2251799813685248.L);
      _C3x[201] = ((real(963889308006.L)-real(599801259897.L)*_n)*_n+
        real(2997989980695.L))/real(4503599627370496.L);
      _C3x[202] = (real(997668129067.L)*_n+real(2834595204413.L))/
        real(4503599627370496.L);
      _C3x[203] = real(2712992943545.L)/real(4503599627370496.L);
      _C3x[204] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(17.L)-_n)*_n-real(135.L))+
        real(663.L))-real(2244.L))+real(5508.L))-real(9996.L))+real(13260.L))-
        real(11934.L))+real(4862.L))/real(2359296.L);
      _C3x[205] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(136.L)-real(9.L)*_n)*
        _n-real(936.L))+real(3842.L))-real(10275.L))+real(18054.L))-
        real(18768.L))+real(4488.L))+real(18054.L))-real(26520.L))+
        real(11934.L))/real(4718592.L);
      _C3x[206] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(2567.L)-
        real(189.L)*_n)*_n-real(15487.L))+real(53599.L))-real(112667.L))+
        real(131157.L))-real(28329.L))-real(150195.L))+real(199614.L))-
        real(25194.L))-real(130458.L))+real(75582.L))/real(37748736.L);
      _C3x[207] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(9350.L)-
        real(759.L)*_n)*_n-real(49896.L))+real(146354.L))-real(237542.L))+
        real(145446.L))+real(176192.L))-real(374794.L))+real(106191.L))+
        real(271320.L))-real(140148.L))-real(197676.L))+real(145962.L))/
        real(75497472.L);
      _C3x[208] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(58259.L)-
        real(5175.L)*_n)*_n-real(276985.L))+real(689963.L))-real(833115.L))+
        real(23288.L))+real(1183630.L))-real(965518.L))-real(626460.L))+
        real(895717.L))+real(598395.L))-real(756789.L))-real(485826.L))+
        real(500616.L))/real(301989888.L);
      _C3x[209] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(163540.L)-real(15795.L)*_n)*_n-real(696752.L))+real(1472490.L))-
        real(1226872.L))-real(898878.L))+real(2433381.L))-real(438036.L))-
        real(1945622.L))+real(191768.L))+real(1865333.L))+real(243618.L))-
        real(1351563.L))-real(734502.L))+real(937890.L))/real(603979776.L);
      _C3x[210] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(3411135.L)*_n-real(13084435.L))+real(23362279.L))-
        real(11260659.L))-real(24451713.L))+real(30899865.L))+real(11812947.L))-
        real(24588885.L))-real(17208913.L))+real(17574277.L))+real(21212775.L))-
        real(2692731.L))-real(20824833.L))-real(7093863.L))+real(13289019.L))/
        real(9663676416.L);
      _C3x[211] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(43689698.L)*_n-real(5958730.L))-real(58684318.L))+
        real(38121490.L))+real(43713954.L))-real(21361844.L))-real(48836030.L))-
        real(1170036.L))+real(39774850.L))+real(29716070.L))-real(17526366.L))-
        real(34032630.L))-real(10553214.L))+real(24826167.L))/
        real(19327352832.L);
      _C3x[212] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(57060070.L)-
        real(239896252.L)*_n)*_n+real(192008346.L))+real(23738214.L))-
        real(167965936.L))-real(108508906.L))+real(65950812.L))+
        real(158255466.L))+real(54741110.L))-real(79830414.L))-
        real(121894878.L))-real(23024433.L))+real(89649252.L))/
        real(77309411328.L);
      _C3x[213] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(332237524.L)*_n+
        real(202527988.L))-real(195238788.L))-real(292948708.L))-
        real(69309528.L))+real(227507784.L))+real(239636144.L))+
        real(43955772.L))-real(186962964.L))-real(195267303.L))-
        real(32022468.L))+real(167997537.L))/real(154618822656.L);
      _C3x[214] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(316302970.L)*_n-
        real(2092640964.L))*_n-real(1624423608.L))+real(567080486.L))+
        real(1825748704.L))+real(1508133060.L))-real(223064628.L))-
        real(1402356933.L))-real(1367776200.L))-real(88340250.L))+
        real(1229783994.L))/real(1236950581248.L);
      _C3x[215] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(4005995580.L)*_n-
        real(1163333234.L))*_n+real(2074189376.L))+real(3614434826.L))+
        real(1812341744.L))-real(887723115.L))-real(2819570688.L))-
        real(2178883707.L))-real(70993392.L))+real(2314640178.L))/
        real(2473901162496.L);
      _C3x[216] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(495196954.L)*_n+
        real(11326346604.L))+real(11510043704.L))+real(4600090009.L))-
        real(5699461548.L))-real(10021567602.L))-real(7579122078.L))+
        real(482976333.L))+real(8558861700.L))/real(9895604649984.L);
      _C3x[217] = (_n*(_n*(_n*(_n*(_n*(_n*(real(21876864256.L)*_n+
        real(19375684513.L))+real(2685032896.L))-real(12164328567.L))-
        real(19087008972.L))-real(12036041373.L))+real(1328161404.L))+
        real(16180133319.L))/real(19791209299968.L);
      _C3x[218] = (_n*(_n*(_n*(_n*(_n*(real(212797419253.L)*_n-
        real(5881704531.L))-real(221254014069.L))-real(265773148287.L))-
        real(167219284173.L))+real(35702422005.L))+real(241259465295.L))/
        real(316659348799488.L);
      _C3x[219] = (_n*(_n*(_n*((-real(134352556562.L)*_n-real(425462412414.L))*
        _n-real(494403318726.L))-real(264259491534.L))+real(74967311898.L))+
        real(457950678939.L))/real(633318697598976.L);
      _C3x[220] = (_n*(_n*((-real(1750233030484.L)*_n-real(1705456537296.L))*_n-
        real(917772006792.L))+real(369214691208.L))+real(1718378129115.L))/
        real(2533274790395904.L);
      _C3x[221] = (_n*((-real(130757894012.L)*_n-real(59879334075.L))*_n+
        real(30560635910.L))+real(136405425687.L))/real(211106232532992.L);
      _C3x[222] = ((real(6537215121417.L)-real(9974798133225.L)*_n)*_n+
        real(24702973639945.L))/real(40532396646334464.L);
      _C3x[223] = (real(1413470393922.L)*_n+real(5246222852159.L))/
        real(9007199254740992.L);
      _C3x[224] = real(310707493777.L)/real(562949953421312.L);
      _C3x[225] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((_n-real(19.L))*_n+
        real(170.L))-real(950.L))+real(3705.L))-real(10659.L))+real(23256.L))-
        real(38760.L))+real(48450.L))-real(41990.L))+real(16796.L))/
        real(10485760.L);
      _C3x[226] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(10.L)*_n-
        real(171.L))+real(1350.L))-real(6479.L))+real(20870.L))-real(46645.L))+
        real(70794.L))-real(62985.L))+real(3876.L))+real(74290.L))-
        real(96900.L))+real(41990.L))/real(20971520.L);
      _C3x[227] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(115.L)*_n-
        real(1786.L))+real(12559.L))-real(52192.L))+real(139141.L))-
        real(236152.L))+real(214809.L))+real(24624.L))-real(329802.L))+
        real(349486.L))-real(4522.L))-real(251940.L))+real(135660.L))/
        real(83886080.L);
      _C3x[228] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(250.L)*
        _n-real(3553.L))+real(22425.L))-real(81173.L))+real(178529.L))-
        real(218584.L))+real(57103.L))+real(247260.L))-real(337423.L))+
        real(24719.L))+real(276146.L))-real(101099.L))-real(197030.L))+
        real(132430.L))/real(83886080.L);
      _C3x[229] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(14625.L)*_n-real(191425.L))+real(1091292.L))-real(3454354.L))+
        real(6209363.L))-real(4870931.L))-real(3209950.L))+real(10153792.L))-
        real(4712349.L))-real(6716637.L))+real(5468074.L))+real(5746550.L))-
        real(5100075.L))-real(4110175.L))+real(3682200.L))/real(2684354560.L);
      _C3x[230] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(2980354.L)-real(575757.L)*_n)*_n-real(8268441.L))+
        real(11934126.L))-real(4107621.L))-real(13286702.L))+real(16812235.L))+
        real(3440642.L))-real(14885329.L))-real(3218218.L))+real(13811795.L))+
        real(4488386.L))-real(9739457.L))-real(6386090.L))+real(6952575.L))/
        real(5368709120.L);
      _C3x[231] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(41224629.L)-real(36685396.L)*_n)*_n+real(5919784.L))-
        real(64785587.L))+real(37953524.L))+real(44317207.L))-real(31628248.L))-
        real(46473241.L))+real(19931624.L))+real(45301625.L))+real(3758668.L))-
        real(39210607.L))-real(16650840.L))+real(24860550.L))/
        real(21474836480.L);
      _C3x[232] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(24558558.L)*
        _n-real(63134341.L))+real(7172062.L))+real(51215875.L))+
        real(3073898.L))-real(47531759.L))-real(18173592.L))+real(31720101.L))+
        real(35985854.L))-real(9668041.L))-real(33402222.L))-real(12817385.L))+
        real(23357270.L))/real(21474836480.L);
      _C3x[233] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(662720255.L)-
        real(275739664.L)*_n)*_n+real(455754408.L))-real(452593555.L))-
        real(595784942.L))+real(27207641.L))+real(596442152.L))+
        real(346569259.L))-real(226480626.L))-real(491143742.L))-
        real(128177010.L))+real(339753540.L))/real(343597383680.L);
      _C3x[234] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(636237516.L)*_n-
        real(59443911.L))-real(573191205.L))-real(365846025.L))+
        real(287585989.L))+real(515311115.L))+real(212001921.L))-
        real(304725172.L))-real(406999182.L))-real(95813570.L))+
        real(319762345.L))/real(343597383680.L);
      _C3x[235] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(2794265030.L)*_n-
        real(4243378712.L))*_n-real(653774542.L))+real(2958697684.L))+
        real(3605909354.L))+real(476022861.L))-real(2448388238.L))-
        real(2904705599.L))-real(415755900.L))+real(2353469524.L))/
        real(2748779069440.L);
      _C3x[236] = (_n*(_n*(_n*(_n*(_n*(_n*((real(675800940.L)-
        real(2696677226.L)*_n)*_n+real(3412012762.L))+real(2582269126.L))-
        real(117369842.L))-real(2590977593.L))-real(2387267425.L))-
        real(279147385.L))+real(2222581950.L))/real(2748779069440.L);
      _C3x[237] = (_n*(_n*(_n*(_n*(_n*(_n*(real(59815399410.L)*_n+
        real(98226145519.L))+real(62281465759.L))-real(25033837790.L))-
        real(76735647798.L))-real(67434894591.L))-real(2267641835.L))+
        real(66031883380.L))/real(87960930222080.L);
      _C3x[238] = (_n*(_n*(_n*(_n*(_n*(real(181849615997.L)*_n+
        real(71599805038.L))-real(65569161565.L))-real(150485430726.L))-
        real(110515260937.L))-real(37069570.L))+real(125183102155.L))/
        real(175921860444160.L);
      _C3x[239] = (_n*(_n*(_n*(_n*(real(168609607336.L)*_n-
        real(343735176087.L))-real(540094461028.L))-real(388906898435.L))+
        real(32479714380.L))+real(468286085070.L))/real(703687441776640.L);
      _C3x[240] = (_n*(_n*((-real(354911705830.L)*_n-real(512388782813.L))*_n-
        real(317944157044.L))+real(41045837835.L))+real(445471143620.L))/
        real(703687441776640.L);
      _C3x[241] = (_n*((-real(7253974348494.L)*_n-real(4472224828261.L))*_n+
        real(982911272875.L))+real(6705646800610.L))/real(11258999068426240.L);
      _C3x[242] = ((real(2069830745930.L)-real(7284964906539.L)*_n)*_n+
        real(12799705021845.L))/real(22517998136852480.L);
      _C3x[243] = (real(992966911831.L)*_n+real(4841011900185.L))/
        real(9007199254740992.L);
      _C3x[244] = real(9267877312991.L)/real(18014398509481984.L);
      _C3x[245] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(21.L)-_n)*_n-
        real(209.L))+real(1309.L))-real(5775.L))+real(19019.L))-real(48279.L))+
        real(95931.L))-real(149226.L))+real(177650.L))-real(149226.L))+
        real(58786.L))/real(46137344.L);
      _C3x[246] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(210.L)-
        real(11.L)*_n)*_n-real(1870.L))+real(10262.L))-real(38544.L))+
        real(103642.L))-real(200970.L))+real(269610.L))-real(206283.L))-
        real(28424.L))+real(298452.L))-real(355300.L))+real(149226.L))/
        real(92274688.L);
      _C3x[247] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(4809.L)-
        real(275.L)*_n)*_n-real(38619.L))+real(187059.L))-real(600734.L))+
        real(1310402.L))-real(1853478.L))+real(1290806.L))+real(724339.L))-
        real(2725569.L))+real(2408307.L))+real(245157.L))-real(1932832.L))+
        real(980628.L))/real(738197504.L);
      _C3x[248] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(20748.L)-real(1287.L)*_n)*_n-real(151250.L))+real(650322.L))-
        real(1788277.L))+real(3114546.L))-real(2872764.L))-real(412620.L))+
        real(4609913.L))-real(4584360.L))-real(647174.L))+real(4316686.L))-
        real(1081993.L))-real(3105322.L))+real(1932832.L))/real(1476395008.L);
      _C3x[249] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(300321.L)*_n-real(1998361.L))+real(7660305.L))-real(18010915.L))+
        real(24284837.L))-real(10011949.L))-real(23317551.L))+real(38530187.L))-
        real(7375295.L))-real(30124457.L))+real(15017321.L))+real(25258189.L))-
        real(16829307.L))-real(16953453.L))+real(13590225.L))/
        real(11811160064.L);
      _C3x[250] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(20156670.L)*_n-real(40336028.L))+real(39676550.L))+
        real(8306300.L))-real(66212650.L))+real(48537730.L))+real(34314526.L))-
        real(50155370.L))-real(27614306.L))+real(48154612.L))+real(25663814.L))-
        real(34398980.L))-real(26948042.L))+real(25840133.L))/
        real(23622320128.L);
      _C3x[251] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(104153330.L)*_n+real(127365836.L))-real(262694988.L))+
        real(43716997.L))+real(211973888.L))-real(46671064.L))-
        real(203959086.L))+real(20997261.L))+real(178886686.L))+
        real(46046000.L))-real(144887864.L))-real(74137635.L))+
        real(93164676.L))/real(94489280512.L);
      _C3x[252] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(411410000.L)*_n-
        real(160724471.L))*_n+real(361492536.L))+real(188744355.L))-
        real(309505556.L))-real(243226535.L))+real(172668364.L))+
        real(313557255.L))-real(19488744.L))-real(256301683.L))-
        real(117080656.L))+real(175992267.L))/real(188978561024.L);
      _C3x[253] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1506793190.L)*_n+
        real(2637181856.L))-real(674535472.L))-real(2539873798.L))-
        real(750167476.L))+real(2037031104.L))+real(1745497468.L))-
        real(534656133.L))-real(1932545296.L))-real(633075718.L))+
        real(1288028302.L))/real(1511828488192.L);
      _C3x[254] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1972567620.L)*_n-
        real(3502706666.L))-real(4082510392.L))+real(818295818.L))+
        real(3983479400.L))+real(2493168689.L))-real(1840890128.L))-
        real(3297474455.L))-real(984759072.L))+real(2434513950.L))/
        real(3023656976384.L);
      _C3x[255] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(34214109966.L)*_n-
        real(17209880808.L))*_n+real(15486859040.L))+real(30592744379.L))+
        real(10296063547.L))-real(16150854049.L))-real(23958254331.L))-
        real(5005992024.L))+real(18006280446.L))/real(24189255811072.L);
      _C3x[256] = (_n*(_n*(_n*(_n*(_n*((real(45022865775.L)-real(11894936144.L)*
        _n)*_n+real(48902957740.L))+real(9804898510.L))-real(36458456034.L))-
        real(40401140725.L))-real(7455320202.L))+real(34121884896.L))/
        real(48378511622144.L);
      _C3x[257] = (_n*(_n*(_n*(_n*(_n*(real(367058948627.L)*_n+
        real(324075281367.L))-real(17040987755.L))-real(281802112383.L))-
        real(289365429727.L))-real(31435288093.L))+real(254437375973.L))/
        real(387028092977152.L);
      _C3x[258] = (_n*(_n*(_n*(_n*(real(451346074082.L)*_n-
        real(124741142592.L))-real(571193854066.L))-real(485934496080.L))-
        real(39721774466.L))+real(483654223701.L))/real(774056185954304.L);
      _C3x[259] = (_n*(_n*((-real(83454510742.L)*_n-real(191937711982.L))*_n-
        real(157335169436.L))-real(1485684057.L))+real(165010560036.L))/
        real(281474976710656.L);
      _C3x[260] = (_n*((-real(372040841280.L)*_n-real(263720191503.L))*_n+
        real(6330619896.L))+real(314637345025.L))/real(562949953421312.L);
      _C3x[261] = ((real(184441277827.L)-real(1875159093121.L)*_n)*_n+
        real(2374498758045.L))/real(4503599627370496.L);
      _C3x[262] = (real(446089464258.L)*_n+real(4540838699861.L))/
        real(9007199254740992.L);
      _C3x[263] = real(17213216597955.L)/real(36028797018963968.L);
      _C3x[264] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((_n-real(23.L))*_n+
        real(252.L))-real(1748.L))+real(8602.L))-real(31878.L))+real(92092.L))-
        real(211508.L))+real(389367.L))-real(572033.L))+real(653752.L))-
        real(534888.L))+real(208012.L))/real(201326592.L);
      _C3x[265] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(12.L)*
        _n-real(253.L))+real(2508.L))-real(15479.L))+real(66296.L))-
        real(207414.L))+real(483736.L))-real(834394.L))+real(1004916.L))-
        real(658559.L))-real(249964.L))+real(1181211.L))-real(1307504.L))+
        real(534888.L))/real(402653184.L);
      _C3x[266] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(162.L)*_n-real(3151.L))+real(28451.L))-real(157250.L))+
        real(589011.L))-real(1554985.L))+real(2878330.L))-real(3451380.L))+
        real(1720400.L))+real(2268145.L))-real(5423055.L))+real(4094690.L))+
        real(919885.L))-real(3692213.L))+real(1782960.L))/real(1610612736.L);
      _C3x[267] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(121176.L)-real(14651.L)*_n)*_n-real(602911.L))+real(1981002.L))-
        real(4392123.L))+real(6229890.L))-real(4125455.L))-real(3090720.L))+
        real(9748665.L))-real(7382540.L))-real(2899035.L))+real(8210310.L))-
        real(1258123.L))-real(6069930.L))+real(3543633.L))/real(3221225472.L);
      _C3x[268] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(22411978.L)-real(7765422.L)*_n)*_n-real(41309194.L))+
        real(41738388.L))-real(89220.L))-real(60325345.L))+real(66668860.L))+
        real(5014995.L))-real(61442890.L))+real(17262880.L))+real(52691390.L))-
        real(27155870.L))-real(34379664.L))+real(25165956.L))/
        real(25769803776.L);
      _C3x[269] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(25393368.L)-
        real(42320204.L)*_n)*_n+real(30201445.L))-real(68744424.L))+
        real(26956276.L))+real(47094413.L))-real(37345995.L))-real(37991705.L))+
        real(39767782.L))+real(31247570.L))-real(29853701.L))-real(27860130.L))+
        real(24075204.L))/real(25769803776.L);
      _C3x[270] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(200558217.L)*_n-
        real(220720771.L))-real(52928069.L))+real(207051666.L))+
        real(27757634.L))-real(198525719.L))-real(30437604.L))+
        real(166989480.L))+real(71325415.L))-real(131847753.L))-
        real(79795809.L))+real(87440158.L))/real(103079215104.L);
      _C3x[271] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(246454750.L)-
        real(306925896.L)*_n)*_n+real(299413923.L))-real(206419186.L))-
        real(301095722.L))+real(87667434.L))+real(319416873.L))+
        real(31563494.L))-real(241568494.L))-real(128598014.L))+
        real(165979293.L))/real(206158430208.L);
      _C3x[272] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(11216502158.L)*_n+
        real(1728199852.L))-real(9241846668.L))-real(5735439412.L))+
        real(6286338280.L))+real(7863903603.L))-real(734752241.L))-
        real(7469114288.L))-real(2929925380.L))+real(4886425026.L))/
        real(6597069766656.L);
      _C3x[273] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(7578673874.L)*_n-
        real(18205621528.L))*_n-real(2346001508.L))+real(14136726404.L))+
        real(12332563557.L))-real(4967567276.L))-real(13077992689.L))-
        real(4681005600.L))+real(9271254276.L))/real(13194139533312.L);
      _C3x[274] = (_n*(_n*(_n*(_n*(_n*((real(12575704206.L)-real(51033493666.L)*
        _n)*_n+real(59718172567.L))+real(31281974437.L))-real(24877719678.L))-
        real(48333688039.L))-real(12934264661.L))+real(34439562144.L))/
        real(52776558133248.L);
      _C3x[275] = (_n*(_n*(_n*(_n*(_n*(real(63207542601.L)*_n+
        real(104113598928.L))+real(40341888927.L))-real(61302821106.L))-
        real(83307817611.L))-real(20216477698.L))+real(65464455585.L))/
        real(105553116266496.L);
      _C3x[276] = (_n*(_n*(_n*(_n*(real(370492702933.L)*_n+real(61228817295.L))-
        real(248814482588.L))-real(302356197395.L))-real(51602035224.L))+
        real(244959842126.L))/real(422212465065984.L);
      _C3x[277] = (_n*(_n*(_n*(real(12372549891.L)*_n-real(524160176530.L))-
        real(518037166452.L))-real(75980818842.L))+real(466818083220.L))/
        real(844424930131968.L);
      _C3x[278] = (_n*((-real(1996652856445.L)*_n-real(1865791589824.L))*_n-
        real(151972497120.L))+real(1757178468582.L))/real(3377699720527872.L);
      _C3x[279] = ((-real(1063027613707.L)*_n-real(57838923424.L))*_n+
        real(1119201890640.L))/real(2251799813685248.L);
      _C3x[280] = (real(446447667014.L)*_n+real(50805589009803.L))/
        real(108086391056891904.L);
      _C3x[281] = real(16221720992423.L)/real(36028797018963968.L);
      _C3x[282] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(25.L)-_n)*
        _n-real(299.L))+real(2275.L))-real(12350.L))+real(50830.L))-
        real(164450.L))+real(427570.L))-real(904475.L))+real(1562275.L))-
        real(2187185.L))+real(2414425.L))-real(1931540.L))+real(742900.L))/
        real(872415232.L);
      _C3x[283] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(300.L)-
        real(13.L)*_n)*_n-real(3276.L))+real(22450.L))-real(107861.L))+
        real(383890.L))-real(1040000.L))+real(2158780.L))-real(3371225.L))+
        real(3683680.L))-real(2039180.L))-real(1420250.L))+real(4630015.L))-
        real(4828850.L))+real(1931540.L))/real(1744830464.L);
      _C3x[284] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(8075.L)*_n-real(80975.L))+real(502623.L))-real(2147433.L))+
        real(6621687.L))-real(14928615.L))+real(24005475.L))-real(24563565.L))+
        real(7261215.L))+real(23039445.L))-real(42091725.L))+real(27515475.L))+
        real(9970155.L))-real(28149355.L))+real(13037895.L))/
        real(13958643712.L);
      _C3x[285] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(2104026.L)*_n-real(8021776.L))+real(21419794.L))-
        real(39657388.L))+real(46313010.L))-real(18895500.L))-real(39083070.L))+
        real(77556180.L))-real(44876910.L))-real(32722560.L))+real(61292010.L))-
        real(3988660.L))-real(47209110.L))+real(26104195.L))/
        real(27917287424.L);
      _C3x[286] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(115415958.L)*_n-real(171259312.L))+real(123396800.L))+
        real(73063651.L))-real(268623624.L))+real(211690128.L))+
        real(83307510.L))-real(235701765.L))+real(23761530.L))+
        real(212511260.L))-real(85340580.L))-real(137813585.L))+
        real(93509260.L))/real(111669149696.L);
      _C3x[287] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(64193336.L)*_n+
        real(372503147.L))-real(504706752.L))+real(48950133.L))+
        real(423583812.L))-real(187611177.L))-real(355652700.L))+
        real(248051245.L))+real(280509840.L))-real(203898565.L))-
        real(227156280.L))+real(179924745.L))/real(223338299392.L);
      _C3x[288] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(2432194332.L)*_n-
        real(2008438660.L))*_n+real(2794677696.L))+real(1418942328.L))-
        real(2829240720.L))-real(1157295672.L))+real(2383906980.L))+
        real(1459577730.L))-real(1895592725.L))-real(1343118465.L))+
        real(1315544685.L))/real(1786706395136.L);
      _C3x[289] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1667557336.L)*_n+
        real(5652102444.L))-real(1495648992.L))-real(5175513024.L))+
        real(113261616.L))+real(4963996830.L))+real(1203201350.L))-
        real(3592582695.L))-real(2201348630.L))+real(2508249705.L))/
        real(3573412790272.L);
      _C3x[290] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(21346023138.L)*_n-
        real(29192992584.L))-real(30575586912.L))+real(16916968029.L))+
        real(33093230235.L))+real(2168937615.L))-real(28465619715.L))-
        real(13020909590.L))+real(18554938350.L))/real(28587302322176.L);
      _C3x[291] = (_n*(_n*(_n*(_n*(_n*((-real(70894213840.L)*_n-
        real(28489507799.L))*_n+real(46164901068.L))+real(55630104244.L))-
        real(10688422706.L))-real(51032159763.L))-real(21224059610.L))+
        real(35330117550.L))/real(57174604644352.L);
      _C3x[292] = (_n*(_n*(_n*(_n*((real(435915317669.L)-real(43359239675.L)*
        _n)*_n+real(316340447943.L))-real(138828099527.L))-
        real(383542159013.L))-real(123646495947.L))+real(263550729195.L))/
        real(457396837154816.L);
      _C3x[293] = (_n*(_n*(_n*(_n*(real(63319218806.L)*_n+real(35533884650.L))-
        real(30091079262.L))-real(51844082046.L))-real(15314049870.L))+
        real(38647569279.L))/real(70368744177664.L);
      _C3x[294] = (_n*(_n*(_n*(real(80687574846.L)*_n-real(129801737586.L))-
        real(190580127312.L))-real(42850331919.L))+real(145097370796.L))/
        real(281474976710656.L);
      _C3x[295] = (_n*((-real(286639949368.L)*_n-real(332189237881.L))*_n-
        real(67008401568.L))+real(277172296351.L))/real(562949953421312.L);
      _C3x[296] = ((-real(1209108246262.L)*_n-real(170172833119.L))*_n+
        real(1046221099377.L))/real(2251799813685248.L);
      _C3x[297] = (real(2003125009747.L)-real(245906210356.L)*_n)/
        real(4503599627370496.L);
      _C3x[298] = real(15190705138949.L)/real(36028797018963968.L);
      _C3x[299] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((_n-
        real(27.L))*_n+real(350.L))-real(2898.L))+real(17199.L))-real(77805.L))+
        real(278460.L))-real(807300.L))+real(1924065.L))-real(3798795.L))+
        real(6216210.L))-real(8351070.L))+real(8947575.L))-real(7020405.L))+
        real(2674440.L))/real(3758096384.L);
      _C3x[300] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        ((real(4186.L)-real(351.L)*_n)*_n-real(31527.L))+real(167790.L))-
        real(668367.L))+real(2055690.L))-real(4950855.L))+real(9316710.L))-
        real(13342875.L))+real(13320450.L))-real(6059235.L))-real(6969690.L))+
        real(18032805.L))-real(17895150.L))+real(7020405.L))/
        real(7516192768.L);
      _C3x[301] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(1847371.L)-
        real(385352.L)*_n)*_n-real(6497740.L))+real(17148019.L))-
        real(33865272.L))+real(48041487.L))-real(41824380.L))+real(2765295.L))+
        real(53066520.L))-real(80250105.L))+real(45662820.L))+real(23881935.L))-
        real(53593680.L))+real(23951970.L))/real(30064771072.L);
      _C3x[302] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(26526277.L)-
        real(11773326.L)*_n)*_n-real(41897402.L))+real(40109813.L))-
        real(4977966.L))-real(49642983.L))+real(73842990.L))-real(31583175.L))-
        real(39519750.L))+real(56398185.L))+real(1077090.L))-real(45732855.L))+
        real(24135510.L))/real(30064771072.L);
      _C3x[303] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(283499531.L)-
        real(649834702.L)*_n)*_n+real(536364048.L))-real(1091940219.L))+
        real(607260174.L))+real(522459795.L))-real(863934360.L))-
        real(60094815.L))+real(837163890.L))-real(258919395.L))-
        real(547849995.L))+real(348527970.L))/real(481036337152.L);
      _C3x[304] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1806590442.L)*_n-
        real(1663844422.L))-real(379236654.L))+real(1705094730.L))-
        real(319841754.L))-real(1525537518.L))+real(718654350.L))+
        real(1198505910.L))-real(684505185.L))-real(916478010.L))+
        real(674048235.L))/real(962072674304.L);
      _C3x[305] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(2025323859.L)-
        real(2724276899.L)*_n)*_n+real(2128566621.L))-real(2324152365.L))-
        real(1668127191.L))+real(2041044360.L))+real(1701264990.L))-
        real(1684197270.L))-real(1390153695.L))+real(1239370755.L))/
        real(1924145348608.L);
      _C3x[306] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(5721023454.L)*_n+
        real(220513290.L))-real(5079273726.L))-real(1028473458.L))+
        real(4645483323.L))+real(1784234052.L))-real(3300848565.L))-
        real(2311480500.L))+real(2372628825.L))/real(3848290697216.L);
      _C3x[307] = (_n*(_n*(_n*(_n*(_n*((-real(76965532978.L)*_n-
        real(139920418129.L))*_n+real(34764704609.L))+real(132826591576.L))+
        real(26730156678.L))-real(107262807981.L))-real(56299660305.L))+
        real(70532356860.L))/real(123145302310912.L);
      _C3x[308] = (_n*(_n*(_n*(_n*((real(136301132102.L)-real(172847147773.L)*
        _n)*_n+real(236611923917.L))-real(8702357306.L))-real(196589634339.L))-
        real(93249400230.L))+real(134741390385.L))/real(246290604621824.L);
      _C3x[309] = (_n*(_n*(_n*(_n*(real(746714952784.L)*_n+
        real(727351416229.L))-real(160669267992.L))-real(750864357495.L))-
        real(281865586632.L))+real(504445199706.L))/real(985162418487296.L);
      _C3x[310] = (_n*(_n*(_n*(real(572596650584.L)*_n-real(290583351929.L))-
        real(671380143616.L))-real(231716125025.L))+real(482124248466.L))/
        real(985162418487296.L);
      _C3x[311] = (_n*((-real(2727777187263.L)*_n-real(4996962842305.L))*_n-
        real(1370255259583.L))+real(3631326099394.L))/real(7881299347898368.L);
      _C3x[312] = ((-real(1263312631342.L)*_n-real(316647060562.L))*_n+
        real(993200127657.L))/real(2251799813685248.L);
      _C3x[313] = (real(13155294584639.L)-real(3120509753760.L)*_n)/
        real(31525197391593472.L);
      _C3x[314] = real(112658685443.L)/real(281474976710656.L);
      _C3x[315] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(29.L)*_n-real(405.L))+real(3625.L))-real(23345.L))+
        real(115101.L))-real(451269.L))+real(1442025.L))-real(3817125.L))+
        real(8454225.L))-real(15737865.L))+real(24582285.L))-real(31865925.L))+
        real(33266625.L))-real(25662825.L))+real(9694845.L))/
        real(16106127360.L);
      _C3x[316] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*
        (real(43094.L)*_n-real(251530.L))+real(1107510.L))-real(3804626.L))+
        real(10385190.L))-real(22658454.L))+real(39189150.L))-real(51969450.L))+
        real(47603790.L))-real(16908450.L))-real(31795890.L))+real(69934950.L))-
        real(66533250.L))+real(25662825.L))/real(32212254720.L);
      _C3x[317] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(11959862.L)*
        _n-real(36019706.L))+real(83522494.L))-real(147083592.L))+
        real(185898816.L))-real(135756192.L))-real(30152460.L))+
        real(231051990.L))-real(302090970.L))+real(149494710.L))+
        real(107313630.L))-real(204001950.L))+real(88394175.L))/
        real(128849018880.L);
      _C3x[318] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(40957510.L)*_n-
        real(55870711.L))+real(42830158.L))+real(10489532.L))-real(75634668.L))+
        real(90632685.L))-real(26106960.L))-real(58821570.L))+real(68440870.L))+
        real(7043520.L))-real(58929450.L))+real(29864925.L))/
        real(42949672960.L);
      _C3x[319] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(252809464.L)*_n+
        real(2880805572.L))-real(4156616400.L))+real(1485669420.L))+
        real(2614417860.L))-real(3049012440.L))-real(757695180.L))+
        real(3242059350.L))-real(745922775.L))-real(2164731825.L))+
        real(1302801075.L))/real(2061584302080.L);
      _C3x[320] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((-real(1620475152.L)*_n-
        real(1108038704.L))*_n+real(2116310728.L))+real(90924860.L))-
        real(2062327576.L))+real(617463534.L))+real(1651903510.L))-
        real(751262545.L))-real(1223044550.L))+real(843771675.L))/
        real(1374389534720.L);
      _C3x[321] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(9260759332.L)*_n+
        real(20594887944.L))-real(13937053160.L))-real(16234003844.L))+
        real(13394690955.L))+real(15015391074.L))-real(11843027685.L))-
        real(11373753600.L))+real(9358276800.L))/real(16492674416640.L);
      _C3x[322] = (_n*(_n*(_n*(_n*(_n*(_n*(real(6789452600.L)*_n-
        real(18600270446.L))-real(7942929306.L))+real(16848959603.L))+
        real(9023494101.L))-real(12009375687.L))-real(9575143875.L))+
        real(8991175950.L))/real(16492674416640.L);
      _C3x[323] = (_n*(_n*(_n*(_n*((real(15799330939.L)-real(584759046085.L)*
        _n)*_n+real(514339794913.L))+real(169243347297.L))-
        real(400371432543.L))-real(238703136705.L))+real(268417026225.L))/
        real(527765581332480.L);
      _C3x[324] = (_n*(_n*(_n*(_n*(real(342834385826.L)*_n+
        real(965390854428.L))+real(88364795390.L))-real(749421605860.L))-
        real(400666009410.L))+real(514338053985.L))/real(1055531162664960.L);
      _C3x[325] = (_n*(_n*(_n*(real(3151890040866.L)*_n-real(201439301326.L))-
        real(2908612257078.L))-real(1245201885305.L))+real(1932221466180.L))/
        real(4222124650659840.L);
      _C3x[326] = (_n*((-real(1537470445268.L)*_n-real(5286374681359.L))*_n-
        real(2080401399700.L))+real(3702812593665.L))/real(8444249301319680.L);
      _C3x[327] = ((-real(6635920896866.L)*_n-real(2128519568765.L))*_n+
        real(4661531863885.L))/real(11258999068426240.L);
      _C3x[328] = (real(5366286770631.L)-real(2112965972644.L)*_n)/
        real(13510798882111488.L);
      _C3x[329] = real(3392857970671.L)/real(9007199254740992.L);
      _C3x[330] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(15500.L)-
        real(2232.L)*_n)*_n-real(82708.L))+real(352408.L))-real(1229832.L))+
        real(3576222.L))-real(8765250.L))+real(18231720.L))-real(32256120.L))+
        real(48384180.L))-real(60790380.L))+real(62031000.L))-real(47143560.L))+
        real(17678835.L))/real(34359738368.L);
      _C3x[331] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(1669040.L)-
        real(440386.L)*_n)*_n-real(5079350.L))+real(12542848.L))-
        real(25144131.L))+real(40390272.L))-real(49961925.L))+real(42073200.L))-
        real(10545270.L))-real(34737360.L))+real(67613790.L))-real(62031000.L))+
        real(23571780.L))/real(34359738368.L);
      _C3x[332] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(96972898.L)-
        real(46712908.L)*_n)*_n-real(154238733.L))+real(174649629.L))-
        real(103732014.L))-real(65388765.L))+real(242500755.L))-
        real(281539830.L))+real(120340140.L))+real(115997970.L))-
        real(194157030.L))+real(81880920.L))/real(137438953472.L);
      _C3x[333] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(185243476.L)-
        real(319425953.L)*_n)*_n+real(153035871.L))-real(486511830.L))+
        real(486541497.L))-real(72220266.L))-real(375786495.L))+
        real(370405980.L))+real(70095030.L))-real(341170500.L))+
        real(166863390.L))/real(274877906944.L);
      _C3x[334] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(3370444930.L)*_n-
        real(3748297420.L))+real(614678664.L))+real(2940017680.L))-
        real(2596952088.L))-real(1178980065.L))+real(3098731635.L))-
        real(492634020.L))-real(2128903920.L))+real(1220770080.L))/
        real(2199023255552.L);
      _C3x[335] = (_n*(_n*(_n*(_n*(_n*(_n*((real(5535914528.L)-
        real(4611081546.L)*_n)*_n+real(1597875408.L))-real(6026446500.L))+
        real(925958313.L))+real(5005621212.L))-real(1811480505.L))-
        real(3650982840.L))+real(2381990400.L))/real(4398046511104.L);
      _C3x[336] = (_n*(_n*(_n*(_n*(_n*(_n*(real(22283354094.L)*_n-
        real(9139613254.L))-real(18068838281.L))+real(10444117217.L))+
        real(15976483206.L))-real(10297013847.L))-real(11527530885.L))+
        real(8848857000.L))/real(17592186044416.L);
      _C3x[337] = (_n*(_n*(_n*(_n*((-real(31988983111.L)*_n-
        real(21989102280.L))*_n+real(29676530299.L))+real(21031459518.L))-
        real(21645687783.L))-real(19625232930.L))+real(17062152465.L))/
        real(35184372088832.L);
      _C3x[338] = (_n*(_n*(_n*((real(120889443942.L)-real(23535121849.L)*_n)*_n+
        real(55477610943.L))-real(92640075585.L))-real(62341338396.L))+
        real(63917467893.L))/real(140737488355328.L);
      _C3x[339] = (_n*(_n*(_n*(real(238510687402.L)*_n+real(49382418998.L))-
        real(176994259657.L))-real(105839350910.L))+real(122826579816.L))/
        real(281474976710656.L);
      _C3x[340] = (_n*(_n*(real(51290363939.L)*_n-real(697975437421.L))-
        real(336197745329.L))+real(462894666682.L))/real(1125899906842624.L);
      _C3x[341] = ((-real(1288035906098.L)*_n-real(569014747702.L))*_n+
        real(889178086077.L))/real(2251799813685248.L);
      _C3x[342] = (real(1683573628847.L)-real(897506094655.L)*_n)/
        real(4503599627370496.L);
      _C3x[343] = real(3236675038231.L)/real(9007199254740992.L);
      _C3x[344] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(115940.L)*
        _n-real(533324.L))+real(2017356.L))-real(6388294.L))+real(17147526.L))-
        real(39338442.L))+real(77484810.L))-real(131128140.L))+
        real(189814860.L))-real(231995940.L))+real(231995940.L))-
        real(173996955.L))+real(64822395.L))/real(146028888064.L);
      _C3x[345] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1104840.L)*_n-
        real(3036946.L))+real(6882744.L))-real(12816144.L))+real(19278156.L))-
        real(22368918.L))+real(17314740.L))-real(2373360.L))-real(17368680.L))+
        real(30705345.L))-real(27293640.L))+real(10235115.L))/
        real(17179869184.L);
      _C3x[346] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(102029246.L)*_n-
        real(147941486.L))+real(150350558.L))-real(68253878.L))-
        real(89297670.L))+real(233576382.L))-real(245076390.L))+
        real(89230245.L))+real(115067505.L))-real(173996955.L))+
        real(71645805.L))/real(137438953472.L);
      _C3x[347] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(104413828.L)*_n+
        real(215147812.L))-real(470094292.L))+real(399170384.L))+
        real(463884.L))-real(364998495.L))+real(312015930.L))+real(87571590.L))-
        real(309534690.L))+real(146703315.L))/real(274877906944.L);
      _C3x[348] = (_n*(_n*(_n*(_n*(_n*(_n*((-real(1513155632.L)*_n-
        real(71626492.L))*_n+real(1460015556.L))-real(1003151847.L))-
        real(712641795.L))+real(1379893080.L))-real(128013105.L))-
        real(982018155.L))+real(539669700.L))/real(1099511627776.L);
      _C3x[349] = (_n*(_n*(_n*(_n*(_n*(_n*(real(2128736024.L)*_n+
        real(1259901225.L))-real(2677693076.L))+real(49560971.L))+
        real(2338994826.L))-real(663815307.L))-real(1701995790.L))+
        real(1057102635.L))/real(2199023255552.L);
      _C3x[350] = (_n*(_n*(_n*(_n*((-real(8516465049.L)*_n-real(35796602007.L))*
        _n+real(14292320879.L))+real(31191967407.L))-real(16655962689.L))-
        real(21840324879.L))+real(15777490455.L))/real(35184372088832.L);
      _C3x[351] = (_n*(_n*(_n*((real(47767847424.L)-real(50056173742.L)*_n)*_n+
        real(43901080962.L))-real(36394040796.L))-real(37551598590.L))+
        real(30518839359.L))/real(70368744177664.L);
      _C3x[352] = (_n*(_n*(_n*(real(208539541814.L)*_n+real(124953092706.L))-
        real(160216498650.L))-real(121237396839.L))+real(114736496796.L))/
        real(281474976710656.L);
      _C3x[353] = (_n*(_n*(real(137698286324.L)*_n-real(312369078125.L))-
        real(207884878668.L))+real(221067686111.L))/real(562949953421312.L);
      _C3x[354] = ((-real(1251698398080.L)*_n-real(672205282129.L))*_n+
        real(835594467657.L))/real(2251799813685248.L);
      _C3x[355] = (real(1608685463477.L)-real(1150098738176.L)*_n)/
        real(4503599627370496.L);
      _C3x[356] = real(3053457067201.L)/real(9007199254740992.L);
      _C3x[357] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(10956330.L)-
        real(3199944.L)*_n)*_n-real(31941470.L))+real(80021788.L))-
        real(173309220.L))+real(325436202.L))-real(529555950.L))+
        real(742753800.L))-real(885802680.L))+real(869984775.L))-
        real(644812245.L))+real(238819350.L))/real(618475290624.L);
      _C3x[358] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(530611004.L)-
        real(252273846.L)*_n)*_n-real(925691310.L))+real(1312442908.L))-
        real(1434615210.L))+real(1018397988.L))-real(9628290.L))-
        real(1231045650.L))+real(2012595795.L))-real(1739969550.L))+
        real(644812245.L))/real(1236950581248.L);
      _C3x[359] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(4555551370.L)-
        real(5004223360.L)*_n)*_n-real(1375225472.L))-real(3810143598.L))+
        real(7979926752.L))-real(7659304695.L))+real(2320094250.L))+
        real(4051932345.L))-real(5636756970.L))+real(2272195530.L))/
        real(4947802324992.L);
      _C3x[360] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(4628126648.L)*_n-
        real(7948221378.L))+real(5763280432.L))+real(992967076.L))-
        real(6264254475.L))+real(4713533415.L))+real(1790376480.L))-
        real(5069780145.L))+real(2336397615.L))/real(4947802324992.L);
      _C3x[361] = (_n*(_n*(_n*(_n*(_n*((real(200707555277.L)-
        real(51289545632.L)*_n)*_n-real(107193842745.L))-real(113591548376.L))+
        real(176169339954.L))-real(5145819363.L))-real(130592664135.L))+
        real(69110571060.L))/real(158329674399744.L);
      _C3x[362] = (_n*(_n*(_n*(_n*(_n*(real(225029316355.L)*_n-
        real(335016982702.L))-real(38933502733.L))+real(311769774810.L))-
        real(67157945757.L))-real(228512219706.L))+real(135856345095.L))/
        real(316659348799488.L);
      _C3x[363] = (_n*(_n*(_n*((real(314495779743.L)-real(1231713700496.L)*_n)*
        _n+real(1079454335352.L))-real(480191057193.L))-real(743381985240.L))+
        real(508971814902.L))/real(1266637395197952.L);
      _C3x[364] = (_n*(_n*(_n*(real(672102628636.L)*_n+real(800464776369.L))-
        real(547684062048.L))-real(644784625023.L))+real(493715101662.L))/
        real(1266637395197952.L);
      _C3x[365] = (_n*(_n*(real(1604700872215.L)*_n-real(1656940587997.L))-
        real(1407328813243.L))+real(1241503046346.L))/real(3377699720527872.L);
      _C3x[366] = ((-real(1099001447104.L)*_n-real(811486323394.L))*_n+
        real(799328663817.L))/real(2251799813685248.L);
      _C3x[367] = (real(4544405605851.L)-real(3995012033458.L)*_n)/
        real(13510798882111488.L);
      _C3x[368] = real(1461210130553.L)/real(4503599627370496.L);
      _C3x[369] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(57054074.L)*_n-
        real(154861058.L))+real(365294266.L))-real(752076430.L))+
        real(1353737574.L))-real(2127301902.L))+real(2900866230.L))-
        real(3384343935.L))+real(3270584475.L))-real(2398428615.L))+
        real(883631595.L))/real(2611340115968.L);
      _C3x[370] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(123779652.L)*_n-
        real(203725552.L))+real(273687964.L))-real(282735500.L))+
        real(183212604.L))+real(25446195.L))-real(266436630.L))+
        real(408137010.L))-real(344272050.L))+real(126233085.L))/
        real(274877906944.L);
      _C3x[371] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(1663576684.L)*_n-
        real(210511204.L))-real(1866054300.L))+real(3330058719.L))-
        real(2949064317.L))+real(714788607.L))+real(1744810665.L))-
        real(2263214520.L))+real(895107330.L))/real(2199023255552.L);
      _C3x[372] = (_n*(_n*(_n*(_n*(_n*((real(4006284039.L)-real(6494726920.L)*
        _n)*_n+real(1527836116.L))-real(5240186494.L))+real(3502793478.L))+
        real(1715472699.L))-real(4112304690.L))+real(1848092280.L))/
        real(4398046511104.L);
      _C3x[373] = (_n*(_n*(_n*(_n*(_n*(real(41375557803.L)*_n-
        real(16713828219.L))-real(26560472237.L))+real(34587765501.L))+
        real(1154658519.L))-real(26834758935.L))+real(13730467455.L))/
        real(35184372088832.L);
      _C3x[374] = (_n*(_n*(_n*((-real(63390820022.L)*_n-real(16426349874.L))*_n+
        real(63674363010.L))-real(9770340990.L))-real(47366046318.L))+
        real(27079840719.L))/real(70368744177664.L);
      _C3x[375] = (_n*(_n*(_n*(real(33010336546.L)*_n+real(228008897754.L))-
        real(84361620600.L))-real(156005632095.L))+real(101829086316.L))/
        real(281474976710656.L);
      _C3x[376] = (_n*(_n*(real(352964190344.L)*_n-real(202196269517.L))-
        real(272819932752.L))+real(198096366191.L))/real(562949953421312.L);
      _C3x[377] = ((-real(948664554276.L)*_n-real(904154339527.L))*_n+
        real(749484543777.L))/real(2251799813685248.L);
      _C3x[378] = (real(1451001339947.L)-real(1576416475160.L)*_n)/
        real(4503599627370496.L);
      _C3x[379] = real(11027253546199.L)/real(36028797018963968.L);
      _C3x[380] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*((real(1637526020.L)-
        real(732070456.L)*_n)*_n-real(3223184700.L))+real(5586853480.L))-
        real(8509207608.L))+real(11313378297.L))-real(12940138575.L))+
        real(12323941500.L))-real(8951073300.L))+real(3282060210.L))/
        real(10995116277760.L);
      _C3x[381] = (_n*(_n*(_n*(_n*(_n*(_n*((real(20420912720.L)-
        real(15973658796.L)*_n)*_n-real(19983745140.L))+real(11732392308.L))+
        real(3725622315.L))-real(20605630188.L))+real(29869342425.L))-
        real(24647883000.L))+real(8951073300.L))/real(21990232555520.L);
      _C3x[382] = (_n*(_n*(_n*(_n*(_n*(_n*(real(4739192952.L)*_n-
        real(78713134222.L))+real(124091978953.L))-real(102015059749.L))+
        real(18484714782.L))+real(67175859231.L))-real(82083936675.L))+
        real(31912522200.L))/real(87960930222080.L);
      _C3x[383] = (_n*(_n*(_n*(_n*(_n*(real(121450754525.L)*_n+
        real(79644174768.L))-real(195393281759.L))+real(116627994594.L))+
        real(71611679859.L))-real(150520729710.L))+real(66127675575.L))/
        real(175921860444160.L);
      _C3x[384] = (_n*(_n*(_n*((-real(214208596351.L)*_n-real(541082739303.L))*
        _n+real(609449342820.L))+real(58731314895.L))-real(496985144040.L))+
        real(246699363690.L))/real(703687441776640.L);
      _C3x[385] = (_n*(_n*((real(1164065632618.L)-real(443970412913.L)*_n)*_n-
        real(111394194596.L))-real(884235983710.L))+real(488030079180.L))/
        real(1407374883553280.L);
      _C3x[386] = (_n*(_n*(real(4297898591023.L)*_n-real(1310904290456.L))-
        real(2944700211400.L))+real(1841448661010.L))/real(5629499534213120.L);
      _C3x[387] = ((-real(3332997588781.L)*_n-real(5187672613040.L))*_n+
        real(3591489932760.L))/real(11258999068426240.L);
      _C3x[388] = (real(10901346434011.L)-real(13902928656794.L)*_n)/
        real(36028797018963968.L);
      _C3x[389] = real(10575447871909.L)/real(36028797018963968.L);
      _C3x[390] = (_n*(_n*(_n*(_n*(_n*(_n*(_n*(real(7230307196.L)*_n-
        real(13670748900.L))+real(22906099268.L))-real(33918646993.L))+
        real(44076102777.L))-real(49517596947.L))+real(46539094875.L))-
        real(33508148310.L))+real(12233133510.L))/real(46179488366592.L);
      _C3x[391] = (_n*(_n*(_n*(_n*(_n*(_n*(real(79594138040.L)*_n-
        real(73897992665.L))+real(38867815500.L))+real(21040444124.L))-
        real(83226221358.L))+real(115044642531.L))-real(93078189750.L))+
        real(33508148310.L))/real(92358976733184.L);
      _C3x[392] = (_n*(_n*(_n*(_n*((real(965405949493.L)-real(677248006993.L)*
        _n)*_n-real(739894554773.L))+real(87436219479.L))+real(540626765511.L))-
        real(627346998915.L))+real(240141729555.L))/real(738871813865472.L);
      _C3x[393] = (_n*(_n*(_n*(_n*(real(776740350354.L)*_n-
        real(1519217204724.L))+real(811262682946.L))+real(614411517160.L))-
        real(1160069278158.L))+real(499271409819.L))/real(1477743627730944.L);
      _C3x[394] = (_n*(_n*((real(4499566403230.L)-real(4527638700610.L)*_n)*_n+
        real(724119676812.L))-real(3871718566765.L))+real(1869926512356.L))/
        real(5910974510923776.L);
      _C3x[395] = (_n*(_n*(real(1271470127648.L)*_n-real(51597036833.L))-
        real(991295676776.L))+real(529940161923.L))/real(1688849860263936.L);
      _C3x[396] = ((-real(795233577023.L)*_n-real(2222831651699.L))*_n+
        real(1337288787659.L))/real(4503599627370496.L);
      _C3x[397] = (real(7843287728577.L)-real(11827401953990.L)*_n)/
        real(27021597764222976.L);
      _C3x[398] = real(9945743757025.L)/real(36028797018963968.L);
      _C3x[399] = (_n*(_n*(_n*(_n*(_n*((real(93401594429.L)-real(57477904264.L)*
        _n)*_n-real(134819496031.L))+real(171588449494.L))-
        real(189650391546.L))+real(176103935007.L))-real(125788525005.L))+
        real(45741281820.L))/real(193514046488576.L);
      _C3x[400] = (_n*(_n*(_n*(_n*((real(11449475582.L)-real(24704740937.L)*_n)*
        _n+real(9558755047.L))-real(30376902542.L))+real(40287513603.L))-
        real(32018897274.L))+real(11435320455.L))/real(35184372088832.L);
      _C3x[401] = (_n*(_n*(_n*(_n*(real(169554155528.L)*_n-
        real(121489410719.L))+real(6333408252.L))+real(98343755913.L))-
        real(109075364340.L))+real(41167153638.L))/real(140737488355328.L);
      _C3x[402] = (_n*(_n*((real(63574719679.L)-real(133207833714.L)*_n)*_n+
        real(58877239676.L))-real(101627745377.L))+real(42926433708.L))/
        real(140737488355328.L);
      _C3x[403] = (_n*(_n*(real(1503383684094.L)*_n+real(343057591489.L))-
        real(1370068675847.L))+real(645421215014.L))/real(2251799813685248.L);
      _C3x[404] = (_n*(real(40279424959.L)*_n-real(2472087778178.L))+
        real(1283746667079.L))/real(4503599627370496.L);
      _C3x[405] = (real(2436782869279.L)-real(4195124656575.L)*_n)/
        real(9007199254740992.L);
      _C3x[406] = real(4774615334345.L)/real(18014398509481984.L);
      _C3x[407] = (_n*(_n*(_n*(_n*(_n*(real(379100589153.L)*_n-
        real(534629035985.L))+real(667646786565.L))-real(726993167593.L))+
        real(667646786565.L))-real(473422266837.L))+real(171529806825.L))/
        real(809240558043136.L);
      _C3x[408] = (_n*(_n*(_n*(_n*(real(17216678754.L)*_n+real(21398507702.L))-
        real(58056242310.L))+real(74241618954.L))-real(58056242310.L))+
        real(20583576819.L))/real(70368744177664.L);
      _C3x[409] = (_n*(_n*((-real(209091447400.L)*_n-real(3801662680.L))*_n+
        real(187421970124.L))-real(199736930614.L))+real(74417546961.L))/
        real(281474976710656.L);
      _C3x[410] = (_n*(_n*(real(103785391164.L)*_n+real(117206473721.L))-
        real(187421970124.L))+real(77818821763.L))/real(281474976710656.L);
      _C3x[411] = (_n*(real(787609465729.L)*_n-real(2549380838081.L))+
        real(1173967590711.L))/real(4503599627370496.L);
      _C3x[412] = (real(2340726177457.L)-real(4628714396034.L)*_n)/
        real(9007199254740992.L);
      _C3x[413] = real(556913250205.L)/real(2251799813685248.L);
      _C3x[414] = (_n*(_n*(_n*((real(1298457888354.L)-real(1058002723844.L)*_n)*
        _n-real(1394639954158.L))+real(1267854503780.L))-real(893010563532.L))+
        real(322476036831.L))/real(1688849860263936.L);
      _C3x[415] = (_n*(_n*(_n*(real(553046878373.L)*_n-real(1329061272928.L))+
        real(1645454649471.L))-real(1267854503780.L))+real(446505281766.L))/
        real(1688849860263936.L);
      _C3x[416] = (_n*((real(4279816803577.L)-real(405827491090.L)*_n)*_n-
        real(4401660092471.L))+real(1620648800484.L))/real(6755399441055744.L);
      _C3x[417] = (_n*(real(1851694849861.L)*_n-real(2771602176854.L))+
        real(1132800437073.L))/real(4503599627370496.L);
      _C3x[418] = (real(3214210754373.L)-real(7127072067763.L)*_n)/
        real(13510798882111488.L);
      _C3x[419] = real(1045457237.L)/real(4398046511104.L);
      _C3x[420] = (_n*(_n*(_n*(real(5049558454710.L)*_n-real(5355592300450.L))+
        real(4823359525250.L))-real(3376351667675.L))+real(1215486600363.L))/
        real(7036874417766400.L);
      _C3x[421] = (_n*((real(2538085046735.L)-real(2110302953668.L)*_n)*_n-
        real(1929343810100.L))+real(675270333535.L))/real(2814749767106560.L);
      _C3x[422] = (_n*(real(13560625821127.L)*_n-real(13508733125545.L))+
        real(4919826715755.L))/real(22517998136852480.L);
      _C3x[423] = (real(2068389622621.L)-real(5134715698742.L)*_n)/
        real(9007199254740992.L);
      _C3x[424] = real(1961943067581.L)/real(9007199254740992.L);
      _C3x[425] = (_n*((real(18378663018625.L)-real(20584102580860.L)*_n)*_n-
        real(12791549460963.L))+real(4591838268038.L))/
        real(29273397577908224.L);
      _C3x[426] = (_n*(real(3766212175509.L)*_n-real(2827486618250.L))+
        real(983965343151.L))/real(4503599627370496.L);
      _C3x[427] = (real(1798281489207.L)-real(4987686394593.L)*_n)/
        real(9007199254740992.L);
      _C3x[428] = real(3788832068455.L)/real(18014398509481984.L);
      _C3x[429] = (_n*(real(70132978079073.L)*_n-real(48553600208589.L))+
        real(17383387729001.L))/real(121597189939003392.L);
      _C3x[430] = (real(5394844467621.L)-real(15585106239794.L)*_n)/
        real(27021597764222976.L);
      _C3x[431] = real(6593698793759.L)/real(36028797018963968.L);
      _C3x[432] = (real(4709784852685.L)-real(13187397587518.L)*_n)/
        real(36028797018963968.L);
      _C3x[433] = real(6593698793759.L)/real(36028797018963968.L);
      _C3x[434] = real(125280277081421.L)/real(1044835113549955072.L);
      break;
    default:
      static_assert(nC3_ == 30, "Bad value of nC3_");
    }
  }

  // The coefficients C4[l] in the Fourier expansion of I4
  template<typename real>
  void Geodesic30<real>::C4coeff() {
    switch (nC4_) {
    case 30:
      _C4x[0] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(25874463385137300111360.L)-
        real(24601948792425629614080.L)*_ep2)*_ep2-
        real(27260595352198226903040.L))+real(28775072871764795064320.L))-
        real(30435173229751225548800.L))+real(32261283623536299081728.L))-
        real(34277613850007317774336.L))+real(36513110405442577629184.L))-
        real(39002640660359117012992.L))+real(41788543564670482513920.L))-
        real(44922684332020768702464.L))+real(48469212042443460968448.L))-
        real(52508313045980416049152.L))+real(57141399491213982171136.L))-
        real(62498405693515292999680.L))+real(68748246262866822299648.L))-
        real(76114129791031124688896.L))+real(84896529382303946768384.L))-
        real(95508595555091940114432.L))+real(108532494948968113766400.L))-
        real(124812369191313330831360.L))+real(145614430723198885969920.L))-
        real(172917136483798677089280.L))+real(209970808587469822179840.L))-
        real(262463510734337277724800.L))+real(341202563954638461042240.L))-
        real(469153525437627883933080.L))+real(703730288156441825899620.L))-
        real(1231528004273773195324335.L))+real(12315280042737731953243350.L))/
        real(18472920064106597929865025.L);
      _C4x[1] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(25874463385137300111360.L)-
        real(24601948792425629614080.L)*_ep2)*_ep2-
        real(27260595352198226903040.L))+real(28775072871764795064320.L))-
        real(30435173229751225548800.L))+real(32261283623536299081728.L))-
        real(34277613850007317774336.L))+real(36513110405442577629184.L))-
        real(39002640660359117012992.L))+real(41788543564670482513920.L))-
        real(44922684332020768702464.L))+real(48469212042443460968448.L))-
        real(52508313045980416049152.L))+real(57141399491213982171136.L))-
        real(62498405693515292999680.L))+real(68748246262866822299648.L))-
        real(76114129791031124688896.L))+real(84896529382303946768384.L))-
        real(95508595555091940114432.L))+real(108532494948968113766400.L))-
        real(124812369191313330831360.L))+real(145614430723198885969920.L))-
        real(172917136483798677089280.L))+real(209970808587469822179840.L))-
        real(262463510734337277724800.L))+real(341202563954638461042240.L))-
        real(469153525437627883933080.L))+real(703730288156441825899620.L))-
        real(1231528004273773195324335.L))/real(24630560085475463906486700.L);
      _C4x[2] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(6468615846284325027840.L)-
        real(6150487198106407403520.L)*_ep2)*_ep2-
        real(6815148838049556725760.L))+real(7193768217941198766080.L))-
        real(7608793307437806387200.L))+real(8065320905884074770432.L))-
        real(8569403462501829443584.L))+real(9128277601360644407296.L))-
        real(9750660165089779253248.L))+real(10447135891167620628480.L))-
        real(11230671083005192175616.L))+real(12117303010610865242112.L))-
        real(13127078261495104012288.L))+real(14285349872803495542784.L))-
        real(15624601423378823249920.L))+real(17187061565716705574912.L))-
        real(19028532447757781172224.L))+real(21224132345575986692096.L))-
        real(23877148888772985028608.L))+real(27133123737242028441600.L))-
        real(31203092297828332707840.L))+real(36403607680799721492480.L))-
        real(43229284120949669272320.L))+real(52492702146867455544960.L))-
        real(65615877683584319431200.L))+real(85300640988659615260560.L))-
        real(117288381359406970983270.L))+real(175932572039110456474905.L))/
        real(7389168025642639171946010.L);
      _C4x[3] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(3234307923142162513920.L)-
        real(3075243599053203701760.L)*_ep2)*_ep2-
        real(3407574419024778362880.L))+real(3596884108970599383040.L))-
        real(3804396653718903193600.L))+real(4032660452942037385216.L))-
        real(4284701731250914721792.L))+real(4564138800680322203648.L))-
        real(4875330082544889626624.L))+real(5223567945583810314240.L))-
        real(5615335541502596087808.L))+real(6058651505305432621056.L))-
        real(6563539130747552006144.L))+real(7142674936401747771392.L))-
        real(7812300711689411624960.L))+real(8593530782858352787456.L))-
        real(9514266223878890586112.L))+real(10612066172787993346048.L))-
        real(11938574444386492514304.L))+real(13566561868621014220800.L))-
        real(15601546148914166353920.L))+real(18201803840399860746240.L))-
        real(21614642060474834636160.L))+real(26246351073433727772480.L))-
        real(32807938841792159715600.L))+real(42650320494329807630280.L))-
        real(58644190679703485491635.L))/real(4222381728938650955397720.L);
      _C4x[4] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(404288490392770314240.L)-real(384405449881650462720.L)*
        _ep2)*_ep2-real(425946802378097295360.L))+
        real(449610513621324922880.L))-real(475549581714862899200.L))+
        real(504082556617754673152.L))-real(535587716406364340224.L))+
        real(570517350085040275456.L))-real(609416260318111203328.L))+
        real(652945993197976289280.L))-real(701916942687824510976.L))+
        real(757331438163179077632.L))-real(820442391343444000768.L))+
        real(892834367050218471424.L))-real(976537588961176453120.L))+
        real(1074191347857294098432.L))-real(1189283277984861323264.L))+
        real(1326508271598499168256.L))-real(1492321805548311564288.L))+
        real(1695820233577626777600.L))-real(1950193268614270794240.L))+
        real(2275225480049982593280.L))-real(2701830257559354329520.L))+
        real(3280793884179215971560.L))-real(4100992355224019964450.L))+
        real(5331290061791225953785.L))/real(586441906797034854916350.L);
      _C4x[5] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(202144245196385157120.L)-real(192202724940825231360.L)*_ep2)*
        _ep2-real(212973401189048647680.L))+real(224805256810662461440.L))-
        real(237774790857431449600.L))+real(252041278308877336576.L))-
        real(267793858203182170112.L))+real(285258675042520137728.L))-
        real(304708130159055601664.L))+real(326472996598988144640.L))-
        real(350958471343912255488.L))+real(378665719081589538816.L))-
        real(410221195671722000384.L))+real(446417183525109235712.L))-
        real(488268794480588226560.L))+real(537095673928647049216.L))-
        real(594641638992430661632.L))+real(663254135799249584128.L))-
        real(746160902774155782144.L))+real(847910116788813388800.L))-
        real(975096634307135397120.L))+real(1137612740024991296640.L))-
        real(1350915128779677164760.L))+real(1640396942089607985780.L))-
        real(2050496177612009982225.L))/real(319877403707473557227100.L);
      _C4x[6] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(50536061299096289280.L)-real(48050681235206307840.L)*_ep2)*_ep2-
        real(53243350297262161920.L))+real(56201314202665615360.L))-
        real(59443697714357862400.L))+real(63010319577219334144.L))-
        real(66948464550795542528.L))+real(71314668760630034432.L))-
        real(76177032539763900416.L))+real(81618249149747036160.L))-
        real(87739617835978063872.L))+real(94666429770397384704.L))-
        real(102555298917930500096.L))+real(111604295881277308928.L))-
        real(122067198620147056640.L))+real(134273918482161762304.L))-
        real(148660409748107665408.L))+real(165813533949812396032.L))-
        real(186540225693538945536.L))+real(211977529197203347200.L))-
        real(243774158576783849280.L))+real(284403185006247824160.L))-
        real(337728782194919291190.L))+real(410099235522401996445.L))/
        real(86120839459704419253450.L);
      _C4x[7] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(25268030649548144640.L)-real(24025340617603153920.L)*_ep2)*_ep2-
        real(26621675148631080960.L))+real(28100657101332807680.L))-
        real(29721848857178931200.L))+real(31505159788609667072.L))-
        real(33474232275397771264.L))+real(35657334380315017216.L))-
        real(38088516269881950208.L))+real(40809124574873518080.L))-
        real(43869808917989031936.L))+real(47333214885198692352.L))-
        real(51277649458965250048.L))+real(55802147940638654464.L))-
        real(61033599310073528320.L))+real(67136959241080881152.L))-
        real(74330204874053832704.L))+real(82906766974906198016.L))-
        real(93270112846769472768.L))+real(105988764598601673600.L))-
        real(121887079288391924640.L))+real(142201592503123912080.L))-
        real(168864391097459645595.L))/real(45931114378509023601840.L);
      _C4x[8] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1579251915596759040.L)-real(1501583788600197120.L)*_ep2)*_ep2-
        real(1663854696789442560.L))+real(1756291068833300480.L))-
        real(1857615553573683200.L))+real(1969072486788104192.L))-
        real(2092139517212360704.L))+real(2228583398769688576.L))-
        real(2380532266867621888.L))+real(2550570285929594880.L))-
        real(2741863057374314496.L))+real(2958325930324918272.L))-
        real(3204853091185328128.L))+real(3487634246289915904.L))-
        real(3814599956879595520.L))+real(4196059952567555072.L))-
        real(4645637804628364544.L))+real(5181672935931637376.L))-
        real(5829382052923092048.L))+real(6624297787412604600.L))-
        real(7617942455524495290.L))+real(8887599531445244505.L))/
        real(3039559039754273620710.L);
      _C4x[9] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(789625957798379520.L)-real(750791894300098560.L)*_ep2)*_ep2-
        real(831927348394721280.L))+real(878145534416650240.L))-
        real(928807776786841600.L))+real(984536243394052096.L))-
        real(1046069758606180352.L))+real(1114291699384844288.L))-
        real(1190266133433810944.L))+real(1275285142964797440.L))-
        real(1370931528687157248.L))+real(1479162965162459136.L))-
        real(1602426545592664064.L))+real(1743817123144957952.L))-
        real(1907299978439797760.L))+real(2098029976283777536.L))-
        real(2322818902314182272.L))+real(2590836467965818688.L))-
        real(2914691026461546024.L))+real(3312148893706302300.L))-
        real(3808971227762247645.L))/real(1599767915660144010900.L);
      _C4x[10] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(197406489449594880.L)-
        real(187697973575024640.L)*_ep2)*_ep2-real(207981837098680320.L))+
        real(219536383604162560.L))-real(232201944196710400.L))+
        real(246134060848513024.L))-real(261517439651545088.L))+
        real(278572924846211072.L))-real(297566533358452736.L))+
        real(318821285741199360.L))-real(342732882171789312.L))+
        real(369790741290614784.L))-real(400606636398166016.L))+
        real(435954280786239488.L))-real(476824994609949440.L))+
        real(524507494070944384.L))-real(580704725578545568.L))+
        real(647709116991454672.L))-real(728672756615386506.L))+
        real(828037223426575575.L))/real(418986835053847240950.L);
      _C4x[11] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(98703244724797440.L)-
        real(93848986787512320.L)*_ep2)*_ep2-real(103990918549340160.L))+
        real(109768191802081280.L))-real(116100972098355200.L))+
        real(123067030424256512.L))-real(130758719825772544.L))+
        real(139286462423105536.L))-real(148783266679226368.L))+
        real(159410642870599680.L))-real(171366441085894656.L))+
        real(184895370645307392.L))-real(200303318199083008.L))+
        real(217977140393119744.L))-real(238412497304974720.L))+
        real(262253747035472192.L))-real(290352362789272784.L))+
        real(323854558495727336.L))-real(364336378307693253.L))/
        real(218601826984615951800.L);
      _C4x[12] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(28418237508000073734.L);
      _C4x[13] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(14735382411555593788.L);
      _C4x[14] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(3810874761609205290.L);
      _C4x[15] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(1966903102766041440.L);
      _C4x[16] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(24097471856640.L)-real(22912350289920.L)*_ep2)*_ep2-
        real(25388407848960.L))+real(26798874951680.L))-real(28344963891200.L))+
        real(30045661724672.L))-real(31923515582464.L))+real(34005483990016.L))-
        real(36324039716608.L))+real(38918613982080.L))-real(41837510030736.L))+
        real(45140471348952.L))-real(48902177294698.L))+real(53217075291289.L))/
        real(63328319596633910.L);
      _C4x[17] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(32568850078268868.L);
      _C4x[18] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(8362272317393358.L);
      _C4x[19] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(4288344778150440.L);
      _C4x[20] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/real(549117319153410.L);
      _C4x[21] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(280943744683140.L);
      _C4x[22] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(71796734752358.L);
      _C4x[23] = (_ep2*(_ep2*(_ep2*(_ep2*((real(511580160.L)-real(486420480.L)*
        _ep2)*_ep2-real(538986240.L))+real(568929920.L))-real(601752800.L))+
        real(637857968.L))-real(677724091.L))/real(1594007062032.L);
      _C4x[24] = (_ep2*(_ep2*(_ep2*((real(31973760.L)-real(30401280.L)*_ep2)*
        _ep2-real(33686640.L))+real(35558120.L))-real(37609550.L))+
        real(39866123.L))/real(101658613650.L);
      _C4x[25] = (_ep2*(_ep2*((real(3197376.L)-real(3040128.L)*_ep2)*_ep2-
        real(3368664.L))+real(3555812.L))-real(3760955.L))/real(10365191980.L);
      _C4x[26] = (_ep2*((real(61488.L)-real(58464.L)*_ep2)*_ep2-real(64782.L))+
        real(68381.L))/real(203091570.L);
      _C4x[27] = ((real(3416.L)-real(3248.L)*_ep2)*_ep2-real(3599.L))/
        real(11488008.L);
      _C4x[28] = (real(61.L)-real(58.L)*_ep2)/real(208742.L);
      _C4x[29] = -1/real(3660.L);
      _C4x[30] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(24601948792425629614080.L)*_ep2-
        real(25874463385137300111360.L))+real(27260595352198226903040.L))-
        real(28775072871764795064320.L))+real(30435173229751225548800.L))-
        real(32261283623536299081728.L))+real(34277613850007317774336.L))-
        real(36513110405442577629184.L))+real(39002640660359117012992.L))-
        real(41788543564670482513920.L))+real(44922684332020768702464.L))-
        real(48469212042443460968448.L))+real(52508313045980416049152.L))-
        real(57141399491213982171136.L))+real(62498405693515292999680.L))-
        real(68748246262866822299648.L))+real(76114129791031124688896.L))-
        real(84896529382303946768384.L))+real(95508595555091940114432.L))-
        real(108532494948968113766400.L))+real(124812369191313330831360.L))-
        real(145614430723198885969920.L))+real(172917136483798677089280.L))-
        real(209970808587469822179840.L))+real(262463510734337277724800.L))-
        real(341202563954638461042240.L))+real(469153525437627883933080.L))-
        real(703730288156441825899620.L))+real(1231528004273773195324335.L))/
        real(221675040769279175158380300.L);
      _C4x[31] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(6150487198106407403520.L)*_ep2-
        real(6468615846284325027840.L))+real(6815148838049556725760.L))-
        real(7193768217941198766080.L))+real(7608793307437806387200.L))-
        real(8065320905884074770432.L))+real(8569403462501829443584.L))-
        real(9128277601360644407296.L))+real(9750660165089779253248.L))-
        real(10447135891167620628480.L))+real(11230671083005192175616.L))-
        real(12117303010610865242112.L))+real(13127078261495104012288.L))-
        real(14285349872803495542784.L))+real(15624601423378823249920.L))-
        real(17187061565716705574912.L))+real(19028532447757781172224.L))-
        real(21224132345575986692096.L))+real(23877148888772985028608.L))-
        real(27133123737242028441600.L))+real(31203092297828332707840.L))-
        real(36403607680799721492480.L))+real(43229284120949669272320.L))-
        real(52492702146867455544960.L))+real(65615877683584319431200.L))-
        real(85300640988659615260560.L))+real(117288381359406970983270.L))-
        real(175932572039110456474905.L))/real(44335008153855835031676060.L);
      _C4x[32] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(3075243599053203701760.L)*_ep2-
        real(3234307923142162513920.L))+real(3407574419024778362880.L))-
        real(3596884108970599383040.L))+real(3804396653718903193600.L))-
        real(4032660452942037385216.L))+real(4284701731250914721792.L))-
        real(4564138800680322203648.L))+real(4875330082544889626624.L))-
        real(5223567945583810314240.L))+real(5615335541502596087808.L))-
        real(6058651505305432621056.L))+real(6563539130747552006144.L))-
        real(7142674936401747771392.L))+real(7812300711689411624960.L))-
        real(8593530782858352787456.L))+real(9514266223878890586112.L))-
        real(10612066172787993346048.L))+real(11938574444386492514304.L))-
        real(13566561868621014220800.L))+real(15601546148914166353920.L))-
        real(18201803840399860746240.L))+real(21614642060474834636160.L))-
        real(26246351073433727772480.L))+real(32807938841792159715600.L))-
        real(42650320494329807630280.L))+real(58644190679703485491635.L))/
        real(21111908644693254776988600.L);
      _C4x[33] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(384405449881650462720.L)*_ep2-
        real(404288490392770314240.L))+real(425946802378097295360.L))-
        real(449610513621324922880.L))+real(475549581714862899200.L))-
        real(504082556617754673152.L))+real(535587716406364340224.L))-
        real(570517350085040275456.L))+real(609416260318111203328.L))-
        real(652945993197976289280.L))+real(701916942687824510976.L))-
        real(757331438163179077632.L))+real(820442391343444000768.L))-
        real(892834367050218471424.L))+real(976537588961176453120.L))-
        real(1074191347857294098432.L))+real(1189283277984861323264.L))-
        real(1326508271598499168256.L))+real(1492321805548311564288.L))-
        real(1695820233577626777600.L))+real(1950193268614270794240.L))-
        real(2275225480049982593280.L))+real(2701830257559354329520.L))-
        real(3280793884179215971560.L))+real(4100992355224019964450.L))-
        real(5331290061791225953785.L))/real(2638988580586656847123575.L);
      _C4x[34] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(192202724940825231360.L)*_ep2-
        real(202144245196385157120.L))+real(212973401189048647680.L))-
        real(224805256810662461440.L))+real(237774790857431449600.L))-
        real(252041278308877336576.L))+real(267793858203182170112.L))-
        real(285258675042520137728.L))+real(304708130159055601664.L))-
        real(326472996598988144640.L))+real(350958471343912255488.L))-
        real(378665719081589538816.L))+real(410221195671722000384.L))-
        real(446417183525109235712.L))+real(488268794480588226560.L))-
        real(537095673928647049216.L))+real(594641638992430661632.L))-
        real(663254135799249584128.L))+real(746160902774155782144.L))-
        real(847910116788813388800.L))+real(975096634307135397120.L))-
        real(1137612740024991296640.L))+real(1350915128779677164760.L))-
        real(1640396942089607985780.L))+real(2050496177612009982225.L))/
        real(1343485095571388940353820.L);
      _C4x[35] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(48050681235206307840.L)*_ep2-real(50536061299096289280.L))+
        real(53243350297262161920.L))-real(56201314202665615360.L))+
        real(59443697714357862400.L))-real(63010319577219334144.L))+
        real(66948464550795542528.L))-real(71314668760630034432.L))+
        real(76177032539763900416.L))-real(81618249149747036160.L))+
        real(87739617835978063872.L))-real(94666429770397384704.L))+
        real(102555298917930500096.L))-real(111604295881277308928.L))+
        real(122067198620147056640.L))-real(134273918482161762304.L))+
        real(148660409748107665408.L))-real(165813533949812396032.L))+
        real(186540225693538945536.L))-real(211977529197203347200.L))+
        real(243774158576783849280.L))-real(284403185006247824160.L))+
        real(337728782194919291190.L))-real(410099235522401996445.L))/
        real(344483357838817677013800.L);
      _C4x[36] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(24025340617603153920.L)*_ep2-real(25268030649548144640.L))+
        real(26621675148631080960.L))-real(28100657101332807680.L))+
        real(29721848857178931200.L))-real(31505159788609667072.L))+
        real(33474232275397771264.L))-real(35657334380315017216.L))+
        real(38088516269881950208.L))-real(40809124574873518080.L))+
        real(43869808917989031936.L))-real(47333214885198692352.L))+
        real(51277649458965250048.L))-real(55802147940638654464.L))+
        real(61033599310073528320.L))-real(67136959241080881152.L))+
        real(74330204874053832704.L))-real(82906766974906198016.L))+
        real(93270112846769472768.L))-real(105988764598601673600.L))+
        real(121887079288391924640.L))-real(142201592503123912080.L))+
        real(168864391097459645595.L))/real(177162869745677662464240.L);
      _C4x[37] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(3003167577200394240.L)*_ep2-real(3158503831193518080.L))+
        real(3327709393578885120.L))-real(3512582137666600960.L))+
        real(3715231107147366400.L))-real(3938144973576208384.L))+
        real(4184279034424721408.L))-real(4457166797539377152.L))+
        real(4761064533735243776.L))-real(5101140571859189760.L))+
        real(5483726114748628992.L))-real(5916651860649836544.L))+
        real(6409706182370656256.L))-real(6975268492579831808.L))+
        real(7629199913759191040.L))-real(8392119905135110144.L))+
        real(9291275609256729088.L))-real(10363345871863274752.L))+
        real(11658764105846184096.L))-real(13248595574825209200.L))+
        real(15235884911048990580.L))-real(17775199062890489010.L))/
        real(22796692798157052155325.L);
      _C4x[38] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(750791894300098560.L)*_ep2-real(789625957798379520.L))+
        real(831927348394721280.L))-real(878145534416650240.L))+
        real(928807776786841600.L))-real(984536243394052096.L))+
        real(1046069758606180352.L))-real(1114291699384844288.L))+
        real(1190266133433810944.L))-real(1275285142964797440.L))+
        real(1370931528687157248.L))-real(1479162965162459136.L))+
        real(1602426545592664064.L))-real(1743817123144957952.L))+
        real(1907299978439797760.L))-real(2098029976283777536.L))+
        real(2322818902314182272.L))-real(2590836467965818688.L))+
        real(2914691026461546024.L))-real(3312148893706302300.L))+
        real(3808971227762247645.L))/real(5865815690753861373300.L);
      _C4x[39] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(187697973575024640.L)*_ep2-real(197406489449594880.L))+
        real(207981837098680320.L))-real(219536383604162560.L))+
        real(232201944196710400.L))-real(246134060848513024.L))+
        real(261517439651545088.L))-real(278572924846211072.L))+
        real(297566533358452736.L))-real(318821285741199360.L))+
        real(342732882171789312.L))-real(369790741290614784.L))+
        real(400606636398166016.L))-real(435954280786239488.L))+
        real(476824994609949440.L))-real(524507494070944384.L))+
        real(580704725578545568.L))-real(647709116991454672.L))+
        real(728672756615386506.L))-real(828037223426575575.L))/
        real(1508352606193850067420.L);
      _C4x[40] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(93848986787512320.L)*
        _ep2-real(98703244724797440.L))+real(103990918549340160.L))-
        real(109768191802081280.L))+real(116100972098355200.L))-
        real(123067030424256512.L))+real(130758719825772544.L))-
        real(139286462423105536.L))+real(148783266679226368.L))-
        real(159410642870599680.L))+real(171366441085894656.L))-
        real(184895370645307392.L))+real(200303318199083008.L))-
        real(217977140393119744.L))+real(238412497304974720.L))-
        real(262253747035472192.L))+real(290352362789272784.L))-
        real(323854558495727336.L))+real(364336378307693253.L))/
        real(775042841127274738200.L);
      _C4x[41] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11731123348439040.L)*_ep2-
        real(12337905590599680.L))+real(12998864818667520.L))-
        real(13721023975260160.L))+real(14512621512294400.L))-
        real(15383378803032064.L))+real(16344839978221568.L))-
        real(17410807802888192.L))+real(18597908334903296.L))-
        real(19926330358824960.L))+real(21420805135736832.L))-
        real(23111921330663424.L))+real(25037914774885376.L))-
        real(27247142549139968.L))+real(29801562163121840.L))-
        real(32781718379434024.L))+real(36294045348659098.L))-
        real(40481819811965917.L))/real(99463831278000258069.L);
      _C4x[42] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(51007092963077055420.L);
      _C4x[43] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(13065856325517275280.L);
      _C4x[44] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(6687470549404540896.L);
      _C4x[45] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(91649401159680.L)*_ep2-real(96389887426560.L))+
        real(101553631395840.L))-real(107195499806720.L))+
        real(113379855564800.L))-real(120182646898688.L))+
        real(127694062329856.L))-real(136021935960064.L))+
        real(145296158866432.L))-real(155674455928320.L))+
        real(167350040122944.L))-real(180561885395808.L))+
        real(195608709178792.L))-real(212868301165156.L))/
        real(854932314554557785.L);
      _C4x[46] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(109201438497725028.L);
      _C4x[47] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(27874241057977860.L);
      _C4x[48] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(14219248474919880.L);
      _C4x[49] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(1812087153206253.L);
      _C4x[50] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(89501368320.L)*
        _ep2-real(94130749440.L))+real(99173468160.L))-real(104683105280.L))+
        real(110722515200.L))-real(117365866112.L))+real(124701232744.L))-
        real(132833921836.L))+real(141890780143.L))/real(923100875387460.L);
      _C4x[51] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(234971131916808.L);
      _C4x[52] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(119550529652400.L);
      _C4x[53] = (_ep2*(_ep2*(_ep2*(_ep2*(real(60802560.L)*_ep2-
        real(63947520.L))+real(67373280.L))-real(71116240.L))+real(75219100.L))-
        real(79732246.L))/real(660780988725.L);
      _C4x[54] = (_ep2*(_ep2*(_ep2*(real(15200640.L)*_ep2-real(15986880.L))+
        real(16843320.L))-real(17779060.L))+real(18804775.L))/
        real(167916110076.L);
      _C4x[55] = (_ep2*(_ep2*(real(760032.L)*_ep2-real(799344.L))+
        real(842166.L))-real(888953.L))/real(8529845940.L);
      _C4x[56] = (_ep2*(real(9744.L)*_ep2-real(10248.L))+real(10797.L))/
        real(111050744.L);
      _C4x[57] = (real(406.L)*_ep2-real(427.L))/real(4696695.L);
      _C4x[58] = real(29.L)/real(340380.L);
      _C4x[59] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(6468615846284325027840.L)-
        real(6150487198106407403520.L)*_ep2)*_ep2-
        real(6815148838049556725760.L))+real(7193768217941198766080.L))-
        real(7608793307437806387200.L))+real(8065320905884074770432.L))-
        real(8569403462501829443584.L))+real(9128277601360644407296.L))-
        real(9750660165089779253248.L))+real(10447135891167620628480.L))-
        real(11230671083005192175616.L))+real(12117303010610865242112.L))-
        real(13127078261495104012288.L))+real(14285349872803495542784.L))-
        real(15624601423378823249920.L))+real(17187061565716705574912.L))-
        real(19028532447757781172224.L))+real(21224132345575986692096.L))-
        real(23877148888772985028608.L))+real(27133123737242028441600.L))-
        real(31203092297828332707840.L))+real(36403607680799721492480.L))-
        real(43229284120949669272320.L))+real(52492702146867455544960.L))-
        real(65615877683584319431200.L))+real(85300640988659615260560.L))-
        real(117288381359406970983270.L))+real(175932572039110456474905.L))/
        real(369458401282131958597300500.L);
      _C4x[60] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(3234307923142162513920.L)-
        real(3075243599053203701760.L)*_ep2)*_ep2-
        real(3407574419024778362880.L))+real(3596884108970599383040.L))-
        real(3804396653718903193600.L))+real(4032660452942037385216.L))-
        real(4284701731250914721792.L))+real(4564138800680322203648.L))-
        real(4875330082544889626624.L))+real(5223567945583810314240.L))-
        real(5615335541502596087808.L))+real(6058651505305432621056.L))-
        real(6563539130747552006144.L))+real(7142674936401747771392.L))-
        real(7812300711689411624960.L))+real(8593530782858352787456.L))-
        real(9514266223878890586112.L))+real(10612066172787993346048.L))-
        real(11938574444386492514304.L))+real(13566561868621014220800.L))-
        real(15601546148914166353920.L))+real(18201803840399860746240.L))-
        real(21614642060474834636160.L))+real(26246351073433727772480.L))-
        real(32807938841792159715600.L))+real(42650320494329807630280.L))-
        real(58644190679703485491635.L))/real(105559543223466273884943000.L);
      _C4x[61] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(404288490392770314240.L)-real(384405449881650462720.L)*
        _ep2)*_ep2-real(425946802378097295360.L))+
        real(449610513621324922880.L))-real(475549581714862899200.L))+
        real(504082556617754673152.L))-real(535587716406364340224.L))+
        real(570517350085040275456.L))-real(609416260318111203328.L))+
        real(652945993197976289280.L))-real(701916942687824510976.L))+
        real(757331438163179077632.L))-real(820442391343444000768.L))+
        real(892834367050218471424.L))-real(976537588961176453120.L))+
        real(1074191347857294098432.L))-real(1189283277984861323264.L))+
        real(1326508271598499168256.L))-real(1492321805548311564288.L))+
        real(1695820233577626777600.L))-real(1950193268614270794240.L))+
        real(2275225480049982593280.L))-real(2701830257559354329520.L))+
        real(3280793884179215971560.L))-real(4100992355224019964450.L))+
        real(5331290061791225953785.L))/real(10262733368948109961036125.L);
      _C4x[62] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(202144245196385157120.L)-real(192202724940825231360.L)*_ep2)*
        _ep2-real(212973401189048647680.L))+real(224805256810662461440.L))-
        real(237774790857431449600.L))+real(252041278308877336576.L))-
        real(267793858203182170112.L))+real(285258675042520137728.L))-
        real(304708130159055601664.L))+real(326472996598988144640.L))-
        real(350958471343912255488.L))+real(378665719081589538816.L))-
        real(410221195671722000384.L))+real(446417183525109235712.L))-
        real(488268794480588226560.L))+real(537095673928647049216.L))-
        real(594641638992430661632.L))+real(663254135799249584128.L))-
        real(746160902774155782144.L))+real(847910116788813388800.L))-
        real(975096634307135397120.L))+real(1137612740024991296640.L))-
        real(1350915128779677164760.L))+real(1640396942089607985780.L))-
        real(2050496177612009982225.L))/real(4478283651904629801179400.L);
      _C4x[63] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(50536061299096289280.L)-real(48050681235206307840.L)*_ep2)*_ep2-
        real(53243350297262161920.L))+real(56201314202665615360.L))-
        real(59443697714357862400.L))+real(63010319577219334144.L))-
        real(66948464550795542528.L))+real(71314668760630034432.L))-
        real(76177032539763900416.L))+real(81618249149747036160.L))-
        real(87739617835978063872.L))+real(94666429770397384704.L))-
        real(102555298917930500096.L))+real(111604295881277308928.L))-
        real(122067198620147056640.L))+real(134273918482161762304.L))-
        real(148660409748107665408.L))+real(165813533949812396032.L))-
        real(186540225693538945536.L))+real(211977529197203347200.L))-
        real(243774158576783849280.L))+real(284403185006247824160.L))-
        real(337728782194919291190.L))+real(410099235522401996445.L))/
        real(1033450073516453031041400.L);
      _C4x[64] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(25268030649548144640.L)-real(24025340617603153920.L)*_ep2)*_ep2-
        real(26621675148631080960.L))+real(28100657101332807680.L))-
        real(29721848857178931200.L))+real(31505159788609667072.L))-
        real(33474232275397771264.L))+real(35657334380315017216.L))-
        real(38088516269881950208.L))+real(40809124574873518080.L))-
        real(43869808917989031936.L))+real(47333214885198692352.L))-
        real(51277649458965250048.L))+real(55802147940638654464.L))-
        real(61033599310073528320.L))+real(67136959241080881152.L))-
        real(74330204874053832704.L))+real(82906766974906198016.L))-
        real(93270112846769472768.L))+real(105988764598601673600.L))-
        real(121887079288391924640.L))+real(142201592503123912080.L))-
        real(168864391097459645595.L))/real(492119082626882395734000.L);
      _C4x[65] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3158503831193518080.L)-real(3003167577200394240.L)*_ep2)*_ep2-
        real(3327709393578885120.L))+real(3512582137666600960.L))-
        real(3715231107147366400.L))+real(3938144973576208384.L))-
        real(4184279034424721408.L))+real(4457166797539377152.L))-
        real(4761064533735243776.L))+real(5101140571859189760.L))-
        real(5483726114748628992.L))+real(5916651860649836544.L))-
        real(6409706182370656256.L))+real(6975268492579831808.L))-
        real(7629199913759191040.L))+real(8392119905135110144.L))-
        real(9291275609256729088.L))+real(10363345871863274752.L))-
        real(11658764105846184096.L))+real(13248595574825209200.L))-
        real(15235884911048990580.L))+real(17775199062890489010.L))/
        real(59705623995173231835375.L);
      _C4x[66] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(789625957798379520.L)-real(750791894300098560.L)*_ep2)*_ep2-
        real(831927348394721280.L))+real(878145534416650240.L))-
        real(928807776786841600.L))+real(984536243394052096.L))-
        real(1046069758606180352.L))+real(1114291699384844288.L))-
        real(1190266133433810944.L))+real(1275285142964797440.L))-
        real(1370931528687157248.L))+real(1479162965162459136.L))-
        real(1602426545592664064.L))+real(1743817123144957952.L))-
        real(1907299978439797760.L))+real(2098029976283777536.L))-
        real(2322818902314182272.L))+real(2590836467965818688.L))-
        real(2914691026461546024.L))+real(3312148893706302300.L))-
        real(3808971227762247645.L))/real(14664539226884653433250.L);
      _C4x[67] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(197406489449594880.L)-
        real(187697973575024640.L)*_ep2)*_ep2-real(207981837098680320.L))+
        real(219536383604162560.L))-real(232201944196710400.L))+
        real(246134060848513024.L))-real(261517439651545088.L))+
        real(278572924846211072.L))-real(297566533358452736.L))+
        real(318821285741199360.L))-real(342732882171789312.L))+
        real(369790741290614784.L))-real(400606636398166016.L))+
        real(435954280786239488.L))-real(476824994609949440.L))+
        real(524507494070944384.L))-real(580704725578545568.L))+
        real(647709116991454672.L))-real(728672756615386506.L))+
        real(828037223426575575.L))/real(3631219237133342754900.L);
      _C4x[68] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(98703244724797440.L)-
        real(93848986787512320.L)*_ep2)*_ep2-real(103990918549340160.L))+
        real(109768191802081280.L))-real(116100972098355200.L))+
        real(123067030424256512.L))-real(130758719825772544.L))+
        real(139286462423105536.L))-real(148783266679226368.L))+
        real(159410642870599680.L))-real(171366441085894656.L))+
        real(184895370645307392.L))-real(200303318199083008.L))+
        real(217977140393119744.L))-real(238412497304974720.L))+
        real(262253747035472192.L))-real(290352362789272784.L))+
        real(323854558495727336.L))-real(364336378307693253.L))/
        real(1808433295963641055800.L);
      _C4x[69] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(226054161995455131975.L);
      _C4x[70] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(113349095473504567600.L);
      _C4x[71] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(28476866350486369200.L);
      _C4x[72] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(14330294034438301920.L);
      _C4x[73] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(96389887426560.L)-real(91649401159680.L)*_ep2)*_ep2-
        real(101553631395840.L))+real(107195499806720.L))-
        real(113379855564800.L))+real(120182646898688.L))-
        real(127694062329856.L))+real(136021935960064.L))-
        real(145296158866432.L))+real(155674455928320.L))-
        real(167350040122944.L))+real(180561885395808.L))-
        real(195608709178792.L))+real(212868301165156.L))/
        real(1804857108504066435.L);
      _C4x[74] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(227502996870260475.L);
      _C4x[75] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(57388143354660300.L);
      _C4x[76] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(28965135782244200.L);
      _C4x[77] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/real(3655965309100335.L);
      _C4x[78] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(1846201750774920.L);
      _C4x[79] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(466212563327000.L);
      _C4x[80] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(235478315982000.L);
      _C4x[81] = (_ep2*(_ep2*(_ep2*((real(1470792960.L)-real(1398458880.L)*
        _ep2)*_ep2-real(1549585440.L))+real(1635673520.L))-real(1730039300.L))+
        real(1833841658.L))/real(29735144492625.L);
      _C4x[82] = (_ep2*(_ep2*((real(3197376.L)-real(3040128.L)*_ep2)*_ep2-
        real(3368664.L))+real(3555812.L))-real(3760955.L))/real(65300709474.L);
      _C4x[83] = (_ep2*((real(799344.L)-real(760032.L)*_ep2)*_ep2-
        real(842166.L))+real(888953.L))/real(16491035484.L);
      _C4x[84] = ((real(133224.L)-real(126672.L)*_ep2)*_ep2-real(140361.L))/
        real(2776268600.L);
      _C4x[85] = (real(3843.L)-real(3654.L)*_ep2)/real(80887525.L);
      _C4x[86] = -real(203.L)/real(4538400.L);
      _C4x[87] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(3075243599053203701760.L)*_ep2-
        real(3234307923142162513920.L))+real(3407574419024778362880.L))-
        real(3596884108970599383040.L))+real(3804396653718903193600.L))-
        real(4032660452942037385216.L))+real(4284701731250914721792.L))-
        real(4564138800680322203648.L))+real(4875330082544889626624.L))-
        real(5223567945583810314240.L))+real(5615335541502596087808.L))-
        real(6058651505305432621056.L))+real(6563539130747552006144.L))-
        real(7142674936401747771392.L))+real(7812300711689411624960.L))-
        real(8593530782858352787456.L))+real(9514266223878890586112.L))-
        real(10612066172787993346048.L))+real(11938574444386492514304.L))-
        real(13566561868621014220800.L))+real(15601546148914166353920.L))-
        real(18201803840399860746240.L))+real(21614642060474834636160.L))-
        real(26246351073433727772480.L))+real(32807938841792159715600.L))-
        real(42650320494329807630280.L))+real(58644190679703485491635.L))/
        real(1034483523589969484072441400.L);
      _C4x[88] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(384405449881650462720.L)*_ep2-
        real(404288490392770314240.L))+real(425946802378097295360.L))-
        real(449610513621324922880.L))+real(475549581714862899200.L))-
        real(504082556617754673152.L))+real(535587716406364340224.L))-
        real(570517350085040275456.L))+real(609416260318111203328.L))-
        real(652945993197976289280.L))+real(701916942687824510976.L))-
        real(757331438163179077632.L))+real(820442391343444000768.L))-
        real(892834367050218471424.L))+real(976537588961176453120.L))-
        real(1074191347857294098432.L))+real(1189283277984861323264.L))-
        real(1326508271598499168256.L))+real(1492321805548311564288.L))-
        real(1695820233577626777600.L))+real(1950193268614270794240.L))-
        real(2275225480049982593280.L))+real(2701830257559354329520.L))-
        real(3280793884179215971560.L))+real(4100992355224019964450.L))-
        real(5331290061791225953785.L))/real(57471306866109415781802300.L);
      _C4x[89] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(192202724940825231360.L)*_ep2-
        real(202144245196385157120.L))+real(212973401189048647680.L))-
        real(224805256810662461440.L))+real(237774790857431449600.L))-
        real(252041278308877336576.L))+real(267793858203182170112.L))-
        real(285258675042520137728.L))+real(304708130159055601664.L))-
        real(326472996598988144640.L))+real(350958471343912255488.L))-
        real(378665719081589538816.L))+real(410221195671722000384.L))-
        real(446417183525109235712.L))+real(488268794480588226560.L))-
        real(537095673928647049216.L))+real(594641638992430661632.L))-
        real(663254135799249584128.L))+real(746160902774155782144.L))-
        real(847910116788813388800.L))+real(975096634307135397120.L))-
        real(1137612740024991296640.L))+real(1350915128779677164760.L))-
        real(1640396942089607985780.L))+real(2050496177612009982225.L))/
        real(18808791337999445164953480.L);
      _C4x[90] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(48050681235206307840.L)*_ep2-real(50536061299096289280.L))+
        real(53243350297262161920.L))-real(56201314202665615360.L))+
        real(59443697714357862400.L))-real(63010319577219334144.L))+
        real(66948464550795542528.L))-real(71314668760630034432.L))+
        real(76177032539763900416.L))-real(81618249149747036160.L))+
        real(87739617835978063872.L))-real(94666429770397384704.L))+
        real(102555298917930500096.L))-real(111604295881277308928.L))+
        real(122067198620147056640.L))-real(134273918482161762304.L))+
        real(148660409748107665408.L))-real(165813533949812396032.L))+
        real(186540225693538945536.L))-real(211977529197203347200.L))+
        real(243774158576783849280.L))-real(284403185006247824160.L))+
        real(337728782194919291190.L))-real(410099235522401996445.L))/
        real(3617075257307585608644900.L);
      _C4x[91] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(24025340617603153920.L)*_ep2-real(25268030649548144640.L))+
        real(26621675148631080960.L))-real(28100657101332807680.L))+
        real(29721848857178931200.L))-real(31505159788609667072.L))+
        real(33474232275397771264.L))-real(35657334380315017216.L))+
        real(38088516269881950208.L))-real(40809124574873518080.L))+
        real(43869808917989031936.L))-real(47333214885198692352.L))+
        real(51277649458965250048.L))-real(55802147940638654464.L))+
        real(61033599310073528320.L))-real(67136959241080881152.L))+
        real(74330204874053832704.L))-real(82906766974906198016.L))+
        real(93270112846769472768.L))-real(105988764598601673600.L))+
        real(121887079288391924640.L))-real(142201592503123912080.L))+
        real(168864391097459645595.L))/real(1515726774490797778860720.L);
      _C4x[92] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1501583788600197120.L)*_ep2-real(1579251915596759040.L))+
        real(1663854696789442560.L))-real(1756291068833300480.L))+
        real(1857615553573683200.L))-real(1969072486788104192.L))+
        real(2092139517212360704.L))-real(2228583398769688576.L))+
        real(2380532266867621888.L))-real(2550570285929594880.L))+
        real(2741863057374314496.L))-real(2958325930324918272.L))+
        real(3204853091185328128.L))-real(3487634246289915904.L))+
        real(3814599956879595520.L))-real(4196059952567555072.L))+
        real(4645637804628364544.L))-real(5181672935931637376.L))+
        real(5829382052923092048.L))-real(6624297787412604600.L))+
        real(7617942455524495290.L))-real(8887599531445244505.L))/
        real(83587873593242524569525.L);
      _C4x[93] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(750791894300098560.L)*_ep2-real(789625957798379520.L))+
        real(831927348394721280.L))-real(878145534416650240.L))+
        real(928807776786841600.L))-real(984536243394052096.L))+
        real(1046069758606180352.L))-real(1114291699384844288.L))+
        real(1190266133433810944.L))-real(1275285142964797440.L))+
        real(1370931528687157248.L))-real(1479162965162459136.L))+
        real(1602426545592664064.L))-real(1743817123144957952.L))+
        real(1907299978439797760.L))-real(2098029976283777536.L))+
        real(2322818902314182272.L))-real(2590836467965818688.L))+
        real(2914691026461546024.L))-real(3312148893706302300.L))+
        real(3808971227762247645.L))/real(38127801989900098926450.L);
      _C4x[94] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(187697973575024640.L)*_ep2-real(197406489449594880.L))+
        real(207981837098680320.L))-real(219536383604162560.L))+
        real(232201944196710400.L))-real(246134060848513024.L))+
        real(261517439651545088.L))-real(278572924846211072.L))+
        real(297566533358452736.L))-real(318821285741199360.L))+
        real(342732882171789312.L))-real(369790741290614784.L))+
        real(400606636398166016.L))-real(435954280786239488.L))+
        real(476824994609949440.L))-real(524507494070944384.L))+
        real(580704725578545568.L))-real(647709116991454672.L))+
        real(728672756615386506.L))-real(828037223426575575.L))/
        real(8896487130976689749505.L);
      _C4x[95] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(93848986787512320.L)*
        _ep2-real(98703244724797440.L))+real(103990918549340160.L))-
        real(109768191802081280.L))+real(116100972098355200.L))-
        real(123067030424256512.L))+real(130758719825772544.L))-
        real(139286462423105536.L))+real(148783266679226368.L))-
        real(159410642870599680.L))+real(171366441085894656.L))-
        real(184895370645307392.L))+real(200303318199083008.L))-
        real(217977140393119744.L))+real(238412497304974720.L))-
        real(262253747035472192.L))+real(290352362789272784.L))-
        real(323854558495727336.L))+real(364336378307693253.L))/
        real(4219677690581829130200.L);
      _C4x[96] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11731123348439040.L)*_ep2-
        real(12337905590599680.L))+real(12998864818667520.L))-
        real(13721023975260160.L))+real(14512621512294400.L))-
        real(15383378803032064.L))+real(16344839978221568.L))-
        real(17410807802888192.L))+real(18597908334903296.L))-
        real(19926330358824960.L))+real(21420805135736832.L))-
        real(23111921330663424.L))+real(25037914774885376.L))-
        real(27247142549139968.L))+real(29801562163121840.L))-
        real(32781718379434024.L))+real(36294045348659098.L))-
        real(40481819811965917.L))/real(506361322869819495624.L);
      _C4x[97] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(245246224751764428080.L);
      _C4x[98] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(59801419336021375320.L);
      _C4x[99] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(29321986255081448544.L);
      _C4x[100] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(45824700579840.L)*_ep2-real(48194943713280.L))+
        real(50776815697920.L))-real(53597749903360.L))+real(56689927782400.L))-
        real(60091323449344.L))+real(63847031164928.L))-real(68010967980032.L))+
        real(72648079433216.L))-real(77837227964160.L))+real(83675020061472.L))-
        real(90280942697904.L))+real(97804354589396.L))-
        real(106434150582578.L))/real(1804857108504066435.L);
      _C4x[101] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(445905873865710531.L);
      _C4x[102] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(5728087572480.L)*_ep2-real(6024367964160.L))+
        real(6347101962240.L))-real(6699718737920.L))+real(7086240972800.L))-
        real(7511415431168.L))+real(7980878895616.L))-real(8501370997504.L))+
        real(9081009929152.L))-real(9729653495520.L))+real(10459377507684.L))-
        real(11285117837238.L))/real(220944351915442155.L);
      _C4x[103] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(54863374834603720.L);
      _C4x[104] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(6824468576987292.L);
      _C4x[105] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(3400897961953800.L);
      _C4x[106] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(848506865255140.L);
      _C4x[107] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(423860968767600.L);
      _C4x[108] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(26491310547975.L);
      _C4x[109] = (_ep2*(_ep2*(_ep2*(real(349614720.L)*_ep2-real(367698240.L))+
        real(387396360.L))-real(408918380.L))+real(432509825.L))/
        real(13256044023222.L);
      _C4x[110] = (_ep2*(_ep2*(real(760032.L)*_ep2-real(799344.L))+
        real(842166.L))-real(888953.L))/real(28859312097.L);
      _C4x[111] = (_ep2*(real(633360.L)*_ep2-real(666120.L))+real(701805.L))/
        real(24098011448.L);
      _C4x[112] = (real(6786.L)*_ep2-real(7137.L))/real(258840080.L);
      _C4x[113] = real(87.L)/real(3328160.L);
      _C4x[114] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(404288490392770314240.L)-real(384405449881650462720.L)*
        _ep2)*_ep2-real(425946802378097295360.L))+
        real(449610513621324922880.L))-real(475549581714862899200.L))+
        real(504082556617754673152.L))-real(535587716406364340224.L))+
        real(570517350085040275456.L))-real(609416260318111203328.L))+
        real(652945993197976289280.L))-real(701916942687824510976.L))+
        real(757331438163179077632.L))-real(820442391343444000768.L))+
        real(892834367050218471424.L))-real(976537588961176453120.L))+
        real(1074191347857294098432.L))-real(1189283277984861323264.L))+
        real(1326508271598499168256.L))-real(1492321805548311564288.L))+
        real(1695820233577626777600.L))-real(1950193268614270794240.L))+
        real(2275225480049982593280.L))-real(2701830257559354329520.L))+
        real(3280793884179215971560.L))-real(4100992355224019964450.L))+
        real(5331290061791225953785.L))/real(665025122307837525475140900.L);
      _C4x[115] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(202144245196385157120.L)-real(192202724940825231360.L)*_ep2)*
        _ep2-real(212973401189048647680.L))+real(224805256810662461440.L))-
        real(237774790857431449600.L))+real(252041278308877336576.L))-
        real(267793858203182170112.L))+real(285258675042520137728.L))-
        real(304708130159055601664.L))+real(326472996598988144640.L))-
        real(350958471343912255488.L))+real(378665719081589538816.L))-
        real(410221195671722000384.L))+real(446417183525109235712.L))-
        real(488268794480588226560.L))+real(537095673928647049216.L))-
        real(594641638992430661632.L))+real(663254135799249584128.L))-
        real(746160902774155782144.L))+real(847910116788813388800.L))-
        real(975096634307135397120.L))+real(1137612740024991296640.L))-
        real(1350915128779677164760.L))+real(1640396942089607985780.L))-
        real(2050496177612009982225.L))/real(120913658601425004631843800.L);
      _C4x[116] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(50536061299096289280.L)-real(48050681235206307840.L)*_ep2)*_ep2-
        real(53243350297262161920.L))+real(56201314202665615360.L))-
        real(59443697714357862400.L))+real(63010319577219334144.L))-
        real(66948464550795542528.L))+real(71314668760630034432.L))-
        real(76177032539763900416.L))+real(81618249149747036160.L))-
        real(87739617835978063872.L))+real(94666429770397384704.L))-
        real(102555298917930500096.L))+real(111604295881277308928.L))-
        real(122067198620147056640.L))+real(134273918482161762304.L))-
        real(148660409748107665408.L))+real(165813533949812396032.L))-
        real(186540225693538945536.L))+real(211977529197203347200.L))-
        real(243774158576783849280.L))+real(284403185006247824160.L))-
        real(337728782194919291190.L))+real(410099235522401996445.L))/
        real(17051926213021475012183100.L);
      _C4x[117] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(25268030649548144640.L)-real(24025340617603153920.L)*_ep2)*_ep2-
        real(26621675148631080960.L))+real(28100657101332807680.L))-
        real(29721848857178931200.L))+real(31505159788609667072.L))-
        real(33474232275397771264.L))+real(35657334380315017216.L))-
        real(38088516269881950208.L))+real(40809124574873518080.L))-
        real(43869808917989031936.L))+real(47333214885198692352.L))-
        real(51277649458965250048.L))+real(55802147940638654464.L))-
        real(61033599310073528320.L))+real(67136959241080881152.L))-
        real(74330204874053832704.L))+real(82906766974906198016.L))-
        real(93270112846769472768.L))+real(105988764598601673600.L))-
        real(121887079288391924640.L))+real(142201592503123912080.L))-
        real(168864391097459645595.L))/real(5846374701607362861319920.L);
      _C4x[118] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1579251915596759040.L)-real(1501583788600197120.L)*_ep2)*_ep2-
        real(1663854696789442560.L))+real(1756291068833300480.L))-
        real(1857615553573683200.L))+real(1969072486788104192.L))-
        real(2092139517212360704.L))+real(2228583398769688576.L))-
        real(2380532266867621888.L))+real(2550570285929594880.L))-
        real(2741863057374314496.L))+real(2958325930324918272.L))-
        real(3204853091185328128.L))+real(3487634246289915904.L))-
        real(3814599956879595520.L))+real(4196059952567555072.L))-
        real(4645637804628364544.L))+real(5181672935931637376.L))-
        real(5829382052923092048.L))+real(6624297787412604600.L))-
        real(7617942455524495290.L))+real(8887599531445244505.L))/
        real(279422320297410724989555.L);
      _C4x[119] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(789625957798379520.L)-real(750791894300098560.L)*_ep2)*_ep2-
        real(831927348394721280.L))+real(878145534416650240.L))-
        real(928807776786841600.L))+real(984536243394052096.L))-
        real(1046069758606180352.L))+real(1114291699384844288.L))-
        real(1190266133433810944.L))+real(1275285142964797440.L))-
        real(1370931528687157248.L))+real(1479162965162459136.L))-
        real(1602426545592664064.L))+real(1743817123144957952.L))-
        real(1907299978439797760.L))+real(2098029976283777536.L))-
        real(2322818902314182272.L))+real(2590836467965818688.L))-
        real(2914691026461546024.L))+real(3312148893706302300.L))-
        real(3808971227762247645.L))/real(114383405969700296779350.L);
      _C4x[120] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(197406489449594880.L)-
        real(187697973575024640.L)*_ep2)*_ep2-real(207981837098680320.L))+
        real(219536383604162560.L))-real(232201944196710400.L))+
        real(246134060848513024.L))-real(261517439651545088.L))+
        real(278572924846211072.L))-real(297566533358452736.L))+
        real(318821285741199360.L))-real(342732882171789312.L))+
        real(369790741290614784.L))-real(400606636398166016.L))+
        real(435954280786239488.L))-real(476824994609949440.L))+
        real(524507494070944384.L))-real(580704725578545568.L))+
        real(647709116991454672.L))-real(728672756615386506.L))+
        real(828037223426575575.L))/real(24510729850650063595575.L);
      _C4x[121] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(98703244724797440.L)-
        real(93848986787512320.L)*_ep2)*_ep2-real(103990918549340160.L))+
        real(109768191802081280.L))-real(116100972098355200.L))+
        real(123067030424256512.L))-real(130758719825772544.L))+
        real(139286462423105536.L))-real(148783266679226368.L))+
        real(159410642870599680.L))-real(171366441085894656.L))+
        real(184895370645307392.L))-real(200303318199083008.L))+
        real(217977140393119744.L))-real(238412497304974720.L))+
        real(262253747035472192.L))-real(290352362789272784.L))+
        real(323854558495727336.L))-real(364336378307693253.L))/
        real(10850599775781846334800.L);
      _C4x[122] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(1229734641255275917944.L);
      _C4x[123] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(567569834425511962128.L);
      _C4x[124] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(132805749434540976360.L);
      _C4x[125] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(62832827689460246880.L);
      _C4x[126] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(48194943713280.L)-real(45824700579840.L)*_ep2)*_ep2-
        real(50776815697920.L))+real(53597749903360.L))-real(56689927782400.L))+
        real(60091323449344.L))-real(63847031164928.L))+real(68010967980032.L))-
        real(72648079433216.L))+real(77837227964160.L))-real(83675020061472.L))+
        real(90280942697904.L))-real(97804354589396.L))+
        real(106434150582578.L))/real(3748549379200753365.L);
      _C4x[127] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(900911867606231481.L);
      _C4x[128] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(6024367964160.L)-real(5728087572480.L)*_ep2)*_ep2-
        real(6347101962240.L))+real(6699718737920.L))-real(7086240972800.L))+
        real(7511415431168.L))-real(7980878895616.L))+real(8501370997504.L))-
        real(9081009929152.L))+real(9729653495520.L))-real(10459377507684.L))+
        real(11285117837238.L))/real(435576008061871677.L);
      _C4x[129] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(105807937181021460.L);
      _C4x[130] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/real(12903406973295300.L);
      _C4x[131] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(6315953357914200.L);
      _C4x[132] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(1550279460578940.L);
      _C4x[133] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(762949743781680.L);
      _C4x[134] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(47035592197425.L);
      _C4x[135] = (_ep2*(_ep2*((real(73539648.L)-real(69922944.L)*_ep2)*_ep2-
        real(77479272.L))+real(81783676.L))-real(86501965.L))/
        real(4648223228922.L);
      _C4x[136] = (_ep2*((real(18384912.L)-real(17480736.L)*_ep2)*_ep2-
        real(19369818.L))+real(20445919.L))/real(1150249725009.L);
      _C4x[137] = ((real(222040.L)-real(211120.L)*_ep2)*_ep2-real(233935.L))/
        real(13770292256.L);
      _C4x[138] = (real(27755.L)-real(26390.L)*_ep2)/real(1708344528.L);
      _C4x[139] = -real(2639.L)/real(169736160.L);
      _C4x[140] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(192202724940825231360.L)*_ep2-
        real(202144245196385157120.L))+real(212973401189048647680.L))-
        real(224805256810662461440.L))+real(237774790857431449600.L))-
        real(252041278308877336576.L))+real(267793858203182170112.L))-
        real(285258675042520137728.L))+real(304708130159055601664.L))-
        real(326472996598988144640.L))+real(350958471343912255488.L))-
        real(378665719081589538816.L))+real(410221195671722000384.L))-
        real(446417183525109235712.L))+real(488268794480588226560.L))-
        real(537095673928647049216.L))+real(594641638992430661632.L))-
        real(663254135799249584128.L))+real(746160902774155782144.L))-
        real(847910116788813388800.L))+real(975096634307135397120.L))-
        real(1137612740024991296640.L))+real(1350915128779677164760.L))-
        real(1640396942089607985780.L))+real(2050496177612009982225.L))/
        real(1625616965641380617828122200.L);
      _C4x[141] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(48050681235206307840.L)*_ep2-real(50536061299096289280.L))+
        real(53243350297262161920.L))-real(56201314202665615360.L))+
        real(59443697714357862400.L))-real(63010319577219334144.L))+
        real(66948464550795542528.L))-real(71314668760630034432.L))+
        real(76177032539763900416.L))-real(81618249149747036160.L))+
        real(87739617835978063872.L))-real(94666429770397384704.L))+
        real(102555298917930500096.L))-real(111604295881277308928.L))+
        real(122067198620147056640.L))-real(134273918482161762304.L))+
        real(148660409748107665408.L))-real(165813533949812396032.L))+
        real(186540225693538945536.L))-real(211977529197203347200.L))+
        real(243774158576783849280.L))-real(284403185006247824160.L))+
        real(337728782194919291190.L))-real(410099235522401996445.L))/
        real(125047458895490816756009400.L);
      _C4x[142] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(24025340617603153920.L)*_ep2-real(25268030649548144640.L))+
        real(26621675148631080960.L))-real(28100657101332807680.L))+
        real(29721848857178931200.L))-real(31505159788609667072.L))+
        real(33474232275397771264.L))-real(35657334380315017216.L))+
        real(38088516269881950208.L))-real(40809124574873518080.L))+
        real(43869808917989031936.L))-real(47333214885198692352.L))+
        real(51277649458965250048.L))-real(55802147940638654464.L))+
        real(61033599310073528320.L))-real(67136959241080881152.L))+
        real(74330204874053832704.L))-real(82906766974906198016.L))+
        real(93270112846769472768.L))-real(105988764598601673600.L))+
        real(121887079288391924640.L))-real(142201592503123912080.L))+
        real(168864391097459645595.L))/real(30964132678883440339583280.L);
      _C4x[143] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(3003167577200394240.L)*_ep2-real(3158503831193518080.L))+
        real(3327709393578885120.L))-real(3512582137666600960.L))+
        real(3715231107147366400.L))-real(3938144973576208384.L))+
        real(4184279034424721408.L))-real(4457166797539377152.L))+
        real(4761064533735243776.L))-real(5101140571859189760.L))+
        real(5483726114748628992.L))-real(5916651860649836544.L))+
        real(6409706182370656256.L))-real(6975268492579831808.L))+
        real(7629199913759191040.L))-real(8392119905135110144.L))+
        real(9291275609256729088.L))-real(10363345871863274752.L))+
        real(11658764105846184096.L))-real(13248595574825209200.L))+
        real(15235884911048990580.L))-real(17775199062890489010.L))/
        real(2390613184766736202688415.L);
      _C4x[144] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(750791894300098560.L)*_ep2-real(789625957798379520.L))+
        real(831927348394721280.L))-real(878145534416650240.L))+
        real(928807776786841600.L))-real(984536243394052096.L))+
        real(1046069758606180352.L))-real(1114291699384844288.L))+
        real(1190266133433810944.L))-real(1275285142964797440.L))+
        real(1370931528687157248.L))-real(1479162965162459136.L))+
        real(1602426545592664064.L))-real(1743817123144957952.L))+
        real(1907299978439797760.L))-real(2098029976283777536.L))+
        real(2322818902314182272.L))-real(2590836467965818688.L))+
        real(2914691026461546024.L))-real(3312148893706302300.L))+
        real(3808971227762247645.L))/real(419405821888901088190950.L);
      _C4x[145] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(187697973575024640.L)*_ep2-real(197406489449594880.L))+
        real(207981837098680320.L))-real(219536383604162560.L))+
        real(232201944196710400.L))-real(246134060848513024.L))+
        real(261517439651545088.L))-real(278572924846211072.L))+
        real(297566533358452736.L))-real(318821285741199360.L))+
        real(342732882171789312.L))-real(369790741290614784.L))+
        real(400606636398166016.L))-real(435954280786239488.L))+
        real(476824994609949440.L))-real(524507494070944384.L))+
        real(580704725578545568.L))-real(647709116991454672.L))+
        real(728672756615386506.L))-real(828037223426575575.L))/
        real(79886823216933540607800.L);
      _C4x[146] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(93848986787512320.L)*
        _ep2-real(98703244724797440.L))+real(103990918549340160.L))-
        real(109768191802081280.L))+real(116100972098355200.L))-
        real(123067030424256512.L))+real(130758719825772544.L))-
        real(139286462423105536.L))+real(148783266679226368.L))-
        real(159410642870599680.L))+real(171366441085894656.L))-
        real(184895370645307392.L))+real(200303318199083008.L))-
        real(217977140393119744.L))+real(238412497304974720.L))-
        real(262253747035472192.L))+real(290352362789272784.L))-
        real(323854558495727336.L))+real(364336378307693253.L))/
        real(32207335842400083565200.L);
      _C4x[147] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11731123348439040.L)*_ep2-
        real(12337905590599680.L))+real(12998864818667520.L))-
        real(13721023975260160.L))+real(14512621512294400.L))-
        real(15383378803032064.L))+real(16344839978221568.L))-
        real(17410807802888192.L))+real(18597908334903296.L))-
        real(19926330358824960.L))+real(21420805135736832.L))-
        real(23111921330663424.L))+real(25037914774885376.L))-
        real(27247142549139968.L))+real(29801562163121840.L))-
        real(32781718379434024.L))+real(36294045348659098.L))-
        real(40481819811965917.L))/real(3381770263452008774346.L);
      _C4x[148] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(1464470313517679013392.L);
      _C4x[149] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(324636276395544608880.L);
      _C4x[150] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(146609931275407242720.L);
      _C4x[151] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(91649401159680.L)*_ep2-real(96389887426560.L))+
        real(101553631395840.L))-real(107195499806720.L))+
        real(113379855564800.L))-real(120182646898688.L))+
        real(127694062329856.L))-real(136021935960064.L))+
        real(145296158866432.L))-real(155674455928320.L))+
        real(167350040122944.L))-real(180561885395808.L))+
        real(195608709178792.L))-real(212868301165156.L))/
        real(16799054625307079895.L);
      _C4x[152] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(1948125662430568929.L);
      _C4x[153] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(456317722731484614.L);
      _C4x[154] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(215534686850228900.L);
      _C4x[155] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(358005473280.L)*_ep2-real(376522997760.L))+real(396693872640.L))-
        real(418732421120.L))+real(442890060800.L))-real(469463464448.L))+
        real(498804930976.L))-real(531335687344.L))+real(567563120572.L))-
        real(608103343470.L))/real(51255199921700775.L);
      _C4x[156] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(12260380047715800.L);
      _C4x[157] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(2947444900359960.L);
      _C4x[158] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(1423280516177520.L);
      _C4x[159] = (_ep2*(_ep2*(_ep2*(_ep2*(real(1398458880.L)*_ep2-
        real(1470792960.L))+real(1549585440.L))-real(1635673520.L))+
        real(1730039300.L))-real(1833841658.L))/real(172463838057225.L);
      _C4x[160] = (_ep2*(_ep2*(_ep2*(real(69922944.L)*_ep2-real(73539648.L))+
        real(77479272.L))-real(81783676.L))+real(86501965.L))/
        real(8386476831018.L);
      _C4x[161] = (_ep2*(_ep2*(real(17480736.L)*_ep2-real(18384912.L))+
        real(19369818.L))-real(20445919.L))/real(2044888400016.L);
      _C4x[162] = (_ep2*(real(14567280.L)*_ep2-real(15320760.L))+
        real(16141515.L))/real(1666205362976.L);
      _C4x[163] = (real(237510.L)*_ep2-real(249795.L))/real(26621702228.L);
      _C4x[164] = real(1131.L)/real(124473184.L);
      _C4x[165] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(50536061299096289280.L)-real(48050681235206307840.L)*_ep2)*_ep2-
        real(53243350297262161920.L))+real(56201314202665615360.L))-
        real(59443697714357862400.L))+real(63010319577219334144.L))-
        real(66948464550795542528.L))+real(71314668760630034432.L))-
        real(76177032539763900416.L))+real(81618249149747036160.L))-
        real(87739617835978063872.L))+real(94666429770397384704.L))-
        real(102555298917930500096.L))+real(111604295881277308928.L))-
        real(122067198620147056640.L))+real(134273918482161762304.L))-
        real(148660409748107665408.L))+real(165813533949812396032.L))-
        real(186540225693538945536.L))+real(211977529197203347200.L))-
        real(243774158576783849280.L))+real(284403185006247824160.L))-
        real(337728782194919291190.L))+real(410099235522401996445.L))/
        real(1921183686667086184705962600.L);
      _C4x[166] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(25268030649548144640.L)-real(24025340617603153920.L)*_ep2)*_ep2-
        real(26621675148631080960.L))+real(28100657101332807680.L))-
        real(29721848857178931200.L))+real(31505159788609667072.L))-
        real(33474232275397771264.L))+real(35657334380315017216.L))-
        real(38088516269881950208.L))+real(40809124574873518080.L))-
        real(43869808917989031936.L))+real(47333214885198692352.L))-
        real(51277649458965250048.L))+real(55802147940638654464.L))-
        real(61033599310073528320.L))+real(67136959241080881152.L))-
        real(74330204874053832704.L))+real(82906766974906198016.L))-
        real(93270112846769472768.L))+real(105988764598601673600.L))-
        real(121887079288391924640.L))+real(142201592503123912080.L))-
        real(168864391097459645595.L))/real(256157824888944824627461680.L);
      _C4x[167] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3158503831193518080.L)-real(3003167577200394240.L)*_ep2)*_ep2-
        real(3327709393578885120.L))+real(3512582137666600960.L))-
        real(3715231107147366400.L))+real(3938144973576208384.L))-
        real(4184279034424721408.L))+real(4457166797539377152.L))-
        real(4761064533735243776.L))+real(5101140571859189760.L))-
        real(5483726114748628992.L))+real(5916651860649836544.L))-
        real(6409706182370656256.L))+real(6975268492579831808.L))-
        real(7629199913759191040.L))+real(8392119905135110144.L))-
        real(9291275609256729088.L))+real(10363345871863274752.L))-
        real(11658764105846184096.L))+real(13248595574825209200.L))-
        real(15235884911048990580.L))+real(17775199062890489010.L))/
        real(14126350637257986652249725.L);
      _C4x[168] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(789625957798379520.L)-real(750791894300098560.L)*_ep2)*_ep2-
        real(831927348394721280.L))+real(878145534416650240.L))-
        real(928807776786841600.L))+real(984536243394052096.L))-
        real(1046069758606180352.L))+real(1114291699384844288.L))-
        real(1190266133433810944.L))+real(1275285142964797440.L))-
        real(1370931528687157248.L))+real(1479162965162459136.L))-
        real(1602426545592664064.L))+real(1743817123144957952.L))-
        real(1907299978439797760.L))+real(2098029976283777536.L))-
        real(2322818902314182272.L))+real(2590836467965818688.L))-
        real(2914691026461546024.L))+real(3312148893706302300.L))-
        real(3808971227762247645.L))/real(1982645703474805144175400.L);
      _C4x[169] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(197406489449594880.L)-
        real(187697973575024640.L)*_ep2)*_ep2-real(207981837098680320.L))+
        real(219536383604162560.L))-real(232201944196710400.L))+
        real(246134060848513024.L))-real(261517439651545088.L))+
        real(278572924846211072.L))-real(297566533358452736.L))+
        real(318821285741199360.L))-real(342732882171789312.L))+
        real(369790741290614784.L))-real(400606636398166016.L))+
        real(435954280786239488.L))-real(476824994609949440.L))+
        real(524507494070944384.L))-real(580704725578545568.L))+
        real(647709116991454672.L))-real(728672756615386506.L))+
        real(828037223426575575.L))/real(320999780562587499533160.L);
      _C4x[170] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(98703244724797440.L)-
        real(93848986787512320.L)*_ep2)*_ep2-real(103990918549340160.L))+
        real(109768191802081280.L))-real(116100972098355200.L))+
        real(123067030424256512.L))-real(130758719825772544.L))+
        real(139286462423105536.L))-real(148783266679226368.L))+
        real(159410642870599680.L))-real(171366441085894656.L))+
        real(184895370645307392.L))-real(200303318199083008.L))+
        real(217977140393119744.L))-real(238412497304974720.L))+
        real(262253747035472192.L))-real(290352362789272784.L))+
        real(323854558495727336.L))-real(364336378307693253.L))/
        real(114189645259418478094800.L);
      _C4x[171] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(10848016299644755419006.L);
      _C4x[172] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(4326844108120415266840.L);
      _C4x[173] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(895209125818016951760.L);
      _C4x[174] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(381185821316058831072.L);
      _C4x[175] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(96389887426560.L)-real(91649401159680.L)*_ep2)*_ep2-
        real(101553631395840.L))+real(107195499806720.L))-
        real(113379855564800.L))+real(120182646898688.L))-
        real(127694062329856.L))+real(136021935960064.L))-
        real(145296158866432.L))+real(155674455928320.L))-
        real(167350040122944.L))+real(180561885395808.L))-
        real(195608709178792.L))+real(212868301165156.L))/
        real(41511713495593528005.L);
      _C4x[176] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(4604660656654072014.L);
      _C4x[177] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(1037085733480646850.L);
      _C4x[178] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(473056650359593300.L);
      _C4x[179] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(376522997760.L)-real(358005473280.L)*_ep2)*_ep2-
        real(396693872640.L))+real(418732421120.L))-real(442890060800.L))+
        real(469463464448.L))-real(498804930976.L))+real(531335687344.L))-
        real(567563120572.L))+real(608103343470.L))/real(109033788924345285.L);
      _C4x[180] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(25356695098684950.L);
      _C4x[181] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(5942175013025160.L);
      _C4x[182] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(2803431319743600.L);
      _C4x[183] = (_ep2*(_ep2*(_ep2*((real(1470792960.L)-real(1398458880.L)*
        _ep2)*_ep2-real(1549585440.L))+real(1635673520.L))-real(1730039300.L))+
        real(1833841658.L))/real(332549888694075.L);
      _C4x[184] = (_ep2*(_ep2*((real(367698240.L)-real(349614720.L)*_ep2)*_ep2-
        real(387396360.L))+real(408918380.L))-real(432509825.L))/
        real(79290326402352.L);
      _C4x[185] = (_ep2*((real(1414224.L)-real(1344672.L)*_ep2)*_ep2-
        real(1489986.L))+real(1572763.L))/real(292126914288.L);
      _C4x[186] = ((real(1178520.L)-real(1120560.L)*_ep2)*_ep2-real(1241655.L))/
        real(234094968352.L);
      _C4x[187] = (real(12627.L)-real(12006.L)*_ep2)/real(2420154748.L);
      _C4x[188] = -real(29.L)/real(5657872.L);
      _C4x[189] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(24025340617603153920.L)*_ep2-real(25268030649548144640.L))+
        real(26621675148631080960.L))-real(28100657101332807680.L))+
        real(29721848857178931200.L))-real(31505159788609667072.L))+
        real(33474232275397771264.L))-real(35657334380315017216.L))+
        real(38088516269881950208.L))-real(40809124574873518080.L))+
        real(43869808917989031936.L))-real(47333214885198692352.L))+
        real(51277649458965250048.L))-real(55802147940638654464.L))+
        real(61033599310073528320.L))-real(67136959241080881152.L))+
        real(74330204874053832704.L))-real(82906766974906198016.L))+
        real(93270112846769472768.L))-real(105988764598601673600.L))+
        real(121887079288391924640.L))-real(142201592503123912080.L))+
        real(168864391097459645595.L))/real(4433500815385583503167606000.L);
      _C4x[190] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1501583788600197120.L)*_ep2-real(1579251915596759040.L))+
        real(1663854696789442560.L))-real(1756291068833300480.L))+
        real(1857615553573683200.L))-real(1969072486788104192.L))+
        real(2092139517212360704.L))-real(2228583398769688576.L))+
        real(2380532266867621888.L))-real(2550570285929594880.L))+
        real(2741863057374314496.L))-real(2958325930324918272.L))+
        real(3204853091185328128.L))-real(3487634246289915904.L))+
        real(3814599956879595520.L))-real(4196059952567555072.L))+
        real(4645637804628364544.L))-real(5181672935931637376.L))+
        real(5829382052923092048.L))-real(6624297787412604600.L))+
        real(7617942455524495290.L))-real(8887599531445244505.L))/
        real(65198541402729169164229500.L);
      _C4x[191] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(750791894300098560.L)*_ep2-real(789625957798379520.L))+
        real(831927348394721280.L))-real(878145534416650240.L))+
        real(928807776786841600.L))-real(984536243394052096.L))+
        real(1046069758606180352.L))-real(1114291699384844288.L))+
        real(1190266133433810944.L))-real(1275285142964797440.L))+
        real(1370931528687157248.L))-real(1479162965162459136.L))+
        real(1602426545592664064.L))-real(1743817123144957952.L))+
        real(1907299978439797760.L))-real(2098029976283777536.L))+
        real(2322818902314182272.L))-real(2590836467965818688.L))+
        real(2914691026461546024.L))-real(3312148893706302300.L))+
        real(3808971227762247645.L))/real(12963452676566033634993000.L);
      _C4x[192] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(187697973575024640.L)*_ep2-real(197406489449594880.L))+
        real(207981837098680320.L))-real(219536383604162560.L))+
        real(232201944196710400.L))-real(246134060848513024.L))+
        real(261517439651545088.L))-real(278572924846211072.L))+
        real(297566533358452736.L))-real(318821285741199360.L))+
        real(342732882171789312.L))-real(369790741290614784.L))+
        real(400606636398166016.L))-real(435954280786239488.L))+
        real(476824994609949440.L))-real(524507494070944384.L))+
        real(580704725578545568.L))-real(647709116991454672.L))+
        real(728672756615386506.L))-real(828037223426575575.L))/
        real(1666729629844204324499100.L);
      _C4x[193] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(93848986787512320.L)*
        _ep2-real(98703244724797440.L))+real(103990918549340160.L))-
        real(109768191802081280.L))+real(116100972098355200.L))-
        real(123067030424256512.L))+real(130758719825772544.L))-
        real(139286462423105536.L))+real(148783266679226368.L))-
        real(159410642870599680.L))+real(171366441085894656.L))-
        real(184895370645307392.L))+real(200303318199083008.L))-
        real(217977140393119744.L))+real(238412497304974720.L))-
        real(262253747035472192.L))+real(290352362789272784.L))-
        real(323854558495727336.L))+real(364336378307693253.L))/
        real(500677675368219480877200.L);
      _C4x[194] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11731123348439040.L)*_ep2-
        real(12337905590599680.L))+real(12998864818667520.L))-
        real(13721023975260160.L))+real(14512621512294400.L))-
        real(15383378803032064.L))+real(16344839978221568.L))-
        real(17410807802888192.L))+real(18597908334903296.L))-
        real(19926330358824960.L))+real(21420805135736832.L))-
        real(23111921330663424.L))+real(25037914774885376.L))-
        real(27247142549139968.L))+real(29801562163121840.L))-
        real(32781718379434024.L))+real(36294045348659098.L))-
        real(40481819811965917.L))/real(41723139614018290073100.L);
      _C4x[195] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(14977537297339899000600.L);
      _C4x[196] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(2840567418461015327700.L);
      _C4x[197] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(1124009473111455527520.L);
      _C4x[198] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(22912350289920.L)*_ep2-real(24097471856640.L))+
        real(25388407848960.L))-real(26798874951680.L))+real(28344963891200.L))-
        real(30045661724672.L))+real(31923515582464.L))-real(34005483990016.L))+
        real(36324039716608.L))-real(38918613982080.L))+real(41837510030736.L))-
        real(45140471348952.L))+real(48902177294698.L))-real(53217075291289.L))/
        real(28738878573872442465.L);
      _C4x[199] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(12075159064652286750.L);
      _C4x[200] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(2592714333701617125.L);
      _C4x[201] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(1133656469796658500.L);
      _C4x[202] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(125808217989629175.L);
      _C4x[203] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(56564935220143350.L);
      _C4x[204] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(12855667095487125.L);
      _C4x[205] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(5898622460094000.L);
      _C4x[206] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(341076808917000.L);
      _C4x[207] = (_ep2*(_ep2*(_ep2*(real(69922944.L)*_ep2-real(73539648.L))+
        real(77479272.L))-real(81783676.L))+real(86501965.L))/
        real(31780333254384.L);
      _C4x[208] = (_ep2*(_ep2*(real(17480736.L)*_ep2-real(18384912.L))+
        real(19369818.L))-real(20445919.L))/real(7449236314344.L);
      _C4x[209] = (_ep2*(real(2913456.L)*_ep2-real(3064152.L))+real(3228303.L))/
        real(1170474841760.L);
      _C4x[210] = (real(17342.L)*_ep2-real(18239.L))/real(6600422040.L);
      _C4x[211] = real(8671.L)/real(3140118960.L);
      _C4x[212] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1579251915596759040.L)-real(1501583788600197120.L)*_ep2)*_ep2-
        real(1663854696789442560.L))+real(1756291068833300480.L))-
        real(1857615553573683200.L))+real(1969072486788104192.L))-
        real(2092139517212360704.L))+real(2228583398769688576.L))-
        real(2380532266867621888.L))+real(2550570285929594880.L))-
        real(2741863057374314496.L))+real(2958325930324918272.L))-
        real(3204853091185328128.L))+real(3487634246289915904.L))-
        real(3814599956879595520.L))+real(4196059952567555072.L))-
        real(4645637804628364544.L))+real(5181672935931637376.L))-
        real(5829382052923092048.L))+real(6624297787412604600.L))-
        real(7617942455524495290.L))+real(8887599531445244505.L))/
        real(1256158564359248659230821700.L);
      _C4x[213] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(789625957798379520.L)-real(750791894300098560.L)*_ep2)*_ep2-
        real(831927348394721280.L))+real(878145534416650240.L))-
        real(928807776786841600.L))+real(984536243394052096.L))-
        real(1046069758606180352.L))+real(1114291699384844288.L))-
        real(1190266133433810944.L))+real(1275285142964797440.L))-
        real(1370931528687157248.L))+real(1479162965162459136.L))-
        real(1602426545592664064.L))+real(1743817123144957952.L))-
        real(1907299978439797760.L))+real(2098029976283777536.L))-
        real(2322818902314182272.L))+real(2590836467965818688.L))-
        real(2914691026461546024.L))+real(3312148893706302300.L))-
        real(3808971227762247645.L))/real(132227217300973543076928600.L);
      _C4x[214] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(197406489449594880.L)-
        real(187697973575024640.L)*_ep2)*_ep2-real(207981837098680320.L))+
        real(219536383604162560.L))-real(232201944196710400.L))+
        real(246134060848513024.L))-real(261517439651545088.L))+
        real(278572924846211072.L))-real(297566533358452736.L))+
        real(318821285741199360.L))-real(342732882171789312.L))+
        real(369790741290614784.L))-real(400606636398166016.L))+
        real(435954280786239488.L))-real(476824994609949440.L))+
        real(524507494070944384.L))-real(580704725578545568.L))+
        real(647709116991454672.L))-real(728672756615386506.L))+
        real(828037223426575575.L))/real(11963414898659511040293540.L);
      _C4x[215] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(98703244724797440.L)-
        real(93848986787512320.L)*_ep2)*_ep2-real(103990918549340160.L))+
        real(109768191802081280.L))-real(116100972098355200.L))+
        real(123067030424256512.L))-real(130758719825772544.L))+
        real(139286462423105536.L))-real(148783266679226368.L))+
        real(159410642870599680.L))-real(171366441085894656.L))+
        real(184895370645307392.L))-real(200303318199083008.L))+
        real(217977140393119744.L))-real(238412497304974720.L))+
        real(262253747035472192.L))-real(290352362789272784.L))+
        real(323854558495727336.L))-real(364336378307693253.L))/
        real(2837173493753243724970800.L);
      _C4x[216] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(198602144562727060747956.L);
      _C4x[217] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(62239988324501358069160.L);
      _C4x[218] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(10577732005888161839340.L);
      _C4x[219] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(3821632208578948793568.L);
      _C4x[220] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(24097471856640.L)-real(22912350289920.L)*_ep2)*_ep2-
        real(25388407848960.L))+real(26798874951680.L))-real(28344963891200.L))+
        real(30045661724672.L))-real(31923515582464.L))+real(34005483990016.L))-
        real(36324039716608.L))+real(38918613982080.L))-real(41837510030736.L))+
        real(45140471348952.L))-real(48902177294698.L))+real(53217075291289.L))/
        real(90474247362191022575.L);
      _C4x[221] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(35581468710508738290.L);
      _C4x[222] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(7212459873751771275.L);
      _C4x[223] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(2997891553462274700.L);
      _C4x[224] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/real(318068981891729145.L);
      _C4x[225] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(137371985534633850.L);
      _C4x[226] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(30110829152540955.L);
      _C4x[227] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(13370210909546400.L);
      _C4x[228] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(750368979617400.L);
      _C4x[229] = (_ep2*(_ep2*((real(367698240.L)-real(349614720.L)*_ep2)*_ep2-
        real(387396360.L))+real(408918380.L))-real(432509825.L))/
        real(340167270759888.L);
      _C4x[230] = (_ep2*((real(18384912.L)-real(17480736.L)*_ep2)*_ep2-
        real(19369818.L))+real(20445919.L))/real(15551914410648.L);
      _C4x[231] = ((real(5106920.L)-real(4855760.L)*_ep2)*_ep2-real(5380505.L))/
        real(3979614461984.L);
      _C4x[232] = (real(383019.L)-real(364182.L)*_ep2)/real(276777697544.L);
      _C4x[233] = -real(8671.L)/real(6147020752.L);
      _C4x[234] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(750791894300098560.L)*_ep2-real(789625957798379520.L))+
        real(831927348394721280.L))-real(878145534416650240.L))+
        real(928807776786841600.L))-real(984536243394052096.L))+
        real(1046069758606180352.L))-real(1114291699384844288.L))+
        real(1190266133433810944.L))-real(1275285142964797440.L))+
        real(1370931528687157248.L))-real(1479162965162459136.L))+
        real(1602426545592664064.L))-real(1743817123144957952.L))+
        real(1907299978439797760.L))-real(2098029976283777536.L))+
        real(2322818902314182272.L))-real(2590836467965818688.L))+
        real(2914691026461546024.L))-real(3312148893706302300.L))+
        real(3808971227762247645.L))/real(2807883849744202885339483800.L);
      _C4x[235] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(187697973575024640.L)*_ep2-real(197406489449594880.L))+
        real(207981837098680320.L))-real(219536383604162560.L))+
        real(232201944196710400.L))-real(246134060848513024.L))+
        real(261517439651545088.L))-real(278572924846211072.L))+
        real(297566533358452736.L))-real(318821285741199360.L))+
        real(342732882171789312.L))-real(369790741290614784.L))+
        real(400606636398166016.L))-real(435954280786239488.L))+
        real(476824994609949440.L))-real(524507494070944384.L))+
        real(580704725578545568.L))-real(647709116991454672.L))+
        real(728672756615386506.L))-real(828037223426575575.L))/
        real(133708754749723946920927800.L);
      _C4x[236] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(93848986787512320.L)*
        _ep2-real(98703244724797440.L))+real(103990918549340160.L))-
        real(109768191802081280.L))+real(116100972098355200.L))-
        real(123067030424256512.L))+real(130758719825772544.L))-
        real(139286462423105536.L))+real(148783266679226368.L))-
        real(159410642870599680.L))+real(171366441085894656.L))-
        real(184895370645307392.L))+real(200303318199083008.L))-
        real(217977140393119744.L))+real(238412497304974720.L))-
        real(262253747035472192.L))+real(290352362789272784.L))-
        real(323854558495727336.L))+real(364336378307693253.L))/
        real(22196710274657730318889200.L);
      _C4x[237] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11731123348439040.L)*_ep2-
        real(12337905590599680.L))+real(12998864818667520.L))-
        real(13721023975260160.L))+real(14512621512294400.L))-
        real(15383378803032064.L))+real(16344839978221568.L))-
        real(17410807802888192.L))+real(18597908334903296.L))-
        real(19926330358824960.L))+real(21420805135736832.L))-
        real(23111921330663424.L))+real(25037914774885376.L))-
        real(27247142549139968.L))+real(29801562163121840.L))-
        real(32781718379434024.L))+real(36294045348659098.L))-
        real(40481819811965917.L))/real(1220819065106175167538906.L);
      _C4x[238] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(319986763503612864426152.L);
      _C4x[239] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(47288684261617664693520.L);
      _C4x[240] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(15254414277941182159200.L);
      _C4x[241] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(91649401159680.L)*_ep2-real(96389887426560.L))+
        real(101553631395840.L))-real(107195499806720.L))+
        real(113379855564800.L))-real(120182646898688.L))+
        real(127694062329856.L))-real(136021935960064.L))+
        real(145296158866432.L))-real(155674455928320.L))+
        real(167350040122944.L))-real(180561885395808.L))+
        real(195608709178792.L))-real(212868301165156.L))/
        real(1314537594027128386825.L);
      _C4x[242] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(119302571558764593090.L);
      _C4x[243] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(22570756781387895990.L);
      _C4x[244] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(8833359604051943100.L);
      _C4x[245] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(358005473280.L)*_ep2-real(376522997760.L))+real(396693872640.L))-
        real(418732421120.L))+real(442890060800.L))-real(469463464448.L))+
        real(498804930976.L))-real(531335687344.L))+real(567563120572.L))-
        real(608103343470.L))/real(1777444310571427575.L);
      _C4x[246] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(366118097194114650.L);
      _C4x[247] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(76921782036743280.L);
      _C4x[248] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(32874989177590560.L);
      _C4x[249] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(1782126326591325.L);
      _C4x[250] = (_ep2*(_ep2*(_ep2*(real(69922944.L)*_ep2-real(73539648.L))+
        real(77479272.L))-real(81783676.L))+real(86501965.L))/
        real(156547567512336.L);
      _C4x[251] = (_ep2*(_ep2*(real(17480736.L)*_ep2-real(18384912.L))+
        real(19369818.L))-real(20445919.L))/real(34763102800272.L);
      _C4x[252] = (_ep2*(real(4855760.L)*_ep2-real(5106920.L))+real(5380505.L))/
        real(8661513829024.L);
      _C4x[253] = (real(1820910.L)*_ep2-real(1915095.L))/real(2938727906276.L);
      _C4x[254] = real(4669.L)/real(6870199664.L);
      _C4x[255] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(197406489449594880.L)-
        real(187697973575024640.L)*_ep2)*_ep2-real(207981837098680320.L))+
        real(219536383604162560.L))-real(232201944196710400.L))+
        real(246134060848513024.L))-real(261517439651545088.L))+
        real(278572924846211072.L))-real(297566533358452736.L))+
        real(318821285741199360.L))-real(342732882171789312.L))+
        real(369790741290614784.L))-real(400606636398166016.L))+
        real(435954280786239488.L))-real(476824994609949440.L))+
        real(524507494070944384.L))-real(580704725578545568.L))+
        real(647709116991454672.L))-real(728672756615386506.L))+
        real(828037223426575575.L))/real(3103450570769908452217324200.L);
      _C4x[256] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(98703244724797440.L)-
        real(93848986787512320.L)*_ep2)*_ep2-real(103990918549340160.L))+
        real(109768191802081280.L))-real(116100972098355200.L))+
        real(123067030424256512.L))-real(130758719825772544.L))+
        real(139286462423105536.L))-real(148783266679226368.L))+
        real(159410642870599680.L))-real(171366441085894656.L))+
        real(184895370645307392.L))-real(200303318199083008.L))+
        real(217977140393119744.L))-real(238412497304974720.L))+
        real(262253747035472192.L))-real(290352362789272784.L))+
        real(323854558495727336.L))-real(364336378307693253.L))/
        real(269865267023470300192810800.L);
      _C4x[257] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(10344835235899694840724414.L);
      _C4x[258] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(2122017484287116890405008.L);
      _C4x[259] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(261332202498413410148400.L);
      _C4x[260] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(73060615752244609288800.L);
      _C4x[261] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(96389887426560.L)-real(91649401159680.L)*_ep2)*_ep2-
        real(101553631395840.L))+real(107195499806720.L))-
        real(113379855564800.L))+real(120182646898688.L))-
        real(127694062329856.L))+real(136021935960064.L))-
        real(145296158866432.L))+real(155674455928320.L))-
        real(167350040122944.L))+real(180561885395808.L))-
        real(195608709178792.L))+real(212868301165156.L))/
        real(5604081321905126280675.L);
      _C4x[262] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(461512579451010399585.L);
      _C4x[263] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(80383572396872682210.L);
      _C4x[264] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(29289560792382758700.L);
      _C4x[265] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(376522997760.L)-real(358005473280.L)*_ep2)*_ep2-
        real(396693872640.L))+real(418732421120.L))-real(442890060800.L))+
        real(469463464448.L))-real(498804930976.L))+real(531335687344.L))-
        real(567563120572.L))+real(608103343470.L))/
        real(5536441369291862925.L);
      _C4x[266] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(1079084918045811600.L);
      _C4x[267] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(215816983609162320.L);
      _C4x[268] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(88243392003006240.L);
      _C4x[269] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(4596010000156575.L);
      _C4x[270] = (_ep2*(_ep2*((real(73539648.L)-real(69922944.L)*_ep2)*_ep2-
        real(77479272.L))+real(81783676.L))-real(86501965.L))/
        real(389309082366204.L);
      _C4x[271] = (_ep2*((real(18384912.L)-real(17480736.L)*_ep2)*_ep2-
        real(19369818.L))+real(20445919.L))/real(83625172990128.L);
      _C4x[272] = ((real(15320760.L)-real(14567280.L)*_ep2)*_ep2-
        real(16141515.L))/real(60630596803168.L);
      _C4x[273] = (real(7015.L)-real(6670.L)*_ep2)/real(24421561548.L);
      _C4x[274] = -real(667.L)/real(2169536736.L);
      _C4x[275] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(93848986787512320.L)*
        _ep2-real(98703244724797440.L))+real(103990918549340160.L))-
        real(109768191802081280.L))+real(116100972098355200.L))-
        real(123067030424256512.L))+real(130758719825772544.L))-
        real(139286462423105536.L))+real(148783266679226368.L))-
        real(159410642870599680.L))+real(171366441085894656.L))-
        real(184895370645307392.L))+real(200303318199083008.L))-
        real(217977140393119744.L))+real(238412497304974720.L))-
        real(262253747035472192.L))+real(290352362789272784.L))-
        real(323854558495727336.L))+real(364336378307693253.L))/
        real(6798034583591228038190329200.L);
      _C4x[276] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11731123348439040.L)*_ep2-
        real(12337905590599680.L))+real(12998864818667520.L))-
        real(13721023975260160.L))+real(14512621512294400.L))-
        real(15383378803032064.L))+real(16344839978221568.L))-
        real(17410807802888192.L))+real(18597908334903296.L))-
        real(19926330358824960.L))+real(21420805135736832.L))-
        real(23111921330663424.L))+real(25037914774885376.L))-
        real(27247142549139968.L))+real(29801562163121840.L))-
        real(32781718379434024.L))+real(36294045348659098.L))-
        real(40481819811965917.L))/real(135960691671824560763806584.L);
      _C4x[277] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(19367619896271304952109200.L);
      _C4x[278] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(1860436393976800229389800.L);
      _C4x[279] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(432101356020418117793760.L);
      _C4x[280] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(45824700579840.L)*_ep2-real(48194943713280.L))+
        real(50776815697920.L))-real(53597749903360.L))+real(56689927782400.L))-
        real(60091323449344.L))+real(63847031164928.L))-real(68010967980032.L))+
        real(72648079433216.L))-real(77837227964160.L))+real(83675020061472.L))-
        real(90280942697904.L))+real(97804354589396.L))-
        real(106434150582578.L))/real(14321541155979767161725.L);
      _C4x[281] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(2094074084991999568185.L);
      _C4x[282] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(5728087572480.L)*_ep2-real(6024367964160.L))+
        real(6347101962240.L))-real(6699718737920.L))+real(7086240972800.L))-
        real(7511415431168.L))+real(7980878895616.L))-real(8501370997504.L))+
        real(9081009929152.L))-real(9729653495520.L))+real(10459377507684.L))-
        real(11285117837238.L))/real(660293630402882746725.L);
      _C4x[283] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(110494480661211147900.L);
      _C4x[284] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(9701954399520978840.L);
      _C4x[285] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(3545564730721952400.L);
      _C4x[286] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(669717782469702120.L);
      _C4x[287] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(486420480.L)*_ep2-
        real(511580160.L))+real(538986240.L))-real(568929920.L))+
        real(601752800.L))-real(637857968.L))+real(677724091.L))/
        real(11313255385000800.L);
      _C4x[288] = (_ep2*(_ep2*(_ep2*(_ep2*(real(30401280.L)*_ep2-
        real(31973760.L))+real(33686640.L))-real(35558120.L))+real(37609550.L))-
        real(39866123.L))/real(562776734713050.L);
      _C4x[289] = (_ep2*(_ep2*(_ep2*(real(15200640.L)*_ep2-real(15986880.L))+
        real(16843320.L))-real(17779060.L))+real(18804775.L))/
        real(228641842024596.L);
      _C4x[290] = (_ep2*(_ep2*(real(760032.L)*_ep2-real(799344.L))+
        real(842166.L))-real(888953.L))/real(9457608850074.L);
      _C4x[291] = (_ep2*(real(48720.L)*_ep2-real(51240.L))+real(53985.L))/
        real(509500813472.L);
      _C4x[292] = (real(3654.L)*_ep2-real(3843.L))/real(32562082064.L);
      _C4x[293] = real(203.L)/real(1560543968.L);
      _C4x[294] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(12337905590599680.L)-
        real(11731123348439040.L)*_ep2)*_ep2-real(12998864818667520.L))+
        real(13721023975260160.L))-real(14512621512294400.L))+
        real(15383378803032064.L))-real(16344839978221568.L))+
        real(17410807802888192.L))-real(18597908334903296.L))+
        real(19926330358824960.L))-real(21420805135736832.L))+
        real(23111921330663424.L))-real(25037914774885376.L))+
        real(27247142549139968.L))-real(29801562163121840.L))+
        real(32781718379434024.L))-real(36294045348659098.L))+
        real(40481819811965917.L))/real(3694584012821319585973005000.L);
      _C4x[295] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*((real(6168952795299840.L)-
        real(5865561674219520.L)*_ep2)*_ep2-real(6499432409333760.L))+
        real(6860511987630080.L))-real(7256310756147200.L))+
        real(7691689401516032.L))-real(8172419989110784.L))+
        real(8705403901444096.L))-real(9298954167451648.L))+
        real(9963165179412480.L))-real(10710402567868416.L))+
        real(11555960665331712.L))-real(12518957387442688.L))+
        real(13623571274569984.L))-real(14900781081560920.L))+
        real(16390859189717012.L))-real(18147022674329549.L))/
        real(273672889838616265627630000.L);
      _C4x[296] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(18199921245425219635335000.L);
      _C4x[297] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(3287727708851007417996000.L);
      _C4x[298] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(48194943713280.L)-real(45824700579840.L)*_ep2)*_ep2-
        real(50776815697920.L))+real(53597749903360.L))-real(56689927782400.L))+
        real(60091323449344.L))-real(63847031164928.L))+real(68010967980032.L))-
        real(72648079433216.L))+real(77837227964160.L))-real(83675020061472.L))+
        real(90280942697904.L))-real(97804354589396.L))+
        real(106434150582578.L))/real(90287976852915923410875.L);
      _C4x[299] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(11380837418434780261875.L);
      _C4x[300] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(6024367964160.L)-real(5728087572480.L)*_ep2)*_ep2-
        real(6347101962240.L))+real(6699718737920.L))-real(7086240972800.L))+
        real(7511415431168.L))-real(7980878895616.L))+real(8501370997504.L))-
        real(9081009929152.L))+real(9729653495520.L))-real(10459377507684.L))+
        real(11285117837238.L))/real(3178432071815118811875.L);
      _C4x[301] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(480410785483526730000.L);
      _C4x[302] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/
        real(38667209563308249000.L);
      _C4x[303] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(13103174004841998000.L);
      _C4x[304] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(2316217627118535000.L);
      _C4x[305] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(848494153875060000.L);
      _C4x[306] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(40043729200736250.L);
      _C4x[307] = (_ep2*(_ep2*((real(73539648.L)-real(69922944.L)*_ep2)*_ep2-
        real(77479272.L))+real(81783676.L))-real(86501965.L))/
        real(3102996427476660.L);
      _C4x[308] = (_ep2*((real(1414224.L)-real(1344672.L)*_ep2)*_ep2-
        real(1489986.L))+real(1572763.L))/real(47288044250370.L);
      _C4x[309] = ((real(235704.L)-real(224112.L)*_ep2)*_ep2-real(248331.L))/
        real(6368760168400.L);
      _C4x[310] = (real(88389.L)-real(84042.L)*_ep2)/real(1963302006800.L);
      _C4x[311] = -real(2001.L)/real(39013599200.L);
      _C4x[312] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(5865561674219520.L)*_ep2-
        real(6168952795299840.L))+real(6499432409333760.L))-
        real(6860511987630080.L))+real(7256310756147200.L))-
        real(7691689401516032.L))+real(8172419989110784.L))-
        real(8705403901444096.L))+real(9298954167451648.L))-
        real(9963165179412480.L))+real(10710402567868416.L))-
        real(11555960665331712.L))+real(12518957387442688.L))-
        real(13623571274569984.L))+real(14900781081560920.L))-
        real(16390859189717012.L))+real(18147022674329549.L))/
        real(7980301467694050305701690800.L);
      _C4x[313] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(_ep2*(real(1466390418554880.L)*_ep2-
        real(1542238198824960.L))+real(1624858102333440.L))-
        real(1715127996907520.L))+real(1814077689036800.L))-
        real(1922922350379008.L))+real(2043104997277696.L))-
        real(2176350975361024.L))+real(2324738541862912.L))-
        real(2490791294853120.L))+real(2677600641967104.L))-
        real(2888990166332928.L))+real(3129739346860672.L))-
        real(3405892818642496.L))+real(3725195270390230.L))-
        real(4097714797429253.L))/real(275182809230829320886265200.L);
      _C4x[314] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(34323877280404517443878240.L);
      _C4x[315] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(91649401159680.L)*_ep2-real(96389887426560.L))+
        real(101553631395840.L))-real(107195499806720.L))+
        real(113379855564800.L))-real(120182646898688.L))+
        real(127694062329856.L))-real(136021935960064.L))+
        real(145296158866432.L))-real(155674455928320.L))+
        real(167350040122944.L))-real(180561885395808.L))+
        real(195608709178792.L))-real(212868301165156.L))/
        real(1462665225017237959256175.L);
      _C4x[316] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(76206087353839288633515.L);
      _C4x[317] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(9153884366827542178200.L);
      _C4x[318] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(2445977199233270379600.L);
      _C4x[319] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(177482491895584862910.L);
      _C4x[320] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(55033330820336391600.L);
      _C4x[321] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(9005454134236864080.L);
      _C4x[322] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(3082347853531581600.L);
      _C4x[323] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(136949553866517975.L);
      _C4x[324] = (_ep2*(_ep2*(_ep2*(real(349614720.L)*_ep2-real(367698240.L))+
        real(387396360.L))-real(408918380.L))+real(432509825.L))/
        real(50268542125121892.L);
      _C4x[325] = (_ep2*(_ep2*(real(1344672.L)*_ep2-real(1414224.L))+
        real(1489986.L))-real(1572763.L))/real(145917393686856.L);
      _C4x[326] = (_ep2*(real(373520.L)*_ep2-real(392840.L))+real(413885.L))/
        real(31334300028528.L);
      _C4x[327] = (real(1334.L)*_ep2-real(1403.L))/real(88348590306.L);
      _C4x[328] = real(667.L)/real(35525324448.L);
      _C4x[329] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*((real(1542238198824960.L)-real(1466390418554880.L)*
        _ep2)*_ep2-real(1624858102333440.L))+real(1715127996907520.L))-
        real(1814077689036800.L))+real(1922922350379008.L))-
        real(2043104997277696.L))+real(2176350975361024.L))-
        real(2324738541862912.L))+real(2490791294853120.L))-
        real(2677600641967104.L))+real(2888990166332928.L))-
        real(3129739346860672.L))+real(3405892818642496.L))-
        real(3725195270390230.L))+real(4097714797429253.L))/
        real(8571434909745461439457371600.L);
      _C4x[330] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*((real(771119099412480.L)-real(733195209277440.L)*_ep2)*
        _ep2-real(812429051166720.L))+real(857563998453760.L))-
        real(907038844518400.L))+real(961461175189504.L))-
        real(1021552498638848.L))+real(1088175487680512.L))-
        real(1162369270931456.L))+real(1245395647426560.L))-
        real(1338800320983552.L))+real(1444495083166464.L))-
        real(1564869673430336.L))+real(1702946409321248.L))-
        real(1862597635195115.L))/real(552995800628739447706927200.L);
      _C4x[331] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(96389887426560.L)-real(91649401159680.L)*_ep2)*_ep2-
        real(101553631395840.L))+real(107195499806720.L))-
        real(113379855564800.L))+real(120182646898688.L))-
        real(127694062329856.L))+real(136021935960064.L))-
        real(145296158866432.L))+real(155674455928320.L))-
        real(167350040122944.L))+real(180561885395808.L))-
        real(195608709178792.L))+real(212868301165156.L))/
        real(16233778238154283029275325.L);
      _C4x[332] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(654807861707063517147240.L);
      _C4x[333] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(64890869178177465663240.L);
      _C4x[334] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(14887243941012374038800.L);
      _C4x[335] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/
        real(953146715735548337850.L);
      _C4x[336] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(265994432298292559400.L);
      _C4x[337] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/real(39764824222371008880.L);
      _C4x[338] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(12580545683673344160.L);
      _C4x[339] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(521514967754315925.L);
      _C4x[340] = (_ep2*(_ep2*((real(73539648.L)-real(69922944.L)*_ep2)*_ep2-
        real(77479272.L))+real(81783676.L))-real(86501965.L))/
        real(35994758558729256.L);
      _C4x[341] = (_ep2*((real(18384912.L)-real(17480736.L)*_ep2)*_ep2-
        real(19369818.L))+real(20445919.L))/real(6425769670135992.L);
      _C4x[342] = ((real(1178520.L)-real(1120560.L)*_ep2)*_ep2-real(1241655.L))/
        real(302898233609104.L);
      _C4x[343] = (real(63135.L)-real(60030.L)*_ep2)/real(12241188012398.L);
      _C4x[344] = -real(69.L)/real(10854960248.L);
      _C4x[345] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(_ep2*(real(733195209277440.L)*_ep2-
        real(771119099412480.L))+real(812429051166720.L))-
        real(857563998453760.L))+real(907038844518400.L))-
        real(961461175189504.L))+real(1021552498638848.L))-
        real(1088175487680512.L))+real(1162369270931456.L))-
        real(1245395647426560.L))+real(1338800320983552.L))-
        real(1444495083166464.L))+real(1564869673430336.L))-
        real(1702946409321248.L))+real(1862597635195115.L))/
        real(18325136703593745146426104800.L);
      _C4x[346] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(_ep2*(real(22912350289920.L)*_ep2-real(24097471856640.L))+
        real(25388407848960.L))-real(26798874951680.L))+real(28344963891200.L))-
        real(30045661724672.L))+real(31923515582464.L))-real(34005483990016.L))+
        real(36324039716608.L))-real(38918613982080.L))+real(41837510030736.L))-
        real(45140471348952.L))+real(48902177294698.L))-real(53217075291289.L))/
        real(69413396604521761918280700.L);
      _C4x[347] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(7699637270417539977489960.L);
      _C4x[348] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(589611863049991800078060.L);
      _C4x[349] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(111397652937920178152400.L);
      _C4x[350] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(6113285831959034166900.L);
      _C4x[351] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(1502934058355377170600.L);
      _C4x[352] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(201909322991176933020.L);
      _C4x[353] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(58275401270348939040.L);
      _C4x[354] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(2229926069018454300.L);
      _C4x[355] = (_ep2*(_ep2*(_ep2*(real(69922944.L)*_ep2-real(73539648.L))+
        real(77479272.L))-real(81783676.L))+real(86501965.L))/
        real(143414853066284904.L);
      _C4x[356] = (_ep2*(_ep2*(real(17480736.L)*_ep2-real(18384912.L))+
        real(19369818.L))-real(20445919.L))/real(24041241696888108.L);
      _C4x[357] = (_ep2*(real(14567280.L)*_ep2-real(15320760.L))+
        real(16141515.L))/real(13922873979342608.L);
      _C4x[358] = (real(420210.L)*_ep2-real(441945.L))/real(287878973257084.L);
      _C4x[359] = real(667.L)/real(336503767688.L);
      _C4x[360] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*((real(24097471856640.L)-real(22912350289920.L)*_ep2)*_ep2-
        real(25388407848960.L))+real(26798874951680.L))-real(28344963891200.L))+
        real(30045661724672.L))-real(31923515582464.L))+real(34005483990016.L))-
        real(36324039716608.L))+real(38918613982080.L))-real(41837510030736.L))+
        real(45140471348952.L))-real(48902177294698.L))+real(53217075291289.L))/
        real(2438425448462070926742183300.L);
      _C4x[361] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(12048735928320.L)-real(11456175144960.L)*_ep2)*_ep2-
        real(12694203924480.L))+real(13399437475840.L))-real(14172481945600.L))+
        real(15022830862336.L))-real(15961757791232.L))+real(17002741995008.L))-
        real(18162019858304.L))+real(19459306991040.L))-real(20918755015368.L))+
        real(22570235674476.L))-real(24451088647349.L))/
        real(139338597054975481528124760.L);
      _C4x[362] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(7322598944330543323550100.L);
      _C4x[363] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(1067261384598783642298800.L);
      _C4x[364] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/
        real(48156916134335359469580.L);
      _C4x[365] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(10132684457944317053400.L);
      _C4x[366] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/
        real(1197499072302510197220.L);
      _C4x[367] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(310175522890566933600.L);
      _C4x[368] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(10813942549756160100.L);
      _C4x[369] = (_ep2*(_ep2*((real(367698240.L)-real(349614720.L)*_ep2)*_ep2-
        real(387396360.L))+real(408918380.L))-real(432509825.L))/
        real(3206015908868885112.L);
      _C4x[370] = (_ep2*((real(18384912.L)-real(17480736.L)*_ep2)*_ep2-
        real(19369818.L))+real(20445919.L))/real(100042586416082772.L);
      _C4x[371] = ((real(15320760.L)-real(14567280.L)*_ep2)*_ep2-
        real(16141515.L))/real(54344121016143728.L);
      _C4x[372] = (real(127673.L)-real(121394.L)*_ep2)/real(306451810241412.L);
      _C4x[373] = -real(203.L)/real(358213688184.L);
      _C4x[374] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (_ep2*(real(11456175144960.L)*_ep2-real(12048735928320.L))+
        real(12694203924480.L))-real(13399437475840.L))+real(14172481945600.L))-
        real(15022830862336.L))+real(15961757791232.L))-real(17002741995008.L))+
        real(18162019858304.L))-real(19459306991040.L))+real(20918755015368.L))-
        real(22570235674476.L))+real(24451088647349.L))/
        real(5172417617949847420362207000.L);
      _C4x[375] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(2864043786240.L)*_ep2-real(3012183982080.L))+
        real(3173550981120.L))-real(3349859368960.L))+real(3543120486400.L))-
        real(3755707715584.L))+real(3990439447808.L))-real(4250685498752.L))+
        real(4540504964576.L))-real(4864826747760.L))+real(5229688753842.L))-
        real(5642558918619.L))/real(139795070755401281631411000.L);
      _C4x[376] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(13960641343994190068454000.L);
      _C4x[377] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(485217412565651727988950.L);
      _C4x[378] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(83824935061175713805400.L);
      _C4x[379] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(8467165157694516546000.L);
      _C4x[380] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(1926847945229279436000.L);
      _C4x[381] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(60213998288414982375.L);
      _C4x[382] = (_ep2*(_ep2*(_ep2*(real(69922944.L)*_ep2-real(73539648.L))+
        real(77479272.L))-real(81783676.L))+real(86501965.L))/
        real(3249194574308196696.L);
      _C4x[383] = (_ep2*(_ep2*(real(17480736.L)*_ep2-real(18384912.L))+
        real(19369818.L))-real(20445919.L))/real(466865403275052936.L);
      _C4x[384] = (_ep2*(real(971152.L)*_ep2-real(1021384.L))+real(1076101.L))/
        real(15719373847644880.L);
      _C4x[385] = (real(2262.L)*_ep2-real(2379.L))/real(23216046230410.L);
      _C4x[386] = real(377.L)/real(2550915658280.L);
      _C4x[387] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(3012183982080.L)-real(2864043786240.L)*_ep2)*_ep2-
        real(3173550981120.L))+real(3349859368960.L))-real(3543120486400.L))+
        real(3755707715584.L))-real(3990439447808.L))+real(4250685498752.L))-
        real(4540504964576.L))+real(4864826747760.L))-real(5229688753842.L))+
        real(5642558918619.L))/real(5467984338975552987240047400.L);
      _C4x[388] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(1506091991040.L)-real(1432021893120.L)*_ep2)*_ep2-
        real(1586775490560.L))+real(1674929684480.L))-real(1771560243200.L))+
        real(1877853857792.L))-real(1995219723904.L))+real(2125342749376.L))-
        real(2270252482288.L))+real(2432413373880.L))-real(2614844376921.L))/
        real(280409453280797589089233200.L);
      _C4x[389] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/
        real(6668273584116528033219570.L);
      _C4x[390] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(886149313503857545942800.L);
      _C4x[391] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/
        real(73398225966986180573040.L);
      _C4x[392] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(14258674794696667826400.L);
      _C4x[393] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(391022331742319334525.L);
      _C4x[394] = (_ep2*(_ep2*((real(367698240.L)-real(349614720.L)*_ep2)*_ep2-
        real(387396360.L))+real(408918380.L))-real(432509825.L))/
        real(94458727981674003948.L);
      _C4x[395] = (_ep2*((real(18384912.L)-real(17480736.L)*_ep2)*_ep2-
        real(19369818.L))+real(20445919.L))/real(2467717131596708376.L);
      _C4x[396] = ((real(222040.L)-real(211120.L)*_ep2)*_ep2-real(233935.L))/
        real(16617623781796016.L);
      _C4x[397] = (real(16653.L)-real(15834.L)*_ep2)/real(734049170812418.L);
      _C4x[398] = -real(2639.L)/real(75507103485088.L);
      _C4x[399] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(1432021893120.L)*_ep2-real(1506091991040.L))+
        real(1586775490560.L))-real(1674929684480.L))+real(1771560243200.L))-
        real(1877853857792.L))+real(1995219723904.L))-real(2125342749376.L))+
        real(2270252482288.L))-real(2432413373880.L))+real(2614844376921.L))/
        real(11527102120002517108235775600.L);
      _C4x[400] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(179002736640.L)*_ep2-real(188261498880.L))+real(198346936320.L))-
        real(209366210560.L))+real(221445030400.L))-real(234731732224.L))+
        real(249402465488.L))-real(265667843672.L))+real(283781560286.L))-
        real(304051671735.L))/real(140574416097591672051655800.L);
      _C4x[401] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(12765340110744758702365200.L);
      _C4x[402] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(812339825229211917423240.L);
      _C4x[403] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(129252960165980064566880.L);
      _C4x[404] = (_ep2*(_ep2*(_ep2*(_ep2*(real(699229440.L)*_ep2-
        real(735396480.L))+real(774792720.L))-real(817836760.L))+
        real(865019650.L))-real(916920829.L))/real(3022496942656846747950.L);
      _C4x[405] = (_ep2*(_ep2*(_ep2*(real(69922944.L)*_ep2-real(73539648.L))+
        real(77479272.L))-real(81783676.L))+real(86501965.L))/
        real(128011635218407626972.L);
      _C4x[406] = (_ep2*(_ep2*(real(58464.L)*_ep2-real(61488.L))+real(64782.L))-
        real(68381.L))/real(50021293208041386.L);
      _C4x[407] = (_ep2*(real(48720.L)*_ep2-real(51240.L))+real(53985.L))/
        real(21108873452551696.L);
      _C4x[408] = (real(2030.L)*_ep2-real(2135.L))/real(476140002689136.L);
      _C4x[409] = real(29.L)/real(3895943914464.L);
      _C4x[410] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        ((real(188261498880.L)-real(179002736640.L)*_ep2)*_ep2-
        real(198346936320.L))+real(209366210560.L))-real(221445030400.L))+
        real(234731732224.L))-real(249402465488.L))+real(265667843672.L))-
        real(283781560286.L))+real(304051671735.L))/
        real(6059117781026964120995728200.L);
      _C4x[411] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(94130749440.L)-
        real(89501368320.L)*_ep2)*_ep2-real(99173468160.L))+
        real(104683105280.L))-real(110722515200.L))+real(117365866112.L))-
        real(124701232744.L))+real(132833921836.L))-real(141890780143.L))/
        real(281819431675672749813754800.L);
      _C4x[412] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/
        real(12240641981872654789890360.L);
      _C4x[413] = (_ep2*(_ep2*(_ep2*(_ep2*((real(11766343680.L)-
        real(11187671040.L)*_ep2)*_ep2-real(12396683520.L))+
        real(13085388160.L))-real(13840314400.L))+real(14670733264.L))-
        real(15587654093.L))/real(1494694488073256644093920.L);
      _C4x[414] = (_ep2*(_ep2*(_ep2*((real(735396480.L)-real(699229440.L)*_ep2)*
        _ep2-real(774792720.L))+real(817836760.L))-real(865019650.L))+
        real(916920829.L))/real(28597471072830165384450.L);
      _C4x[415] = (_ep2*(_ep2*((real(3197376.L)-real(3040128.L)*_ep2)*_ep2-
        real(3368664.L))+real(3555812.L))-real(3760955.L))/
        real(44858778153459082956.L);
      _C4x[416] = (_ep2*((real(799344.L)-real(760032.L)*_ep2)*_ep2-
        real(842166.L))+real(888953.L))/real(4590049143423607182.L);
      _C4x[417] = ((real(666120.L)-real(633360.L)*_ep2)*_ep2-real(701805.L))/
        real(1730927623109239072.L);
      _C4x[418] = (real(35685.L)-real(33930.L)*_ep2)/real(45550726923927344.L);
      _C4x[419] = -real(377.L)/real(266222834155040.L);
      _C4x[420] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*
        (real(89501368320.L)*_ep2-real(94130749440.L))+real(99173468160.L))-
        real(104683105280.L))+real(110722515200.L))-real(117365866112.L))+
        real(124701232744.L))-real(132833921836.L))+real(141890780143.L))/
        real(12709369004105339375747137200.L);
      _C4x[421] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(22375342080.L)*_ep2-
        real(23532687360.L))+real(24793367040.L))-real(26170776320.L))+
        real(27680628800.L))-real(29341466528.L))+real(31175308186.L))-
        real(33208480459.L))/real(282430422313451986127714160.L);
      _C4x[422] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(11187671040.L)*_ep2-
        real(11766343680.L))+real(12396683520.L))-real(13085388160.L))+
        real(13840314400.L))-real(14670733264.L))+real(15587654093.L))/
        real(23514096214810988669282400.L);
      _C4x[423] = (_ep2*(_ep2*(_ep2*(_ep2*(real(30401280.L)*_ep2-
        real(31973760.L))+real(33686640.L))-real(35558120.L))+real(37609550.L))-
        real(39866123.L))/real(14996234830874355018675.L);
      _C4x[424] = (_ep2*(_ep2*(_ep2*(real(15200640.L)*_ep2-real(15986880.L))+
        real(16843320.L))-real(17779060.L))+real(18804775.L))/
        real(2211209528003434308636.L);
      _C4x[425] = (_ep2*(_ep2*(real(760032.L)*_ep2-real(799344.L))+
        real(842166.L))-real(888953.L))/real(38511631837505387088.L);
      _C4x[426] = (_ep2*(real(633360.L)*_ep2-real(666120.L))+real(701805.L))/
        real(12707541818436120992.L);
      _C4x[427] = (real(6786.L)*_ep2-real(7137.L))/real(59715892003929140.L);
      _C4x[428] = real(1131.L)/real(4746558335788640.L);
      _C4x[429] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*((real(23532687360.L)-
        real(22375342080.L)*_ep2)*_ep2-real(24793367040.L))+
        real(26170776320.L))-real(27680628800.L))+real(29341466528.L))-
        real(31175308186.L))+real(33208480459.L))/
        real(13300502446156750509502818000.L);
      _C4x[430] = (_ep2*(_ep2*(_ep2*(_ep2*((real(511580160.L)-real(486420480.L)*
        _ep2)*_ep2-real(538986240.L))+real(568929920.L))-real(601752800.L))+
        real(637857968.L))-real(677724091.L))/
        real(24607775108523127677156000.L);
      _C4x[431] = (_ep2*(_ep2*(_ep2*((real(31973760.L)-real(30401280.L)*_ep2)*
        _ep2-real(33686640.L))+real(35558120.L))-real(37609550.L))+
        real(39866123.L))/real(245868501296893495073625.L);
      _C4x[432] = (_ep2*(_ep2*((real(3197376.L)-real(3040128.L)*_ep2)*_ep2-
        real(3368664.L))+real(3555812.L))-real(3760955.L))/
        real(5553735558706300124016.L);
      _C4x[433] = (_ep2*((real(799344.L)-real(760032.L)*_ep2)*_ep2-
        real(842166.L))+real(888953.L))/real(394968131170694783856.L);
      _C4x[434] = ((real(44408.L)-real(42224.L)*_ep2)*_ep2-real(46787.L))/
        real(7388105708393093600.L);
      _C4x[435] = (real(793.L)-real(754.L)*_ep2)/real(50589875285720700.L);
      _C4x[436] = -real(29.L)/real(827888081823600.L);
      _C4x[437] = (_ep2*(_ep2*(_ep2*(_ep2*(_ep2*(real(486420480.L)*_ep2-
        real(511580160.L))+real(538986240.L))-real(568929920.L))+
        real(601752800.L))-real(637857968.L))+real(677724091.L))/
        real(1207968338105057534196391200.L);
      _C4x[438] = (_ep2*(_ep2*(_ep2*(_ep2*(real(30401280.L)*_ep2-
        real(31973760.L))+real(33686640.L))-real(35558120.L))+real(37609550.L))-
        real(39866123.L))/real(6163103765842130276512200.L);
      _C4x[439] = (_ep2*(_ep2*(_ep2*(real(15200640.L)*_ep2-real(15986880.L))+
        real(16843320.L))-real(17779060.L))+real(18804775.L))/
        real(473713073766689229096624.L);
      _C4x[440] = (_ep2*(_ep2*(real(760032.L)*_ep2-real(799344.L))+
        real(842166.L))-real(888953.L))/real(5156528379172959678120.L);
      _C4x[441] = (_ep2*(real(126672.L)*_ep2-real(133224.L))+real(140361.L))/
        real(236123858440243271456.L);
      _C4x[442] = (real(522.L)*_ep2-real(549.L))/real(317029885123849720.L);
      _C4x[443] = real(87.L)/real(19640659160215120.L);
      _C4x[444] = (_ep2*(_ep2*(_ep2*((real(31973760.L)-real(30401280.L)*_ep2)*
        _ep2-real(33686640.L))+real(35558120.L))-real(37609550.L))+
        real(39866123.L))/real(314842811527382016891612600.L);
      _C4x[445] = (_ep2*(_ep2*((real(3197376.L)-real(3040128.L)*_ep2)*_ep2-
        real(3368664.L))+real(3555812.L))-real(3760955.L))/
        real(2469355384528486406993040.L);
      _C4x[446] = (_ep2*((real(799344.L)-real(760032.L)*_ep2)*_ep2-
        real(842166.L))+real(888953.L))/real(91391237018107987486680.L);
      _C4x[447] = ((real(10248.L)-real(9744.L)*_ep2)*_ep2-real(10797.L))/
        real(246171682203657878752.L);
      _C4x[448] = (real(549.L)-real(522.L)*_ep2)/real(3503517496453777544.L);
      _C4x[449] = -real(29.L)/real(61429295671311120.L);
      _C4x[450] = (_ep2*(_ep2*(_ep2*(real(3040128.L)*_ep2-real(3197376.L))+
        real(3368664.L))-real(3555812.L))+real(3760955.L))/
        real(131077415411399860093650960.L);
      _C4x[451] = (_ep2*(_ep2*(real(58464.L)*_ep2-real(61488.L))+real(64782.L))-
        real(68381.L))/real(190242983180551320890640.L);
      _C4x[452] = (_ep2*(real(9744.L)*_ep2-real(10248.L))+real(10797.L))/
        real(4526544605418280586848.L);
      _C4x[453] = (real(58.L)*_ep2-real(61.L))/real(5469777315892122084.L);
      _C4x[454] = real(29.L)/real(703302752481745680.L);
      _C4x[455] = (_ep2*((real(61488.L)-real(58464.L)*_ep2)*_ep2-real(64782.L))+
        real(68381.L))/real(10478285093218993340819760.L);
      _C4x[456] = ((real(3416.L)-real(3248.L)*_ep2)*_ep2-real(3599.L))/
        real(42336505427147447841696.L);
      _C4x[457] = (real(61.L)-real(58.L)*_ep2)/real(104211770430232260620.L);
      _C4x[458] = -real(29.L)/real(10232365536106966560.L);
      _C4x[459] = (_ep2*(real(3248.L)*_ep2-real(3416.L))+real(3599.L))/
        real(2416376017304170372096800.L);
      _C4x[460] = (real(58.L)*_ep2-real(61.L))/real(3028040121935050591600.L);
      _C4x[461] = real(29.L)/real(201751358211920378400.L);
      _C4x[462] = (real(61.L)-real(58.L)*_ep2)/
        real(178874588293945079492880.L);
      _C4x[463] = -1/real(209087771237808392160.L);
      _C4x[464] = 1/real(12769026871558087949280.L);
      break;
    default:
      static_assert(nC4_ == 30, "Bad value of nC4_");
    }
  }

  template class Geodesic30<double>;
#if GEOGRAPHICLIB_HAVE_LONG_DOUBLE
  template class Geodesic30<long double>;
#endif

} // namespace GeographicLib
