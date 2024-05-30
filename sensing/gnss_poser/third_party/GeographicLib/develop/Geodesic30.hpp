/**
 * \file Geodesic30.hpp
 * \brief Header for GeographicLib::Geodesic30 class
 *
 * Copyright (c) Charles Karney (2009-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEODESICEXACT_HPP)
#define GEOGRAPHICLIB_GEODESICEXACT_HPP 1

#include <GeographicLib/Constants.hpp>

#if !defined(GEOGRAPHICLIB_GEODESICEXACT_ORDER)
/**
 * The order of the expansions used by Geodesic30.
 **********************************************************************/
#  define GEOGRAPHICLIB_GEODESICEXACT_ORDER 30
#endif

namespace GeographicLib {

  template<typename real> class GeodesicLine30;

  /**
   * \brief %Geodesic calculations
   *
   * The shortest path between two points on an ellipsoid at (\e lat1, \e lon1)
   * and (\e lat2, \e lon2) is called the geodesic.  Its length is \e s12 and
   * the geodesic from point 1 to point 2 has azimuths \e azi1 and \e azi2 at
   * the two end points.  (The azimuth is the heading measured clockwise from
   * north.  \e azi2 is the "forward" azimuth, i.e., the heading that takes you
   * beyond point 2 not back to point 1.)
   *
   * Given \e lat1, \e lon1, \e azi1, and \e s12, we can determine \e lat2, \e
   * lon2, and \e azi2.  This is the \e direct geodesic problem and its
   * solution is given by the function Geodesic30::Direct.  (If \e s12 is
   * sufficiently large that the geodesic wraps more than halfway around the
   * earth, there will be another geodesic between the points with a smaller \e
   * s12.)
   *
   * Given \e lat1, \e lon1, \e lat2, and \e lon2, we can determine \e azi1, \e
   * azi2, and \e s12.  This is the \e inverse geodesic problem, whose solution
   * is given by Geodesic30::Inverse.  Usually, the solution to the inverse
   * problem is unique.  In cases where there are multiple solutions (all with
   * the same \e s12, of course), all the solutions can be easily generated
   * once a particular solution is provided.
   *
   * The standard way of specifying the direct problem is the specify the
   * distance \e s12 to the second point.  However it is sometimes useful
   * instead to specify the arc length \e a12 (in degrees) on the auxiliary
   * sphere.  This is a mathematical construct used in solving the geodesic
   * problems.  The solution of the direct problem in this form is provided by
   * Geodesic30::ArcDirect.  An arc length in excess of 180&deg; indicates that
   * the geodesic is not a shortest path.  In addition, the arc length between
   * an equatorial crossing and the next extremum of latitude for a geodesic is
   * 90&deg;.
   *
   * This class can also calculate several other quantities related to
   * geodesics.  These are:
   * - <i>reduced length</i>.  If we fix the first point and increase \e azi1
   *   by \e dazi1 (radians), the second point is displaced \e m12 \e dazi1 in
   *   the direction \e azi2 + 90&deg;.  The quantity \e m12 is called
   *   the "reduced length" and is symmetric under interchange of the two
   *   points.  On a curved surface the reduced length obeys a symmetry
   *   relation, \e m12 + \e m21 = 0.  On a flat surface, we have \e m12 = \e
   *   s12.  The ratio <i>s12</i>/\e m12 gives the azimuthal scale for an
   *   azimuthal equidistant projection.
   * - <i>geodesic scale</i>.  Consider a reference geodesic and a second
   *   geodesic parallel to this one at point 1 and separated by a small
   *   distance \e dt.  The separation of the two geodesics at point 2 is \e
   *   M12 \e dt where \e M12 is called the "geodesic scale".  \e M21 is
   *   defined similarly (with the geodesics being parallel at point 2).  On a
   *   flat surface, we have \e M12 = \e M21 = 1.  The quantity 1/\e M12 gives
   *   the scale of the Cassini-Soldner projection.
   * - <i>area</i>.  Consider the quadrilateral bounded by the following lines:
   *   the geodesic from point 1 to point 2, the meridian from point 2 to the
   *   equator, the equator from \e lon2 to \e lon1, the meridian from the
   *   equator to point 1.  The area of this quadrilateral is represented by \e
   *   S12 with a clockwise traversal of the perimeter counting as a positive
   *   area and it can be used to compute the area of any simple geodesic
   *   polygon.
   *
   * Overloaded versions of Geodesic30::Direct, Geodesic30::ArcDirect,
   * and Geodesic30::Inverse allow these quantities to be returned.  In
   * addition there are general functions Geodesic30::GenDirect, and
   * Geodesic30::GenInverse which allow an arbitrary set of results to be
   * computed.  The quantities \e m12, \e M12, \e M21 which all specify the
   * behavior of nearby geodesics obey addition rules.  Let points 1, 2, and 3
   * all lie on a single geodesic, then
   * - \e m13 = \e m12 \e M23 + \e m23 \e M21
   * - \e M13 = \e M12 \e M23 &minus; (1 &minus; \e M12 \e M21) \e m23 / \e m12
   * - \e M31 = \e M32 \e M21 &minus; (1 &minus; \e M23 \e M32) \e m12 / \e m23
   *
   * Additional functionality is provided by the GeodesicLine30 class, which
   * allows a sequence of points along a geodesic to be computed.
   *
   * The calculations are accurate to better than 15 nm (15 nanometers).  See
   * Sec. 9 of
   * <a href="https://arxiv.org/abs/1102.1215v1">arXiv:1102.1215v1</a>
   * for details.
   *
   * The algorithms are described in
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   addenda: <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
   *   geod-addenda.html</a>.
   * .
   * For more information on geodesics see \ref geodesic.
   **********************************************************************/

  template<typename real>
  class Geodesic30 {
  private:
    friend class GeodesicLine30<real>;
    static const int nA1_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nC1_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nC1p_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nA2_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nC2_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nA3_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nA3x_ = nA3_;
    static const int nC3_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nC3x_ = (nC3_ * (nC3_ - 1)) / 2;
    static const int nC4_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nC4x_ = (nC4_ * (nC4_ + 1)) / 2;
    static const unsigned maxit_ = 50;

    static const real tiny_;
    static const real tol0_;
    static const real tol1_;
    static const real tol2_;
    static const real xthresh_;

    enum captype {
      CAP_NONE = 0U,
      CAP_C1   = 1U<<0,
      CAP_C1p  = 1U<<1,
      CAP_C2   = 1U<<2,
      CAP_C3   = 1U<<3,
      CAP_C4   = 1U<<4,
      CAP_ALL  = 0x1FU,
      OUT_ALL  = 0x7F80U,
    };

    static real SinCosSeries(bool sinp,
                             real sinx, real cosx, const real c[], int n)
     ;
    static inline real AngRound(real x) {
      // The makes the smallest gap in x = 1/16 - nextafter(1/16, 0) = 1/2^57
      // for reals = 0.7 pm on the earth if x is an angle in degrees.  (This
      // is about 1000 times more resolution than we get with angles around 90
      // degrees.)  We use this to avoid having to deal with near singular
      // cases when x is non-zero but tiny (e.g., 1.0e-200).
      const real z = real(0.0625); // 1/16
      volatile real y = std::abs(x);
      // The compiler mustn't "simplify" z - (z - y) to y
      y = y < z ? z - (z - y) : y;
      return x < 0 ? -y : y;
    }
    static inline void SinCosNorm(real& sinx, real& cosx) {
      using std::hypot;
      real r = hypot(sinx, cosx);
      sinx /= r;
      cosx /= r;
    }
    static real Astroid(real x, real y);

    real _a, _f, _f1, _e2, _ep2, _n, _b, _c2, _etol2;
    real _A3x[nA3x_], _C3x[nC3x_], _C4x[nC4x_];

    void Lengths(real eps, real sig12,
                 real ssig1, real csig1, real ssig2, real csig2,
                 real cbet1, real cbet2,
                 real& s12s, real& m12a, real& m0,
                 bool scalep, real& M12, real& M21,
                 real C1a[], real C2a[]) const;
    real InverseStart(real sbet1, real cbet1, real sbet2, real cbet2,
                      real lam12,
                      real& salp1, real& calp1,
                      real& salp2, real& calp2,
                      real C1a[], real C2a[]) const;
    real Lambda12(real sbet1, real cbet1, real sbet2, real cbet2,
                  real salp1, real calp1,
                  real& salp2, real& calp2, real& sig12,
                  real& ssig1, real& csig1, real& ssig2, real& csig2,
                  real& eps, real& domg12, bool diffp, real& dlam12,
                  real C1a[], real C2a[], real C3a[])
      const;

    // These are Maxima generated functions to provide series approximations to
    // the integrals for the ellipsoidal geodesic.
    static real A1m1f(real eps);
    static void C1f(real eps, real c[]);
    static void C1pf(real eps, real c[]);
    static real A2m1f(real eps);
    static void C2f(real eps, real c[]);

    void A3coeff();
    real A3f(real eps) const;
    void C3coeff();
    void C3f(real eps, real c[]) const;
    void C4coeff();
    void C4f(real k2, real c[]) const;

  public:

    /**
     * Bit masks for what calculations to do.  These masks do double duty.
     * They signify to the GeodesicLine30::GeodesicLine30 constructor and
     * to Geodesic30::Line what capabilities should be included in the
     * GeodesicLine30 object.  They also specify which results to return in
     * the general routines Geodesic30::GenDirect and
     * Geodesic30::GenInverse routines.  GeodesicLine30::mask is a
     * duplication of this enum.
     **********************************************************************/
    enum mask {
      /**
       * No capabilities, no output.
       * @hideinitializer
       **********************************************************************/
      NONE          = 0U,
      /**
       * Calculate latitude \e lat2.  (It's not necessary to include this as a
       * capability to GeodesicLine30 because this is included by default.)
       * @hideinitializer
       **********************************************************************/
      LATITUDE      = 1U<<7  | CAP_NONE,
      /**
       * Calculate longitude \e lon2.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE     = 1U<<8  | CAP_C3,
      /**
       * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
       * include this as a capability to GeodesicLine30 because this is
       * included by default.)
       * @hideinitializer
       **********************************************************************/
      AZIMUTH       = 1U<<9  | CAP_NONE,
      /**
       * Calculate distance \e s12.
       * @hideinitializer
       **********************************************************************/
      DISTANCE      = 1U<<10 | CAP_C1,
      /**
       * Allow distance \e s12 to be used as input in the direct geodesic
       * problem.
       * @hideinitializer
       **********************************************************************/
      DISTANCE_IN   = 1U<<11 | CAP_C1 | CAP_C1p,
      /**
       * Calculate reduced length \e m12.
       * @hideinitializer
       **********************************************************************/
      REDUCEDLENGTH = 1U<<12 | CAP_C1 | CAP_C2,
      /**
       * Calculate geodesic scales \e M12 and \e M21.
       * @hideinitializer
       **********************************************************************/
      GEODESICSCALE = 1U<<13 | CAP_C1 | CAP_C2,
      /**
       * Calculate area \e S12.
       * @hideinitializer
       **********************************************************************/
      AREA          = 1U<<14 | CAP_C4,
      /**
       * All capabilities, calculate everything.
       * @hideinitializer
       **********************************************************************/
      ALL           = OUT_ALL| CAP_ALL,
    };

    /** \name Constructor
     **********************************************************************/
    ///@{
    /**
     * Constructor for an ellipsoid with
     *
     * @param[in] a equatorial radius (meters).
     * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
     *   Negative \e f gives a prolate ellipsoid.  If \e f > 1, set flattening
     *   to 1/\e f.
     * @exception GeographicErr if \e a or (1 &minus; \e f ) \e a is not
     *   positive.
     **********************************************************************/
    Geodesic30(real a, real f);
    ///@}

    /** \name Direct geodesic problem specified in terms of distance.
     **********************************************************************/
    ///@{
    /**
     * Perform the direct geodesic calculation where the length of the geodesic
     * is specified in terms of distance.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] s12 distance between point 1 and point 2 (meters); it can be
     *   signed.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).  The values of
     * \e lon2 and \e azi2 returned are in the range [&minus;180&deg;,
     * 180&deg;).
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed and writing \e lat = 90&deg; &minus; &epsilon; or
     * &minus;90&deg; + &epsilon; and taking the limit &epsilon; &rarr; 0 from
     * above.  An arc length greater that 180&deg; signifies a geodesic which
     * is not a shortest path.  (For a prolate ellipsoid, an additional
     * condition is necessary for a shortest path: the longitudinal extent must
     * not exceed of 180&deg;.)
     *
     * The following functions are overloaded versions of Geodesic30::Direct
     * which omit some of the output parameters.  Note, however, that the arc
     * length is always computed and returned as the function value.
     **********************************************************************/
    real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& m12, real& M12, real& M21, real& S12)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH |
                       REDUCEDLENGTH | GEODESICSCALE | AREA,
                       lat2, lon2, azi2, t, m12, M12, M21, S12);
    }

    /**
     * See the documentation for Geodesic30::Direct.
     **********************************************************************/
    real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE,
                       lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Direct.
     **********************************************************************/
    real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH,
                       lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Direct.
     **********************************************************************/
    real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2, real& m12)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH | REDUCEDLENGTH,
                       lat2, lon2, azi2, t, m12, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Direct.
     **********************************************************************/
    real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& M12, real& M21)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH | GEODESICSCALE,
                       lat2, lon2, azi2, t, t, M12, M21, t);
    }

    /**
     * See the documentation for Geodesic30::Direct.
     **********************************************************************/
    real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& m12, real& M12, real& M21)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH |
                       REDUCEDLENGTH | GEODESICSCALE,
                       lat2, lon2, azi2, t, m12, M12, M21, t);
    }
    ///@}

    /** \name Direct geodesic problem specified in terms of arc length.
     **********************************************************************/
    ///@{
    /**
     * Perform the direct geodesic calculation where the length of the geodesic
     * is specified in terms of arc length.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] a12 arc length between point 1 and point 2 (degrees); it can
     *   be signed.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).  The values of
     * \e lon2 and \e azi2 returned are in the range [&minus;180&deg;,
     * 180&deg;).
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed and writing \e lat = 90&deg; &minus; &epsilon; or
     * &minus;90&deg; + &epsilon; and taking the limit &epsilon; &rarr; 0 from
     * above.  An arc length greater that 180&deg; signifies a geodesic which
     * is not a shortest path.  (For a prolate ellipsoid, an additional
     * condition is necessary for a shortest path: the longitudinal extent must
     * not exceed of 180&deg;.)
     *
     * The following functions are overloaded versions of Geodesic30::Direct
     * which omit some of the output parameters.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& m12, real& M12, real& M21, real& S12)
      const {
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH | GEODESICSCALE | AREA,
                lat2, lon2, azi2, s12, m12, M12, M21, S12);
    }

    /**
     * See the documentation for Geodesic30::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE,
                lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH,
                lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12)
      const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE,
                lat2, lon2, azi2, s12, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2,
                   real& s12, real& m12) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH,
                lat2, lon2, azi2, s12, m12, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& M12, real& M21) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                GEODESICSCALE,
                lat2, lon2, azi2, s12, t, M12, M21, t);
    }

    /**
     * See the documentation for Geodesic30::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& m12, real& M12, real& M21) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH | GEODESICSCALE,
                lat2, lon2, azi2, s12, m12, M12, M21, t);
    }
    ///@}

    /** \name General version of the direct geodesic solution.
     **********************************************************************/
    ///@{

    /**
     * The general direct geodesic calculation.  Geodesic30::Direct and
     * Geodesic30::ArcDirect are defined in terms of this function.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] arcmode boolean flag determining the meaning of the second
     *   parameter.
     * @param[in] s12_a12 if \e arcmode is false, this is the distance between
     *   point 1 and point 2 (meters); otherwise it is the arc length between
     *   point 1 and point 2 (degrees); it can be signed.
     * @param[in] outmask a bitor'ed combination of Geodesic30::mask values
     *   specifying which of the following parameters should be set.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * The Geodesic30::mask values possible for \e outmask are
     * - \e outmask |= Geodesic30::LATITUDE for the latitude \e lat2.
     * - \e outmask |= Geodesic30::LONGITUDE for the latitude \e lon2.
     * - \e outmask |= Geodesic30::AZIMUTH for the latitude \e azi2.
     * - \e outmask |= Geodesic30::DISTANCE for the distance \e s12.
     * - \e outmask |= Geodesic30::REDUCEDLENGTH for the reduced length \e
     *   m12.
     * - \e outmask |= Geodesic30::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21.
     * - \e outmask |= Geodesic30::AREA for the area \e S12.
     * .
     * The function value \e a12 is always computed and returned and this
     * equals \e s12_a12 is \e arcmode is true.  If \e outmask includes
     * Geodesic30::DISTANCE and \e arcmode is false, then \e s12 = \e
     * s12_a12.  It is not necessary to include Geodesic30::DISTANCE_IN in
     * \e outmask; this is automatically included is \e arcmode is false.
     **********************************************************************/
    real GenDirect(real lat1, real lon1, real azi1,
                         bool arcmode, real s12_a12, unsigned outmask,
                         real& lat2, real& lon2, real& azi2,
                         real& s12, real& m12, real& M12, real& M21,
                         real& S12) const;
    ///@}

    /** \name Inverse geodesic problem.
     **********************************************************************/
    ///@{
    /**
     * Perform the inverse geodesic calculation.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] lat2 latitude of point 2 (degrees).
     * @param[in] lon2 longitude of point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] azi1 azimuth at point 1 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * \e lat1 and \e lat2 should be in the range [&minus;90&deg;, 90&deg;]; \e
     * lon1 and \e lon2 should be in the range [&minus;540&deg;, 540&deg;).
     * The values of \e azi1 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;).
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed and writing \e lat = 90&deg; &minus; &epsilon; or
     * &minus;90&deg; + &epsilon; and taking the limit &epsilon; &rarr; 0 from
     * above.  If the routine fails to converge, then all the requested outputs
     * are set to Math::NaN().  (Test for such results with Math::isnan.)  This
     * is not expected to happen with ellipsoidal models of the earth; please
     * report all cases where this occurs.
     *
     * The following functions are overloaded versions of Geodesic30::Inverse
     * which omit some of the output parameters.  Note, however, that the arc
     * length is always computed and returned as the function value.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12,
                       real& M12, real& M21, real& S12) const {
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH |
                        REDUCEDLENGTH | GEODESICSCALE | AREA,
                        s12, azi1, azi2, m12, M12, M21, S12);
    }

    /**
     * See the documentation for Geodesic30::Inverse.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE,
                        s12, t, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Inverse.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& azi1, real& azi2) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        AZIMUTH,
                        t, azi1, azi2, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Inverse.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2)
      const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH,
                        s12, azi1, azi2, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Inverse.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12)
      const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH | REDUCEDLENGTH,
                        s12, azi1, azi2, m12, t, t, t);
    }

    /**
     * See the documentation for Geodesic30::Inverse.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2,
                       real& M12, real& M21) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH | GEODESICSCALE,
                        s12, azi1, azi2, t, M12, M21, t);
    }

    /**
     * See the documentation for Geodesic30::Inverse.
     **********************************************************************/
    real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12,
                       real& M12, real& M21) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH |
                        REDUCEDLENGTH | GEODESICSCALE,
                        s12, azi1, azi2, m12, M12, M21, t);
    }
    ///@}

    /** \name General version of inverse geodesic solution.
     **********************************************************************/
    ///@{
    /**
     * The general inverse geodesic calculation.  Geodesic30::Inverse is
     * defined in terms of this function.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] lat2 latitude of point 2 (degrees).
     * @param[in] lon2 longitude of point 2 (degrees).
     * @param[in] outmask a bitor'ed combination of Geodesic30::mask values
     *   specifying which of the following parameters should be set.
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] azi1 azimuth at point 1 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * The Geodesic30::mask values possible for \e outmask are
     * - \e outmask |= Geodesic30::DISTANCE for the distance \e s12.
     * - \e outmask |= Geodesic30::AZIMUTH for the latitude \e azi2.
     * - \e outmask |= Geodesic30::REDUCEDLENGTH for the reduced length \e
     *   m12.
     * - \e outmask |= Geodesic30::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21.
     * - \e outmask |= Geodesic30::AREA for the area \e S12.
     * .
     * The arc length is always computed and returned as the function value.
     **********************************************************************/
    real GenInverse(real lat1, real lon1, real lat2, real lon2,
                          unsigned outmask,
                          real& s12, real& azi1, real& azi2,
                          real& m12, real& M12, real& M21, real& S12)
      const;
    ///@}

    /** \name Interface to GeodesicLine30.
     **********************************************************************/
    ///@{

    /**
     * Set up to compute several points on a single geodesic.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] caps bitor'ed combination of Geodesic30::mask values
     *   specifying the capabilities the GeodesicLine30 object should
     *   possess, i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).
     *
     * The Geodesic30::mask values are
     * - \e caps |= Geodesic30::LATITUDE for the latitude \e lat2; this is
     *   added automatically
     * - \e caps |= Geodesic30::LONGITUDE for the latitude \e lon2
     * - \e caps |= Geodesic30::AZIMUTH for the latitude \e azi2; this is
     *   added automatically
     * - \e caps |= Geodesic30::DISTANCE for the distance \e s12
     * - \e caps |= Geodesic30::REDUCEDLENGTH for the reduced length \e m12
     * - \e caps |= Geodesic30::GEODESICSCALE for the geodesic scales \e M12
     *   and \e M21
     * - \e caps |= Geodesic30::AREA for the area \e S12
     * - \e caps |= Geodesic30::DISTANCE_IN permits the length of the
     *   geodesic to be given in terms of \e s12; without this capability the
     *   length can only be specified in terms of arc length.
     * .
     * The default value of \e caps is Geodesic30::ALL which turns on all
     * the capabilities.
     *
     * If the point is at a pole, the azimuth is defined by keeping the \e lon1
     * fixed and writing \e lat1 = &plusmn;(90&deg; &minus; &epsilon;) and
     * taking the limit &epsilon; &rarr; 0+.
     **********************************************************************/
    GeodesicLine30<real>
      Line(real lat1, real lon1, real azi1, unsigned caps = ALL)
      const;

    ///@}

    /** \name Inspector functions.
     **********************************************************************/
    ///@{

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    real EquatorialRadius() const { return _a; }

    /**
     * @return \e f the  flattening of the ellipsoid.  This is the
     *   value used in the constructor.
     **********************************************************************/
    real Flattening() const { return _f; }

    /// \cond SKIP
    /**
     * <b>DEPRECATED</b>
     * @return \e r the inverse flattening of the ellipsoid.
     **********************************************************************/
    real InverseFlattening() const { return 1/_f; }
    /// \endcond

    /**
     * @return total area of ellipsoid in meters<sup>2</sup>.  The area of a
     *   polygon encircling a pole can be found by adding
     *   Geodesic30::EllipsoidArea()/2 to the sum of \e S12 for each side of
     *   the polygon.
     **********************************************************************/
    real EllipsoidArea() const
    { return 4 * Math::pi<real>() * _c2; }
    ///@}

    /**
     * A global instantiation of Geodesic30 with the parameters for the WGS84
     * ellipsoid.
     **********************************************************************/
    static const Geodesic30 WGS84;

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEODESICEXACT_HPP
