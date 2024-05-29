/**
 * \file GeodesicLine30.hpp
 * \brief Header for GeographicLib::GeodesicLine30 class
 *
 * Copyright (c) Charles Karney (2009-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEODESICLINEEXACT_HPP)
#define GEOGRAPHICLIB_GEODESICLINEEXACT_HPP 1

#include <GeographicLib/Constants.hpp>
#include "Geodesic30.hpp"

namespace GeographicLib {

  /**
   * \brief A geodesic line
   *
   * GeodesicLine30 facilitates the determination of a series of points on a
   * single geodesic.  The starting point (\e lat1, \e lon1) and the azimuth \e
   * azi1 are specified in the constructor.  GeodesicLine30.Position returns
   * the location of point 2 a distance \e s12 along the geodesic.
   * Alternatively GeodesicLine30.ArcPosition gives the position of point 2
   * an arc length \e a12 along the geodesic.
   *
   * The default copy constructor and assignment operators work with this
   * class.  Similarly, a vector can be used to hold GeodesicLine30 objects.
   *
   * The calculations are accurate to better than 15 nm (15 nanometers).  See
   * Sec. 9 of
   * <a href="https://arxiv.org/abs/1102.1215v1">arXiv:1102.1215v1</a> for
   * details.
   *
   * The algorithms are described in
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
   *   addenda</a>.
   * .
   * For more information on geodesics see \ref geodesic.
   **********************************************************************/

  template<typename real>
  class GeodesicLine30 {
  private:
    friend class Geodesic30<real>;
    static const int nC1_ = Geodesic30<real>::nC1_;
    static const int nC1p_ = Geodesic30<real>::nC1p_;
    static const int nC2_ = Geodesic30<real>::nC2_;
    static const int nC3_ = Geodesic30<real>::nC3_;
    static const int nC4_ = Geodesic30<real>::nC4_;

    real _lat1, _lon1, _azi1;
    real _a, _f, _b, _c2, _f1, _salp0, _calp0, _k2,
      _salp1, _calp1, _ssig1, _csig1, _stau1, _ctau1, _somg1, _comg1,
      _A1m1, _A2m1, _A3c, _B11, _B21, _B31, _A4, _B41;
    // index zero elements of _C1a, _C1pa, _C2a, _C3a are unused
    real _C1a[nC1_ + 1], _C1pa[nC1p_ + 1], _C2a[nC2_ + 1], _C3a[nC3_],
      _C4a[nC4_];    // all the elements of _C4a are used
    unsigned _caps;

    enum captype {
      CAP_NONE = Geodesic30<real>::CAP_NONE,
      CAP_C1   = Geodesic30<real>::CAP_C1,
      CAP_C1p  = Geodesic30<real>::CAP_C1p,
      CAP_C2   = Geodesic30<real>::CAP_C2,
      CAP_C3   = Geodesic30<real>::CAP_C3,
      CAP_C4   = Geodesic30<real>::CAP_C4,
      CAP_ALL  = Geodesic30<real>::CAP_ALL,
      OUT_ALL  = Geodesic30<real>::OUT_ALL,
    };
  public:

    /**
     * Bit masks for what calculations to do.  They signify to the
     * GeodesicLine30::GeodesicLine30 constructor and to
     * Geodesic30::Line what capabilities should be included in the
     * GeodesicLine30 object.  This is merely a duplication of
     * Geodesic30::mask.
     **********************************************************************/
    enum mask {
      /**
       * No capabilities, no output.
       * @hideinitializer
       **********************************************************************/
      NONE          = Geodesic30<real>::NONE,
      /**
       * Calculate latitude \e lat2.  (It's not necessary to include this as a
       * capability to GeodesicLine30 because this is included by default.)
       * @hideinitializer
       **********************************************************************/
      LATITUDE      = Geodesic30<real>::LATITUDE,
      /**
       * Calculate longitude \e lon2.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE     = Geodesic30<real>::LONGITUDE,
      /**
       * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
       * include this as a capability to GeodesicLine30 because this is
       * included by default.)
       * @hideinitializer
       **********************************************************************/
      AZIMUTH       = Geodesic30<real>::AZIMUTH,
      /**
       * Calculate distance \e s12.
       * @hideinitializer
       **********************************************************************/
      DISTANCE      = Geodesic30<real>::DISTANCE,
      /**
       * Allow distance \e s12 to be used as input in the direct geodesic
       * problem.
       * @hideinitializer
       **********************************************************************/
      DISTANCE_IN   = Geodesic30<real>::DISTANCE_IN,
      /**
       * Calculate reduced length \e m12.
       * @hideinitializer
       **********************************************************************/
      REDUCEDLENGTH = Geodesic30<real>::REDUCEDLENGTH,
      /**
       * Calculate geodesic scales \e M12 and \e M21.
       * @hideinitializer
       **********************************************************************/
      GEODESICSCALE = Geodesic30<real>::GEODESICSCALE,
      /**
       * Calculate area \e S12.
       * @hideinitializer
       **********************************************************************/
      AREA          = Geodesic30<real>::AREA,
      /**
       * All capabilities, calculate everything.
       * @hideinitializer
       **********************************************************************/
      ALL           = Geodesic30<real>::ALL,
    };

    /** \name Constructors
     **********************************************************************/
    ///@{

    /**
     * Constructor for a geodesic line staring at latitude \e lat1, longitude
     * \e lon1, and azimuth \e azi1 (all in degrees).
     *
     * @param[in] g A Geodesic30 object used to compute the necessary
     *   information about the GeodesicLine30.
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] caps bitor'ed combination of GeodesicLine30::mask values
     *   specifying the capabilities the GeodesicLine30 object should
     *   possess, i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).
     *
     * The GeodesicLine30::mask values are
     * - \e caps |= GeodesicLine30::LATITUDE for the latitude \e lat2; this
     *   is added automatically
     * - \e caps |= GeodesicLine30::LONGITUDE for the latitude \e lon2
     * - \e caps |= GeodesicLine30::AZIMUTH for the latitude \e azi2; this is
     *   added automatically
     * - \e caps |= GeodesicLine30::DISTANCE for the distance \e s12
     * - \e caps |= GeodesicLine30::REDUCEDLENGTH for the reduced length \e
         m12
     * - \e caps |= GeodesicLine30::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21
     * - \e caps |= GeodesicLine30::AREA for the area \e S12
     * - \e caps |= GeodesicLine30::DISTANCE_IN permits the length of the
     *   geodesic to be given in terms of \e s12; without this capability the
     *   length can only be specified in terms of arc length.
     * .
     * The default value of \e caps is GeodesicLine30::ALL which turns on
     * all the capabilities.
     *
     * If the point is at a pole, the azimuth is defined by keeping the \e lon1
     * fixed and writing \e lat1 = &plusmn;(90&deg; &minus; &epsilon;) and
     * taking the limit &epsilon; &rarr; 0+.
     **********************************************************************/
    GeodesicLine30(const Geodesic30<real>& g, real lat1, real lon1, real azi1,
                 unsigned caps = ALL)
     ;

    /**
     * A default constructor.  If GeodesicLine30::Position is called on the
     * resulting object, it returns immediately (without doing any
     * calculations).  The object can be set with a call to
     * Geodesic30::Line.  Use Init() to test whether object is still in this
     * uninitialized state.
     **********************************************************************/
    GeodesicLine30() : _caps(0U) {}
    ///@}

    /** \name Position in terms of distance
     **********************************************************************/
    ///@{

    /**
     * Compute the position of point 2 which is a distance \e s12 (meters)
     * from point 1.
     *
     * @param[in] s12 distance between point 1 and point 2 (meters); it can be
     *   signed.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees); requires that the
     *   GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::LONGITUDE.
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters); requires that the
     *   GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::REDUCEDLENGTH.
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless); requires that the GeodesicLine30 object was
     *   constructed with \e caps |= GeodesicLine30::GEODESICSCALE.
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless); requires that the GeodesicLine30 object was
     *   constructed with \e caps |= GeodesicLine30::GEODESICSCALE.
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
     *   that the GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::AREA.
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * The values of \e lon2 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;).
     *
     * The GeodesicLine30 object \e must have been constructed with \e caps
     * |= GeodesicLine30::DISTANCE_IN; otherwise Math::NaN() is returned and
     * no parameters are set.  Requesting a value which the GeodesicLine30
     * object is not capable of computing is not an error; the corresponding
     * argument will not be altered.
     *
     * The following functions are overloaded versions of
     * GeodesicLine30::Position which omit some of the output parameters.
     * Note, however, that the arc length is always computed and returned as
     * the function value.
     **********************************************************************/
    real Position(real s12,
                        real& lat2, real& lon2, real& azi2,
                        real& m12, real& M12, real& M21,
                        real& S12) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE | AZIMUTH |
                         REDUCEDLENGTH | GEODESICSCALE | AREA,
                         lat2, lon2, azi2, t, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicLine30::Position.
     **********************************************************************/
    real Position(real s12, real& lat2, real& lon2) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE,
                         lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::Position.
     **********************************************************************/
    real Position(real s12, real& lat2, real& lon2,
                        real& azi2) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE | AZIMUTH,
                         lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::Position.
     **********************************************************************/
    real Position(real s12, real& lat2, real& lon2,
                        real& azi2, real& m12) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE |
                         AZIMUTH | REDUCEDLENGTH,
                         lat2, lon2, azi2, t, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::Position.
     **********************************************************************/
    real Position(real s12, real& lat2, real& lon2,
                        real& azi2, real& M12, real& M21)
      const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE |
                         AZIMUTH | GEODESICSCALE,
                         lat2, lon2, azi2, t, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicLine30::Position.
     **********************************************************************/
    real Position(real s12,
                        real& lat2, real& lon2, real& azi2,
                        real& m12, real& M12, real& M21)
      const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE | AZIMUTH |
                         REDUCEDLENGTH | GEODESICSCALE,
                         lat2, lon2, azi2, t, m12, M12, M21, t);
    }

    ///@}

    /** \name Position in terms of arc length
     **********************************************************************/
    ///@{

    /**
     * Compute the position of point 2 which is an arc length \e a12 (degrees)
     * from point 1.
     *
     * @param[in] a12 arc length between point 1 and point 2 (degrees); it can
     *   be signed.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees); requires that the
     *   GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::LONGITUDE.
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters); requires
     *   that the GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::DISTANCE.
     * @param[out] m12 reduced length of geodesic (meters); requires that the
     *   GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::REDUCEDLENGTH.
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless); requires that the GeodesicLine30 object was
     *   constructed with \e caps |= GeodesicLine30::GEODESICSCALE.
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless); requires that the GeodesicLine30 object was
     *   constructed with \e caps |= GeodesicLine30::GEODESICSCALE.
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
     *   that the GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::AREA.
     *
     * The values of \e lon2 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;).
     *
     * Requesting a value which the GeodesicLine30 object is not capable of
     * computing is not an error; the corresponding argument will not be
     * altered.
     *
     * The following functions are overloaded versions of
     * GeodesicLine30::ArcPosition which omit some of the output parameters.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& m12, real& M12, real& M21,
                     real& S12) const {
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                  REDUCEDLENGTH | GEODESICSCALE | AREA,
                  lat2, lon2, azi2, s12, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicLine30::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE,
                  lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12,
                     real& lat2, real& lon2, real& azi2)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH,
                  lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12) const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH | DISTANCE,
                  lat2, lon2, azi2, s12, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& m12) const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH |
                  DISTANCE | REDUCEDLENGTH,
                  lat2, lon2, azi2, s12, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicLine30::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& M12, real& M21)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH |
                  DISTANCE | GEODESICSCALE,
                  lat2, lon2, azi2, s12, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicLine30::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& m12, real& M12, real& M21)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH |
                  DISTANCE | REDUCEDLENGTH | GEODESICSCALE,
                  lat2, lon2, azi2, s12, m12, M12, M21, t);
    }
    ///@}

    /** \name The general position function.
     **********************************************************************/
    ///@{

    /**
     * The general position function.  GeodesicLine30::Position and
     * GeodesicLine30::ArcPosition are defined in terms of this function.
     *
     * @param[in] arcmode boolean flag determining the meaning of the second
     *   parameter; if arcmode is false, then the GeodesicLine30 object must
     *   have been constructed with \e caps |= GeodesicLine30::DISTANCE_IN.
     * @param[in] s12_a12 if \e arcmode is false, this is the distance between
     *   point 1 and point 2 (meters); otherwise it is the arc length between
     *   point 1 and point 2 (degrees); it can be signed.
     * @param[in] outmask a bitor'ed combination of GeodesicLine30::mask
     *   values specifying which of the following parameters should be set.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees); requires that the
     *   GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::LONGITUDE.
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters); requires
     *   that the GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::DISTANCE.
     * @param[out] m12 reduced length of geodesic (meters); requires that the
     *   GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::REDUCEDLENGTH.
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless); requires that the GeodesicLine30 object was
     *   constructed with \e caps |= GeodesicLine30::GEODESICSCALE.
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless); requires that the GeodesicLine30 object was
     *   constructed with \e caps |= GeodesicLine30::GEODESICSCALE.
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
     *   that the GeodesicLine30 object was constructed with \e caps |=
     *   GeodesicLine30::AREA.
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * The GeodesicLine30::mask values possible for \e outmask are
     * - \e outmask |= GeodesicLine30::LATITUDE for the latitude \e lat2.
     * - \e outmask |= GeodesicLine30::LONGITUDE for the latitude \e lon2.
     * - \e outmask |= GeodesicLine30::AZIMUTH for the latitude \e azi2.
     * - \e outmask |= GeodesicLine30::DISTANCE for the distance \e s12.
     * - \e outmask |= GeodesicLine30::REDUCEDLENGTH for the reduced length
     *   \e m12.
     * - \e outmask |= GeodesicLine30::GEODESICSCALE for the geodesic scales
     *   \e M12 and \e M21.
     * - \e outmask |= GeodesicLine30::AREA for the area \e S12.
     * .
     * Requesting a value which the GeodesicLine30 object is not capable of
     * computing is not an error; the corresponding argument will not be
     * altered.  Note, however, that the arc length is always computed and
     * returned as the function value.
     **********************************************************************/
    real GenPosition(bool arcmode, real s12_a12, unsigned outmask,
                           real& lat2, real& lon2, real& azi2,
                           real& s12, real& m12, real& M12, real& M21,
                           real& S12) const;

    ///@}

    /** \name Inspector functions
     **********************************************************************/
    ///@{

    /**
     * @return true if the object has been initialized.
     **********************************************************************/
    bool Init() const { return _caps != 0U; }

    /**
     * @return \e lat1 the latitude of point 1 (degrees).
     **********************************************************************/
    real Latitude() const
    { return Init() ? _lat1 : Math::NaN<real>(); }

    /**
     * @return \e lon1 the longitude of point 1 (degrees).
     **********************************************************************/
    real Longitude() const
    { return Init() ? _lon1 : Math::NaN<real>(); }

    /**
     * @return \e azi1 the azimuth (degrees) of the geodesic line at point 1.
     **********************************************************************/
    real Azimuth() const
    { return Init() ? _azi1 : Math::NaN<real>(); }

    /**
     * @return \e azi0 the azimuth (degrees) of the geodesic line as it crosses
     * the equator in a northward direction.
     **********************************************************************/
    real EquatorialAzimuth() const {
      using std::atan2;
      return Init() ?
        atan2(_salp0, _calp0) / Math::degree<real>() : Math::NaN<real>();
    }

    /**
     * @return \e a1 the arc length (degrees) between the northward equatorial
     * crossing and point 1.
     **********************************************************************/
    real EquatorialArc() const {
      using std::atan2;
      return Init() ?
        atan2(_ssig1, _csig1) / Math::degree<real>() : Math::NaN<real>();
    }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value inherited from the Geodesic30 object used in the
     *   constructor.
     **********************************************************************/
    real EquatorialRadius() const
    { return Init() ? _a : Math::NaN<real>(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the Geodesic30 object used in the constructor.
     **********************************************************************/
    real Flattening() const
    { return Init() ? _f : Math::NaN<real>(); }

    /// \cond SKIP
    /**
     * <b>DEPRECATED</b>
     * @return \e r the inverse flattening of the ellipsoid.
     **********************************************************************/
    real InverseFlattening() const
    { return Init() ? 1/_f : Math::NaN<real>(); }
    /// \endcond

    /**
     * @return \e caps the computational capabilities that this object was
     *   constructed with.  LATITUDE and AZIMUTH are always included.
     **********************************************************************/
    unsigned Capabilities() const { return _caps; }

    /**
     * @param[in] testcaps a set of bitor'ed GeodesicLine30::mask values.
     * @return true if the GeodesicLine30 object has all these capabilities.
     **********************************************************************/
    bool Capabilities(unsigned testcaps) const {
      testcaps &= OUT_ALL;
      return (_caps & testcaps) == testcaps;
    }
    ///@}

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEODESICLINEEXACT_HPP
