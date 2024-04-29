// Estimate the required order of the DST for a given GEOGRAPHICLIB_PRECISION
// and n in [-0.91, 0.91].  For each n, vary alp0 to determine the case with
// the maximum error.  Ensure that this is smaller by the machine epsilon than
// the spherical terms.

// This code allows either FFTW or kissfft to be used.  kissfft prefers because
// it allows the use of mpreal and mpreal is needed to reliably measure the
// error for quad precision.

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <limits>

#include <GeographicLib/Math.hpp>
#include <GeographicLib/Utility.hpp>

#if !defined(HAVE_FFTW)
#define HAVE_FFTW 0
#endif

#if HAVE_FFTW
#include <fftw3.h>

#if GEOGRAPHICLIB_PRECISION == 1
#  define fftw_2r2_kind fftwf_2r2_kind
#  define fftw_plan fftwf_plan
#  define fftw_plan_r2r_1d fftwf_plan_r2r_1d
#  define fftw_execute fftwf_execute
#  define fftw_destroy_plan fftwf_destroy_plan
#elif GEOGRAPHICLIB_PRECISION == 2
#elif GEOGRAPHICLIB_PRECISION == 3
#  define fftw_2r2_kind fftwl_2r2_kind
#  define fftw_plan fftwl_plan
#  define fftw_plan_r2r_1d fftwl_plan_r2r_1d
#  define fftw_execute fftwl_execute
#  define fftw_destroy_plan fftwl_destroy_plan
#elif GEOGRAPHICLIB_PRECISION == 4
#  define fftw_2r2_kind fftwq_2r2_kind
#  define fftw_plan fftwq_plan
#  define fftw_plan_r2r_1d fftwq_plan_r2r_1d
#  define fftw_execute fftwq_execute
#  define fftw_destroy_plan fftwq_destroy_plan
#else
#  error "Bad value for GEOGRAPHICLIB_PRECISION"
#endif

#else
#include <../src/kissfft.hh>
#endif

using namespace GeographicLib;
using namespace std;

class I4Integrand {
  Math::real X, tX, tdX, sX, sX1, sXX1, asinhsX, _k2;
  // return asinh(sqrt(x))/sqrt(x)
  static Math::real asinhsqrt(Math::real x) {
    return x == 0 ? 1 :
      (x > 0 ? asinh(sqrt(x))/sqrt(x) :
       asin(sqrt(-x))/sqrt(-x)); // NaNs end up here
  }
  // This differs by from t as defined following Eq 61 in Karney (2013) by
  // the final subtraction of 1.  This changes nothing since Eq 61 uses the
  // difference of two evaluations of t and improves the accuracy(?).
  static Math::real t(Math::real x) {
    // Group terms to minimize roundoff
    // with x = ep2, this is the same as
    // e2/(1-e2) + (atanh(e)/e - 1)
    return x + (sqrt(1 + x) * asinhsqrt(x) - 1);
  }
  // d t(x) / dx
  static Math::real td(Math::real x) {
    return x == 0 ? 4/Math::real(3) :
      // Group terms to minimize roundoff
      1 + (1 - asinhsqrt(x) / sqrt(1+x)) / (2*x);
  }
  // ( t(x) - t(y) ) / (x - y)
  static Math::real Dt(Math::real x, Math::real y) {
    if (x == y) return td(x);
    if (x * y <= 0) return ( t(x) - t(y) ) / (x - y);
    Math::real
      sx = sqrt(fabs(x)), sx1 = sqrt(1 + x),
      sy = sqrt(fabs(y)), sy1 = sqrt(1 + y),
      z = (x - y) / (sx * sy1 + sy * sx1),
      d1 = 2 * sx * sy,
      d2 = 2 * (x * sy * sy1 + y * sx * sx1);
    return x > 0 ?
      ( 1 + (asinh(z)/z) / d1 - (asinh(sx) + asinh(sy)) / d2 ) :
      // NaNs fall through to here
      ( 1 - (asin (z)/z) / d1 - (asin (sx) + asin (sy)) / d2 );
  }
  // ( t(X) - t(y) ) / (X - y)
  Math::real DtX(Math::real y) const {
    if (X == y) return tdX;
    if (X * y <= 0) return ( tX - t(y) ) / (X - y);
    Math::real
      sy = sqrt(fabs(y)), sy1 = sqrt(1 + y),
      z = (X - y) / (sX * sy1 + sy * sX1),
      d1 = 2 * sX * sy,
      d2 = 2 * (X * sy * sy1 + y * sXX1);
    return X > 0 ?
      ( 1 + (asinh(z)/z) / d1 - (asinhsX + asinh(sy)) / d2 ) :
      // NaNs fall through to here
      ( 1 - (asin (z)/z) / d1 - (asinhsX + asin (sy)) / d2 );
  }

public:
  I4Integrand(Math::real ep2, Math::real k2)
    : X( ep2 )
    , tX( t(X) )
    , tdX( td(X) )
    , _k2( k2 )
  {
    sX = sqrt(fabs(X));     // ep
    sX1 =  sqrt(1 + X);     // 1/(1-f)
    sXX1 = sX * sX1;
    asinhsX = X > 0 ? asinh(sX) : asin(sX); // atanh(e)
  }
  Math::real operator()(Math::real sig) const {
    Math::real ssig = sin(sig);
    return - DtX(_k2 * Math::sq(ssig)) * ssig/2;
  }
};

#define GEOGRAPHICLIB_GEODESICEXACT_ORDER 30

class TaylorI4 {
private:
  typedef Math::real real;
  static const int nC4_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
  static const int nC4x_ = (nC4_ * (nC4_ + 1)) / 2;
  real _cC4x[nC4x_];
  real _n, _ep2, _e2, _c2;
  static Math::real reale(long long y, long long z) {
    return ldexp(real(y), 52) + z;
  }
public:
  TaylorI4(real n)
    : _n(n)
    , _ep2(4*_n/Math::sq(1 - _n))
    , _e2(4*_n/Math::sq(1 + _n))
    , _c2( (1 + Math::sq((1-_n)/(1+_n)) *
          (_n == 0 ? 1 :
           (_n > 0 ? asinh(sqrt(_ep2)) : atan(sqrt(-_e2))) /
           sqrt(fabs(_e2))))/2) // authalic radius squared
  {
    C4coeff();
  }
  void C4f(real alp0, vector<Math::real>& c) const {
    c.resize(nC4_);
    real calp0, salp0;
    Math::sincosd(alp0, salp0, calp0);
    real A4 = calp0 * salp0 * _e2,
      k2 = Math::sq(calp0) * _ep2,
      eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2);
    // Evaluate C4 coeffs
    // Elements c[0] thru c[nC4_ - 1] are set
    real mult = A4/_c2;
    int o = 0;
    for (int l = 0; l < nC4_; ++l) { // l is index of C4[l]
      int m = nC4_ - l - 1;          // order of polynomial in eps
      c[l] = mult * Math::polyval(m, _cC4x + o, eps);
      o += m + 1;
      mult *= eps;
    }
    // Post condition: o == nC4x_
    if  (!(o == nC4x_))
      throw GeographicErr("C4 misalignment");
  }

  void C4coeff() {
    // Generated by Maxima on 2017-05-27 10:17:57-04:00
#if GEOGRAPHICLIB_GEODESICEXACT_ORDER == 30
    static const real coeff[] = {
      // C4[0], coeff of eps^29, polynomial in n of order 0
      3361,real(109067695),
      // C4[0], coeff of eps^28, polynomial in n of order 1
      real(121722048),real(30168404),real(0x269c465a0c9LL),
      // C4[0], coeff of eps^27, polynomial in n of order 2
      real(21708121824LL),-real(10786479696LL),real(8048130587LL),
      real(0xbfa33c13e963LL),
      // C4[0], coeff of eps^26, polynomial in n of order 3
      real(0x738319564e0LL),-real(0x4c2475635c0LL),real(0x25d0be52da0LL),
      real(643173496654LL),real(0xa0f21774b90225LL),
      // C4[0], coeff of eps^25, polynomial in n of order 4
      real(0x7a99ea0a52f40LL),-real(0x5a5f53e2c3b50LL),real(0x3b83d2c0c8da0LL),
      -real(0x1d8a81cb5cc70LL),real(0x1605bd50459c1LL),
      real(0x6fb2ae4757107d03LL),
      // C4[0], coeff of eps^24, polynomial in n of order 5
      real(0x2507d929b7f89580LL),-real(0x1ce7bf02c3715a00LL),
      real(0x15463c23456c8680LL),-real(0xdfecff0050dfd00LL),
      real(0x6f141ba97196780LL),real(0x1b71ab9c78b8b48LL),
      reale(1520879,0x957266bcf90f9LL),
      // C4[0], coeff of eps^23, polynomial in n of order 6
      reale(5214,0xb54b8c26f5620LL),-reale(4202,0x4ae5f5bcbf950LL),
      reale(3272,0xab988a50dfac0LL),-reale(2404,0x84ae60c9e7b30LL),
      real(0x62be65b26227b760LL),-real(0x30f2645200be8b10LL),
      real(0x2472ebc3f09ad327LL),reale(9429453,0x6b5ee3606e93bLL),
      // C4[0], coeff of eps^22, polynomial in n of order 7
      reale(213221,0x21fe88963f0e0LL),-reale(174746,0x12fe03af82e40LL),
      reale(140344,0xd3dfad978d4a0LL),-reale(109009,0x13ee03d15f180LL),
      reale(79932,0x9fff01479b460LL),-reale(52447,0x53ea945b584c0LL),
      reale(25976,0xa5a6ee990f820LL),reale(6403,0x87dc4a069efc6LL),
      reale(273454149,0x29bfc1ec86bafLL),
      // C4[0], coeff of eps^21, polynomial in n of order 8
      reale(1513769,0x9572babb99080LL),-reale(1247902,0x66609b16e1250LL),
      reale(1017692,0x228016ac84e60LL),-reale(814136,0x86ec313455df0LL),
      reale(630421,0xa88f591713840LL),-reale(461205,0x487f023b60f90LL),
      reale(302134,0x36942691aea20LL),-reale(149503,0x5a1d9af94cb30LL),
      reale(111169,0xb14ab93d4ba6dLL),reale(1367270745,0xd0bec99ea1a6bLL),
      // C4[0], coeff of eps^20, polynomial in n of order 9
      reale(2196138,0xe1b60fe1808c0LL),-reale(1802572,0x3b4b1c2a34200LL),
      reale(1475191,0x47b8ccbe8340LL),-reale(1196055,0x2e2a401c46980LL),
      reale(952413,0x117e9e1fb75c0LL),-reale(734856,0x2e19f1e7be100LL),
      reale(536171,0x8daa599335040LL),-reale(350594,0xa58d466a3880LL),
      reale(173293,0x7b19cdc9682c0LL),reale(42591,0xb005bdeb82d74LL),
      reale(1367270745,0xd0bec99ea1a6bLL),
      // C4[0], coeff of eps^19, polynomial in n of order 10
      reale(9954363,0x5ecc5371ca720LL),-reale(8035921,0x7cc90565e0670LL),
      reale(6522783,0x32e1ec30d1a80LL),-reale(5291286,0x4172ef2beb090LL),
      reale(4260231,0x65c388ed45de0LL),-reale(3373847,0x4da61e8c704b0LL),
      reale(2592185,0xcd194d02dbd40LL),-reale(1885401,0xa08c9a20ef6d0LL),
      reale(1230164,0x4c527bc6a84a0LL),-reale(607279,0x24d6e51bd7af0LL),
      reale(450701,0xae98337b7d081LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^18, polynomial in n of order 11
      reale(16160603,0x85a3ec5761ce0LL),-reale(12587219,0x97b7f7c505ac0LL),
      reale(9979192,0xa0e43863a93a0LL),-reale(7988280,0xcfaf566027f00LL),
      reale(6410314,0xbffc30c12660LL),-reale(5117692,0xfd9318db4c340LL),
      reale(4026292,0x94c482b815d20LL),-reale(3077917,0x9c480ad851f80LL),
      reale(2230377,0x99db799d8bfe0LL),-reale(1451530,0xb0005d9658bc0LL),
      reale(715485,0xdbe6a2ef6d6a0LL),reale(175141,0x3547b8669b9beLL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^17, polynomial in n of order 12
      reale(30091817,0x8745c27487540LL),-reale(21716256,0x7a4bb1495e170LL),
      reale(16366670,0xd4e8bc19a0660LL),-reale(12670374,0x9eda0f5df2ed0LL),
      reale(9963727,0x5ae4f6d3c8380LL),-reale(7887824,0x191034733ae30LL),
      reale(6231873,0x96448488ef0a0LL),-reale(4863678,0x67c3c74b1b90LL),
      reale(3695513,0x2e7ae0f4851c0LL),-reale(2665992,0xe6864878c32f0LL),
      reale(1729741,0xf881cba41aae0LL),-reale(851104,0x888fd5b7ab050LL),
      reale(629987,0x9ea5a19626943LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^16, polynomial in n of order 13
      reale(79181861,0x46beef62ca900LL),-reale(45969492,0x85a19d8425400LL),
      reale(30736937,0x10d9a95bb4f00LL),-reale(22084618,0xaf3a6659fa600LL),
      reale(16548053,0x58583f22e9500LL),-reale(12711232,0x3d7f1b1be3800LL),
      reale(9889259,0xbbf5d84b2bb00LL),-reale(7711253,0x36b17889dca00LL),
      reale(5958759,0x73d1ebe040100LL),-reale(4493987,0xfa374abbe1c00LL),
      reale(3224517,0x29027e04ea700LL),-reale(2084431,0x8d77e42beee00LL),
      reale(1023433,0xbf113370eed00LL),reale(249103,0x93cdbdabe0fb0LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^15, polynomial in n of order 14
      reale(100415733,0x1c7e0d98777e0LL),-reale(220472579,0x196c2a7ff77f0LL),
      reale(81497972,0xcf48e14d7b2c0LL),-reale(47157604,0xb4c79beff0c90LL),
      reale(31400333,0x3ade51fc905a0LL),-reale(22437640,0x62c8445afeb30LL),
      reale(16688020,0xb49b2cc64ec80LL),-reale(12687475,0x35a524f08d7d0LL),
      reale(9727302,0xc96eb1166e360LL),-reale(7422875,0x3574dc9ff9670LL),
      reale(5546536,0x3897621326640LL),-reale(3953280,0x7a61d237aeb10LL),
      reale(2544043,0x942757fc8f120LL),-reale(1245848,0x5f59e2e2499b0LL),
      reale(918672,0xb7e149f3f515dLL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^14, polynomial in n of order 15
      -reale(410150575,0x33edeefdadd60LL),reale(389451478,0x4a8eb37cf8e40LL),
      reale(102537774,0xdf54e754057e0LL),-reale(228145792,0x9928ef6984980LL),
      reale(84014235,0x8c476a1354120LL),-reale(48417903,0x9486b64af140LL),
      reale(32072368,0xac5157de0d660LL),-reale(22757026,0x6fd3c1d71f100LL),
      reale(16760216,0x75de552320fa0LL),-reale(12564203,0xce657c7ead0c0LL),
      reale(9433140,0xee7b325fde4e0LL),-reale(6966096,0xc0a9d97231880LL),
      reale(4923714,0x7fe1a8c934e20LL),-reale(3150864,0xcacdc5bf45040LL),
      reale(1538058,0xc6e75548f4360LL),reale(371250,0x9b28ca926da22LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^13, polynomial in n of order 16
      reale(10071346,0xbead2787bab00LL),reale(77935892,0xc8037e807a610LL),
      -reale(424974584,0x95c58aa2abc60LL),reale(405632040,0xf37804095de30LL),
      reale(104709205,0x2c34dddf07040LL),-reale(236671973,0xc06ad427a5bb0LL),
      reale(86756233,0x36f6256b264e0LL),-reale(49748360,0xa42ca4c379390LL),
      reale(32735340,0x1aa6eba145580LL),-reale(23012513,0x41e6e60af5570LL),
      reale(16722020,0xa0e65eb557620LL),-reale(12285046,0x712c138942d50LL),
      reale(8933912,0x44131ea6cfac0LL),-reale(6247309,0xac4879043a730LL),
      reale(3969671,0x8774cc7c1760LL),-reale(1929932,0x2a739696c4f10LL),
      reale(1414943,0x9f9bcb791811fLL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^12, polynomial in n of order 17
      reale(1301009,0x7885767b34dc0LL),reale(3139452,0x6299dbe8eac00LL),
      reale(10399899,0xe9c2f692aa40LL),reale(80694987,0xafcfc919b1e80LL),
      -reale(441529449,0x34f14f083e140LL),reale(423985433,0x2e9be95704100LL),
      reale(106892519,0x9a909730adb40LL),-reale(246219322,0x3cc21ecefbc80LL),
      reale(89751674,0x8e9ea1f760fc0LL),-reale(51139306,0x4d1fa35b2aa00LL),
      reale(33357165,0x391836578ec40LL),-reale(23152852,0x670df382e5780LL),
      reale(16502135,0xfb453b1baa0c0LL),-reale(11755175,0x732a395d89500LL),
      reale(8105218,0xa64658fb65d40LL),-reale(5103238,0xc9c658d3f3280LL),
      reale(2468214,0x7d6aacb2351c0LL),reale(588064,0xecbdce72e5104LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^11, polynomial in n of order 18
      reale(365173,0x141eb92882aa0LL),reale(660579,0x721db1cc80890LL),
      reale(1339643,0x6f3cff39e7d00LL),reale(3240370,0xc29100e665970LL),
      reale(10762711,0xac38fa6376f60LL),reale(83769430,0x6edf90fa38050LL),
      -reale(460180081,0xa7a2c15d05240LL),reale(445039582,0xb96af8d66e930LL),
      reale(109020126,0x840edc5d1e420LL),-reale(257005247,0x2ec795996fff0LL),
      reale(93028106,0x54adfb574be80LL),-reale(52565819,0x1d828e2b6cf10LL),
      reale(33879206,0x109475f98e8e0LL),-reale(23088279,0x158dbde3c1830LL),
      reale(15975944,0x7a6ca24c70f40LL),-reale(10806612,0x3c0d699b76f50LL),
      reale(6721635,0xd5a36326ddda0LL),-reale(3228909,0xe44dc20d06870LL),
      reale(2345355,0x81bdf10588059LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^10, polynomial in n of order 19
      reale(142358,0x43f28ef2bce60LL),reale(224104,0xc49bf70fb8540LL),
      reale(374789,0x29edb81ed2220LL),reale(679606,0x56dce126b3a00LL),
      reale(1381751,0x3315a15e701e0LL),reale(3351469,0xe4cb186e3aec0LL),
      reale(11166107,0x295c18ed1d5a0LL),reale(87224183,0xbf27e3cc5cb80LL),
      -reale(481408924,0xf800e4fbbfaa0LL),reale(469519077,0x9e18ca33e7840LL),
      reale(110970854,0x606788cedf920LL),-reale(269315695,0x90dadb20d6300LL),
      reale(96606791,0x8c213171618e0LL),-reale(53972000,0xd509f5454de40LL),
      reale(34191407,0x9021dc5d4cca0LL),-reale(22654105,0x9f8b9187f1180LL),
      reale(14912791,0x946e9b2907c60LL),-reale(9121084,0x6067cd3f714c0LL),
      reale(4341360,0x73b562399020LL),reale(1011849,0x75de66a5bdb46LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^9, polynomial in n of order 20
      reale(66631,0x784cbdfb1b2c0LL),reale(96606,0x3419bb8e05f90LL),
      reale(145459,0xb79bffbfb42e0LL),reale(229589,0x824d22506cd30LL),
      reale(385010,0x35e34fd0f4f00LL),reale(700134,0x4df5413db48d0LL),
      reale(1427794,0x581b23c083b20LL),reale(3474469,0x224df4c0f7670LL),
      reale(11618119,0x6c8cba4306b40LL),reale(91144571,0x713d14f45fa10LL),
      -reale(505869523,0xd3d937aa3bca0LL),reale(498449385,0x686859af477b0LL),
      reale(112524504,0x2ca5b0e042780LL),-reale(283533725,0xba4eec11a6cb0LL),
      reale(100487121,0xc424152de7ba0LL),-reale(55236514,0x8c4dd4ee50f10LL),
      reale(34077723,0x322bbe9b9a3c0LL),-reale(21528502,0x2ca44d130cb70LL),
      reale(12851809,0x7f1d30d5603e0LL),-reale(6038295,0xecbfc0da7fdd0LL),
      reale(4313665,0xa0fbedf62e95bLL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^8, polynomial in n of order 21
      reale(34939,0x4781a8598a880LL),reale(47986,0x870a153a0ba00LL),
      reale(67643,0xf93c5a3d5fb80LL),reale(98366,0xdef5527b5d100LL),
      reale(148567,0x565e4f7b51e80LL),reale(235242,0x766e64b79c800LL),
      reale(395796,0x5614c84bc3180LL),reale(722239,0xc9f1a6fcbf00LL),
      reale(1478257,0xd3352c2795480LL),reale(3611438,0xfdbc40cced600LL),
      reale(12129091,0x5ec9e3d72a780LL),reale(95645231,0xe79e249b02d00LL),
      -reale(534473300,0x6333290e9b580LL),reale(533336700,0xd7635e240e400LL),
      reale(113268651,0x31e09daaa5d80LL),-reale(300181610,0x6cd38634ee500LL),
      reale(104606327,0x6a6e0bd3d0080LL),-reale(56090968,0xcfc000b8f0e00LL),
      reale(33084425,0x428f85e945380LL),-reale(19025074,0x3fea5ea1f7700LL),
      reale(8768855,0x59c11511e7680LL),reale(1959911,0x57aea52b92dd8LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^7, polynomial in n of order 22
      reale(19712,0xac93bc6991f60LL),reale(26064,0x47e63bb6f7b10LL),
      reale(35129,0x85349dd791940LL),reale(48412,0xcf2b50a5e4170LL),
      reale(68486,0xf23457a2e7b20LL),reale(99959,0x1aee9379bdd0LL),
      reale(151547,0xc976e86422100LL),reale(240911,0x67a8290f88c30LL),
      reale(407002,0x79f859786e6e0LL),reale(745880,0xf6e3b80f24890LL),
      reale(1533569,0xcfffb4a9fa8c0LL),reale(3764807,0xab1a08cbd8ef0LL),
      reale(12712489,0x4098eb8542a0LL),reale(100884327,0x9a754746dfb50LL),
      -reale(568536969,0xbcc82f5b36f80LL),reale(576497219,0x10ca042b229b0LL),
      reale(112392819,0xaecaa4a6c6e60LL),-reale(319979712,0xfe05e4aae49f0LL),
      reale(108728942,0x9b1cd9ac3b840LL),-reale(55904982,0xfebe8a174c390LL),
      reale(30158727,0xd0df7149f4a20LL),-reale(13482566,0x2ca2af46da730LL),
      reale(9304222,0x6328f1d67a7f5LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^6, polynomial in n of order 23
      reale(11639,0x4298ebe4bc020LL),reale(14966,0xe9089607c0a40LL),
      reale(19534,0x1996a62965260LL),reale(25928,0xdcaffa7bfcb80LL),
      reale(35089,0x59fa64f7d88a0LL),reale(48563,0x32ed377221cc0LL),
      reale(69004,0xe5c9403173ae0LL),reale(101181,0xf483b00105600LL),
      reale(154143,0xf39432e434120LL),reale(246274,0xfc90899a3cf40LL),
      reale(418255,0xdad9486cf7360LL),reale(770731,0xbf0321b55e080LL),
      reale(1593877,0xd61fe95ba9a0LL),reale(3937200,0x3820413b3e1c0LL),
      reale(13385919,0xf48ca237dbbe0LL),reale(107086956,0x9d1b10f932b00LL),
      -reale(610048075,0x6c1b2715a7de0LL),reale(631706048,0xcac1d46451440LL),
      reale(108187733,0xaf9fd1440d460LL),-reale(343908890,0x37b3c0b50a80LL),
      reale(112109635,0x3a73d439f8aa0LL),-reale(53028119,0x15d1799f5d940LL),
      reale(22454404,0x49a70d2177ce0LL),reale(4553016,0x22f700960daaaLL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^5, polynomial in n of order 24
      reale(7030,0x634f92bbfec80LL),reale(8852,0x183ea9c784b10LL),
      reale(11280,0x864427e0ea420LL),reale(14569,0x4ed71f4155e30LL),
      reale(19103,0x13b2c1ad2ffc0LL),reale(25480,0x35983eb20bf50LL),
      reale(34659,0x18ad59c5f9360LL),reale(48227,0x95f2c0574270LL),
      reale(68917,0x8c5b3ac32f300LL),reale(101660,0x272f49f96bb90LL),
      reale(155850,0xbc628b339b2a0LL),reale(250657,0x122490d07feb0LL),
      reale(428675,0x21f5a97506640LL),reale(795748,0x8d9dd2ee8dfd0LL),
      reale(1658420,0x22b44d2c5a1e0LL),reale(4130702,0x814b60cb632f0LL),
      reale(14171990,0xb8691b29bf980LL),reale(114585240,0x7599d8275cc10LL),
      -reale(662180135,0x55c1167b3fee0LL),reale(705602404,0xf6219ee07f30LL),
      reale(96655880,0xe42cfbbc64cc0LL),-reale(373149978,0xd8d5a94d3dfb0LL),
      reale(112272021,0x704341a757060LL),-reale(42251989,0xbf5a94cca7c90LL),
      reale(26498553,0xea37274059c77LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^4, polynomial in n of order 25
      reale(4244,0x3972351df5940LL),reale(5257,0xaa8f87b5d5600LL),
      reale(6578,0xed6cb3b3fa2c0LL),reale(8324,0xb4008d853180LL),
      reale(10662,0x703b07259b440LL),reale(13846,0x8f2f6ca125d00LL),
      reale(18261,0x3a455b4269dc0LL),reale(24508,0x5045fb81ae880LL),
      reale(33557,0x1b3e945f36f40LL),reale(47022,0x9499ec44e400LL),
      reale(67699,0x7a940285938c0LL),reale(100662,0x403646e1e5f80LL),
      reale(155637,0xf20897fb50a40LL),reale(252593,0x7106d86756b00LL),
      reale(436178,0xe720d891ff3c0LL),reale(818051,0x1d79595b01680LL),
      reale(1723706,0xc365c92e70540LL),reale(4344105,0xb055b91247200LL),
      reale(15096896,0xe96c54f834ec0LL),reale(123888911,0x435c586708d80LL),
      -reale(730395130,0x8d07d85ee1fc0LL),reale(811137162,0xd7ccf03d27900LL),
      reale(66848989,0xdd39a234bc9c0LL),-reale(407950245,0xd67367b7fbb80LL),
      reale(99073631,0x21cb91dfe1b40LL),reale(14205410,0x589c3f44ce7acLL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^3, polynomial in n of order 26
      reale(2481,0x8d2c27b46b620LL),reale(3034,0xe44720f3fdf90LL),
      reale(3743,0xf82fc54a92780LL),reale(4662,0xb922ac44f6b70LL),
      reale(5867,0xae02c805f08e0LL),reale(7469,0x40a687e9b4d50LL),
      reale(9629,0xbb2099bca6640LL),reale(12592,0xa0727e14e5130LL),
      reale(16731,0xdc4cfea134ba0LL),reale(22636,0xbf84f9dc44310LL),
      reale(31263,0xfe99294d5c500LL),reale(44220,0x78f2e666feef0LL),
      reale(64313,0xe77c1f84fde60LL),reale(96684,0x43c9282e120d0LL),
      reale(151281,0x84eb0984fa3c0LL),reale(248729,0xa2c4a502aa4b0LL),
      reale(435615,0xd80deb212120LL),reale(829647,0x194fc60e84690LL),
      reale(1777619,0x17dfea7bc6280LL),reale(4562307,0x417bb8824d270LL),
      reale(16175470,0xd3a7db47373e0LL),reale(135804489,0xbb999e2601450LL),
      -reale(825156505,0xa8162cc9f9ec0LL),reale(977623624,0xd8c5ee7f4d830LL),
      -reale(20397512,0x4ab8f862cc960LL),-reale(435632583,0xf2b7943e115f0LL),
      reale(143237887,0xa8277df5ccab1LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^2, polynomial in n of order 27
      real(0x52cac993243497e0LL),real(0x6437dfaee57b9d40LL),
      real(0x7a3f9cad4d2f48a0LL),reale(2405,0xee01eec3f2b00LL),
      reale(2986,0x65a22988df560LL),reale(3743,0xe8ba104bd58c0LL),
      reale(4745,0x82561551e620LL),reale(6086,0xa7581d3ddee80LL),
      reale(7912,0x8561dfdd262e0LL),reale(10440,0x7aa2aab74b440LL),
      reale(14008,0x9b1a2c148b3a0LL),reale(19155,0xcd3b8407d7200LL),
      reale(26767,0x9792b4f9c2060LL),reale(38350,0xb50c17257efc0LL),
      reale(56574,0xaf828f4edf120LL),reale(86399,0xb1bc40483f580LL),
      reale(137581,0x7d29442656de0LL),reale(230687,0xc9059cc5d4b40LL),
      reale(413025,0xcba5d91bbdea0LL),reale(806439,0xbad85d457b900LL),
      reale(1777226,0xdb254a1088b60LL),reale(4709200,0x187f6563b06c0LL),
      reale(17312174,0x4c53d944cbc20LL),reale(151524377,0x682a2ddefc80LL),
      -reale(970338799,0x73aba5c04720LL),reale(1287957204,0xb756685e76240LL),
      -reale(416692036,0xd1e73fe253660LL),-reale(78129756,0xe75b5bfa6fa32LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[0], coeff of eps^1, polynomial in n of order 28
      real(0xb4c355cd41c92c0LL),real(0xd8fea3a41cc7830LL),
      real(0x1064f0c6b9a6ad20LL),real(0x13f7a88902ef1b10LL),
      real(0x1884a414973fcb80LL),real(0x1e5fa2ae5243d7f0LL),
      real(0x25fe0bb384ddd9e0LL),real(0x3006f6e3e0e25ad0LL),
      real(0x3d6c2c13c34ec440LL),real(0x4f91f34825bd4fb0LL),
      real(0x688ffb74f98676a0LL),reale(2233,0xdec33bb086290LL),
      reale(3036,0xe53843c2cdd00LL),reale(4213,0xb13e1137e3f70LL),
      reale(5984,0xaa1cca8abe360LL),reale(8732,0xb9880d6c69250LL),
      reale(13152,0x1eadcfcfd75c0LL),reale(20566,0x4e1752c3c0730LL),
      reale(33653,0xf4262a5798020LL),reale(58247,0x3a420e3524a10LL),
      reale(108257,0x7934f39e3ee80LL),reale(221025,0xaccc1c0dc06f0LL),
      reale(514222,0xffbb852faace0LL),reale(1456965,0x29e8a4070e9d0LL),
      reale(5827860,0xa7a2901c3a740LL),reale(56821641,0x6270fd1339eb0LL),
      -reale(416692036,0xd1e73fe253660LL),reale(625038055,0x3adadfd37d190LL),
      -reale(273454149,0x29bfc1ec86bafLL),reale(1367270745,0xd0bec99ea1a6bLL),
      // C4[0], coeff of eps^0, polynomial in n of order 29
      reale(42171,0xbca3d5a569b4LL),reale(46862,0xd0a41cdef9cf0LL),
      reale(52277,0xa2d5316ac1b2cLL),reale(58560,0x6f94d669a7a28LL),
      reale(65892,0x788629d238da4LL),reale(74502,0x6b99bdf690d60LL),
      reale(84681,0x87b277eadbb1cLL),reale(96804,0x8c76c6701c898LL),
      reale(111359,0x1427f62cd3d94LL),reale(128987,0x59921e2221dd0LL),
      reale(150546,0xaa0136eb20f0cLL),reale(177198,0x7742592373f08LL),
      reale(210542,0x4360b9bd64984LL),reale(252821,0x8a8c09196de40LL),
      reale(307248,0x66986780ae6fcLL),reale(378530,0x79d0ac77ed78LL),
      reale(473750,0x5114d83948174LL),reale(603901,0x80acdb5cb5eb0LL),
      reale(786661,0x2afc1dbf812ecLL),reale(1051686,0xda8ab314e3e8LL),
      reale(1451326,0xc0ede2017b564LL),reale(2083956,0x5d3b51a63af20LL),
      reale(3149615,0xde5c8fc3f62dcLL),reale(5099378,0x12ae3e18b3258LL),
      reale(9106032,0x45ee012c1b554LL),reale(18940547,0x20d0545bbdf90LL),
      reale(52086504,0x9a3ce7fc4a6ccLL),reale(312519027,0x9d6d6fe9be8c8LL),
      -reale(1093816596,0xa6ff07b21aebcLL),
      reale(2734541491LL,0xa17d933d434d6LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[1], coeff of eps^29, polynomial in n of order 0
      917561,real(273868982145LL),
      // C4[1], coeff of eps^28, polynomial in n of order 1
      -real(125915776),real(90505212),real(0x73d4d30e25bLL),
      // C4[1], coeff of eps^27, polynomial in n of order 2
      -real(0x2f7e4f2fca0LL),real(0x161b06db8f0LL),real(379339642199LL),
      real(0x145a25f15d59339LL),
      // C4[1], coeff of eps^26, polynomial in n of order 3
      -real(0x780f9f651c0LL),real(0x49cd6538080LL),-real(0x275396e6f40LL),
      real(0x1c1406225eaLL),real(0x1e2d6465e2b066fLL),
      // C4[1], coeff of eps^25, polynomial in n of order 4
      -real(0x226e68a74f6c2c0LL),real(0x178fbd94c6e4130LL),
      -real(0x10bafa7048ffb60LL),real(0x7b204e43552d10LL),
      real(0x1ebd785c76c649LL),reale(369943,0xaebaf6655156dLL),
      // C4[1], coeff of eps^24, polynomial in n of order 5
      -real(0x26adfa4c2bcf8500LL),real(0x1be7e116f09bc400LL),
      -real(0x1641521374362300LL),real(0xd7dd4a2b1831200LL),
      -real(0x7449d087ac65100LL),real(0x525502d56a2a1d8LL),
      reale(4562638,0xc0573436eb2ebLL),
      // C4[1], coeff of eps^23, polynomial in n of order 6
      -reale(27299,0x1e7fae46f2ae0LL),reale(20250,0xb050f61211530LL),
      -reale(17170,0x1ccacfb407b40LL),reale(11560,0x5557506ac7a50LL),
      -reale(8300,0x1ee1dfec0f3a0LL),reale(3760,0xc5da39149a170LL),
      real(0x3aaaad07e2dbe15fLL),reale(141441801,0x4a8f52a67aa75LL),
      // C4[1], coeff of eps^22, polynomial in n of order 7
      -reale(223720,0xada70de871dc0LL),reale(168212,0x95f7a36b8e780LL),
      -reale(147708,0x4639d71413140LL),reale(104570,0x398040c96dd00LL),
      -reale(84304,0x27ca2fe2f28c0LL),reale(50205,0xd862a9f308280LL),
      -reale(27426,0xbe7e08935dc40LL),reale(19210,0x9794de13dcf52LL),
      reale(820362447,0x7d3f45c59430dLL),
      // C4[1], coeff of eps^21, polynomial in n of order 8
      -reale(1591044,0x45108afb80980LL),reale(1200725,0xfaaefe8d2aff0LL),
      -reale(1074110,0x244b18cc1fd20LL),reale(779463,0x6e55e2794e4d0LL),
      -reale(667443,0x7f273db50d4c0LL),reale(440073,0xbd38cdf5ffbb0LL),
      -reale(320490,0xb0902bc064460LL),reale(142410,0x1eb038cc00090LL),
      reale(35531,0x5cce3f7afbb81LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[1], coeff of eps^20, polynomial in n of order 9
      -reale(6932123,0xff59c6bb56f80LL),reale(5207764,0x9d4c81592dc00LL),
      -reale(4682178,0xdef9cf054a880LL),reale(3431350,0xdcd7f0ab97d00LL),
      -reale(3036244,0xeb9781cfe3980LL),reale(2097463,0x35c6f48ae00LL),
      -reale(1714507,0xab45478b85280LL),reale(997568,0xe75b4df283f00LL),
      -reale(555001,0x356f72a492380LL),reale(383325,0x3033ad4799914LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^19, polynomial in n of order 10
      -reale(10475274,0x80e3f984eb560LL),reale(7761418,0x6cb2d37d31d50LL),
      -reale(6912729,0x2574b8548f80LL),reale(5061056,0xbff13b9f8e7b0LL),
      -reale(4542234,0x9c8561f8559a0LL),reale(3202970,0x45874de1c0010LL),
      -reale(2776395,0x2331e9957c0LL),reale(1780809,0x24244086de270LL),
      -reale(1321308,0xb7d4404aacde0LL),reale(572110,0xf0d923e3d0ad0LL),
      reale(142666,0x15ad08c690505LL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^18, polynomial in n of order 11
      -reale(16991539,0x3bfa3a952a5c0LL),reale(12232630,0xc216625651e80LL),
      -reale(10582386,0xca84c044c7740LL),reale(7659664,0x22fef68736200LL),
      -reale(6852368,0xbf4b993050cc0LL),reale(4854746,0x78ae9dfa88580LL),
      -reale(4332124,0x5850c11d91e40LL),reale(2896859,0x8330e6242d100LL),
      -reale(2410777,0x3c4e4b27563c0LL),reale(1359574,0x6f5bc7e308c80LL),
      -reale(775169,0xf705a84369540LL),reale(525423,0x9fd72933d2d3aLL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^17, polynomial in n of order 12
      -reale(31605635,0x9b2a6245129c0LL),reale(21349095,0xec111ef51efd0LL),
      -reale(17343382,0xc6b59d854f620LL),reale(12224940,0xad54b9902f0LL),
      -reale(10665275,0xcb2c9d1586680LL),reale(7495419,0x2bbe593f97c10LL),
      -reale(6731026,0x5bd11498926e0LL),reale(4567553,0xbb95797dfef30LL),
      -reale(4019270,0xe17fb3dce340LL),reale(2483542,0x18261977df050LL),
      -reale(1889445,0x252a3b83f47a0LL),reale(789608,0x3727b34041370LL),
      reale(196748,0x5030b26b63d7fLL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^16, polynomial in n of order 13
      -reale(83651327,0x7df35b769ce00LL),reale(46183264,0x6a662d0fec800LL),
      -reale(32523895,0xbf44a3e60200LL),reale(21575930,0xbd1dba7599c00LL),
      -reale(17706525,0xdbcb8c6749600LL),reale(12151631,0x7c587583d3000LL),
      -reale(10707728,0xa79806e6f4a00LL),reale(7245171,0x8aa6d7e27c400LL),
      -reale(6517082,0x9ff2c462fde00LL),reale(4168671,0x7a21919979800LL),
      -reale(3551918,0x26047c5101200LL),reale(1918361,0x786d4fd8aec00LL),
      -reale(1131511,0x7e7a26769a600LL),reale(747310,0xbb693903a2f10LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^15, polynomial in n of order 14
      -reale(63372442,0x2cb5338504ea0LL),reale(236021120,0xed659df2db350LL),
      -reale(86667901,0x5273be9be40LL),reale(47209611,0xc1161d91d1e30LL),
      -reale(33537857,0x3d1f3cdba35e0LL),reale(21739691,0xd5c3b2c9df710LL),
      -reale(18074666,0x2123c601d8980LL),reale(11984705,0x3d2e52a8729f0LL),
      -reale(10682808,0x1cfcfab158d20LL),reale(6875060,0xeec2e9924a2d0LL),
      -reale(6158904,0xf3892aedc14c0LL),reale(3612073,0x775a08e9d4db0LL),
      -reale(2844696,0x4fdad4b74f460LL),reale(1130419,0xe52285ff91690LL),
      reale(281319,0xf8ed6ce679421LL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^14, polynomial in n of order 15
      reale(377918798,0xab0ca9f0672c0LL),-reale(418618018,0x8099eba53f80LL),
      -reale(60854873,0x3eafa33f453c0LL),reale(245263030,0xf5560cf897d00LL),
      -reale(90083330,0xb4182a1e90640LL),reale(48226005,0xa87e22e4ae980LL),
      -reale(34666917,0x2b03feac26cc0LL),reale(21804113,0xa9bac4593e00LL),
      -reale(18434597,0x75e58711b4f40LL),reale(11683388,0x18da60c9eb280LL),
      -reale(10544255,0x717858fde75c0LL),reale(6335167,0xce8110cc57f00LL),
      -reale(5568830,0x1a6ca9ba6a840LL),reale(2826076,0xf4ab3cac7db80LL),
      -reale(1750284,0x2ff80145eaec0LL),reale(1113751,0xd17a5fb748e66LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^13, polynomial in n of order 16
      -reale(7676111,0x5b2a6c5f6c100LL),-reale(64415807,0x4cf1fd08a9430LL),
      reale(389009273,0x614b445047d20LL),-reale(437396877,0xd309fa5941090LL),
      -reale(57368388,0x6af986a1a0c0LL),reale(255600151,0x61702d3245910LL),
      -reale(94005962,0x2924b0b2256a0LL),reale(49188288,0xa4967a4d0acb0LL),
      -reale(35935634,0xccf0586b2e080LL),reale(21713831,0x3869a07cfee50LL),
      -reale(18759173,0xcf3c8197a7a60LL),reale(11187408,0x277eed08021f0LL),
      -reale(10209411,0xbc33094486040LL),reale(5549613,0x5f33e35304b90LL),
      -reale(4590963,0x90f6e6e49ce20LL),reale(1692490,0x5de933ef26f30LL),
      reale(420297,0x50d0b3d8c1d9bLL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^12, polynomial in n of order 17
      -reale(852919,0x6a82cfa963080LL),-reale(2188759,0x20ca5d762f800LL),
      -reale(7786929,0x3421dcca91f80LL),-reale(65787035,0x1d560be049100LL),
      reale(401061675,0x8c48395cfc980LL),-reale(458713135,0x22175c326fa00LL),
      -reale(52544362,0x54a9b8a28c580LL),reale(267237346,0x9f71e62ba7d00LL),
      -reale(98592445,0x567d144d01c80LL),reale(50019657,0x7efcd81e48400LL),
      -reale(37374118,0xabf7952238b80LL),reale(21383288,0xfc61768bbcb00LL),
      -reale(18992011,0x5234632e06280LL),reale(10406178,0xe1fef86250200LL),
      -reale(9523344,0xe57e66503f180LL),reale(4398013,0x8a16c0de4d900LL),
      -reale(2932033,0xa738784cb8880LL),reale(1764194,0xc6396b58af30cLL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^11, polynomial in n of order 18
      -reale(210362,0x76b369d3025e0LL),-reale(399459,0x1eaf9acef0ab0LL),
      -reale(856141,0xe229f972ba700LL),-reale(2206922,0xef935c87bb50LL),
      -reale(7896496,0x6b0bc697c0820LL),-reale(67217074,0x2cc6331df1df0LL),
      reale(414202467,0x2b5605d0252c0LL),-reale(483149583,0xa02db175d690LL),
      -reale(45836711,0xc18042256fa60LL),reale(280420397,0xa9af8baa076d0LL),
      -reale(104078404,0x7a91f5b525380LL),reale(50585814,0x9d940e3bb2630LL),
      -reale(39015494,0x6a69555b81ca0LL),reale(20678727,0x5f0f1f3a9390LL),
      -reale(19012332,0x416957968b9c0LL),reale(9200947,0xc21b589061af0LL),
      -reale(8178296,0xad1e8ab768ee0LL),reale(2676456,0xd6956da2a1850LL),
      reale(661843,0xede00571b821dLL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^10, polynomial in n of order 19
      -reale(73282,0x88acf774cdcc0LL),-reale(119856,0xfafc4232d6980LL),
      -reale(209310,0xc95dad3d9d040LL),-reale(398728,0xc3246fdb30c00LL),
      -reale(857927,0x8ca89fdf097c0LL),-reale(2222415,0x7f22a8f79ee80LL),
      -reale(8002412,0xa401cae100b40LL),-reale(68698832,0xcf05dd2d1e900LL),
      reale(428572510,0x4af905b8fd40LL),-reale(511480829,0xaa7af93dad380LL),
      -reale(36412636,0xa51695c145640LL),reale(295430858,0x62539c3ab7a00LL),
      -reale(110834541,0xf7ac6a286ddc0LL),reale(50648730,0xf42d6a1912780LL),
      -reale(40882711,0xc825af61d7140LL),reale(19389515,0xc578a6be65d00LL),
      -reale(18548541,0x30b0433e6e8c0LL),reale(7353872,0xa4f0c77ab4280LL),
      -reale(5517208,0xc642445621c40LL),reale(3035548,0x619b33f1391d2LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^9, polynomial in n of order 20
      -reale(31116,0x5ced59f2a6a40LL),-reale(46466,0x39ef1648a3c30LL),
      -reale(72339,0x13bec712995a0LL),-reale(118591,0xe96704ee23c10LL),
      -reale(207681,0xf3272ddf69500LL),-reale(396975,0x5586a3fda15f0LL),
      -reale(857776,0x96a9e394d3460LL),-reale(2234014,0x9c760527155d0LL),
      -reale(8101033,0x1f3b77f93fc0LL),-reale(70217181,0xc7476a97287b0LL),
      reale(444320933,0x84d59896b7ce0LL),-reale(544755366,0x60ab42e093790LL),
      -reale(22958170,0x5fc77e584ca80LL),reale(312550991,0xea91e4bc80e90LL),
      -reale(119474190,0x655c7a979e1e0LL),reale(49778595,0x69cfb591beb0LL),
      -reale(42938053,0xad555dfab9540LL),reale(17185991,0x9567a8e814cd0LL),
      -reale(16947236,0xc941a0517b0a0LL),reale(4507394,0xb6bfddcb2cf0LL),
      reale(1103154,0xee71952935057LL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^8, polynomial in n of order 21
      -reale(15013,0x669ca85dbff00LL),-reale(21081,0x7f4d799198400LL),
      -reale(30470,0xbdb587d74d900LL),-reale(45587,0xe4badb51b1a00LL),
      -reale(71124,0x646ea35b6300LL),-reale(116891,0x8adb62aa4d000LL),
      -reale(205315,0x1aa2ab2ec7d00LL),-reale(393884,0x4b8d8eda78600LL),
      -reale(855000,0x2faa553050700LL),-reale(2239966,0xb31164c141c00LL),
      -reale(8186764,0x97347e701e100LL),-reale(71742883,0x7f111739b7200LL),
      reale(461586973,0x9a516d5401500LL),-reale(584418823,0xe1245bd6e6800LL),
      -reale(3315305,0x14110f9c0500LL),reale(331936814,0x28269ca022200LL),
      -reale(131069117,0x7ee7ad0730f00LL),reale(47184778,0x227a729454c00LL),
      -reale(44897669,0x9cd1b2a1e900LL),reale(13574545,0xcd96a182a3600LL),
      -reale(12485695,0x45db16a057300LL),reale(5879734,0x70bef82b8988LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^7, polynomial in n of order 22
      -reale(7900,0x638c66d8a8320LL),-reale(10613,0xf2ac3092c9cb0LL),
      -reale(14565,0xe107ae27501c0LL),-reale(20489,0xead89ce414d0LL),
      -reale(29670,0x849ce08edf860LL),-reale(44482,0xeb1f022729ef0LL),
      -reale(69562,0xbdfcfee35b00LL),-reale(114632,0x975e8fa16f10LL),
      -reale(201989,0x9411d71111da0LL),-reale(389021,0x33d7ff034b930LL),
      -reale(848628,0xc0285ec233440LL),-reale(2237713,0xb97d9ca55b150LL),
      -reale(8250880,0x9132887d792e0LL),-reale(73221392,0xf1ffe05c8b70LL),
      reale(480452831,0x383b5471fd280LL),-reale(632496874,0xca3591eba7b90LL),
      reale(26233104,0x13df159bb07e0LL),reale(353203487,0x101c2c33c4a50LL),
      -reale(147596513,0x7a337ff05e6c0LL),reale(41406718,0x88562e0e69230LL),
      -reale(45513246,0x22b5bfcbced60LL),reale(7934370,0xa8c8e9d8c2810LL),
      reale(1869414,0xdc5c61854a479LL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^6, polynomial in n of order 23
      -reale(4406,0xf939ae5c97c40LL),-reale(5729,0xf863eba5bf80LL),
      -reale(7570,0xa927e082c4c0LL),-reale(10189,0xdc3d2b5930900LL),
      -reale(14011,0xfd72406188940LL),-reale(19751,0x4ee9330f94280LL),
      -reale(28665,0xa6c18d00fb1c0LL),-reale(43078,0xe8ed052a45400LL),
      -reale(67543,0xd4150add2640LL),-reale(111634,0xb28e55bb02580LL),
      -reale(197389,0xccdd68505cec0LL),-reale(381765,0x22e00b9b89f00LL),
      -reale(837258,0xa000eefe9340LL),-reale(2223425,0xd3d15b309a880LL),
      -reale(8279438,0xc28db224c5bc0LL),-reale(74551261,0xb7816e54f2a00LL),
      reale(500824278,0x3891b999befc0LL),-reale(691847154,0x918a2dd450b80LL),
      reale(72461747,0xa045596356740LL),reale(374046829,0x41b777218cb00LL),
      -reale(172833056,0x62b9485f4dd40LL),reale(29915148,0x80284d25e7180LL),
      -reale(39423763,0x40d338467c5c0LL),reale(13659048,0x68e501c228ffeLL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^5, polynomial in n of order 24
      -reale(2545,0x1363104362d80LL),-reale(3226,0xe67b1424a4830LL),
      -reale(4144,0x8c711302fa660LL),-reale(5400,0xc1bfe2853af90LL),
      -reale(7153,0xb2c26c1682b40LL),-reale(9653,0x9e8ef4e7cf0f0LL),
      -reale(13308,0xeb09aee491820LL),-reale(18810,0x561040fe22850LL),
      -reale(27375,0xc35e0fb3fc900LL),-reale(41260,0x7d7f41fc271b0LL),
      -reale(64893,0xc7a96414399e0LL),-reale(107622,0xe02e2157de910LL),
      -reale(191035,0x6ce8a0a1be6c0LL),-reale(371181,0x96988a373aa70LL),
      -reale(818768,0xa91a46aa60ba0LL),-reale(2191167,0x9fde37effd1d0LL),
      -reale(8249435,0xe27cdc35b6480LL),-reale(75540143,0x55cc77d97b30LL),
      reale(522119910,0xf5aa540a8b2a0LL),-reale(766397212,0x64559a510c290LL),
      reale(148547296,0x8152775e2ddc0LL),reale(385247751,0x81b301a133c10LL),
      -reale(213402544,0x90fce845e3f20LL),reale(10198756,0x255c7c31664b0LL),
      reale(1365904,0xd74a19c69db33LL),reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^4, polynomial in n of order 25
      -real(0x5cd20bbc3c672180LL),-real(0x73720b2d98187c00LL),
      -reale(2321,0xc4eb857568680LL),-reale(2952,0xb2617088c8f00LL),
      -reale(3804,0x417bd8fa2e380LL),-reale(4973,0x5ec86f601d200LL),
      -reale(6609,0x998272f30a880LL),-reale(8950,0x197c7ab46b500LL),
      -reale(12382,0xcc481e2a44580LL),-reale(17565,0x5f7861969a800LL),
      -reale(25660,0x4a6f330e22a80LL),-reale(38825,0xe447100991b00LL),
      -reale(61313,0x47573aa0ec780LL),-reale(102123,0xa55bb6037e00LL),
      -reale(182121,0xfb4d0590e8c80LL),-reale(355742,0x340be91b74100LL),
      -reale(789743,0xf318e4285e980LL),-reale(2131260,0x2c59b0f82d400LL),
      -reale(8121193,0x3f9cc7c594e80LL),-reale(75808472,0x814742dd4a700LL),
      reale(542406027,0xe15955752d480LL),-reale(860719085,0xb088c959b2a00LL),
      reale(281794203,0x6d691a09a0f80LL),reale(349671639,0x4a19c69db3300LL),
      -reale(268081590,0x1f35e51280d80LL),reale(42616231,0x9d4bdce6b704LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^3, polynomial in n of order 26
      -real(0x34f88b61ee2c2e60LL),-real(0x40e8b73250ad02b0LL),
      -real(0x50402824a1190680LL),-real(0x643133a56bf6de50LL),
      -real(0x7e70b50d7e53aea0LL),-reale(2583,0x89ee9103c6bf0LL),
      -reale(3343,0x2d56b6f20aac0LL),-reale(4390,0x9150bee746f90LL),
      -reale(5862,0xecb9ee1767ee0LL),-reale(7978,0x9b4551158ad30LL),
      -reale(11096,0x13774a5e7af00LL),-reale(15825,0x3f23db737e8d0LL),
      -reale(23248,0xf45a340cbf20LL),-reale(35380,0xaf4478627e670LL),
      -reale(56209,0x8a81f32e3340LL),-reale(94205,0x2f98ae2576a10LL),
      -reale(169093,0xeae4ad4ee8f60LL),-reale(332577,0xf0ed8664037b0LL),
      -reale(743995,0x906300fb45780LL),-reale(2026493,0x9c6e844791350LL),
      -reale(7821602,0x7531c16940fa0LL),-reale(74557824,0x1ed43b2e7c0f0LL),
      reale(555703654,0x34418f385c440LL),-reale(974709694,0x84f4a67130490LL),
      reale(527421389,0x42f7f1faaa020LL),reale(94702735,0xa411a5cab5dd0LL),
      -reale(117194635,0x5b0909f7a774bLL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^2, polynomial in n of order 27
      -real(0x1bd57a8f504dd3c0LL),-real(0x21b6ff10b9172180LL),
      -real(0x292825cda3a88940LL),-real(0x32aacbfadedfca00LL),
      -real(0x3ef38a62fa0322c0LL),-real(0x4f013a1cfd80d280LL),
      -real(0x64414a4729c69840LL),-reale(2060,0x90ead26a03300LL),
      -reale(2683,0x237c6d92be1c0LL),-reale(3547,0x3d9a05c33e380LL),
      -reale(4770,0x6ec9da59bf740LL),-reale(6541,0x1657e411dc00LL),
      -reale(9170,0x1a8b4944fd0c0LL),-reale(13190,0xb069410801480LL),
      -reale(19554,0x9e393a3b06640LL),-reale(30047,0xba30505448500LL),
      -reale(48224,0x707d4f4f6afc0LL),-reale(81689,0xf05ca40b52580LL),
      -reale(148265,0xab90de58ba540LL),-reale(294962,0x64373b047ee00LL),
      -reale(667587,0xc0c688fa83ec0LL),-reale(1840377,0xc842d822d680LL),
      -reale(7199121,0xfc41489b57440LL),-reale(69934327,0xdb9ec152bd700LL),
      reale(541991040,0xe60e5a413c240LL),-reale(1060670639,0x2d9274118e780LL),
      reale(833384073,0xa3ce7fc4a6cc0LL),-reale(234389270,0xb61213ef4ee96LL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[1], coeff of eps^1, polynomial in n of order 28
      -real(0xb4c355cd41c92c0LL),-real(0xd8fea3a41cc7830LL),
      -real(0x1064f0c6b9a6ad20LL),-real(0x13f7a88902ef1b10LL),
      -real(0x1884a414973fcb80LL),-real(0x1e5fa2ae5243d7f0LL),
      -real(0x25fe0bb384ddd9e0LL),-real(0x3006f6e3e0e25ad0LL),
      -real(0x3d6c2c13c34ec440LL),-real(0x4f91f34825bd4fb0LL),
      -real(0x688ffb74f98676a0LL),-reale(2233,0xdec33bb086290LL),
      -reale(3036,0xe53843c2cdd00LL),-reale(4213,0xb13e1137e3f70LL),
      -reale(5984,0xaa1cca8abe360LL),-reale(8732,0xb9880d6c69250LL),
      -reale(13152,0x1eadcfcfd75c0LL),-reale(20566,0x4e1752c3c0730LL),
      -reale(33653,0xf4262a5798020LL),-reale(58247,0x3a420e3524a10LL),
      -reale(108257,0x7934f39e3ee80LL),-reale(221025,0xaccc1c0dc06f0LL),
      -reale(514222,0xffbb852faace0LL),-reale(1456965,0x29e8a4070e9d0LL),
      -reale(5827860,0xa7a2901c3a740LL),-reale(56821641,0x6270fd1339eb0LL),
      reale(416692036,0xd1e73fe253660LL),-reale(625038055,0x3adadfd37d190LL),
      reale(273454149,0x29bfc1ec86bafLL),
      reale(12305436712LL,0x56b51693aedc3LL),
      // C4[2], coeff of eps^29, polynomial in n of order 0
      185528,real(30429886905LL),
      // C4[2], coeff of eps^28, polynomial in n of order 1
      real(17366491968LL),real(4404238552LL),real(0x74e318fa9c07fLL),
      // C4[2], coeff of eps^27, polynomial in n of order 2
      real(412763643136LL),-real(248137794944LL),real(164642704408LL),
      real(0x4d882f0532d9e9LL),
      // C4[2], coeff of eps^26, polynomial in n of order 3
      real(0x11462b92d913a0LL),-real(0xdd4620ebadc40LL),
      real(0x5974730e46be0LL),real(0x16bcec57851ccLL),
      reale(33547,0x1cf91962af003LL),
      // C4[2], coeff of eps^25, polynomial in n of order 4
      real(0xc83679b433c00LL),-real(0xb29b6d58dfb00LL),real(0x5f4e3bdd4de00LL),
      -real(0x3affd9960e900LL),real(0x2665fb625f490LL),
      reale(15809,0x8f200ee7e2a7dLL),
      // C4[2], coeff of eps^24, polynomial in n of order 5
      real(0x67b92a8524a18e80LL),-real(0x609d7d3ca356ae00LL),
      real(0x39db180d1b52d580LL),-real(0x2fa1e9183dec9700LL),
      real(0x1294d8f2627edc80LL),real(0x4bc94ddbc9bad70LL),
      reale(22813193,0xc1b4051297e97LL),
      // C4[2], coeff of eps^23, polynomial in n of order 6
      reale(24830,0x3d0fb879bb600LL),-reale(23212,0xa100635ccdb00LL),
      reale(14957,0x147cd156ba400LL),-reale(13653,0x51ea4b9c89d00LL),
      reale(7024,0x2535370909200LL),-reale(4511,0x3af63b60c9f00LL),
      reale(2865,0xf50f5adcce1f0LL),reale(235736335,0x7c44346acc6c3LL),
      // C4[2], coeff of eps^22, polynomial in n of order 7
      reale(1046092,0x25a6222f26060LL),-reale(949436,0x14a3a722f1840LL),
      reale(652845,0xb96689ab42720LL),-reale(615919,0x6f1345ab50580LL),
      reale(356624,0x982d38f2a9de0LL),-reale(303839,0x22c37d5c832c0LL),
      reale(113262,0x286189b57e4a0LL),reale(28978,0x12ae8b059bc84LL),
      reale(6836353729LL,0x13b9f01928417LL),
      // C4[2], coeff of eps^21, polynomial in n of order 8
      reale(4643688,0x71b79cbf7cc00LL),-reale(3959056,0x83e38a4f9d180LL),
      reale(2926140,0x6f81ce5fc3900LL),-reale(2722736,0xdd03df5282c80LL),
      reale(1710940,0xc70403130e600LL),-reale(1602990,0x9ebb76967a780LL),
      reale(787738,0x6bf60987b1300LL),-reale(530212,0xcde2a88ab0280LL),
      reale(326645,0xab9033855e368LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^20, polynomial in n of order 9
      reale(2366152,0x4fc26559c91c0LL),-reale(1830925,0x4d73259824200LL),
      reale(1477489,0x62c9a90a52a40LL),-reale(1299560,0xe7bf798235180LL),
      reale(885946,0x5cb0a99f5e2c0LL),-reale(843740,0x47153eb842100LL),
      reale(469359,0x79db9d7cfb40LL),-reale(417111,0x1a4c5e2477080LL),
      reale(146559,0x51b0aa3dcb3c0LL),reale(37677,0x6dd5ee66abd48LL),
      reale(6836353729LL,0x13b9f01928417LL),
      // C4[2], coeff of eps^19, polynomial in n of order 10
      reale(11390177,0xa8f910291300LL),-reale(7729638,0x6f23cf47c2480LL),
      reale(6929266,0x5fb765e065c00LL),-reale(5514735,0x5eb0876136380LL),
      reale(4148166,0x27d6c40aa500LL),-reale(3788609,0xfef33001c8280LL),
      reale(2322601,0x1de03c2bc2e00LL),-reale(2237878,0x77b7642b94180LL),
      reale(1037457,0x571c66f013700LL),-reale(742165,0x8c39e6d5b6080LL),
      reale(439349,0xf7cfa6e796fc8LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^18, polynomial in n of order 11
      reale(19643005,0x3eb0d373a0e0LL),-reale(11359402,0x98e8f09139c0LL),
      reale(11381255,0xacc1b03fd73a0LL),-reale(7834592,0x92741bdd3b00LL),
      reale(6664656,0xa317edb25b660LL),-reale(5516050,0x3ff87cc43bc40LL),
      reale(3774293,0xd5e83edc68920LL),-reale(3594547,0xbec9f61701d80LL),
      reale(1908400,0x61c5f793c0be0LL),-reale(1786093,0xfaf3f7a19bec0LL),
      reale(579905,0x9d50696085ea0LL),reale(150042,0xa9efa9004c604LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^17, polynomial in n of order 12
      reale(38321815,0x1e48683dc9800LL),-reale(18616913,0x727791f8dfa00LL),
      reale(20113440,0xb841223d75400LL),-reale(11495937,0x9838f29931e00LL),
      reale(11261630,0x21fd3747b1000LL),-reale(7960716,0x75135ee9c200LL),
      reale(6275150,0xa8a2fa972cc00LL),-reale(5471565,0x945df446e600LL),
      reale(3293426,0x6eab44c698800LL),-reale(3257897,0x559df659f8a00LL),
      reale(1401057,0x756ea738a4400LL),-reale(1086629,0xf49cb94a8ae00LL),
      reale(610116,0x479bdc6c290e0LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^16, polynomial in n of order 13
      reale(102781113,0x98fe5a9192500LL),-reale(40336104,0xccc089a851400LL),
      reale(40165652,0x6e617f3b73300LL),-reale(18616625,0x95536d5576600LL),
      reale(20514709,0xd39b96f5ec100LL),-reale(11691503,0x7c1154bb0b800LL),
      reale(10980290,0x40d1adbe6cf00LL),-reale(8104717,0x4a433bfb60a00LL),
      reale(5726151,0xc3b2b2965d00LL),-reale(5331323,0xa4559d80c5c00LL),
      reale(2689333,0x7cf2f82446b00LL),-reale(2678624,0x7904ff2b8ae00LL),
      reale(779755,0xfacbca777f900LL),reale(203539,0xb4670b88476e0LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^15, polynomial in n of order 14
      -reale(23295494,0x8be82e34e6400LL),-reale(256522224,0x1264f586eb600LL),
      reale(109420782,0x9692235ce1800LL),-reale(40005401,0x76f47ac799a00LL),
      reale(42210732,0x9175627089400LL),-reale(18637789,0x360d04338fe00LL),
      reale(20777547,0x32d7f69c1000LL),-reale(11978808,0x3c6fce691e200LL),
      reale(10467739,0x890cbd2438c00LL),-reale(8246695,0x5d95a89294600LL),
      reale(4981450,0x2e83f5dba0800LL),-reale(4997884,0x48d2490e42a00LL),
      reale(1949724,0xd6b9d613a8400LL),-reale(1687002,0x42840cd678e00LL),
      reale(881316,0x5154c853b06e0LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^14, polynomial in n of order 15
      -reale(315852553,0x127aa1fb9560LL),reale(452067016,0x32f06289dc340LL),
      -reale(36389203,0xc905d2dd0bc20LL),-reale(265701999,0x414c3c9652f80LL),
      reale(117462481,0xb44ff33f8ed20LL),-reale(39375172,0xb9e521c5c6240LL),
      reale(44443567,0x98c20ae94660LL),-reale(18737379,0x9088d09ce7500LL),
      reale(20789662,0x74772cb6e2fa0LL),-reale(12399165,0xc39cbc16e07c0LL),
      reale(9634015,0x48be8ec7788e0LL),-reale(8326007,0x8f1246dddba80LL),
      reale(4012687,0x8a9763f933220LL),-reale(4283805,0xe15bd5742d40LL),
      reale(1064918,0x3e0322e890b60LL),reale(281445,0x189dacfa2913cLL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^13, polynomial in n of order 16
      reale(4607575,0xc9d7900c88800LL),reale(44527228,0x61b96ac1eb380LL),
      -reale(320302478,0xa276d3450e900LL),reale(471382647,0x4d0623cc86a80LL),
      -reale(52535715,0x404f1a5b09a00LL),-reale(275262322,0xf3348bb543e80LL),
      reale(127364360,0xbf0504ec13500LL),-reale(38376532,0x74833ebc78780LL),
      reale(46801690,0x6a3245e5c4400LL),-reale(19021914,0x3bda110f1b080LL),
      reale(20372666,0xf7fc04d85300LL),-reale(12992077,0x825700022f980LL),
      reale(8374681,0xba502a56d2200LL),-reale(8187369,0x8d48a8bba280LL),
      reale(2818780,0x7113503f27100LL),-reale(2834494,0xf2038f04beb80LL),
      reale(1337917,0xc906f381aecf8LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^12, polynomial in n of order 17
      reale(388658,0x19c7c6f8ea2c0LL),reale(1117971,0xaadcbdb38ac00LL),
      reale(4519560,0xaee28ee393540LL),reale(44278119,0xe09b9f50af680LL),
      -reale(324493551,0x5c00bae29840LL),reale(492697628,0x7d1cc3fd18100LL),
      -reale(72657626,0xb42806bf185c0LL),-reale(284925253,0x57cc84a557480LL),
      reale(139770748,0x33e950dc3acc0LL),-reale(36961790,0xef70c005baa00LL),
      reale(49119876,0xa052562f03f40LL),-reale(19681131,0xbaa50226adf80LL),
      reale(19252422,0xc3af9265b71c0LL),-reale(13755373,0x2f0960c0cd500LL),
      reale(6600104,0x6565773f88440LL),-reale(7462805,0xbfb982e534a80LL),
      reale(1452711,0x6b2cd84feb6c0LL),reale(390635,0x965de9321fbe8LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^11, polynomial in n of order 18
      reale(73868,0xf53613318fd00LL),reale(155158,0x6bea1fc037e80LL),
      reale(370865,0xe686995a3a800LL),reale(1077531,0xb6b00d00e5180LL),
      reale(4409046,0x1d5f244685300LL),reale(43860006,0xf94485a638480LL),
      -reale(328226208,0x254b380304200LL),reale(516242826,0x48cfde1d3d780LL),
      -reale(98028430,0xc7227901d5700LL),-reale(294125055,0xf41dd5cbff580LL),
      reale(155591277,0xc58331ae9d400LL),-reale(35168366,0x6c3820d072280LL),
      reale(51023141,0xfcae9f00dff00LL),-reale(21033813,0x6b0840ce0ef80LL),
      reale(17035669,0xa0ab037f7ea00LL),-reale(14520825,0x209891efc9c80LL),
      reale(4321952,0xda1143d705500LL),-reale(5322397,0x9ed9b44796980LL),
      reale(2165443,0xa5af00ad58358LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^10, polynomial in n of order 19
      reale(19809,0x63304b335a660LL),reale(35566,0xcb4164f348e40LL),
      reale(68577,0xe86c972757e20LL),reale(145245,0xbc9cc7446e200LL),
      reale(350489,0x7e29a3d4285e0LL),reale(1029750,0x45087f82835c0LL),
      reale(4270842,0x2203011585da0LL),reale(43220702,0xa65b618eca980LL),
      -reale(331199124,0xa89ccd5235aa0LL),reale(542217711,0x200e3727c5d40LL),
      -reale(130429686,0x3b8b1d50d02e0LL),-reale(301749371,0x2c4d836f88f00LL),
      reale(176097282,0x8ddfe73d104e0LL),-reale(33280999,0x8c12e2a85fb40LL),
      reale(51717673,0x23cc103525ca0LL),-reale(23558374,0x76fe0e70fc780LL),
      reale(13250268,0x69c1c450ca460LL),-reale(14595460,0xd8a80a3d5d3c0LL),
      reale(1848614,0x7d3564e37c20LL),reale(506231,0x2a6100a6a6db4LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^9, polynomial in n of order 20
      reale(6397,0xfcd62c9faa400LL),reale(10440,0x3fc8ff8e75700LL),
      reale(17841,0xb7bede1dba00LL),reale(32272,0x7935213063d00LL),
      reale(62742,0x8933a9bfd5000LL),reale(134128,0x223daf23d6300LL),
      reale(327129,0xfca43cca0e600LL),reale(973230,0x31dda9e44900LL),
      reale(4098328,0x3528b970ffc00LL),reale(42289297,0xe5d54d5326f00LL),
      -reale(332951092,0xecfda756dee00LL),reale(570709002,0x2878cf4ff5500LL),
      -reale(172380399,0x5788b53115800LL),-reale(305626020,0x9c65fcc7d8500LL),
      reale(202987914,0xbd0aab0ad3e00LL),-reale(32233434,0x3f0406dec9f00LL),
      reale(49604551,0xc747777555400LL),-reale(27757216,0x323bffb167900LL),
      reale(7652705,0x1c15203ae6a00LL),-reale(11782806,0x2b7827f239300LL),
      reale(3811565,0x362856b8e6d30LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^8, polynomial in n of order 21
      reale(2297,0xe5959dcaf9680LL),reale(3515,0xaf44e93439a00LL),
      reale(5557,0xf844363205d80LL),reale(9134,0x3148872cf3100LL),
      reale(15730,0x1f27208afe480LL),reale(28695,0xbe2e993314800LL),
      reale(56314,0x2c7b05479ab80LL),reale(121661,0x287926e675f00LL),
      reale(300328,0xfc8a376113280LL),reale(906274,0xf1fb199eef600LL),
      reale(3883000,0x5f528c391f980LL),reale(40968060,0xe6e08c5558d00LL),
      -reale(332763533,0x8282a4a507f80LL),reale(601507851,0xf6ba284c8a400LL),
      -reale(227453313,0x642fd223ab880LL),-reale(301473974,0xbe5976c5a4500LL),
      reale(238209921,0x57c5b91e6ce80LL),-reale(34582562,0x41ecac4f5ae00LL),
      reale(41696071,0xee870caef9580LL),-reale(33183269,0xa456f79c1700LL),
      reale(1407347,0x27b05f0931c80LL),reale(329283,0x26010fabff570LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^7, polynomial in n of order 22
      real(0x367dbe5da7953e00LL),real(0x4f9a921ac6fb1900LL),
      real(0x773454548df74400LL),reale(2938,0xbc18faed4af00LL),
      reale(4681,0x407a350a64a00LL),reale(7756,0xa0ed83ee90500LL),
      reale(13477,0x2fbfd87edd000LL),reale(24826,0x9ea174e739b00LL),
      reale(49249,0xd3391f1d95600LL),reale(107696,0xcac2013cff100LL),
      reale(269571,0xe064d3a745c00LL),reale(826840,0x70825da398700LL),
      reale(3613882,0x7ef0aa40a6200LL),reale(39120270,0xc5673698bdd00LL),
      -reale(329492011,0x53f65ac991800LL),reale(633695353,0xfeb5c44027300LL),
      -reale(300630213,0xecf09fbea9200LL),-reale(280700646,0xcee0a2073700LL),
      reale(282664342,0x7b726e8a17400LL),-reale(46720160,0x11dfe8c55a100LL),
      reale(23527957,0x90f427ad67a00LL),-reale(33848503,0x5eac35f0d4b00LL),
      reale(7456233,0x7c1f0b332cab0LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^6, polynomial in n of order 23
      real(0x14f52a063dc5fc20LL),real(0x1d93a1e9ceb48740LL),
      real(0x2a911c303b723a60LL),real(0x3ea26bba66a54980LL),
      real(0x5e84fad71b3608a0LL),reale(2349,0x85d3117e94bc0LL),
      reale(3776,0x1c9d51cf2c6e0LL),reale(6317,0x5193932d16e00LL),
      reale(11091,0xc7716ff97d520LL),reale(20667,0xe33c2c4a29040LL),
      reale(41523,0x1a30a42ae9360LL),reale(92100,0xbd0a1f1419280LL),
      reale(234309,0x70b77706661a0LL),reale(732507,0x72fafb4df54c0LL),
      reale(3276808,0xe462aef209fe0LL),reale(36551902,0x4c4d10a4b700LL),
      -reale(321265885,0x720bf168351e0LL),reale(664675522,0x65892c55e9940LL),
      -reale(398339257,0x2b82ef41c13a0LL),-reale(225754486,0xf240500d62480LL),
      reale(330356701,0xbb7252695baa0LL),-reale(82401980,0x37f104ae0a240LL),
      -reale(4970822,0x52bf5cccc8720LL),-reale(3278171,0x9e4b710fe0e14LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^5, polynomial in n of order 24
      real(0x7d5242068d47400LL),real(0xac3832c9e621080LL),
      real(0xf0840d5e59cf500LL),real(0x155fabefd3362980LL),
      real(0x1f01ffac4c30b600LL),real(0x2e0489bbd6aca280LL),
      real(0x461560bdbc05f700LL),real(0x6df6210d29c3bb80LL),
      reale(2857,0xf2e1b87d2f800LL),reale(4836,0xd8d8f4249b480LL),
      reale(8600,0x17271d36df900LL),reale(16248,0x163bc1ffccd80LL),
      reale(33146,0xc23750bad3a00LL),reale(74792,0x260310eab4680LL),
      reale(194024,0xef2cdae46fb00LL),reale(620545,0xfcf47db535f80LL),
      reale(2853712,0x7228ad7b17c00LL),reale(32984640,0x1c4ce82435880LL),
      -reale(304937768,0x83ef272fd0300LL),reale(687819348,0xf9e0f9c397180LL),
      -reale(526420007,0xa1ce2482e4200LL),-reale(101220737,0xb065c6f7c1580LL),
      reale(344186593,0xf79ee4a13ff00LL),-reale(151524377,0x682a2ddefc80LL),
      reale(15298134,0x380aba4a19708LL),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^4, polynomial in n of order 25
      real(0x2b077c634ede840LL),real(0x39e80232e455600LL),
      real(0x4f004399e9803c0LL),real(0x6d6a8dd96e7d980LL),
      real(0x9a16639c690ff40LL),real(0xdd0eb6a29ee1d00LL),
      real(0x143ca2e567649ac0LL),real(0x1e583a687f6ce080LL),
      real(0x2ebb5ae27bca9640LL),real(0x4a366ef6d0a8e400LL),
      real(0x7a244f6987aeb1c0LL),reale(3355,0xff6a995ee780LL),
      reale(6059,0x95d9afc38ad40LL),reale(11647,0x91c4ac30bab00LL),
      reale(24220,0xbe377a4d448c0LL),reale(55835,0xd9394a033ee80LL),
      reale(148417,0x27a782b394440LL),reale(488256,0xe5126fdac7200LL),
      reale(2322515,0xb040a0735fc0LL),reale(28019858,0x3d9464fe1f580LL),
      -reale(275064197,0x290d46715a4c0LL),reale(686424553,0x6984a82213900LL),
      -reale(677745912,0x9f6fb36960940LL),reale(151524377,0x682a2ddefc80LL),
      reale(169007958,0xfd6a53329f240LL),-reale(85232462,0x13a97b9cd6e08LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^3, polynomial in n of order 26
      real(0xc4c78b5f73e700LL),real(0x1046756e5efb980LL),
      real(0x15cbc98d9fba400LL),real(0x1d9279681ffce80LL),
      real(0x28b2f34344c6100LL),real(0x38e6214caec8380LL),
      real(0x50f0f0d0c655e00LL),real(0x7563dc0de2d1880LL),
      real(0xadfad5eb325db00LL),real(0x1083ab8775a8cd80LL),
      real(0x19c9d8efc1ad1800LL),real(0x29945e7f0056e280LL),
      real(0x4594bf2102ba5500LL),real(0x79a9d12705de9780LL),
      reale(3587,0xb2b264e0cd200LL),reale(7053,0x1d58043372c80LL),
      reale(15040,0x44c8073c3cf00LL),reale(35667,0x702872e47e180LL),
      reale(97902,0x6929355be8c00LL),reale(334186,0x1d1de4e87f680LL),
      reale(1659947,0xed2beccfc4900LL),reale(21110207,0x53559189eab80LL),
      -reale(222144335,0x8c70c0703ba00LL),reale(617753229,0x694fabb034080LL),
      -reale(769277606,0x6fd24e8e23d00LL),reale(454573131,0x1387e899cf580LL),
      -reale(104173009,0x3479cff894d98LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[2], coeff of eps^2, polynomial in n of order 27
      real(0x24546bc28a93e0LL),real(0x2f6c4d745b8e40LL),
      real(0x3e90f252c210a0LL),real(0x5380c389acd700LL),
      real(0x70da9adde57d60LL),real(0x9aa08aca5a9fc0LL),
      real(0xd7127fe199fa20LL),real(0x130248120008880LL),
      real(0x1b6103e1c56a6e0LL),real(0x283fa247b6e3140LL),
      real(0x3c89da46fe8a3a0LL),real(0x5d71643158b3a00LL),
      real(0x948b363af771060LL),real(0xf445a32263b42c0LL),
      real(0x1a1d56e9fe070d20LL),real(0x2ecb290f0241eb80LL),
      real(0x58a5da95527fb9e0LL),reale(2876,0x680343126d440LL),
      reale(6354,0x3e35c062e36a0LL),reale(15689,0x7d2910c199d00LL),
      reale(45107,0x47d6102c9a360LL),reale(162386,0x35cf6d6d5e5c0LL),
      reale(857038,0x54e3334f72020LL),reale(11655721,0x4f45203874e80LL),
      -reale(131126864,0xbbc9aa7b23320LL),reale(378810942,0x9046972ad7740LL),
      -reale(416692036,0xd1e73fe253660LL),reale(156259513,0xceb6b7f4df464LL),
      reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[3], coeff of eps^29, polynomial in n of order 0
      594728,real(456448303575LL),
      // C4[3], coeff of eps^28, polynomial in n of order 1
      -real(3245452288LL),real(1965206256),real(0x17609e98859b3LL),
      // C4[3], coeff of eps^27, polynomial in n of order 2
      -real(0x15f49b7dd3600LL),real(0x7876e24c6900LL),real(0x1f5dd75c0b28LL),
      reale(4837,0x68f14547adebLL),
      // C4[3], coeff of eps^26, polynomial in n of order 3
      -real(0x33418e8004000LL),real(0x17b00d59dc000LL),
      -real(0x11669ade1c000LL),real(0xa37322475bc0LL),
      reale(6709,0x6c31d1e089667LL),
      // C4[3], coeff of eps^25, polynomial in n of order 4
      -real(0xc3e38d2fc36800LL),real(0x6a604d6faf7a00LL),
      -real(0x650b3de948f400LL),real(0x20a6596010be00LL),
      real(0x88f534a1fae70LL),reale(275086,0x53fa9cf60167fLL),
      // C4[3], coeff of eps^24, polynomial in n of order 5
      -real(0xdd5f9d233a5800LL),real(0x8b724926c9e000LL),
      -real(0x8af41510346800LL),real(0x3d05686ce77000LL),
      -real(0x2f9901c72df800LL),real(0x1ae74f29ea4ce0LL),
      reale(223345,0xf3eec944ed143LL),
      // C4[3], coeff of eps^23, polynomial in n of order 6
      -reale(81630,0xcf55ff9c68c00LL),reale(60811,0x59dd5ef6a6e00LL),
      -reale(57592,0x6457f059a8800LL),reale(30387,0x2572e53b9c200LL),
      -reale(30167,0xe11b4690d8400LL),reale(9044,0xd72699d03d600LL),
      reale(2392,0x21f43a8f7f830LL),reale(990092609,0x9eb428d5a933LL),
      // C4[3], coeff of eps^22, polynomial in n of order 7
      -reale(3070961,0xf14af9164000LL),reale(2767073,0x4d2d51bbc4000LL),
      -reale(2322170,0xf623e90f3c000LL),reale(1476552,0x4ed8bf53f8000LL),
      -reale(1490469,0x7e13eaba44000LL),reale(616004,0x8b84c9ea6c000LL),
      -reale(517487,0xf3178ed39c000LL),reale(279040,0x23dc4dd774ec0LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^21, polynomial in n of order 8
      -reale(3998482,0x374a7520d6800LL),reale(4351696,0x89a9dbf785900LL),
      -reale(3077852,0x4b8dc9fbd6e00LL),reale(2436308,0x9b47462d3fb00LL),
      -reale(2230379,0xda399323b400LL),reale(1147885,0x7a5199072bd00LL),
      -reale(1196012,0x91bb473d37a00LL),reale(325643,0x5e75ef9e35f00LL),
      reale(87110,0x728c765d95698LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^20, polynomial in n of order 9
      -reale(5536106,0x41a6dc97e5400LL),reale(6819318,0x7020ae33aa000LL),
      -reale(3996497,0x7d04a5d65ec00LL),reale(4026336,0x4a526eb153800LL),
      -reale(3081046,0x922df73cac400LL),reale(2027203,0x8c3cc70035000LL),
      -reale(2046086,0x4cc9bc51b5c00LL),reale(787253,0x8fa9057e6800LL),
      -reale(725367,0x21dd9ffc63400LL),reale(368582,0x69a43eb914890LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^19, polynomial in n of order 10
      -reale(8942538,0x3b8622ae62a00LL),reale(10481872,0x1e7c948175300LL),
      -reale(5381394,0x830498d800800LL),reale(6645195,0x535f47efddd00LL),
      -reale(4043713,0x9ba9cf138e600LL),reale(3563786,0x6253b3df24700LL),
      -reale(3045580,0xe2f1f7a110400LL),reale(1548984,0x4828fbf665100LL),
      -reale(1694435,0x63dcfc138a200LL),reale(406057,0xe76a74dc3bb00LL),
      reale(110280,0xa64ca1bbeb438LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^18, polynomial in n of order 11
      -reale(18204995,0x3f490d6ed8000LL),reale(15367333,0xa666c37198000LL),
      -reale(8424707,0xb9613a5da8000LL),reale(10765521,3190860555LL<<17),
      -reale(5300295,0xd300940f58000LL),reale(6273886,0xba1b2aa228000LL),
      -reale(4137511,0x6a32b5bc28000LL),reale(2951915,0x3ffeb65fb0000LL),
      -reale(2898950,0x38c8743c58000LL),reale(1027617,0x2c3889c5b8000LL),
      -reale(1062542,0x7c8a4a4828000LL),reale(500325,0x147f19cd83980LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^17, polynomial in n of order 12
      -reale(46659673,0x7940546261000LL),reale(20576887,0xb72d09f420c00LL),
      -reale(17371112,0xc460beb873800LL),reale(16552256,0x8d133b2d84400LL),
      -reale(7883306,0x3c181b1016000LL),reale(10867815,0x95ba8c80bfc00LL),
      -reale(5343012,0x31a34980f8800LL),reale(5640245,0x12558783a3400LL),
      -reale(4241979,0x47a64b12cb000LL),reale(2204426,0xf7d60f21fec00LL),
      -reale(2506924,0x6e46ed413d800LL),reale(503732,0xa322eb69a2400LL),
      reale(139663,0x777cb98300b20LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^16, polynomial in n of order 13
      -reale(156865464,0x9b4a437ced000LL),reale(26751997,0x84cabd1d8c000LL),
      -reale(47510066,0xf418e3e50b000LL),reale(22667291,0xeea5410a3a000LL),
      -reale(16175537,0xc4ceea20b9000LL),reale(17818506,0xfb6c54d608000LL),
      -reale(7402653,0x2459922697000LL),reale(10650742,0xeb52d29456000LL),
      -reale(5558253,0xfdda6aad45000LL),reale(4690304,0xc3737ed884000LL),
      -reale(4248624,0xb4bb4dab63000LL),reale(1382140,0xc755b095f2000LL),
      -reale(1646389,0x4c787b5791000LL),reale(701746,0xdc0286e009640LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^15, polynomial in n of order 14
      reale(158569992,0x763cf17d39800LL),reale(242045827,0xf358b9d531400LL),
      -reale(171801710,0xfbdaa54751000LL),reale(26564510,0xe59a1e6b54c00LL),
      -reale(47715397,0x8fdbdb93bb800LL),reale(25503418,0x124aa89300400LL),
      -reale(14593564,0x65519680b6000LL),reale(19028249,0x27fd86c303c00LL),
      -reale(7127523,0x40a42052f0800LL),reale(9926805,0x1876eddc2f400LL),
      -reale(5956098,0xfb7e2f3f1b000LL),reale(3422018,0xde3cf0f552c00LL),
      -reale(3909386,0x4ce6da2de5800LL),reale(606166,0xec68c0e73e400LL),
      reale(172919,0x9ad62b665b520LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^14, polynomial in n of order 15
      reale(234628808,0x48818da828000LL),-reale(452308383,0x26baa88038000LL),
      reale(184630907,0xde7b734758000LL),reale(240946965,0x4db221ae90000LL),
      -reale(189474421,0xed4c1e36d8000LL),reale(27214973,0x55324802d8000LL),
      -reale(46882338,0xe5fcdfdca8000LL),reale(29262846,2319362995LL<<17),
      -reale(12682237,0x3cee53d458000LL),reale(19904432,0x70537f02e8000LL),
      -reale(7274198,0xbf917ba828000LL),reale(8480909,0x438c3da230000LL),
      -reale(6415713,0xc95c9b8258000LL),reale(1960896,0x685dc04df8000LL),
      -reale(2745254,0xf883406d28000LL),reale(1023946,0x4eef421f04580LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^13, polynomial in n of order 16
      -reale(2272755,0x57fd708a77000LL),-reale(26091168,0x1366cec7d9d00LL),
      reale(231976719,0xafe6927fcde00LL),-reale(464894868,0x24c5c39795700LL),
      reale(215184123,0xaf8273d716c00LL),reale(236438336,0xab29f0bfd4f00LL),
      -reale(210344218,0x367ffa8b78600LL),reale(29454299,0x2f129bee9500LL),
      -reale(44460297,0xf9cfdfb8bb800LL),reale(34058265,0xda8305b9abb00LL),
      -reale(10677799,0x93543d448ea00LL),reale(19950418,0xbb16c712a0100LL),
      -reale(8097327,0xc3857f1ecdc00LL),reale(6164437,0x8a1d8a85ca700LL),
      -reale(6487914,0xa92c56ec54e00LL),reale(653539,0x4a58f163aed00LL),
      reale(193289,0xc4fa7fb371708LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^12, polynomial in n of order 17
      -reale(136365,0x73a1fcfe6ac00LL),-reale(450638,0xd074750f34000LL),
      -reale(2128024,0x54e7feac4d400LL),-reale(24952088,0x92a9c1fc91800LL),
      reale(228113259,0x85d44607e4400LL),-reale(477191195,0x7e69e50f07000LL),
      reale(251096618,0x1896eb4cd1c00LL),reale(226763725,0xac7cda7d93800LL),
      -reale(234776156,0x14cc4b0edcc00LL),reale(34557325,0x4230b4bd66000LL),
      -reale(39741101,0x3a85821c7f400LL),reale(39764072,0x42dd69fc98800LL),
      -reale(9161206,0x9c1a792d6dc00LL),reale(18380268,0xf302f56753000LL),
      -reale(9708385,0x581708d300400LL),reale(3148914,0x8380fab1bd800LL),
      -reale(5050904,0x8a565e3e8ec00LL),reale(1566765,0x6fd98617e9df0LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^11, polynomial in n of order 18
      -reale(18810,0x4977f6cdda600LL),-reale(44617,0xf507aa2256700LL),
      -reale(121680,0x26c8d0378b000LL),-reale(408670,0xadcc6d8f87900LL),
      -reale(1967116,0xd731d207dba00LL),-reale(23614778,0x5c1a1fadbeb00LL),
      reale(222693980,0x695506ba87c00LL),-reale(488598159,0xe2ab67bc47d00LL),
      reale(293333811,0x10f016a3f3200LL),reale(209273530,0x4db1c2b811100LL),
      -reale(262769616,0x9b49f60945800LL),reale(44647130,0x3acb33bfff00LL),
      -reale(31983858,0x227f1389ce200LL),reale(45626356,0x9e16c6ccb8d00LL),
      -reale(9276161,0xf8fb16a652c00LL),reale(14205372,0x289c377eefb00LL),
      -reale(11490116,0xc948e407f600LL),reale(414830,0x163387d5d8900LL),
      reale(117690,0xc756ec17c4aa8LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^10, polynomial in n of order 19
      -reale(3667,0x8ba48fb7ec000LL),-reale(7355,0xde5d961edc000LL),
      -reale(15963,0x138d280434000LL),-reale(38393,53315683LL<<17),
      -reale(106358,0x1cca460dcc000LL),-reale(363723,0x77fed5aee4000LL),
      -reale(1788619,0xb46088e414000LL),-reale(22045766,0x7d53064fc8000LL),
      reale(215267089,0x7c4e47994000LL),-reale(498143540,0xc077eb386c000LL),
      reale(342855614,0x4b25e0bbcc000LL),reale(179961617,0x7ca6ea4dd0000LL),
      -reale(293329289,0xb4e43f9ccc000LL),reale(63137066,0xbcee02f98c000LL),
      -reale(20920174,0xdceb909f94000LL),reale(49479848,0x7088e98168000LL),
      -reale(12768344,0x1ee1d8cbec000LL),reale(6948560,0xd8f6969c04000LL),
      -reale(10643749,0x466c677134000LL),reale(2529930,0x161dcdf222440LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^9, polynomial in n of order 20
      -real(0x354d49acec3dd800LL),-real(0x606a7d34c50a0200LL),
      -reale(2939,0xdc47a7c209c00LL),-reale(5971,0x671f2d9dad600LL),
      -reale(13140,0xcdf9f327fe000LL),-reale(32101,0x6baea5bb9ea00LL),
      -reale(90511,0x408ba9a232400LL),-reale(315893,0xc97e5e852be00LL),
      -reale(1591343,0xfce30d8d1e800LL),-reale(20207205,0x8b4272e60d200LL),
      reale(205238828,0x21c1cf60c5400LL),-reale(504251582,0xb2b181bcfa600LL),
      reale(400330413,0xa384192d01000LL),reale(132810886,0x4094526254600LL),
      -reale(323039224,0xd5680dd0e3400LL),reale(95085342,0xbfbbc74d27200LL),
      -reale(8279837,0x6ce790195f800LL),reale(46514941,0x8e0e73ffc5e00LL),
      -reale(20732718,0x38ef4b2eebc00LL),-reale(922541,0xf2a1d94487600LL),
      -reale(491669,0x5bd07d195db30LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^8, polynomial in n of order 21
      -real(0xd828cefda55a800LL),-real(0x16c6eac98e7b6000LL),
      -real(0x27e1e798049c9800LL),-real(0x490330552dbbf000LL),
      -reale(2255,0x88ea2b8740800LL),-reale(4647,0x88c66c31f8000LL),
      -reale(10390,0xd13f35560f800LL),-reale(25836,0xfcd55e2db1000LL),
      -reale(74324,0xc0bfff0e86800LL),-reale(265480,0xf5ce67923a000LL),
      -reale(1374647,0xa0b10ca8f5800LL),-reale(18058373,0x723761b2e3000LL),
      reale(191831943,0xc85920c253800LL),-reale(504361484,0x6e935002fc000LL),
      reale(465423127,0xbaa71ebb04800LL),reale(59036306,0xf120275a2b000LL),
      -reale(342905949,0x5a93131732800LL),reale(146354899,0x9f9c2b8142000LL),
      -reale(1641748,0x1e8ba62ca1800LL),reale(28969072,0x51c8dabef9000LL),
      -reale(27136540,0x3d9359d98800LL),reale(4249105,0xd55e5a0325120LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^7, polynomial in n of order 22
      -real(0x38123cee860f400LL),-real(0x59d375c04e8be00LL),
      -real(0x942bf86bd4c1800LL),-real(0xfcbda8858afb200LL),
      -real(0x1c02af2dc3443c00LL),-real(0x33fc822f8d2b6600LL),
      -real(0x65e35fc07de4e000LL),-reale(3414,0xc7eb297eb5a00LL),
      -reale(7775,0x1c0e884298400LL),-reale(19731,0x6a31912ef0e00LL),
      -reale(58089,0x9471e600da800LL),-reale(213111,0x15a6331c60200LL),
      -reale(1139019,0x77ee6ce2ccc00LL),-reale(15560104,0x33d66a0afb600LL),
      reale(174045800,0x2f0a20e9d9000LL),-reale(494300177,0xd9e4761bbaa00LL),
      reale(535087920,0xe9f8f195ec00LL),-reale(53102016,0x93f6bbbe95e00LL),
      -reale(331738553,0x77bff637f3800LL),reale(216985631,0x987f3afb7ae00LL),
      -reale(21074121,0x8043eaffd5c00LL),-reale(4185955,0xa3ff769180600LL),
      -reale(4713710,0xd2e19a34f30b0LL),reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^6, polynomial in n of order 23
      -real(0xe0ca252d14c000LL),-real(0x15a70af15f24000LL),
      -real(0x222b3f817554000LL),-real(0x375f97b48cd8000LL),
      -real(0x5c7b9631f8ac000LL),-real(0x9fe2527c7fcc000LL),
      -real(0x11face3d5ef34000LL),-real(0x21e77d8dabde0000LL),
      -real(0x439dcbf7fdccc000LL),-reale(2310,0x1731d0ccf4000LL),
      -reale(5373,0x35ee2c1554000LL),-reale(13965,0xf39edc32e8000LL),
      -reale(42247,0xa0aa0b1cac000LL),-reale(159930,0xa2319a759c000LL),
      -reale(887131,0xc123fa86b4000LL),-reale(12685735,0x6243721af0000LL),
      reale(150650948,0x968da6a8b4000LL),-reale(467294064,0x1610ada8c4000LL),
      reale(599544322,0x5feb9b1dac000LL),-reale(214883240,0x150075a4f8000LL),
      -reale(244806233,0x53bd4b2bac000LL),reale(272520146,0x88b0e96a94000LL),
      -reale(87760725,0x27ae1fc734000LL),reale(5827860,0xa7a2901c3a740LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^5, polynomial in n of order 24
      -real(0x32b69e04189800LL),-real(0x4bd39320660300LL),
      -real(0x73a508e7ef1600LL),-real(0xb44a7ec206b900LL),
      -real(0x1200d9d52c6d400LL),-real(0x1d916a5ad4bcf00LL),
      -real(0x321a3f994641200LL),-real(0x57fce6d660f8500LL),
      -real(0xa10c564a22b1000LL),-real(0x1356fa3ebba41b00LL),
      -real(0x275fd13435900e00LL),-real(0x5604e2d76283d100LL),
      -reale(3283,0xdf8f52c874c00LL),-reale(8783,0x8ddc09700e700LL),
      -reale(27451,0x143e179f50a00LL),-reale(107903,0xe48c7d6f59d00LL),
      -reale(625732,0xe2abef41d8800LL),-reale(9446536,0xacc19c0743300LL),
      reale(120325828,0x5507fb0eafa00LL),-reale(412649247,0xc3fe82376e900LL),
      reale(633089704,0xd19d26ed03c00LL),-reale(418090362,0x84d33548fff00LL),
      -reale(13712613,0x4e3334f720200LL),reale(163180098,0x55c7c31664b00LL),
      -reale(61921019,0x751f3b2bed108LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[3], coeff of eps^4, polynomial in n of order 25
      -real(0x30fab48eb2c00LL),-real(0x4779db0cde000LL),
      -real(0x6a1a5308c1400LL),-real(0xa07c7893bf800LL),
      -real(0xf7d15b087bc00LL),-real(0x1878e181999000LL),
      -real(0x27ab652bf7a400LL),-real(0x422ed0b6682800LL),
      -real(0x721448fff54c00LL),-real(0xcc1e5699294000LL),
      -real(0x17d5829db9a3400LL),-real(0x2ed74923dde5800LL),
      -real(0x61c84aba5ffdc00LL),-real(0xdbaa1b53c88f000LL),
      -real(0x21cc8beefe3fc400LL),-real(0x5da8efb832aa8800LL),
      -reale(4876,0x5d83861736c00LL),-reale(20082,0x8bb9af0c4a000LL),
      -reale(123005,0x97d1502b45400LL),-reale(1983151,0x65e045fd8b800LL),
      reale(27425226,0x9c6669ee40400LL),-reale(105081920,0xe8c662ae85000LL),
      reale(191976586,0x46cce583c1c00LL),-reale(186491540,0xf45203874e800LL),
      reale(93245770,0x7a2901c3a7400LL),-reale(18940547,0x20d0545bbdf90LL),
      reale(9570895220LL,0xb53783566b8edLL),
      // C4[3], coeff of eps^3, polynomial in n of order 26
      -real(0x10330cb256200LL),-real(0x172cb16211100LL),
      -real(0x21a8187537800LL),-real(0x31b06260f1f00LL),
      -real(0x4ab014ab28e00LL),-real(0x7280309c9cd00LL),
      -real(0xb366eef7be400LL),-real(0x11ff8a58b05b00LL),
      -real(0x1dae666558ba00LL),-real(0x327547ac4a0900LL),
      -real(0x58c9207d125000LL),-real(0xa2826b77361700LL),
      -real(0x137557a5841e600LL),-real(0x275355b4b1bc500LL),
      -real(0x54b37d85300bc00LL),-real(0xc517d06239a5300LL),
      -real(0x1f8f2f623d981200LL),-real(0x5b85a3034c390100LL),
      -reale(5020,0xa2ee6bc312800LL),-reale(21965,0x48d3177570f00LL),
      -reale(144343,0x4c469a2853e00LL),-reale(2526007,0xb6d389c1bbd00LL),
      reale(38395317,0x415c2de726c00LL),-reale(163180098,0x55c7c31664b00LL),
      reale(326360196,0xab8f862cc9600LL),-reale(303048754,0xd0545bbdf900LL),
      reale(104173009,0x3479cff894d98LL),
      reale(28712685662LL,0x1fa68a0342ac7LL),
      // C4[4], coeff of eps^29, polynomial in n of order 0
      4519424,real(0x13ed3512585LL),
      // C4[4], coeff of eps^28, polynomial in n of order 1
      real(322327509504LL),real(86419033792LL),real(0x12e7203d54087bdLL),
      // C4[4], coeff of eps^27, polynomial in n of order 2
      real(0xdf868e997000LL),-real(0xc54488fde800LL),real(0x67996a8dfb80LL),
      reale(6219,0x86ed0fee71e5LL),
      // C4[4], coeff of eps^26, polynomial in n of order 3
      real(0x1e30d5f17398800LL),-real(0x20335f44c005000LL),
      real(0x8656a9da59d800LL),real(0x246f3281df3200LL),
      reale(1871928,0xea4bbbb5bea41LL),
      // C4[4], coeff of eps^25, polynomial in n of order 4
      real(0x640278dc982000LL),-real(0x64de2b5e388800LL),
      real(0x266cf1cb211000LL),-real(0x24af02897bd800LL),
      real(0x125236c4932c80LL),reale(225070,0xa1cd0c0f186c5LL),
      // C4[4], coeff of eps^24, polynomial in n of order 5
      real(0x183393315f62f400LL),-real(0x147c8a635ba4f000LL),
      real(0xaadb07a361e2c00LL),-real(0xbd0a07cdca37800LL),
      real(0x2c490db64a86400LL),real(0xc3000bbe3e2580LL),
      reale(8327613,0x62a2be2e87a79LL),
      // C4[4], coeff of eps^23, polynomial in n of order 6
      reale(7399,0xe4703b1ceb000LL),-reale(4925,0x718bf750ef800LL),
      reale(3656,0xc01290e152000LL),-reale(3594,0x9ae0aefbbc800LL),
      real(0x5080258211e79000LL),-real(0x5458466826cf9800LL),
      real(0x27a09e95cf36b080LL),reale(97921247,0xc3bd6c206251LL),
      // C4[4], coeff of eps^22, polynomial in n of order 7
      reale(4319137,0xe5044c1364800LL),-reale(2259378,0xc043aee633000LL),
      reale(2431286,0xcceb783bf5800LL),-reale(1865690,0x884902c9a2000LL),
      reale(996566,0x94ae3b7946800LL),-reale(1135368,0x2cb1c30811000LL),
      reale(231629,0x92b25177d7800LL),reale(64961,0x89605803fda00LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^21, polynomial in n of order 8
      reale(6174501,0x53f34a829c000LL),-reale(2885765,0xddf01a0f35800LL),
      reale(4089976,0x588848e445000LL),-reale(2309244,0x73683320c8800LL),
      reale(1950621,0xac1b944ace000LL),-reale(1810054,0xa24c07eb4b800LL),
      reale(609590,0x74daa18497000LL),-reale(712107,0x16cff78e5e800LL),
      reale(310317,0x16957f6a36b80LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^20, polynomial in n of order 9
      reale(7763095,0xd98a0c3214600LL),-reale(4551997,0xf65d38a54d000LL),
      reale(6348004,0x7dcc619ba1a00LL),-reale(2777846,0x11091dc381c00LL),
      reale(3645151,0x5af876afd6e00LL),-reale(2403756,0x12692c3266800LL),
      reale(1377366,0xde24866584200LL),-reale(1585712,0xf2192bea6b400LL),
      reale(268682,0xb0f056b079600LL),reale(77255,0xca5a822ebf740LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^19, polynomial in n of order 10
      reale(8073134,0x8bff962f2e000LL),-reale(9331256,0xe8e10405e1000LL),
      reale(8608510,0x42ad0321d8000LL),-reale(3959617,0x4c778c1e2f000LL),
      reale(6283090,0x55033b3d82000LL),-reale(2832307,0xbbdb17809d000LL),
      reale(2955095,0x929c8347ec000LL),-reale(2459067,0xd43d49c36b000LL),
      reale(787004,0x9cc4866d6000LL),-reale(1039103,0x6b1983acd9000LL),
      reale(412222,0xf695367aa1b00LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^18, polynomial in n of order 11
      reale(8586281,0xffd2991fd000LL),-reale(20926106,0xdd733d721a000LL),
      reale(9282973,0x193483c94f000LL),-reale(8121077,0x9b55004148000LL),
      reale(9430655,0x90c0e29221000LL),-reale(3512067,0x80c2ac76000LL),
      reale(5840995,0x1886eb4173000LL),-reale(3061324,0xab1a78b4a4000LL),
      reale(2049544,0x4067911445000LL),-reale(2292525,0x617c054ad2000LL),
      reale(297833,0x966e637f97000LL),reale(88539,0x9a2e50b8c6400LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^17, polynomial in n of order 12
      reale(32196457,0xd679f8ae1c000LL),-reale(40594018,0x37167c5ef5000LL),
      reale(8052650,0x2eda271162000LL),-reale(20325613,0xcd34eeff17000LL),
      reale(11030346,0x5827875768000LL),-reale(6662972,0x9685f0fc59000LL),
      reale(10015916,0xfa65faac6e000LL),-reale(3377057,0x1ef6021e7b000LL),
      reale(4892320,0x94cb79bcb4000LL),-reale(3369439,0x93437f1d3d000LL),
      reale(1068721,0xdee482d47a000LL),-reale(1596884,0xcb3e26805f000LL),
      reale(562334,0xcf5270735f500LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^16, polynomial in n of order 13
      reale(239019678,0x7928c61a8b800LL),-reale(41200119,0x147c0b11e000LL),
      reale(27063572,0xac3757be98800LL),-reale(45155983,0xc412cf1f79000LL),
      reale(8354845,0xf8b6ea7445800LL),-reale(18750027,0x4e7377c014000LL),
      reale(13292220,0xfed958edd2800LL),-reale(5165101,0x26aa3105af000LL),
      reale(10025000,0x43fec217f800LL),-reale(3715677,0xed5a4430a000LL),
      reale(3405288,0xc16fe1018c800LL),-reale(3440521,0x6cb0e4f2e5000LL),
      reale(291108,0x30be23439800LL),reale(90314,0xe93f4121c6900LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^15, polynomial in n of order 14
      -reale(301344600,0x1f7a69f35a000LL),-reale(137666269,0x81776c9d9b000LL),
      reale(257500426,0xa27a71193c000LL),-reale(52745704,0xa8e59f44d000LL),
      reale(20527629,0x3707e00852000LL),-reale(49389175,0x1679a6a55f000LL),
      reale(10057417,0xa546ce8428000LL),-reale(15960633,0x79a78f6a91000LL),
      reale(15828795,0x3b7a7e96fe000LL),-reale(4041479,0x5385608da3000LL),
      reale(9015452,0x8a056dcb14000LL),-reale(4531739,0xb18fd7c855000LL),
      reale(1608583,0x5c81da4aaa000LL),-reale(2620079,0xb9c03a2467000LL),
      reale(790676,0xf12036cb88d00LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^14, polynomial in n of order 15
      -reale(152316078,0x9ee9710b1f000LL),reale(396132268,0xf6300698d2000LL),
      -reale(331944543,0x2a26efc8bd000LL),-reale(111967823,0x409ccb544c000LL),
      reale(276102802,0x8592b62d25000LL),-reale(69409637,0x2e4659b6a000LL),
      reale(12806364,0xaa4a38387000LL),-reale(52382533,0xaa3aad6588000LL),
      reale(13858261,0x7d9fda6f69000LL),-reale(11925525,0x17f68feba6000LL),
      reale(17994828,0x2633a57dcb000LL),-reale(3926621,0x9c334da6c4000LL),
      reale(6610729,0xa84ec063ad000LL),-reale(5341800,0xcfe0c57fe2000LL),
      reale(171304,0xc92dc0ce0f000LL),reale(53498,0x8a12fdd94c400LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^13, polynomial in n of order 16
      reale(945329,0x3e694a5630000LL),reale(13046260,0xd11553dc81000LL),
      -reale(145063327,0x6c5bbd04f6000LL),reale(395288944,0x9758cc3483000LL),
      -reale(364989750,0x4da45c465c000LL),-reale(77659847,0x7f601a5fdb000LL),
      reale(293261136,0xdb46a6c9be000LL),-reale(92956699,0x68d702f4d9000LL),
      reale(4748491,0xd717292318000LL),-reale(52641236,0xde7217eeb7000LL),
      reale(20401071,0xa831b35d72000LL),-reale(7165143,0xe2daef21b5000LL),
      reale(18530179,0x70f1fa908c000LL),-reale(5449998,0x995f61f213000LL),
      reale(2985284,0xf423c13426000LL),-reale(4674955,0x4c99b17411000LL),
      reale(1148405,0xaa811667d8300LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^12, polynomial in n of order 17
      reale(39064,0xc457745427a00LL),reale(149707,0xe179ab818a000LL),
      reale(834482,0xb3de3faf4c600LL),reale(11844090,0x43801d34c0c00LL),
      -reale(136492367,0x606ac4f4b6e00LL),reale(391413380,0x8b1b355567800LL),
      -reale(399991879,0xf56c51d232200LL),-reale(32313943,0x670cb1cd91c00LL),
      reale(306137820,0x47c0d4df8aa00LL),-reale(125355715,0x12c37db13b000LL),
      -reale(1549012,0x61de67b1d0a00LL),-reale(48002827,0x1ef791fca4400LL),
      reale(29707099,0x80264b6e6c200LL),-reale(3304868,0xd90dacdedd800LL),
      reale(15595740,0x1c41b85df0e00LL),-reale(8339676,0x731c5b6cf6c00LL),
      -reale(264319,0x3253133a92600LL),-reale(128183,0x1fd72f4c70540LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^11, polynomial in n of order 18
      reale(3796,0xb8b80a685d000LL),reale(10243,0xe5415b1644800LL),
      reale(32134,0x75fe9c2f28000LL),reale(125896,0x13cc0b67cb800LL),
      reale(720062,0x2eb5ef2cf3000LL),reale(10542664,0x8e7784ebe2800LL),
      -reale(126401502,0xa942d02d22000LL),reale(383396973,0xa914c081a9800LL),
      -reale(435856143,0x9e18e4ddf7000LL),reale(26921352,0xa17bcee040800LL),
      reale(309790567,0x432113bb94000LL),-reale(168177156,0xf5a6b5d938800LL),
      -reale(1732899,0x7848d10f61000LL),-reale(36033193,0x6ff05a93a1800LL),
      reale(39850986,0x4a7ce5d24a000LL),-reale(3520516,0x12d4d9afda800LL),
      reale(7904559,0x47211641b5000LL),-reale(9293198,0x11e52b76c3800LL),
      reale(1712350,0xd1c47193d5a80LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^10, polynomial in n of order 19
      real(0x20b0c3dbe662b800LL),real(0x49a4ee6b654d5000LL),
      reale(2895,0xbb9a481b3e800LL),reale(7963,0xd6290c9168000LL),
      reale(25525,0x742091bd91800LL),reale(102493,0xec03f49fb000LL),
      reale(603292,0x6fe940faa4800LL),reale(9144553,0x3f081030e000LL),
      -reale(114581171,0x9502f66408800LL),reale(369767644,0x159b783921000LL),
      -reale(470438620,0x42537ac0f5800LL),reale(102998223,0x33db2118b4000LL),
      reale(295924658,0xfd504b0d5d800LL),-reale(220875824,0xd68590c9b9000LL),
      reale(12088406,0x3b87c77470800LL),-reale(15966308,0xf7cc70b9a6000LL),
      reale(44660638,0xbb68d3ddc3800LL),-reale(11155854,0x316b572a93000LL),
      -reale(1400757,0x91d7719929800LL),-reale(909990,0x5b4dcbdcd9200LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^9, polynomial in n of order 20
      real(0x55091490e3fe000LL),real(0xab3101736f26800LL),
      real(0x16d77945c4e3b000LL),real(0x345d2a91137d7800LL),
      reale(2099,0xc55d2c398000LL),reale(5898,0x424192198800LL),
      reale(19366,0xa6f5f449f5000LL),reale(79943,0x847cdfac49800LL),
      reale(486014,0x6a1dc16732000LL),reale(7659629,0x94cc8fca800LL),
      -reale(100839015,0x651046eed1000LL),reale(348607247,0x22ddc22bfb800LL),
      -reale(499815073,0x4df2756234000LL),reale(197958555,0x77a0b2f8bc800LL),
      reale(251323198,0x2663cfb2e9000LL),-reale(276534810,0xe292670a12800LL),
      reale(51555588,0x6a67a23666000LL),reale(5587968,0x5e92831b6e800LL),
      reale(32523682,0xed2ae23e23000LL),-reale(21111776,0x46401336e0800LL),
      reale(2489921,0xe3c1e337a6d80LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^8, polynomial in n of order 21
      real(0xeb8379f6b27c00LL),real(0x1b6c4de1f1d7000LL),
      real(0x355a1dadc956400LL),real(0x6d308de46411800LL),
      real(0xed54313f63d4c00LL),real(0x22ae87428a2ac000LL),
      real(0x58ce5dd980bc3400LL),reale(4090,0xd3c824bc46800LL),
      reale(13806,0x44b4a8a441c00LL),reale(58809,0x7ab991df81000LL),
      reale(370898,0xe410033e70400LL),reale(6109620,0x6402b9f6fb800LL),
      -reale(85053139,0x4bf446ca91400LL),reale(317515928,0x1b63894556000LL),
      -reale(517123103,0xa7a388b5a2c00LL),reale(310296682,0xe98bc80130800LL),
      reale(156996715,0xaa3cf3c05bc00LL),-reale(312601560,0xdd28200ed5000LL),
      reale(125126811,0xf01e02788a400LL),reale(4091818,0xb5091207e5800LL),
      -reale(866059,0xc9a79cf1f7400LL),-reale(4943757,0xf4721fe538b80LL),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^7, polynomial in n of order 22
      real(0x2814d49c0c5000LL),real(0x468b0d3a3db800LL),
      real(0x80724d98876000LL),real(0xf31dbc49b20800LL),
      real(0x1e12cb4a6a67000LL),real(0x3eb5a58b5455800LL),
      real(0x8b1eef20fbf8000LL),real(0x14cb29a266eda800LL),
      real(0x36974c82ca289000LL),reale(2585,0xefae20720f800LL),
      reale(9007,0x1d6baf437a000LL),reale(39779,0x24ec74fd54800LL),
      reale(261696,0x442f64f42b000LL),reale(4534975,0xa5b17f809800LL),
      -reale(67279179,0x4d9bf05604000LL),reale(273758534,0xd27122c18e800LL),
      -reale(510920394,0x40d515b3000LL),reale(428723861,0x53ee2b6143800LL),
      -reale(7330129,0x37be948582000LL),-reale(275708250,0xae16364977800LL),
      reale(204390109,0xe684af0fef000LL),-reale(52540960,0x7463315742800LL),
      reale(2056891,0xfeee14beab380LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^6, polynomial in n of order 23
      real(0x628e4f4bb7800LL),real(0xa60e374943000LL),real(0x11fae77940e800LL),
      real(0x2022ddc061a000LL),real(0x3b7f2e2d7a5800LL),
      real(0x72aa26ca9f1000LL),real(0xe77392a11fc800LL),
      real(0x1ed1e51d0348000LL),real(0x460248a5fa93800LL),
      real(0xabd9e84dc89f000LL),real(0x1d078c2cd5cea800LL),
      real(0x58c9fda5cf076000LL),reale(5134,0xa77137081800LL),
      reale(23653,0x63d76094d000LL),reale(163469,0x772f4630d8800LL),
      reale(3004667,0x8d384291a4000LL),-reale(47956830,0xd53f134a90800LL),
      reale(214953528,0xfe0a5a4ffb000LL),-reale(463620631,0xbff95a7639800LL),
      reale(519033396,0x411553aad2000LL),-reale(237300381,0xd565fafaa2800LL),
      -reale(84296486,0x10fabff57000LL),reale(142611178,0x607af3a3b4800LL),
      -reale(46622885,0x3d1480e1d3a00LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^5, polynomial in n of order 24
      real(0xc0b5b2cac000LL),real(0x139ac5d2ed800LL),real(0x20abe97223000LL),
      real(0x37e2f8cba0800LL),real(0x6269b1d1ba000LL),real(0xb3074a8a43800LL),
      real(0x151de1e3911000LL),real(0x298e5ccaa76800LL),
      real(0x55d208375c8000LL),real(0xbb7ea958fd9800LL),
      real(0x1b5e1854857f000LL),real(0x4547c4b8360c800LL),
      real(0xc1cdc899e5d6000LL),real(0x2682d6f5e00af800LL),
      reale(2326,0xf44888e46d000LL),reale(11275,0x7d4afe8b62800LL),
      reale(82638,0x859516eee4000LL),reale(1628359,0xc1653179c5800LL),
      -reale(28286265,0xc31f9b1d25000LL),reale(141205400,0x2bb5164778800LL),
      -reale(353352393,0x632221a20e000LL),reale(504046796,0x730ece181b800LL),
      -reale(416863444,0x7c7b16f237000LL),reale(186491540,0xf45203874e800LL),
      -reale(34967163,0xedcf60a95eb80LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[4], coeff of eps^4, polynomial in n of order 25
      real(0xe07098dae00LL),real(0x16338b625000LL),real(0x23dda179f200LL),
      real(0x3b41a69cf400LL),real(0x645a89a6b600LL),real(0xaeabe0e09800LL),
      real(0x1397028dcfa00LL),real(0x246014e923c00LL),real(0x4633de275be00LL),
      real(0x8d95c8a56e000LL),real(0x12c670f9ba0200LL),
      real(0x2a433484738400LL),real(0x6608a70542c600LL),
      real(0x10c10ac322d2800LL),real(0x30ddb4b92590a00LL),
      real(0xa2e30513d28cc00LL),real(0x289386109855ce00LL),
      reale(3347,0x17499d2cb7000LL),reale(26358,0x5763b5c021200LL),
      reale(564821,0x99c65b39a1400LL),-reale(10825747,0x58af29d092a00LL),
      reale(60624185,0x23d4ea299b800LL),-reale(172778927,0xa61ece902e600LL),
      reale(279737311,0x6e7b054af5c00LL),-reale(233114426,0x3166846922200LL),
      reale(75762188,0x8341516ef7e40LL),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[5], coeff of eps^29, polynomial in n of order 0
      3108352,real(0x4338129a0b3LL),
      // C4[5], coeff of eps^28, polynomial in n of order 1
      -real(4961047LL<<17),real(304969986048LL),real(0x171a7cbcbc0a5e7LL),
      // C4[5], coeff of eps^27, polynomial in n of order 2
      -real(0xb7a8cf8589000LL),real(0x25cdf8a9f5800LL),real(0xaa8ee05df480LL),
      reale(53207,0x4825dfa147919LL),
      // C4[5], coeff of eps^26, polynomial in n of order 3
      -real(0x4519d2e6066000LL),real(0x17b1d503134000LL),
      -real(0x1b53dc2d3c2000LL),real(0xc104a529c3b00LL),
      reale(207992,0x1a086a30a3679LL),
      // C4[5], coeff of eps^25, polynomial in n of order 4
      -real(0xe48436400f9e000LL),real(0x825cbe3b5113800LL),
      -real(0x9657faac8f9f000LL),real(0x1ac735d19d16800LL),
      real(0x7b639e59c13780LL),reale(8527676,0x2b5901ca2b961LL),
      // C4[5], coeff of eps^24, polynomial in n of order 5
      -real(0x13b86e0d5c5dc000LL),real(0x135f9b0385fb0000LL),
      -real(0x10df1064c3304000LL),real(0x58b0ae17a818000LL),
      -real(0x70d05036b8ec000LL),real(0x2e5299a0b610e00LL),
      reale(10178194,0x2338af8e3405bLL),
      // C4[5], coeff of eps^23, polynomial in n of order 6
      -reale(126383,0x5f6b81564f000LL),reale(192332,0x2215a4d90d800LL),
      -reale(113392,0x893928fcaa000LL),reale(71665,0x3fb557978e800LL),
      -reale(81791,0xa6f9503f45000LL),reale(12036,0x1a6fad5adf800LL),
      reale(3561,0x9aef6f2cefa80LL),reale(3470764200LL,0xea81d86b4b937LL),
      // C4[5], coeff of eps^22, polynomial in n of order 7
      -reale(191647,0x188f775ada000LL),reale(308186,0x45ee8f2434000LL),
      -reale(124928,0xd21a49314e000LL),reale(153616,0xaed0e35eb8000LL),
      -reale(118466,0xc4b6a2a9a2000LL),reale(38029,0x77ad4b77bc000LL),
      -reale(53612,0x41f60b8316000LL),reale(20169,0xecfa5f7fa8900LL),
      reale(3470764200LL,0xea81d86b4b937LL),
      // C4[5], coeff of eps^21, polynomial in n of order 8
      -reale(5169843,0xc81db86efc000LL),reale(5341939,0xe957aa505800LL),
      -reale(2049228,0x2e9753666d000LL),reale(3734678,0xdcd2e44998800LL),
      -reale(1762099,0xebebc251fe000LL),reale(1337844,0xa441c7cbb800LL),
      -reale(1455577,0x7e18adc04f000LL),reale(163809,0xd9aab3cbce800LL),
      reale(50215,0x8f7a6f7ead780LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^20, polynomial in n of order 9
      -reale(11201228,0x9af12fea90000LL),reale(5330620,7096189457LL<<19),
      -reale(4084126,0xa473ecba70000LL),reale(5776338,0xc1238f4360000LL),
      -reale(1850318,0x7e36514750000LL),reale(3091001,2788978033LL<<18),
      -reale(1978996,0x9854b5b30000LL),reale(651396,0xde4e2e0920000LL),
      -reale(1009381,0x5e1878c010000LL),reale(341219,0x67868049b6800LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^19, polynomial in n of order 10
      -reale(19364139,0xf3aad6c27e000LL),reale(3661269,0x231a8ee911000LL),
      -reale(10171658,0x9bc1444518000LL),reale(6650152,0x1449aa44ff000LL),
      -reale(2982446,0xb2f133d6b2000LL),reale(5796709,0x225c7b8fcd000LL),
      -reale(2004712,0xb33d0f538c000LL),reale(2087887,0x2718a4e53b000LL),
      -reale(2041244,0xb9c4a8d7e6000LL),reale(150337,0x64e8ec0109000LL),
      reale(48205,0x4eea8f2f13300LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^18, polynomial in n of order 11
      -reale(17821498,0x43ce2fe394000LL),reale(8113989,0x34042cf6f8000LL),
      -reale(21055211,0x1d823792dc000LL),reale(4458324,0xaba1762760000LL),
      -reale(8384573,0x54084121e4000LL),reale(8079221,0xcbb99849c8000LL),
      -reale(2172398,0x503335ed2c000LL),reale(5129813,0x3b8a4c21b0000LL),
      -reale(2481567,0xadec795134000LL),reale(934125,9279934035LL<<15),
      -reale(1531704,0x9cc504aa7c000LL),reale(453383,0xd34e451346a00LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^17, polynomial in n of order 12
      reale(4095301,0x789aeb9e64000LL),reale(49542396,0x46ab457e8d000LL),
      -reale(24303219,0x1ccf0dd62000LL),reale(4679495,0x21a30e03df000LL),
      -reale(21666597,0xecbbb1868000LL),reale(6429258,0x6611bb6911000LL),
      -reale(5963806,0x7f45fe6c6e000LL),reale(9141324,0xab5773fc63000LL),
      -reale(2043796,0x5ca6f33334000LL),reale(3626747,0xd85dd12c15000LL),
      -reale(2919955,0xba0fdf867a000LL),reale(85758,0x333e03c667000LL),
      reale(28339,0x9119c9ad54d00LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^16, polynomial in n of order 13
      -reale(273240474,0x43c43c74c8000LL),reale(133674826,0x952bfc30e0000LL),
      reale(7048142,0x68e4684408000LL),reale(44883009,0xdb6a70b90000LL),
      -reale(32370151,0x153b9e91a8000LL),reale(2006331,0xa0ac245340000LL),
      -reale(20459012,0x9d1a27ed8000LL),reale(9634139,0x6e1e5ebef0000LL),
      -reale(3415127,0x8d101d0c88000LL),reale(9090639,8214448173LL<<17),
      -reale(2849328,0xea461fc3b8000LL),reale(1554483,7516134885LL<<16),
      -reale(2460922,0x6540542d68000LL),reale(615586,0x6f27f96118400LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^15, polynomial in n of order 14
      reale(385255297,0xc522d651da000LL),-reale(58599463,0x810289e63d000LL),
      -reale(271784816,0x96bdc01bbc000LL),reale(164665597,0xfc4f4e3665000LL),
      reale(6169937,0xa7ea1cfd2e000LL),reale(36278794,0xf1d4bf77a7000LL),
      -reale(41327996,0x5935502f28000LL),reale(1406713,0xae66a659c9000LL),
      -reale(16753028,0x6b0d0fac7e000LL),reale(13550589,0x7d5a3390b000LL),
      -reale(1765295,0x851b6e8694000LL),reale(7142364,0xca525091ad000LL),
      -reale(4183412,0x818c59892a000LL),-reale(96164,0xa4307ac011000LL),
      -reale(44020,0x281c2d0515b00LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^14, polynomial in n of order 15
      reale(85300002,0xc7e70a9f1c000LL),-reale(294351273,0xafb8edef98000LL),
      reale(403760509,0xda2cbc2e94000LL),-reale(107444454,0x9ae8f34870000LL),
      -reale(261509454,0x4bda846b4000LL),reale(200593259,0xcaf344c1b8000LL),
      -reale(1492598,0x1c0b3e713c000LL),reale(23203659,0x98196f9e60000LL),
      -reale(49434335,0xf8209c0184000LL),reale(4620325,0x4eb0e8bd08000LL),
      -reale(10475101,0x343acca80c000LL),reale(16597245,8542632147LL<<16),
      -reale(2356576,0x3bbee61554000LL),reale(3249396,0x1edbdd7e58000LL),
      -reale(4240477,0x930e83f9dc000LL),reale(851256,0x2b979a0197a00LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^13, polynomial in n of order 16
      -reale(334885,0xc6bdc7fcb0000LL),-reale(5563880,0xa3a405a9f1000LL),
      reale(77196254,0x955c2ca786000LL),-reale(280592470,0x60fd2cd013000LL),
      reale(419465490,0x135ebd637c000LL),-reale(164134806,0xd03e535795000LL),
      -reale(238238642,0xf95f61c30e000LL),reale(239782224,0x6d53e5d49000LL),
      -reale(20068072,0x4afa414658000LL),reale(6399560,0x53e56b4c47000LL),
      -reale(53380994,0xb54d3160a2000LL),reale(13179100,0x7f23319325000LL),
      -reale(3190623,0x71f1454c2c000LL),reale(15946535,0x7112262fa3000LL),
      -reale(5597132,0xd891768336000LL),-reale(517466,0x3872db407f000LL),
      -reale(280398,0x37b65ce5ca500LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^12, polynomial in n of order 17
      -reale(9362,0x69735ac9d0000LL),-reale(41698,3327447843LL<<20),
      -reale(274851,0x56e2bdf830000LL),-reale(4724425,0xa83b5c01a0000LL),
      reale(68370240,0x5baadc4870000LL),-reale(262946254,0xff686b9240000LL),
      reale(430395020,0x66a0aab610000LL),-reale(228360148,0x64a23696e0000LL),
      -reale(196492193,0xc6f6cbf150000LL),reale(277855749,243039325LL<<19),
      -reale(54565881,0x3f0390efb0000LL),-reale(10430670,3478671393LL<<17),
      -reale(48232829,0x9769bd8710000LL),reale(26504611,0xd8be140f40000LL),
      reale(733724,0x9fb250690000LL),reale(8992810,0x9e09f3a6a0000LL),
      -reale(7946224,0xca1f6288d0000LL),reale(1176502,0x79934ee544800LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^11, polynomial in n of order 18
      -real(0x274a66713f785000LL),-real(0x78cbe0a9df914800LL),
      -reale(6986,0x5cd0ed6f68000LL),-reale(31980,0xbaca6835fb800LL),
      -reale(217574,0x7dc41d384b000LL),-reale(3882916,0x2edd7dacd2800LL),
      reale(58859398,0xdc7c0f67f2000LL),-reale(240755855,0x78dc5ddf79800LL),
      reale(433769587,0x318800cb6f000LL),-reale(298315443,0xab75c9fd0800LL),
      -reale(129660149,0x66ef2473b4000LL),reale(305615878,0x94b6a51048800LL),
      -reale(109156237,0x593300db57000LL),-reale(18007247,0x43b21e10e800LL),
      -reale(29424146,0x61ad17715a000LL),reale(38156138,0xf0096c8a4a800LL),
      -reale(4683041,0xee399b1b9d000LL),-reale(1149725,0xbf46657f8c800LL),
      -reale(1106736,0x8c2ceac93e180LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^10, polynomial in n of order 19
      -real(0x3bd4906e474e000LL),-real(0x97941b80ce3c000LL),
      -real(0x1a66716bc5afa000LL),-real(0x532298a0bc3e0000LL),
      -reale(4939,0xda9250746000LL),-reale(23308,0x7863f72384000LL),
      -reale(164254,0x558c90eef2000LL),-reale(3056120,0xcef6e5fe8000LL),
      reale(48766418,0xafc6204b42000LL),-reale(213414260,0xdc9b1ebcc000LL),
      reale(425806905,0x15318e0496000LL),-reale(369415923,0x757d6c39f0000LL),
      -reale(31178847,0x2c748765b6000LL),reale(306118804,0x213b4942ec000LL),
      -reale(181898310,0x263b289662000LL),reale(568685,0x4686791808000LL),
      -reale(309548,0x34bb55302e000LL),reale(32975540,0x34fcc4d2a4000LL),
      -reale(16246779,0x8dca2dd5da000LL),reale(1477949,0xdae92a7065f00LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^9, polynomial in n of order 20
      -real(0x69d018a3b9e000LL),-real(0xed437c3919a800LL),
      -real(0x237e48279feb000LL),-real(0x5bea2151a0b3800LL),
      -real(0x10666acb6ec18000LL),-real(0x350c7e1643d3c800LL),
      -reale(3247,0xe2be74bf45000LL),-reale(15860,0x268da19a55800LL),
      -reale(116263,0x5e4790b892000LL),-reale(2266502,0x8314b6fb1e800LL),
      reale(38294967,0xecf46ee8e1000LL),-reale(180538484,0x555f9ed2b7800LL),
      reale(401643505,0x9c33fda5f4000LL),-reale(432258273,0xf8da98e440800LL),
      reale(101814780,0x5dd5e11f87000LL),reale(252370005,0x80f91f9d26800LL),
      -reale(252307179,0x99e21a8986000LL),reale(63455824,0x191a53ee5d800LL),
      reale(12621880,0x95e41abad000LL),reale(2033357,0xc3307b9c44800LL),
      -reale(4727243,0x20838a8bae80LL),reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^8, polynomial in n of order 21
      -real(0xc09a6adbf4000LL),-real(0x18cab6e3030000LL),
      -real(0x359d0ace62c000LL),-real(0x7ab7d9cc438000LL),
      -real(0x12c67ab580a4000LL),-real(856171152199LL<<18),
      -real(0x9233f1c13ddc000LL),-real(0x1e779de654b48000LL),
      -real(0x789f22a00b054000LL),-reale(9796,7021023797LL<<16),
      -reale(75089,0xae07706a8c000LL),-reale(1543001,0x638fcd4c58000LL),
      reale(27798321,0x1e96e700fc000LL),-reale(142306959,0xd3ad6eb8e0000LL),
      reale(355697955,0xce7f78ffc4000LL),-reale(469861249,0x5989105b68000LL),
      reale(259457720,0x1370b4ff4c000LL),reale(112194489,0x36d40ed990000LL),
      -reale(260872269,0xf8005192ec000LL),reale(151422395,0x58f7b5f388000LL),
      -reale(32332898,0xbdc6e34964000LL),reale(433029,0xe4d3ce78fba00LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^7, polynomial in n of order 22
      -real(0x1441fa2f35000LL),-real(0x272c726527800LL),
      -real(0x4ebdd7b856000LL),-real(0xa564301b74800LL),
      -real(0x16d6333bd37000LL),-real(0x3580dec1951800LL),
      -real(0x865ae53c178000LL),-real(0x16ec61d7f65e800LL),
      -real(0x455fa2e228b9000LL),-real(0xef77f4cbfa3b800LL),
      -real(0x3d9c6e708569a000LL),-reale(5230,0x8a511fbc88800LL),
      -reale(42196,0xcfdba8cebb000LL),-reale(920786,0xf57a80c4e5800LL),
      reale(17837247,0x2fc56aab44000LL),-reale(100064916,0x5e72032af2800LL),
      reale(283253574,0xc37962f3c3000LL),-reale(455567530,0xe21e28364f800LL),
      reale(400948026,0xf028b16722000LL),-reale(118913774,0x549816fe9c800LL),
      -reale(112010399,0x36034a3e3f000LL),reale(121825743,0x78c43cf486800LL),
      -reale(36338425,0x426e19287b880LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^6, polynomial in n of order 23
      -real(0x1b5badebe000LL),-real(0x326332ca4000LL),-real(0x5fd1bd93a000LL),
      -real(0xbcd8e5378000LL),-real(0x1837bef256000LL),
      -real(0x3404424ccc000LL),-real(0x75bf8cd1d2000LL),
      -real(38025986691LL<<17),-real(0x2dc96f11f6e000LL),
      -real(0x811a6e895f4000LL),-real(0x195036bc82ea000LL),
      -real(0x5af70d135548000LL),-real(0x187d57cdaa406000LL),
      -reale(2189,0x32d399c61c000LL),-reale(18742,0x385cb42a82000LL),
      -reale(438375,0xd6a8872030000LL),reale(9224813,0x89f7eb41e2000LL),
      -reale(57288808,0xfdc8999b44000LL),reale(184899999,0x331692f966000LL),
      -reale(357870966,0x3154fb6f18000LL),reale(431875147,0x7929b7544a000LL),
      -reale(318710001,0xe0f19bd36c000LL),reale(131641087,0xbb852faace000LL),
      -reale(23311442,0x9e8a4070e9d00LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[5], coeff of eps^5, polynomial in n of order 24
      -real(92116035LL<<14),-real(0x26e7bc2d800LL),-real(0x46d3779b000LL),
      -real(0x84e1d0c0800LL),-real(0x101cbc30a000LL),-real(0x2073376e3800LL),
      -real(0x442adb8b9000LL),-real(0x963884ff6800LL),-real(0x15dbd71e08000LL),
      -real(0x363ebc6d59800LL),-real(0x9122bbd857000LL),
      -real(0x1a90a4ab06c800LL),-real(0x56f0a68cd06000LL),
      -real(0x147a29992a8f800LL),-real(0x5d1402e6c175000LL),
      -real(0x228e263277d22800LL),-reale(5078,0x584c613b04000LL),
      -reale(128863,0x92233985800LL),reale(2982258,0xd360aa0ed000LL),
      -reale(20710125,0x5bbe664118800LL),reale(76213261,0x519df32cfe000LL),
      -reale(171479837,0xf7a363253b800LL),reale(241341994,0x2d1ed763cf000LL),
      -reale(186491540,0xf45203874e800LL),reale(58278606,0x8c59a11a48880LL),
      reale(45119934611LL,0xe897fd72d67cbLL),
      // C4[6], coeff of eps^29, polynomial in n of order 0
      139264,real(63626127165LL),
      // C4[6], coeff of eps^28, polynomial in n of order 1
      real(247833LL<<16),real(4782743552LL),real(0x219ae3fb400f15LL),
      // C4[6], coeff of eps^27, polynomial in n of order 2
      real(420150473LL<<18),-real(0x876551ce0000LL),real(0x350bfa156000LL),
      reale(4837,0x68f14547adebLL),
      // C4[6], coeff of eps^26, polynomial in n of order 3
      real(0x297e6b0e9e1000LL),-real(0x2e90de909aa000LL),
      real(0x6148b0a84b000LL),real(0x1d77336bca600LL),
      reale(207992,0x1a086a30a3679LL),
      // C4[6], coeff of eps^25, polynomial in n of order 4
      real(0x10bc6a9e4ee30000LL),-real(0xc179e3d40c9c000LL),
      real(0x3edf483df118000LL),-real(0x5c91fff78634000LL),
      real(0x216fdab58654400LL),reale(10078162,0xbedd8dc0620e7LL),
      // C4[6], coeff of eps^24, polynomial in n of order 5
      reale(17715,0xdb1cfba26000LL),-reale(7689,0x9976d7f948000LL),
      reale(6474,0xb1047d5d4a000LL),-reale(6855,0xa6eeabbaa4000LL),
      real(0x2ac3e335ea26e000LL),real(0xd6d2e7c22e28400LL),
      reale(372892021,0x96057cce2c163LL),
      // C4[6], coeff of eps^23, polynomial in n of order 6
      reale(279883,0xa92c150938000LL),-reale(86797,0xd10c69f53c000LL),
      reale(160072,0xfd9d58a4d0000LL),-reale(96731,0xc2b3d16724000LL),
      reale(32938,0x46d62be868000LL),-reale(52162,0xc27e2d9b0c000LL),
      reale(17103,0x67a9fde667c00LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[6], coeff of eps^22, polynomial in n of order 7
      reale(293467,0x7db7c77729000LL),-reale(146628,0x46fd92fe6000LL),
      reale(282074,0xcdca0f3f8b000LL),-reale(92435,0x174eb2c344000LL),
      reale(105774,0xf5edeb18ed000LL),-reale(100726,0x78839052a2000LL),
      reale(6619,0xde4489894f000LL),reale(2174,0xdeb0a21cf2e00LL),
      reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[6], coeff of eps^21, polynomial in n of order 8
      reale(183603,8337878185LL<<19),-reale(387951,0x8934978f10000LL),
      reale(363243,0x9b8677d760000LL),-reale(100927,0x6adc79e30000LL),
      reale(246790,7131746729LL<<18),-reale(115867,0xce56197550000LL),
      reale(45470,0x976a005d20000LL),-reale(74789,0x6bec0ac470000LL),
      reale(21823,0x7d1eb3d72b000LL),reale(4101812237LL,0x723c5cdbe4f41LL),
      // C4[6], coeff of eps^20, polynomial in n of order 9
      reale(2390210,0x71ea4526d8000LL),-reale(11473167,6397281565LL<<18),
      reale(3566140,0xe9fdb6daa8000LL),-reale(3459649,0xbdbfad5d70000LL),
      reale(5328875,0xe507b89678000LL),-reale(1202839,0xbeff1963a0000LL),
      reale(2208040,0x527339ea48000LL),-reale(1770989,0xb71cae09d0000LL),
      reale(48626,0x557ebf6618000LL),reale(16670,0x4a1716aa8d000LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^19, polynomial in n of order 10
      reale(16170911,0xf66942f9a0000LL),-reale(15946100,0x87937e1ff0000LL),
      reale(1191966,5683381737LL<<19),-reale(10381645,0x67a9610710000LL),
      reale(5401104,0xec5f94af60000LL),-reale(1916345,0x9f2b7d6630000LL),
      reale(5166787,7293640425LL<<18),-reale(1681428,0xa094a5ad50000LL),
      reale(912008,0xad6a83a520000LL),-reale(1452992,0x3f13404c70000LL),
      reale(367621,0xca46f4fdbb000LL),reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^18, polynomial in n of order 11
      reale(51879505,0x1c6021da42000LL),-reale(3388727,0x452f2e2244000LL),
      reale(10993546,0x58785d1036000LL),-reale(19450323,0x2862de39d0000LL),
      reale(1456775,0xebc764482a000LL),-reale(7922511,0x8d8f4f815c000LL),
      reale(7390372,0xfe1ce59e1e000LL),-reale(1065019,0x2a2a06ce8000LL),
      reale(3871757,0x7ef447ee12000LL),-reale(2395461,0x8df44bf074000LL),
      -reale(40351,0xb597a7abfa000LL),-reale(17707,0xeba2dcf1c1400LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^17, polynomial in n of order 12
      reale(18941665,0xd940803e20000LL),-reale(2462456,0xc647b5b638000LL),
      reale(55543449,0x9a9f25d270000LL),-reale(10182797,0xdffcb19ee8000LL),
      reale(4836527,0xb44e233ec0000LL),-reale(21402374,0x58dcab98000LL),
      reale(3817083,0xbef1c88b10000LL),-reale(4459099,0x992120d448000LL),
      reale(8502561,0xac3fb5bf60000LL),-reale(1525489,0x80b8b610f8000LL),
      reale(1649611,0x4cebe6e3b0000LL),-reale(2280763,0x4f507e59a8000LL),
      reale(482782,0x1ffc428c24800LL),reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^16, polynomial in n of order 13
      reale(169672066,0xfc4e53058c000LL),-reale(255936417,0xcd4166f930000LL),
      reale(43044311,0x58bada2414000LL),reale(10984552,0x79ecf34458000LL),
      reale(54615551,0xb3c2ab069c000LL),-reale(20672829,0x547b9ae620000LL),
      -reale(762958,0xc96d76adc000LL),-reale(20252510,0xad74c43098000LL),
      reale(8266131,0x9541dc37ac000LL),-reale(1263055,0x9458475310000LL),
      reale(7416125,0xebded0d634000LL),-reale(3121438,0x16f54c0588000LL),
      -reale(225538,0xf843322744000LL),-reale(111163,0x41ef8785bb800LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^15, polynomial in n of order 14
      -reale(371272727,0xe93844d330000LL),reale(258600199,0x3ab9b44ef8000LL),
      reale(127447726,0xd7dad2fc20000LL),-reale(278220404,0x7730102b8000LL),
      reale(77869881,0xad9b189b70000LL),reale(21813766,0xb09d2ff98000LL),
      reale(46644312,9197745227LL<<18),-reale(33841430,0x25b28aa218000LL),
      -reale(3096455,0x6fa54a95f0000LL),-reale(14807144,0xa86ee6dfc8000LL),
      reale(13281582,0xf66e06a960000LL),-reale(452377,0x35cd9cb178000LL),
      reale(3621811,0x85d91d8b0000LL),-reale(3791781,0x3a80710f28000LL),
      reale(636887,0x5f8cc1d1bc800LL),reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^14, polynomial in n of order 15
      -reale(40751652,0x879256f716000LL),reale(182461023,0x62c00442f4000LL),
      -reale(366891419,0xe235688602000LL),reale(303920923,0x2a6218fe88000LL),
      reale(70640959,0xa70aa30512000LL),-reale(290919308,0xf0cc1f4de4000LL),
      reale(124435738,0x116d522626000LL),reale(24575054,0x49539549b0000LL),
      reale(29829722,0x6d4c4f193a000LL),-reale(46205497,0xcd680acebc000LL),
      reale(1253661,0x8798d15a4e000LL),-reale(5829398,0x329c172b28000LL),
      reale(15178042,0x87d0f72562000LL),-reale(3413258,0x604057df94000LL),
      -reale(544537,0x1343d1098a000LL),-reale(371792,0x5ec0380ab3400LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^13, polynomial in n of order 16
      reale(100946,21976965LL<<20),reale(2010862,0x3c46708bb0000LL),
      -reale(34502092,0x6e09dbf3a0000LL),reale(163298206,0x527fb2e110000LL),
      -reale(355839921,948516465LL<<18),reale(347383598,0x3243b82e70000LL),
      -reale(2611762,0xae3f6124e0000LL),-reale(286060486,421499843LL<<16),
      reale(181022396,2339564421LL<<19),reale(11053843,0x8ea9e8f130000LL),
      reale(5354229,0xc704cb69e0000LL),-reale(50862137,0xf12aeaf970000LL),
      reale(14064844,5665935493LL<<18),reale(1748678,0x2e869553f0000LL),
      reale(9719088,0x671cfc38a0000LL),-reale(6714197,0x76aa8fd6b0000LL),
      reale(816805,0x9ce5b98e4f000LL),reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^12, polynomial in n of order 17
      real(0x75cff722d22b8000LL),reale(9742,5260669319LL<<19),
      reale(75734,0x79163f0448000LL),reale(1568684,0xd935dd4310000LL),
      -reale(28213944,0x88db35f228000LL),reale(141802366,0xe4716652a0000LL),
      -reale(336424367,0x7aaa4f7098000LL),reale(384795625,0xe2aff0230000LL),
      -reale(92516926,0xbd45322708000LL),-reale(252728877,4730701433LL<<18),
      reale(239978666,0xfd893c3a88000LL),-reale(28528394,5445461995LL<<16),
      -reale(18370370,0x5cd8a4fbe8000LL),-reale(38961300,0x78b7628f20000LL),
      reale(30014507,0xb37b1485a8000LL),-reale(654615,0xa96a2bf90000LL),
      -reale(667571,0x85c41bf0c8000LL),-reale(1181523,0x1c81baa857000LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^11, polynomial in n of order 18
      real(0x55d873de6520000LL),real(0x12c7cfeef6810000LL),
      real(0x4e200e3f1e1LL<<20),reale(6671,0xd2467fb9f0000LL),
      reale(53806,3275978471LL<<17),reale(1163348,0xd1cfb7f3d0000LL),
      -reale(22032298,0xf3cc53d740000LL),reale(118198962,4397370971LL<<16),
      -reale(306929389,0x72efa76b60000LL),reale(409945031,0xba4df5f90000LL),
      -reale(195574008,5584443935LL<<19),-reale(178055138,0x4cd4f3ce90000LL),
      reale(282861404,0xd715020c60000LL),-reale(99637722,0xf11193d4b0000LL),
      -reale(20986520,0xfb661347c0000LL),-reale(8771627,7018708525LL<<16),
      reale(31360164,0xdb2c51c420000LL),-reale(12477955,8590832271LL<<16),
      reale(873590,0xbe0d3e9693000LL),reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^10, polynomial in n of order 19
      real(0x5808512b12b000LL),real(0xfaa729276e2000LL),
      real(0x3175560e4519000LL),real(0xb21b680b3a90000LL),
      real(0x2fcbc5fe71407000LL),reale(4229,0xf0de326e3e000LL),
      reale(35532,0x38e22907f5000LL),reale(805604,0x42db4fa3ec000LL),
      -reale(16150031,0xfe4d67d51d000LL),reale(93034137,0xf6628ead9a000LL),
      -reale(265995225,0x398943192f000LL),reale(414315266,0x970145dd48000LL),
      -reale(301204836,0xc549c7ba41000LL),-reale(51738066,0x4e1063bb0a000LL),
      reale(275650719,0x10481031ad000LL),-reale(187610845,0x85f00095c000LL),
      reale(25230256,0x4ada23b49b000LL),reale(13917204,0x3da6dc4452000LL),
      reale(4066715,0x8660f73889000LL),-reale(4361677,0xea98323d07e00LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^9, polynomial in n of order 20
      real(0x65fa8c6bf0000LL),real(0xfe88642ae4000LL),real(0x2aa82304e58000LL),
      real(0x7ca8bddcccc000LL),real(434853972467LL<<18),
      real(0x5e16320d44b4000LL),real(0x1a2859bf40b28000LL),
      reale(2409,0x1b825da69c000LL),reale(21179,0xabe6860d90000LL),
      reale(506292,0x5b6e5f0684000LL),-reale(10810252,0xeee1886808000LL),
      reale(67327238,0xa18a80786c000LL),-reale(213364581,0xe79aac41a0000LL),
      reale(387619687,0x51e3ba1054000LL),-reale(387180015,0xd550406b38000LL),
      reale(121695298,0x2400c6e23c000LL),reale(172879787,0x9e57682f30000LL),
      -reale(230507460,0xb74e70fddc000LL),reale(112381926,0x4eee70a198000LL),
      -reale(20283371,0x42949e7bf4000LL),-reale(288686,0x988d3450a7c00LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^8, polynomial in n of order 21
      real(0x72e86a7de000LL),real(8772831327LL<<15),real(0x273ffc1812000LL),
      real(0x64635c5cac000LL),real(0x11473cdd246000LL),
      real(0x33fd816c260000LL),real(0xae6e2137a7a000LL),
      real(0x29ff10928814000LL),real(0xc26a115cf4ae000LL),
      real(0x492994f20c1c8000LL),reale(10833,0x80f3c9e4e2000LL),
      reale(274842,0xd406a2037c000LL),-reale(6296293,0xca802ed0ea000LL),
      reale(42731189,0xb6f3d1e130000LL),-reale(151191524,0x41a7e788b6000LL),
      reale(320575109,0xae49526ee4000LL),-reale(416345568,0xb8c8445e82000LL),
      reale(298319523,0xb52957c098000LL),-reale(42956565,0x78799bae4e000LL),
      -reale(119892798,0x70342c95b4000LL),reale(103927174,0x8691916be6000LL),
      -reale(29157346,0x2fb5a3d22ec00LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[6], coeff of eps^7, polynomial in n of order 22
      real(74709635LL<<15),real(0x4ba47734000LL),real(0xa7b994d0000LL),
      real(0x1869c5c6c000LL),real(0x3c23e3d88000LL),real(0x9e1c8b7a4000LL),
      real(1882100649LL<<18),real(0x573ad5a4dc000LL),real(0x12f915ab6f8000LL),
      real(0x4c1f4084014000LL),real(0x170ced7cbfb0000LL),
      real(0x921b89aca54c000LL),real(0x599b4a7922068000LL),
      reale(38914,0x1efa73f084000LL),-reale(964915,0x51a6da0ae0000LL),
      reale(7200274,0x92a23dbc000LL),-reale(28652022,0x356dea628000LL),
      reale(70837833,0x39cdeca8f4000LL),-reale(114872161,0xfcdf3a9570000LL),
      reale(122704354,0xd9bfe74e2c000LL),-reale(83141739,0x9edadabcb8000LL),
      reale(32332898,0xbdc6e34964000LL),-reale(5485045,0x527ae1fc73400LL),
      reale(17774519695LL,0x99b03d0e3576fLL),
      // C4[6], coeff of eps^6, polynomial in n of order 23
      real(257316433920LL),real(517719121920LL),real(0xfb6e649000LL),
      real(0x221f7064000LL),real(0x4d84a37f000LL),real(0xb958155a000LL),
      real(0x1d5dd0db5000LL),real(0x4faa5a050000LL),real(0xea04686eb000LL),
      real(0x2f40e3db46000LL),real(0xab8623d121000LL),real(0x2d147c4903c000LL),
      real(0xe63ae874e57000LL),real(0x60cd21bcc932000LL),
      real(0x3f869e23e408d000LL),reale(29814,0xcc97221028000LL),
      -reale(808726,0x6d837bf63d000LL),reale(6700876,0x1daf27af1e000LL),
      -reale(30153942,0x8594329407000LL),reale(86154121,0x7da76bf014000LL),
      -reale(165128732,0xdb80e436d1000LL),reale(210163841,0xd18cc55d0a000LL),
      -reale(153581269,0x570b79c9b000LL),reale(46622885,0x3d1480e1d3a00LL),
      reale(53323559086LL,0xcd10b72aa064dLL),
      // C4[7], coeff of eps^29, polynomial in n of order 0
      real(13087612928LL),real(0x90e6983c364f3dLL),
      // C4[7], coeff of eps^28, polynomial in n of order 1
      -real(161707LL<<21),real(7239297LL<<14),real(0xcf8f801ee602cdLL),
      // C4[7], coeff of eps^27, polynomial in n of order 2
      -real(3500022825LL<<20),real(630513507LL<<19),real(0x6038c37fa000LL),
      reale(72555,0x626230f3330c5LL),
      // C4[7], coeff of eps^26, polynomial in n of order 3
      -real(92252949633LL<<21),real(16187170389LL<<22),
      -real(51975912235LL<<21),real(0x7c00d0f2b78000LL),
      reale(3119881,0x867e38d993117LL),
      // C4[7], coeff of eps^25, polynomial in n of order 4
      -real(0x64d0a86bae7c0000LL),real(0x7c07ce24c65f0000LL),
      -real(0x739ece76489e0000LL),real(0x6e7bce15f550000LL),
      real(0x24fc420030b8400LL),reale(127915142,0x8a371ad88dcafLL),
      // C4[7], coeff of eps^24, polynomial in n of order 5
      -reale(5990,0xbd2326cc40000LL),reale(14992,4018200301LL<<20),
      -reale(6873,8929851351LL<<18),reale(2782,8051012645LL<<19),
      -reale(4583,0xc89924b340000LL),real(0x52aed30dcf988800LL),
      reale(430260024,0xe82db7640b7c1LL),
      // C4[7], coeff of eps^23, polynomial in n of order 6
      -reale(169326,4206873009LL<<17),reale(261065,0x25b4e353d0000LL),
      -reale(59142,0xf0c50992c0000LL),reale(111182,4597550539LL<<16),
      -reale(88869,504433083LL<<17),reale(2313,0xe34bfe3f90000LL),
      real(0x32dc48b9e1d23400LL),reale(4732860273LL,0xf9f6e14c7e54bLL),
      // C4[7], coeff of eps^22, polynomial in n of order 7
      -reale(467157,1100000847LL<<20),reale(258178,755278933LL<<21),
      -reale(91474,559664221LL<<20),reale(248285,171426119LL<<22),
      -reale(82821,231309675LL<<20),reale(44668,65972935LL<<21),
      -reale(71456,2669582201LL<<20),reale(18220,0x9846e079d4000LL),
      reale(4732860273LL,0xf9f6e14c7e54bLL),
      // C4[7], coeff of eps^21, polynomial in n of order 8
      -reale(10858183,1145150433LL<<21),reale(1155453,0xa514064740000LL),
      -reale(4408275,1110140307LL<<19),reale(4494002,0xa8330ec1c0000LL),
      -reale(693747,3759921697LL<<20),reale(2336198,8880970129LL<<18),
      -reale(1499288,4981657777LL<<19),-reale(18466,6402610053LL<<18),
      -reale(7818,0x6ee4879b83000LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^20, polynomial in n of order 9
      -reale(7907170,4058896835LL<<20),reale(1601483,335338375LL<<23),
      -reale(11238504,3427529005LL<<20),reale(2745284,1787777405LL<<21),
      -reale(2325455,2252860775LL<<20),reale(4939939,712213223LL<<22),
      -reale(1021126,555773201LL<<20),reale(952760,1631005375LL<<21),
      -reale(1365312,965324491LL<<20),reale(299618,0x2f589c3f22000LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^19, polynomial in n of order 10
      reale(5811147,7891888051LL<<19),reale(19155879,6260648859LL<<18),
      -reale(13234724,832589145LL<<21),-reale(473729,0xaccee67ac0000LL),
      -reale(9690431,2460044795LL<<19),reale(5218195,5282091375LL<<18),
      -reale(699193,3313511321LL<<20),reale(4032431,0xb01d955a40000LL),
      -reale(1901524,5999844905LL<<19),-reale(111197,715304509LL<<18),
      -reale(51622,0xdda253af9f000LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^18, polynomial in n of order 11
      -reale(34477536,1085877825LL<<20),reale(44845230,817114545LL<<21),
      reale(4606432,1572161669LL<<20),reale(12496576,210421693LL<<23),
      -reale(18271471,1543698101LL<<20),-reale(128700,742574025LL<<21),
      -reale(6139017,689151983LL<<20),reale(7385046,100502461LL<<22),
      -reale(590509,2783893289LL<<20),reale(1800602,699157181LL<<21),
      -reale(2092277,3566080099LL<<20),reale(381025,0x99466ecd7c000LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^17, polynomial in n of order 12
      -reale(152644671,981125379LL<<19),-reale(24136152,0xd3514f38e0000LL),
      -reale(16909786,8097141141LL<<18),reale(53988238,0xc115854860000LL),
      -reale(2192558,3293732289LL<<20),reale(3853073,2819007469LL<<17),
      -reale(20689919,5309095411LL<<18),reale(3514368,0xf1b4463ee0000LL),
      -reale(1814216,3975618817LL<<19),reale(7354899,0xbd88356420000LL),
      -reale(2207882,191252177LL<<18),-reale(269543,3717910997LL<<17),
      -reale(156646,0x7bcb3b3a6a800LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^16, polynomial in n of order 13
      reale(52565396,753292423LL<<19),reale(252855342,568744119LL<<21),
      -reale(197183211,7281644191LL<<19),-reale(6678358,3552459447LL<<20),
      reale(4519131,7283469291LL<<19),reale(56648760,112164189LL<<22),
      -reale(15289276,2020707835LL<<19),-reale(3713103,1403767329LL<<20),
      -reale(17880720,7304289905LL<<19),reale(9494998,1497636157LL<<21),
      reale(492167,2907561065LL<<19),reale(3952538,4294903605LL<<20),
      -reale(3358139,5130468237LL<<19),reale(480004,0x1e727719e9000LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^15, polynomial in n of order 14
      reale(279617399,0xd9972cba40000LL),-reale(353187715,0x687b832220000LL),
      reale(118965967,4456434973LL<<19),reale(220096359,3595022681LL<<17),
      -reale(240814657,8170991797LL<<18),reale(28075084,0xec7a345460000LL),
      reale(24758769,1818605983LL<<20),reale(48013974,0xb5345431a0000LL),
      -reale(32373431,0xc7bac8f4c0000LL),-reale(5075135,8642954025LL<<17),
      -reale(9094832,6469786017LL<<19),reale(13639028,3685620545LL<<17),
      -reale(1773068,1431802737LL<<18),-reale(460476,0x51ab5a8ea0000LL),
      -reale(423738,0x5d98934922800LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^14, polynomial in n of order 15
      reale(16417106,2408387839LL<<20),-reale(93245803,1562234793LL<<21),
      reale(256985456,250552029LL<<20),-reale(365861944,857240429LL<<22),
      reale(190902238,1499270843LL<<20),reale(163412998,1423242741LL<<21),
      -reale(274443985,2668181351LL<<20),reale(82958237,163620913LL<<23),
      reale(33859016,1729347703LL<<20),reale(25275487,1495319443LL<<21),
      -reale(45844273,3794232747LL<<20),reale(4490176,231613489LL<<22),
      reale(1010900,690735667LL<<20),reale(10013483,1036831025LL<<21),
      -reale(5637707,2068106223LL<<20),reale(570308,0x8f0afe45ec000LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^13, polynomial in n of order 16
      -reale(25657,393048869LL<<22),-reale(608651,0xacbc40d5c0000LL),
      reale(12764052,2144856077LL<<19),-reale(76823449,3121867141LL<<18),
      reale(228672619,4131243473LL<<20),-reale(367062288,0xf756dca4c0000LL),
      reale(263470997,8569317111LL<<19),reale(78573490,0xffb10f8fc0000LL),
      -reale(283548774,1794660389LL<<21),reale(154702622,9227087281LL<<18),
      reale(16937276,1939608161LL<<19),-reale(6432822,7897704317LL<<18),
      -reale(43016670,946798949LL<<20),reale(22087851,0xaa7600dd40000LL),
      reale(1665577,8523064651LL<<19),-reale(163221,0xe0acad2e40000LL),
      -reale(1189371,0x766c2260a3000LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^12, polynomial in n of order 17
      -real(0x13bc5107d5fLL<<20),-real(506650109317LL<<24),
      -reale(17217,2185571073LL<<20),-reale(426469,557216187LL<<21),
      reale(9411503,1140836685LL<<20),-reale(60299258,66945391LL<<22),
      reale(194753933,1835852139LL<<20),-reale(352928157,2046106529LL<<21),
      reale(328231147,2405007161LL<<20),-reale(34561926,360605189LL<<23),
      -reale(248371006,3915897705LL<<20),reale(226668375,1401273273LL<<21),
      -reale(40114392,2920598683LL<<20),-reale(25898188,1028871717LL<<22),
      -reale(16043876,2538787453LL<<20),reale(28698456,1825641427LL<<21),
      -reale(9602688,2437057327LL<<20),reale(502063,0xa52218333a000LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^11, polynomial in n of order 18
      -real(81880241733LL<<19),-real(651169421489LL<<18),
      -real(194261131981LL<<22),-real(0x4616f301f1bc0000LL),
      -reale(10659,7786635659LL<<19),-reale(276843,0xf150eaf340000LL),
      reale(6459374,425055961LL<<20),-reale(44283297,2370521611LL<<18),
      reale(156003403,8328479919LL<<19),-reale(319848045,0xab86b09a40000LL),
      reale(372382116,1407449139LL<<21),-reale(166870261,0xbaeb2e09c0000LL),
      -reale(148815577,7753476247LL<<19),reale(260443738,1330203003LL<<18),
      -reale(131653575,428167437LL<<20),reale(2775725,691412797LL<<18),
      reale(12306214,6299226531LL<<19),reale(5355345,9401097695LL<<18),
      -reale(3966302,0xcbc08bfb17000LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^10, polynomial in n of order 19
      -real(1704454843LL<<20),-real(2722537665LL<<21),-real(19434970697LL<<20),
      -real(4989045369LL<<24),-real(394962411735LL<<20),
      -real(0x128b33efecfLL<<21),-reale(5903,789230693LL<<20),
      -reale(161527,569013611LL<<22),reale(4006338,1271698701LL<<20),
      -reale(29564239,1312346333LL<<21),reale(114267945,2347153791LL<<20),
      -reale(265827046,63320697LL<<23),reale(379233361,4202669809LL<<20),
      -reale(292689947,1148927723LL<<21),reale(19915451,3747715939LL<<20),
      reale(197711494,385979271LL<<22),-reale(197401730,911113003LL<<20),
      reale(83971818,387288839LL<<21),-reale(12852829,1345691321LL<<20),
      -reale(602476,0x5fc28370ac000LL),reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^9, polynomial in n of order 20
      -real(304621785LL<<18),-real(0xc9814e4b0000LL),-real(5069418237LL<<17),
      -real(0x7c4fe70d90000LL),-real(7691534469LL<<20),
      -real(0x7a02179d470000LL),-real(0x274586580a60000LL),
      -real(0x10907db87bd50000LL),-reale(2773,9732262223LL<<18),
      -reale(80424,78339267LL<<16),reale(2134032,0xd8c3d9bae0000LL),
      -reale(17066460,0x8709888510000LL),reale(72842964,6932239995LL<<19),
      -reale(192914141,0x448548ebf0000LL),reale(332328916,0xa61d5e5020000LL),
      -reale(364348462,0x260e7984d0000LL),reale(215166704,0xfd6630ec0000LL),
      reale(5301792,6304582341LL<<16),-reale(118567350,0x6a550b6aa0000LL),
      reale(89166503,0x5c73fd2370000LL),-reale(23960987,0x75c7f62663400LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^8, polynomial in n of order 21
      -real(11869221LL<<18),-real(7450235LL<<20),-real(79397539LL<<18),
      -real(113271327LL<<19),-real(700448177LL<<18),-real(148973407LL<<22),
      -real(9118660335LL<<18),-real(20216702289LL<<19),
      -real(0xcadd965ff40000LL),-real(386512744317LL<<20),
      -real(0xf93c68aca7bLL<<18),-reale(30847,5279995331LL<<19),
      reale(882325,8584251383LL<<18),-reale(7706931,2116826591LL<<21),
      reale(36580048,2730390969LL<<18),-reale(110604386,3847062005LL<<19),
      reale(227103584,0x9e98f54ac0000LL),-reale(323034443,1752619391LL<<20),
      reale(314251676,0xb5ebcf2b40000LL),-reale(199218854,3061725287LL<<19),
      reale(73903768,9476063903LL<<18),-reale(12124837,0x72a953b85800LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[7], coeff of eps^7, polynomial in n of order 22
      -real(575575LL<<17),-real(2681133LL<<16),-real(1637545LL<<18),
      -real(16890107LL<<16),-real(23159565LL<<17),-real(0x8210e690000LL),
      -real(27276821LL<<20),-real(0x5bebf1b70000LL),-real(3075032387LL<<17),
      -real(0x6a5f183250000LL),-real(40477467135LL<<18),
      -real(0x11b5c31caf30000LL),-real(0xd14cd352ff20000LL),
      -reale(6969,0xb17d189610000LL),reale(216834,7757873387LL<<19),
      -reale(2087035,0xf153506af0000LL),reale(11091105,0x4b9d7f7a20000LL),
      -reale(38290720,0xa99fbe31d0000LL),reale(91897729,0x9718fbaac0000LL),
      -reale(156643857,0x418d7e6eb0000LL),reale(184759421,0x6102c9a360000LL),
      -reale(129331594,0xf71b8d2590000LL),reale(38395317,0x415c2de726c00LL),
      reale(61527183561LL,0xb18970e26a4cfLL),
      // C4[8], coeff of eps^29, polynomial in n of order 0
      real(7241<<16),real(0x112c657acf71bLL),
      // C4[8], coeff of eps^28, polynomial in n of order 1
      real(1165359LL<<20),real(3168035LL<<17),real(0x21ffb4a731cf423fLL),
      // C4[8], coeff of eps^27, polynomial in n of order 2
      real(41827383LL<<21),-real(137865429LL<<20),real(631109843LL<<16),
      reale(4837,0x68f14547adebLL),
      // C4[8], coeff of eps^26, polynomial in n of order 3
      real(54350489115LL<<22),-real(21656377197LL<<23),real(1080358617LL<<22),
      real(0x5c4a2579a0000LL),reale(3535865,0xba8f0d3ad9e09LL),
      // C4[8], coeff of eps^25, polynomial in n of order 4
      reale(4480,63845967LL<<22),-real(0x5f0bc8cec07LL<<20),
      real(0x198015cca1fLL<<21),-real(0x51d1e6f78cdLL<<20),
      real(0x14fb331d33f30000LL),reale(144970494,0xe0e91e6ce4f71LL),
      // C4[8], coeff of eps^24, polynomial in n of order 5
      reale(226427,7535956641LL<<17),-reale(36730,6647829291LL<<19),
      reale(116830,5936429895LL<<17),-reale(76966,613785099LL<<18),
      -real(0x2a948e8d73a60000LL),-real(0x116572b5168a4000LL),
      reale(5363908310LL,0x81b165bd17b55LL),
      // C4[8], coeff of eps^23, polynomial in n of order 6
      reale(151394,3866446399LL<<20),-reale(105723,1435687723LL<<19),
      reale(240417,2090106533LL<<21),-reale(54672,3991575693LL<<19),
      reale(46185,3230210197LL<<20),-reale(67790,4028416911LL<<19),
      reale(15270,0xa469197488000LL),reale(5363908310LL,0x81b165bd17b55LL),
      // C4[8], coeff of eps^22, polynomial in n of order 7
      -reale(105618,1394014919LL<<21),-reale(5351753,377020849LL<<22),
      reale(3446650,1690522763LL<<21),-reale(453181,286167171LL<<23),
      reale(2431204,1637447437LL<<21),-reale(1239333,63204475LL<<22),
      -reale(60030,1665832481LL<<21),-reale(26716,0x6a2a5d69d0000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^21, polynomial in n of order 8
      reale(4362900,465328075LL<<22),-reale(10560212,802403079LL<<19),
      reale(656010,2976408017LL<<20),-reale(3068612,8162681445LL<<19),
      reale(4482659,1990068235LL<<21),-reale(516359,5969201251LL<<19),
      reale(1022585,502576667LL<<20),-reale(1273161,3447687361LL<<19),
      reale(245310,0x45a78ad538000LL),reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^20, polynomial in n of order 9
      reale(23624906,1629010283LL<<19),-reale(5851601,324958949LL<<22),
      reale(255419,6850290885LL<<19),-reale(10559838,1365338319LL<<20),
      reale(3058631,5351542623LL<<19),-reale(782822,266312293LL<<21),
      reale(4071490,3032871865LL<<19),-reale(1457911,2387656005LL<<20),
      -reale(144540,929274797LL<<19),-reale(76349,0x2c1c25d590000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^19, polynomial in n of order 10
      reale(23056909,2395766741LL<<20),reale(8427619,3212023717LL<<19),
      reale(19522568,367619617LL<<22),-reale(12637641,5752438869LL<<19),
      -reale(1859539,2126155853LL<<20),-reale(7720368,2358643951LL<<19),
      reale(5969641,447801057LL<<21),-reale(4464,2860310889LL<<19),
      reale(1954609,2968726289LL<<20),-reale(1904959,7710818563LL<<19),
      reale(302310,0x7de136fc28000LL),reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^18, polynomial in n of order 11
      -reale(34760584,1377594673LL<<21),-reale(45089279,700199389LL<<22),
      reale(38964787,642296389LL<<21),reale(8377867,242649263LL<<24),
      reale(10805340,270655563LL<<21),-reale(18348308,77894251LL<<22),
      -reale(8504,712824639LL<<21),-reale(2874058,104939633LL<<23),
      reale(7002991,271121287LL<<21),-reale(1456031,497148985LL<<22),
      -reale(262921,1640850307LL<<21),-reale(186713,0x66184da2b0000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^17, polynomial in n of order 12
      reale(266733950,1060079417LL<<21),-reale(92315127,311764467LL<<19),
      -reale(39784767,2743383633LL<<20),-reale(24124714,5368290721LL<<19),
      reale(52035773,16490707LL<<22),-reale(214469,3779317103LL<<19),
      -reale(32744,3406796695LL<<20),-reale(19012957,4710528797LL<<19),
      reale(5897141,767669011LL<<21),reale(758445,547828629LL<<19),
      reale(4185368,186448291LL<<20),-reale(2955613,5547446041LL<<19),
      reale(363691,0xed908404b8000LL),reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^16, polynomial in n of order 13
      -reale(254507630,0xe2b8b6bb40000LL),-reale(58148124,1471923579LL<<20),
      reale(270522720,3187458133LL<<18),-reale(149985652,7726894061LL<<19),
      -reale(27328603,0x99e6e9ea40000LL),reale(4011861,407374679LL<<21),
      reale(54943982,0xaf3dd22640000LL),-reale(17478454,3922351195LL<<19),
      -reale(6848089,0x9bbe86d940000LL),-reale(11885922,3297252137LL<<20),
      reale(11706960,678889437LL<<18),-reale(595378,2432546249LL<<19),
      -reale(329167,0xe066968840000LL),-reale(450081,0x595d162958000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^15, polynomial in n of order 14
      -reale(166710239,3480741959LL<<20),reale(313421255,2329933911LL<<19),
      -reale(299209385,1287661491LL<<21),reale(24383199,5307535921LL<<19),
      reale(248029318,3243559867LL<<20),-reale(210032461,8189928917LL<<19),
      reale(10907073,960500783LL<<22),reale(29948106,2038678405LL<<19),
      reale(40306034,2725271357LL<<20),-reale(36745958,6196825729LL<<19),
      -reale(1934460,123839633LL<<21),-reale(492518,4761069735LL<<19),
      reale(9949315,1255380799LL<<20),-reale(4720329,388065197LL<<19),
      reale(398374,0x9081f25c18000LL),reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^14, polynomial in n of order 15
      -reale(5499415,942753073LL<<21),reale(38811064,349279653LL<<22),
      -reale(140017234,1709558915LL<<21),reale(290760665,163783697LL<<23),
      -reale(332595868,714890693LL<<21),reale(118902683,730607999LL<<22),
      reale(189429058,237701737LL<<21),-reale(256456610,243945477LL<<24),
      reale(78365400,1246868327LL<<21),reale(35514427,78282137LL<<22),
      reale(8045132,96899221LL<<21),-reale(42695880,422981029LL<<23),
      reale(15212575,506177875LL<<21),reale(2863173,903918451LL<<22),
      reale(284029,1922203905LL<<21),-reale(1161079,0x6c18f2ad70000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^13, polynomial in n of order 16
      reale(5407,439728533LL<<23),reale(151556,693836399LL<<19),
      -reale(3836797,3870271773LL<<20),reale(28742693,8016450573LL<<19),
      -reale(111747677,1508361473LL<<21),reale(256964119,236840267LL<<19),
      -reale(347691811,711701031LL<<20),reale(216072654,2622689769LL<<19),
      reale(88295276,790755477LL<<22),-reale(263658780,5516898905LL<<19),
      reale(163753678,37603151LL<<20),-reale(1803857,6339257275LL<<19),
      -reale(22560155,108468139LL<<21),-reale(21197875,3801517693LL<<19),
      reale(25658955,3111371333LL<<20),-reale(7416706,6937931487LL<<19),
      reale(268690,0x9ce0757848000LL),reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^12, polynomial in n of order 17
      real(369814360487LL<<19),real(159053188703LL<<23),
      reale(3152,1136779065LL<<19),reale(92558,2295771257LL<<20),
      -reale(2473330,1734833909LL<<19),reale(19757571,360351901LL<<21),
      -reale(83162616,6619531363LL<<19),reale(212413714,2066066043LL<<20),
      -reale(337117487,5524436113LL<<19),reale(299293348,697772127LL<<22),
      -reale(51076102,4535292671LL<<19),-reale(200831521,1217539203LL<<20),
      reale(227963885,2651839699LL<<19),-reale(87452614,163103009LL<<21),
      -reale(9664164,7186873499LL<<19),reale(9739937,3920029055LL<<20),
      reale(6101400,4999938359LL<<19),-reale(3588081,0x84422b3e50000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^11, polynomial in n of order 18
      real(3576016431LL<<20),real(32208729499LL<<19),real(10983028711LL<<23),
      real(0x9286be006280000LL),real(0x65e9f47db41LL<<20),
      reale(50386,4528870031LL<<19),-reale(1428014,1685009291LL<<21),
      reale(12227031,6176103481LL<<19),-reale(56007028,2392678701LL<<20),
      reale(159476659,5817614083LL<<19),-reale(295263705,809939737LL<<22),
      reale(344605761,6427356205LL<<19),-reale(202719833,951923227LL<<20),
      -reale(50658828,5629491977LL<<19),reale(201884255,1512264231LL<<21),
      -reale(166564947,5368120671LL<<19),reale(63259557,2614639991LL<<20),
      -reale(8140125,1990873493LL<<19),-reale(722971,0xa61c9dba68000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^10, polynomial in n of order 19
      real(45596577LL<<21),real(81531441LL<<22),real(656187675LL<<21),
      real(191463201LL<<25),real(17391213765LL<<21),real(65094511967LL<<22),
      real(0x16272ee843fLL<<21),reale(23168,382193603LL<<23),
      -reale(700305,441535191LL<<21),reale(6465118,134564813LL<<22),
      -reale(32414063,913166045LL<<21),reale(103314135,145041825LL<<24),
      -reale(222332965,1267927603LL<<21),reale(326070132,55789563LL<<22),
      -reale(309964302,1355885369LL<<21),reale(149975409,308317249LL<<23),
      reale(35633361,576916401LL<<21),-reale(113104897,1027785623LL<<22),
      reale(77116975,1889590315LL<<21),-reale(20082545,0xcd53c80110000LL),
      reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^9, polynomial in n of order 20
      real(1123785LL<<21),real(13838643LL<<19),real(23159565LL<<20),
      real(171251217LL<<19),real(44667189LL<<23),real(3472549135LL<<19),
      real(10302054723LL<<20),real(162001999341LL<<19),
      real(500351698399LL<<21),reale(8102,6578411627LL<<19),
      -reale(262912,1458939143LL<<20),reale(2634746,8016047177LL<<19),
      -reale(14551212,731365323LL<<22),reale(52133305,8561410375LL<<19),
      -reale(129964645,2819080401LL<<20),reale(232230181,2215647013LL<<19),
      -reale(298362920,1016411147LL<<21),reale(269480987,8039357859LL<<19),
      -reale(161945649,1493828379LL<<20),reale(57837731,7816254593LL<<19),
      -reale(9237971,9476063903LL<<15),reale(69730808036LL,0x96022a9a34351LL),
      // C4[8], coeff of eps^8, polynomial in n of order 21
      real(292383LL<<17),real(202215LL<<19),real(2386137LL<<17),
      real(3789747LL<<18),real(26247507LL<<17),real(6294651LL<<21),
      real(437764365LL<<17),real(1112245757LL<<18),real(0x67551030e0000LL),
      real(28804895217LL<<19),real(0x2c0f1d988820000LL),
      real(0x66a336663d1c0000LL),-reale(57641,8501165381LL<<17),
      reale(631918,3696102011LL<<20),-reale(3870503,720372107LL<<17),
      reale(15639991,0xcc8b836440000LL),-reale(44892569,0xd7d7de220000LL),
      reale(94682509,2360318459LL<<19),-reale(147486216,0x5ecdb08ae0000LL),
      reale(163873573,0xbeaba7b6c0000LL),-reale(110855652,0xd3ce78fba0000LL),
      reale(32332898,0xbdc6e34964000LL),reale(69730808036LL,0x96022a9a34351LL),
      // C4[9], coeff of eps^29, polynomial in n of order 0
      real(16847<<16),real(0x3d2e2985830503LL),
      // C4[9], coeff of eps^28, polynomial in n of order 1
      -real(207753LL<<23),real(1712087LL<<18),real(0x438da32e1600335LL),
      // C4[9], coeff of eps^27, polynomial in n of order 2
      -real(3127493161LL<<21),-real(38277317LL<<20),-real(0xe4960490000LL),
      reale(161925,0x30e683ffe0741LL),
      // C4[9], coeff of eps^26, polynomial in n of order 3
      -real(9299582409LL<<22),real(3656674463LL<<23),-real(10918261107LL<<22),
      real(80278491423LL<<17),reale(1317283,0x4f8aa089603a9LL),
      // C4[9], coeff of eps^25, polynomial in n of order 4
      -real(711479186953LL<<22),reale(3279,1361598081LL<<20),
      -real(0x3749d192179LL<<21),-real(309897952117LL<<20),
      -real(0x1f18264b9990000LL),reale(162025847,0x379b22013c233LL),
      // C4[9], coeff of eps^24, polynomial in n of order 5
      -reale(133856,15001023LL<<25),reale(223946,23087107LL<<27),
      -reale(32028,12079289LL<<25),reale(48931,2142027LL<<26),
      -reale(63933,112742755LL<<25),reale(12842,2153614949LL<<20),
      reale(5994956347LL,0x96bea2db115fLL),
      // C4[9], coeff of eps^23, polynomial in n of order 6
      -reale(5988742,4056322469LL<<20),reale(2349145,7648181561LL<<19),
      -reale(426462,344885543LL<<21),reale(2475174,940948911LL<<19),
      -reale(999559,3441325239LL<<20),-reale(83146,4971496059LL<<19),
      -reale(41198,0x4f02423cb8000LL),reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^22, polynomial in n of order 7
      -reale(8631189,1052889985LL<<21),-reale(629492,634245703LL<<22),
      -reale(3874477,505696163LL<<21),reale(3866974,513650043LL<<23),
      -reale(159710,1408881461LL<<21),reale(1100061,714281683LL<<22),
      -reale(1180171,586380503LL<<21),reale(201643,0x9fcf910730000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^21, polynomial in n of order 8
      reale(341632,721850923LL<<22),reale(2597220,4632100393LL<<19),
      -reale(10372056,1528523471LL<<20),reale(1205419,4719051179LL<<19),
      -reale(1145316,921601685LL<<21),reale(3990959,6117017869LL<<19),
      -reale(1073059,3486842565LL<<20),-reale(153111,7776387185LL<<19),
      -reale(94130,0x280827bb28000LL),reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^20, polynomial in n of order 9
      reale(3783713,134627971LL<<22),reale(23115315,66415493LL<<25),
      -reale(6392067,518305043LL<<22),-reale(1733013,275013225LL<<23),
      -reale(8816564,833972409LL<<22),reale(4468878,247379557LL<<24),
      reale(300534,553592433LL<<22),reale(2085027,317816093LL<<23),
      -reale(1725044,456624309LL<<22),reale(240877,0x8d28f00d60000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^19, polynomial in n of order 10
      -reale(58776429,1067354331LL<<20),reale(18829393,7579267909LL<<19),
      reale(10681211,592856305LL<<22),reale(16946593,1116323851LL<<19),
      -reale(14277877,2775669533LL<<20),-reale(2149268,8027942223LL<<19),
      -reale(4056423,828321103LL<<21),reale(6443828,4480734455LL<<19),
      -reale(857619,751312863LL<<20),-reale(227988,4599783267LL<<19),
      -reale(205805,0x3340b739f8000LL),reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^18, polynomial in n of order 11
      -reale(8326980,196156635LL<<21),-reale(34821146,552451591LL<<22),
      -reale(46791029,557069513LL<<21),reale(38334852,177025053LL<<24),
      reale(8937287,838991609LL<<21),reale(5317827,1033371567LL<<22),
      -reale(18378967,400717301LL<<21),reale(2844411,12754877LL<<23),
      reale(593018,1659418637LL<<21),reale(4310821,76308389LL<<22),
      -reale(2591282,1217948513LL<<21),reale(276451,0x9f1a0fb950000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^17, polynomial in n of order 12
      -reale(182057178,279202431LL<<21),reale(246730983,7282837989LL<<19),
      -reale(61609386,3656132889LL<<20),-reale(45340440,795982425LL<<19),
      -reale(20534253,134894613LL<<22),reale(51951312,243959241LL<<19),
      -reale(4823611,581671439LL<<20),-reale(5624597,8387045493LL<<19),
      -reale(13789372,989768405LL<<21),reale(9667615,6509295661LL<<19),
      reale(215496,1395596667LL<<20),-reale(184969,6596889361LL<<19),
      -reale(459826,0xaf07be5d28000LL),reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^16, polynomial in n of order 13
      reale(316814308,524172905LL<<23),-reale(186563878,63588247LL<<25),
      -reale(110274143,526845649LL<<23),reale(263023219,83035351LL<<24),
      -reale(130904637,509865531LL<<23),-reale(30397924,20206077LL<<26),
      reale(13683269,123318603LL<<23),reale(48158450,141529345LL<<24),
      -reale(26437768,420249375LL<<23),-reale(5662772,129791197LL<<25),
      -reale(2185901,350246489LL<<23),reale(9629298,177188459LL<<24),
      -reale(3949112,493038403LL<<23),reale(276643,3634960421LL<<18),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^15, polynomial in n of order 14
      reale(78761274,365004673LL<<20),-reale(201515576,8484307809LL<<19),
      reale(313194006,1602645493LL<<21),-reale(252751914,5568714583LL<<19),
      -reale(13191170,2167441005LL<<20),reale(241719441,7606152787LL<<19),
      -reale(201822768,404840345LL<<22),reale(21302083,5136432093LL<<19),
      reale(37050656,1255073061LL<<20),reale(20598069,641077127LL<<19),
      -reale(39565379,1270355289LL<<21),reale(9630032,7047419793LL<<19),
      reale(3352897,1867330359LL<<20),reale(651457,7128437307LL<<19),
      -reale(1114108,0x7475455348000LL),reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^14, polynomial in n of order 15
      reale(1503684,1762694997LL<<21),-reale(12945810,457623793LL<<22),
      reale(59109631,1997027375LL<<21),-reale(165337541,436423661LL<<23),
      reale(292248408,1310925625LL<<21),-reale(302358268,80333091LL<<22),
      reale(101329883,1625451155LL<<21),reale(168206436,256940945LL<<24),
      -reale(245462494,1698275235LL<<21),reale(106772813,575322475LL<<22),
      reale(20184277,1515722423LL<<21),-reale(15841583,115367503LL<<23),
      -reale(24343366,297803839LL<<21),reale(22619454,642864953LL<<22),
      -reale(5751128,1751200357LL<<21),reale(119914,0x778fad9290000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^13, polynomial in n of order 16
      -real(494538685723LL<<23),-reale(30244,7532247025LL<<19),
      reale(913230,2357276371LL<<20),-reale(8356271,5886749331LL<<19),
      reale(41054740,50726383LL<<21),-reale(125953300,8377060373LL<<19),
      reale(252781900,3961145001LL<<20),-reale(323011393,7392450487LL<<19),
      reale(214671336,735258085LL<<22),reale(38642307,2838366023LL<<19),
      -reale(221064383,2701482241LL<<20),reale(190191489,7753766309LL<<19),
      -reale(54203341,2021364059LL<<21),-reale(15936650,4639479773LL<<19),
      reale(7069294,1728459477LL<<20),reale(6475976,7904740225LL<<19),
      -reale(3243677,0x65d7af1058000LL),reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^12, polynomial in n of order 17
      -real(4941153649LL<<22),-real(2434362319LL<<26),
      -real(480183190319LL<<22),-reale(15428,422153761LL<<23),
      reale(492912,506448323LL<<22),-reale(4815395,177795021LL<<24),
      reale(25573504,64498885LL<<22),-reale(86374812,63633203LL<<23),
      reale(196725482,856584503LL<<22),-reale(303474922,105041487LL<<25),
      reale(296000607,1045914233LL<<22),-reale(124541003,470657541LL<<23),
      -reale(96946363,592229397LL<<22),reale(194787320,217061649LL<<24),
      -reale(139624521,484247315LL<<22),reale(48044895,435975913LL<<23),
      -reale(5082038,276187937LL<<22),-reale(749748,0x6069872020000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^11, polynomial in n of order 18
      -real(231323121LL<<20),-real(2351460757LL<<19),-real(912558841LL<<23),
      -real(0xdfda7610580000LL),-real(777314384543LL<<20),
      -reale(6590,3852961377LL<<19),reale(223861,17176789LL<<21),
      -reale(2346980,3623268951LL<<19),reale(13542533,3871010099LL<<20),
      -reale(50565862,7643343277LL<<19),reale(130735502,766383623LL<<22),
      -reale(239800507,7719641123LL<<19),reale(308448660,3395101317LL<<20),
      -reale(258446712,3844536697LL<<19),reale(99709743,227483207LL<<21),
      reale(54337873,5199140497LL<<19),-reale(105984135,215955881LL<<20),
      reale(67263140,627338299LL<<19),-reale(17110329,0x5fa94e648000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[9], coeff of eps^10, polynomial in n of order 19
      -real(538707LL<<21),-real(1075491LL<<22),-real(9728097LL<<21),
      -real(3213907LL<<25),-real(333357375LL<<21),-real(1438804621LL<<22),
      -real(39246385997LL<<21),-real(379094211993LL<<23),
      reale(25645,1674653973LL<<21),-reale(290249,472854199LL<<22),
      reale(1830100,1274307463LL<<21),-reale(7588281,99130323LL<<24),
      reale(22282256,82312105LL<<21),-reale(48025833,432719649LL<<22),
      reale(76964476,1304326427LL<<21),-reale(91125940,162742323LL<<23),
      reale(77471536,1478654845LL<<21),-reale(44556474,1023100235LL<<22),
      reale(15423395,377918063LL<<21),-reale(2409905,0x7f0a0dc2b0000LL),
      reale(25978144170LL,0x7e28f6c5ff5f1LL),
      // C4[9], coeff of eps^9, polynomial in n of order 20
      -real(16575LL<<21),-real(226005LL<<19),-real(421083LL<<20),
      -real(3487431LL<<19),-real(1025715LL<<23),-real(90604825LL<<19),
      -real(308056405LL<<20),-real(5606626571LL<<19),-real(20270111449LL<<21),
      -real(0x30ab7cf8dddLL<<19),reale(15220,1707177905LL<<20),
      -reale(187210,7636838095LL<<19),reale(1297995,534056013LL<<22),
      -reale(6003229,1506461473LL<<19),reale(20010763,3942424887LL<<20),
      -reale(50026909,6827222547LL<<19),reale(95435950,2132760845LL<<21),
      -reale(138382128,8075045605LL<<19),reale(146522254,737992253LL<<20),
      -reale(96396219,7300467927LL<<19),reale(27713913,0x34f39e3ee8000LL),
      reale(77934432511LL,0x7a7ae451fe1d3LL),
      // C4[10], coeff of eps^29, polynomial in n of order 0
      real(14059LL<<19),real(0x168a4531304537LL),
      // C4[10], coeff of eps^28, polynomial in n of order 1
      -real(1004279LL<<22),-real(3373361LL<<19),reale(3807,0xdf0925caacfb9LL),
      // C4[10], coeff of eps^27, polynomial in n of order 2
      real(78580619LL<<24),-real(212705597LL<<23),real(705875469LL<<19),
      reale(59656,0xa639fabc960fdLL),
      // C4[10], coeff of eps^26, polynomial in n of order 3
      real(927832218729LL<<21),-real(204500125453LL<<22),
      -real(29157611613LL<<21),-real(0x66c4e2e4040000LL),
      reale(23087123,0x49a60b16d9e77LL),
      // C4[10], coeff of eps^25, polynomial in n of order 4
      real(26024288967LL<<27),-real(7678900515LL<<25),real(13514191015LL<<26),
      -real(31097026337LL<<25),real(89826688809LL<<21),
      reale(25583028,0x820b055e82c23LL),
      // C4[10], coeff of eps^24, polynomial in n of order 5
      reale(1328855,126349401LL<<24),-reale(550962,13774891LL<<26),
      reale(2464835,125518543LL<<24),-reale(784466,25625323LL<<25),
      -reale(93184,68528187LL<<24),-reale(52198,1190112709LL<<21),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^23, polynomial in n of order 6
      -reale(1114607,27405733LL<<26),-reale(4563722,53821803LL<<25),
      reale(3169393,348585LL<<27),reale(68182,92955763LL<<25),
      reale(1172595,45755337LL<<26),-reale(1088988,13585007LL<<25),
      reale(166307,46143431LL<<21),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^22, polynomial in n of order 7
      reale(5480278,504127481LL<<20),-reale(9293162,155326547LL<<21),
      -reale(197247,3072475525LL<<20),-reale(1644302,932629169LL<<22),
      reale(3811061,3287215741LL<<20),-reale(747954,323686257LL<<21),
      -reale(145848,3935467265LL<<20),-reale(106662,0xb8d6e5aaa0000LL),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^21, polynomial in n of order 8
      reale(22796753,23076841LL<<25),-reale(927290,180149865LL<<22),
      -reale(369606,74581989LL<<23),-reale(9303996,552920075LL<<22),
      reale(3036817,71115369LL<<24),reale(396000,898162707LL<<22),
      reale(2181010,484753161LL<<23),-reale(1556188,389640079LL<<22),
      reale(192540,3401040927LL<<18),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^20, polynomial in n of order 9
      reale(862955,1266777839LL<<21),reale(7027949,96012647LL<<24),
      reale(20762590,1932890753LL<<21),-reale(9559408,1057130891LL<<22),
      -reale(3064719,1040728173LL<<21),-reale(5129237,23711641LL<<23),
      reale(5760893,1279205669LL<<21),-reale(394463,240514009LL<<22),
      -reale(178786,1942994377LL<<21),-reale(217080,8651652815LL<<18),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^19, polynomial in n of order 10
      -reale(10913096,468931943LL<<23),-reale(57356320,139563275LL<<22),
      reale(21632622,17971173LL<<25),reale(12352870,1067519707LL<<22),
      reale(10546968,197307279LL<<23),-reale(16476123,321986815LL<<22),
      reale(466396,220388453LL<<24),reale(183297,876676071LL<<22),
      reale(4340035,31265157LL<<23),-reale(2266775,500956659LL<<22),
      reale(210337,0xe1ea7a84c0000LL),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^18, polynomial in n of order 11
      reale(183220667,2590575043LL<<20),reale(3573393,1902991101LL<<21),
      -reale(38433982,657724943LL<<20),-reale(39892891,403988263LL<<23),
      reale(42677900,967722655LL<<20),reale(4292702,1711217099LL<<21),
      -reale(2792039,799969587LL<<20),-reale(14767346,746757159LL<<22),
      reale(7703673,1133060475LL<<20),reale(747527,637790873LL<<21),
      -reale(45502,3704918615LL<<20),-reale(458872,0xc21d355260000LL),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^17, polynomial in n of order 12
      -reale(61780842,49135749LL<<25),-reale(196091506,391376453LL<<23),
      reale(232013693,187926637LL<<24),-reale(59475550,301219495LL<<23),
      -reale(46331600,62864215LL<<26),-reale(5439903,151048009LL<<23),
      reale(49627120,102515675LL<<24),-reale(16690048,509576107LL<<23),
      -reale(7354945,21733079LL<<25),-reale(3769052,366616397LL<<23),
      reale(9145462,214930505LL<<24),-reale(3305318,371590575LL<<23),
      reale(189374,6271289399LL<<19),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^16, polynomial in n of order 13
      -reale(246951312,552772347LL<<22),reale(295555190,29721595LL<<24),
      -reale(151972143,664869293LL<<22),-reale(114414430,423169395LL<<23),
      reale(248915402,492058657LL<<22),-reale(140322148,99500631LL<<25),
      -reale(15757705,299151505LL<<22),reale(29401843,368167099LL<<23),
      reale(29725213,39057725LL<<22),-reale(34936824,2445655LL<<24),
      reale(5290998,483472011LL<<22),reale(3412892,270211305LL<<23),
      reale(939828,308185177LL<<22),-reale(1058440,2262901433LL<<19),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^15, polynomial in n of order 14
      -reale(29237793,21929809LL<<24),reale(96597143,85827693LL<<23),
      -reale(210653294,75090197LL<<25),reale(297719766,264531499LL<<23),
      -reale(232859751,332419LL<<24),reale(779198,466262825LL<<23),
      reale(210565738,49075321LL<<26),-reale(210231291,35636249LL<<23),
      reale(60435795,147172683LL<<24),reale(30790678,95503589LL<<23),
      -reale(8341137,27440903LL<<25),-reale(25891285,147600733LL<<23),
      reale(19770912,119140889LL<<24),-reale(4475853,348372575LL<<23),
      reale(24304,4909664935LL<<19),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^14, polynomial in n of order 15
      -reale(326980,1465789373LL<<20),reale(3379554,1566468779LL<<21),
      -reale(19029758,226591575LL<<20),reale(68313947,940737655LL<<22),
      -reale(165836985,3033756273LL<<20),reale(273579872,472941105LL<<21),
      -reale(286670501,2169744907LL<<20),reale(131674848,489609853LL<<23),
      reale(102478771,1504412891LL<<20),-reale(220713363,225877065LL<<21),
      reale(153302553,1201451329LL<<20),-reale(29968476,1046042243LL<<22),
      -reale(18498599,2914872793LL<<20),reale(4637884,270502717LL<<21),
      reale(6604171,2668065421LL<<20),-reale(2936921,0x8973648be0000LL),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^13, polynomial in n of order 16
      real(8181919521LL<<26),reale(4651,463423847LL<<22),
      -reale(165627,528682553LL<<23),reale(1821092,755660373LL<<22),
      -reale(11021646,91392285LL<<24),reale(43145010,992272131LL<<22),
      -reale(116748177,310953403LL<<23),reale(223068773,47271409LL<<22),
      -reale(294932999,99296991LL<<25),reale(242263024,286305695LL<<22),
      -reale(60276785,30271037LL<<23),-reale(125363778,717272947LL<<22),
      reale(182026841,37372833LL<<24),-reale(116787755,417593029LL<<22),
      reale(36759949,346012225LL<<23),-reale(3061422,962217431LL<<22),
      -reale(731281,0xaaf6b13240000LL),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^12, polynomial in n of order 17
      real(777809483LL<<21),real(436668683LL<<25),real(99139014933LL<<21),
      real(0x1cfc4bfd58dLL<<22),-reale(70023,1623299233LL<<21),
      reale(822756,446933025LL<<23),-reale(5376526,2111656919LL<<21),
      reale(23042396,297910775LL<<22),-reale(69630161,297782669LL<<21),
      reale(153266731,112309515LL<<24),-reale(247130955,1577554627LL<<21),
      reale(284460705,580739169LL<<22),-reale(212091093,1801700473LL<<21),
      reale(61297082,319619083LL<<23),reale(65462218,2096893009LL<<21),
      -reale(98426842,1047683893LL<<22),reale(59152561,1235484315LL<<21),
      -reale(14780753,0xb5d74354c0000LL),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^11, polynomial in n of order 18
      real(1233981LL<<23),real(14104237LL<<22),real(6201077LL<<26),
      real(933195507LL<<22),real(6966040851LL<<23),real(592370721657LL<<22),
      -reale(22184,189431713LL<<24),reale(279989,474035391LL<<22),
      -reale(1985627,52832535LL<<23),reale(9357698,635528005LL<<22),
      -reale(31645438,92987147LL<<25),reale(79909927,996806539LL<<22),
      -reale(153562851,494252097LL<<23),reale(225351987,543734289LL<<22),
      -reale(249473559,252214923LL<<24),reale(201676475,478217047LL<<22),
      -reale(111804841,353712747LL<<23),reale(37701632,700509149LL<<22),
      -reale(5783773,3281237837LL<<18),reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[10], coeff of eps^10, polynomial in n of order 19
      real(57057LL<<20),real(126819LL<<21),real(1284843LL<<20),
      real(478667LL<<24),real(56414325LL<<20),real(279062861LL<<21),
      real(8810413183LL<<20),real(99625441377LL<<22),
      -reale(3997,1800115191LL<<20),reale(54510,559965495LL<<21),
      -reale(421909,1796318189LL<<20),reale(2196607,410595787LL<<23),
      -reale(8328804,1896277603LL<<20),reale(24012916,1506461473LL<<21),
      -reale(53875133,2685050457LL<<20),reale(94820235,193579723LL<<22),
      -reale(129680615,3269639887LL<<20),reale(131955714,608596747LL<<21),
      -reale(84828673,2009615045LL<<20),reale(24099054,0xf664899ae0000LL),
      reale(86138056986LL,0x5ef39e09c8055LL),
      // C4[11], coeff of eps^29, polynomial in n of order 0
      -real(255169LL<<19),real(0xbdc79d6e266b55fLL),
      // C4[11], coeff of eps^28, polynomial in n of order 1
      -real(535829LL<<26),real(6461547LL<<20),real(0x56e2cdab4666fea1LL),
      // C4[11], coeff of eps^27, polynomial in n of order 2
      -real(54075943LL<<25),-real(11012147LL<<24),-real(184884229LL<<19),
      reale(65338,0x3c271ece8bf8fLL),
      // C4[11], coeff of eps^26, polynomial in n of order 3
      -real(29189823LL<<30),real(157366885LL<<32),-real(637753597LL<<30),
      real(13332470307LL<<23),reale(19666808,0xb9ff38da93b23LL),
      // C4[11], coeff of eps^25, polynomial in n of order 4
      -reale(768828,16543417LL<<28),reale(2405043,41201001LL<<26),
      -reale(595679,17511625LL<<27),-reale(94147,42169149LL<<26),
      -reale(60455,661597895LL<<21),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^24, polynomial in n of order 5
      -reale(5042070,2793567LL<<28),reale(2454743,3154771LL<<30),
      reale(191058,8757223LL<<28),reale(1233521,5992667LL<<29),
      -reale(1001395,9125891LL<<28),reale(137539,51052897LL<<22),
      reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^23, polynomial in n of order 6
      -reale(7620321,6390387LL<<27),-reale(1118882,49853273LL<<26),
      -reale(2175696,13938625LL<<28),reale(3557797,45648113LL<<26),
      -reale(479218,13589073LL<<27),-reale(128928,23280229LL<<26),
      -reale(115227,1709406351LL<<21),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^22, polynomial in n of order 7
      reale(3030693,3978133LL<<30),reale(1618004,874507LL<<32),
      -reale(9217736,134985LL<<30),reale(1767041,97063LL<<33),
      reale(345531,3069873LL<<30),reale(2240563,263433LL<<32),
      -reale(1400217,895853LL<<30),reale(154222,296573467LL<<23),
      reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^21, polynomial in n of order 8
      reale(304504,49998275LL<<26),reale(21950325,110791689LL<<23),
      -reale(5005332,65785063LL<<24),-reale(3038456,102319349LL<<23),
      -reale(5977063,34913149LL<<25),reale(5022754,485487661LL<<23),
      -reale(45293,7681997LL<<24),-reale(123970,179449809LL<<23),
      -reale(222792,7672407751LL<<18),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^20, polynomial in n of order 9
      -reale(56432361,120691497LL<<25),reale(6246807,9955417LL<<28),
      reale(11410351,83254233LL<<25),reale(14692579,49664915LL<<26),
      -reale(13833928,124401461LL<<25),-reale(1245123,9835207LL<<27),
      -reale(339985,114545011LL<<25),reale(4291293,22713457LL<<26),
      -reale(1980685,98627585LL<<25),reale(159775,7030690975LL<<19),
      reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^19, polynomial in n of order 10
      reale(40826627,234278683LL<<24),-reale(19124677,161584861LL<<23),
      -reale(51024100,50981393LL<<26),reale(30646271,384869645LL<<23),
      reale(9815197,6859997LL<<24),reale(639236,244693975LL<<23),
      -reale(14951689,12090449LL<<25),reale(5916250,151054657LL<<23),
      reale(1073676,226322463LL<<24),reale(80953,380993803LL<<23),
      -reale(451111,0xff79096c0000LL),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^18, polynomial in n of order 11
      -reale(233788807,3535193LL<<28),reale(177892093,3762413LL<<30),
      -reale(5486729,8981851LL<<28),-reale(45008759,222901LL<<32),
      -reale(22295157,5977845LL<<28),reale(46499690,3347131LL<<30),
      -reale(8381947,991351LL<<28),-reale(7636513,1723229LL<<31),
      -reale(5108235,6476849LL<<28),reale(8568922,940153LL<<30),
      -reale(2769555,11314291LL<<28),reale(126172,1159668425LL<<21),
      reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^17, polynomial in n of order 12
      reale(246666787,23370677LL<<26),-reale(50685638,204720607LL<<24),
      -reale(179803952,51059789LL<<25),reale(226753224,100010811LL<<24),
      -reale(83690537,703961LL<<27),-reale(36110826,15584139LL<<24),
      reale(17880019,26951173LL<<25),reale(35367319,73259919LL<<24),
      -reale(29730133,51577369LL<<26),reale(2029349,105583177LL<<24),
      reale(3224077,120316375LL<<25),reale(1158582,83501667LL<<24),
      -reale(999784,6146420159LL<<19),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^16, polynomial in n of order 13
      reale(135472555,2919565LL<<26),-reale(240891001,9823619LL<<28),
      reale(277187915,48314475LL<<26),-reale(153860890,22130685LL<<27),
      -reale(78071452,50966567LL<<26),reale(224257271,6520287LL<<29),
      -reale(168898235,23643785LL<<26),reale(25365615,30758325LL<<27),
      reale(33991594,22223845LL<<26),-reale(1325267,13358465LL<<28),
      -reale(26274549,1352253LL<<26),reale(17195323,12709799LL<<27),
      -reale(3493469,55138895LL<<26),-reale(37171,2996514251LL<<20),
      reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^15, polynomial in n of order 14
      reale(8369149,65829073LL<<25),-reale(34468304,77612041LL<<24),
      reale(98238486,50466661LL<<26),-reale(197855127,257258223LL<<24),
      reale(275634083,126808451LL<<25),-reale(237329411,109823349LL<<24),
      reale(57709083,378039LL<<27),reale(144028485,10992165LL<<24),
      -reale(208082193,86131531LL<<25),reale(120116321,182851999LL<<24),
      -reale(12733337,27687817LL<<26),-reale(18886416,178008391LL<<24),
      reale(2557866,23705959LL<<25),reale(6572718,173475635LL<<24),
      -reale(2666354,4022017967LL<<19),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^14, polynomial in n of order 15
      reale(54399,8224347LL<<28),-reale(665621,1410497LL<<30),
      reale(4527846,10561865LL<<28),-reale(20181039,1593975LL<<31),
      reale(63299017,4532479LL<<28),-reale(144047710,3737571LL<<30),
      reale(238046961,3527085LL<<28),-reale(274611219,485845LL<<32),
      reale(189061064,3080067LL<<28),-reale(9459768,127989LL<<30),
      -reale(141083601,3627343LL<<28),reale(166868825,1278147LL<<31),
      -reale(97711641,16702617LL<<28),reale(28303864,4120681LL<<30),
      -reale(1707991,14300715LL<<28),-reale(691965,964491519LL<<21),
      reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^13, polynomial in n of order 16
      -real(394848061LL<<27),-real(277855615551LL<<23),
      reale(21507,221049445LL<<24),-reale(280152,15918397LL<<23),
      reale(2046623,76965257LL<<25),-reale(9908745,30179611LL<<23),
      reale(34287090,9035647LL<<24),-reale(88042794,304571673LL<<23),
      reale(170266683,41403331LL<<26),-reale(246546803,514564215LL<<23),
      reale(257558635,22204697LL<<24),-reale(171596509,74164853LL<<23),
      reale(32098211,100286083LL<<25),reale(71621283,185724333LL<<23),
      -reale(91026815,208301517LL<<24),reale(52421624,501338287LL<<23),
      -reale(12919309,7987587551LL<<18),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^12, polynomial in n of order 17
      -real(2506701LL<<25),-real(1595211LL<<29),-real(414133331LL<<25),
      -real(9611154693LL<<26),reale(6318,129560535LL<<25),
      -reale(88018,7994433LL<<27),reale(693686,99032081LL<<25),
      -reale(3663195,37640223LL<<26),reale(14022738,83451835LL<<25),
      -reale(40598902,6803915LL<<28),reale(90961965,119128629LL<<25),
      -reale(159220781,788729LL<<26),reale(217217490,112380639LL<<25),
      -reale(227284915,26366059LL<<27),reale(176075660,9208089LL<<25),
      -reale(94610548,45670931LL<<26),reale(31201351,21556291LL<<25),
      -reale(4712704,700509149LL<<19),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[11], coeff of eps^11, polynomial in n of order 18
      -real(13041LL<<24),-real(166957LL<<23),-real(82777LL<<27),
      -real(14154867LL<<23),-real(121102751LL<<24),-real(11919970777LL<<23),
      real(140288886837LL<<25),-reale(15649,252654239LL<<23),
      reale(133731,225409843LL<<24),-reale(773734,115698949LL<<23),
      reale(3285982,23309223LL<<26),-reale(10716783,181102411LL<<23),
      reale(27557442,232845957LL<<24),-reale(56645854,420384689LL<<23),
      reale(93299054,125728615LL<<25),-reale(121534295,132369527LL<<23),
      reale(119605179,120525655LL<<24),-reale(75403265,163638237LL<<23),
      reale(21207168,6304582341LL<<18),reale(94341681461LL,0x436c57c191ed7LL),
      // C4[12], coeff of eps^29, polynomial in n of order 0
      real(2113LL<<23),real(0x495846bc80a035LL),
      // C4[12], coeff of eps^28, polynomial in n of order 1
      -real(5059597LL<<25),-real(23775299LL<<22),
      reale(61953,0x75e619a89ce07LL),
      // C4[12], coeff of eps^27, polynomial in n of order 2
      real(30823201LL<<29),-real(55301563LL<<28),real(131431881LL<<24),
      reale(497138,0xbe8dd4238d2e7LL),
      // C4[12], coeff of eps^26, polynomial in n of order 3
      real(8059635627LL<<28),-real(757042391LL<<29),-real(311216327LL<<28),
      -real(7273579LL<<33),reale(21376966,0x1d2a1f8b6ccdLL),
      // C4[12], coeff of eps^25, polynomial in n of order 4
      reale(590308,751003LL<<30),reale(77521,16047653LL<<28),
      reale(426657,125003LL<<29),-reale(306166,5244457LL<<28),
      reale(37995,207060411LL<<24),reale(34181768645LL,0x62a1b07dc9473LL),
      // C4[12], coeff of eps^24, polynomial in n of order 5
      -reale(1599658,2394579LL<<27),-reale(2671318,5123391LL<<29),
      reale(3256460,16377243LL<<27),-reale(261261,3982303LL<<28),
      -reale(106562,1204279LL<<27),-reale(120793,1029973LL<<24),
      reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^23, polynomial in n of order 6
      reale(1248773,4542469LL<<29),-reale(2889741,9813249LL<<28),
      reale(234885,3135591LL<<30),reale(66922,9908313LL<<28),
      reale(755418,635863LL<<29),-reale(419245,13200621LL<<28),
      reale(41213,192739239LL<<24),reale(34181768645LL,0x62a1b07dc9473LL),
      // C4[12], coeff of eps^22, polynomial in n of order 7
      reale(20885911,4938503LL<<28),-reale(1107830,1234733LL<<29),
      -reale(2370377,3771643LL<<28),-reale(6561451,624527LL<<30),
      reale(4279851,10797315LL<<28),reale(210660,3761905LL<<29),
      -reale(68724,2033407LL<<28),-reale(224555,1152577LL<<29),
      reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^21, polynomial in n of order 8
      -reale(5562062,38325LL<<31),reale(7741765,4470897LL<<28),
      reale(17399328,6149297LL<<29),-reale(10890962,10744893LL<<28),
      -reale(2368414,446197LL<<30),-reale(891562,9146315LL<<28),
      reale(4183586,393403LL<<29),-reale(1730085,6072185LL<<28),
      reale(120797,18742483LL<<24),reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^20, polynomial in n of order 9
      reale(3332722,179104103LL<<24),-reale(54245156,21887465LL<<27),
      reale(18428910,34326153LL<<24),reale(12293411,106053413LL<<25),
      reale(4024929,78680811LL<<24),-reale(14528270,1262953LL<<26),
      reale(4350569,36952333LL<<24),reale(1251271,92345015LL<<25),
      reale(191236,5049199LL<<24),-reale(439124,1983321823LL<<21),
      reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^19, polynomial in n of order 10
      reale(117817828,9756529LL<<27),reale(29304818,21859033LL<<26),
      -reale(33857646,3348243LL<<29),-reale(34756825,30241097LL<<26),
      reale(40576649,11012471LL<<27),-reale(1809964,37385419LL<<26),
      -reale(7014724,9945043LL<<28),-reale(6163099,62399597LL<<26),
      reale(7950550,2557949LL<<27),-reale(2323999,3159279LL<<26),
      reale(80031,1030811061LL<<22),reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^18, polynomial in n of order 11
      reale(37677439,43610729LL<<26),-reale(212417438,31724009LL<<27),
      reale(188774384,60946099LL<<26),-reale(37276694,6182933LL<<29),
      -reale(44097735,31570179LL<<26),reale(5696664,10497345LL<<27),
      reale(38054298,7154503LL<<26),-reale(24525159,12952085LL<<28),
      -reale(350073,57822319LL<<26),reale(2901654,18012267LL<<27),
      reale(1319354,63457243LL<<26),-reale(941373,59576227LL<<26),
      reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^17, polynomial in n of order 12
      -reale(253394431,1462439LL<<28),reale(237669216,409221LL<<26),
      -reale(76296320,28335185LL<<27),-reale(133872864,17217849LL<<26),
      reale(218174046,2622387LL<<29),-reale(127998192,57914967LL<<26),
      reale(363472,30718057LL<<27),reale(32681168,4189163LL<<26),
      reale(4677048,16088179LL<<28),-reale(25857930,7142835LL<<26),
      reale(14914891,9312419LL<<27),-reale(2731797,52301425LL<<26),
      -reale(76352,726189181LL<<22),reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^16, polynomial in n of order 13
      -reale(53643409,97684423LL<<25),reale(127408278,3174879LL<<27),
      -reale(219370358,126672641LL<<25),reale(262176902,49024297LL<<26),
      -reale(182579118,132254331LL<<25),-reale(3956056,15289739LL<<28),
      reale(167880537,57011019LL<<25),-reale(188895775,46555265LL<<26),
      reale(91621970,25449425LL<<25),-reale(762367,26232331LL<<27),
      -reale(18049661,12932969LL<<25),reale(839158,10679509LL<<26),
      reale(6440415,29973277LL<<25),-reale(2428550,982597961LL<<22),
      reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^15, polynomial in n of order 14
      -reale(1786573,13634499LL<<27),reale(8939902,21088027LL<<26),
      -reale(31907799,6174271LL<<28),reale(84216248,4944333LL<<26),
      -reale(166330228,11364729LL<<27),reale(242706491,8863071LL<<26),
      -reale(246937724,7698133LL<<29),reale(139651898,50060433LL<<26),
      reale(29493809,19297617LL<<27),-reale(148014628,18656733LL<<26),
      reale(151165884,622571LL<<28),-reale(81883041,55488299LL<<26),
      reale(21903841,15379355LL<<27),-reale(792996,14574745LL<<26),
      -reale(644309,457907141LL<<22),reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^14, polynomial in n of order 15
      -reale(6504,28793619LL<<26),reale(93075,33271365LL<<27),
      -reale(752118,6462873LL<<26),reale(4061558,11832697LL<<28),
      -reale(15840997,35193887LL<<26),reale(46482714,19239327LL<<27),
      -reale(104709924,13039269LL<<26),reale(181860520,7828435LL<<29),
      -reale(240159499,36173099LL<<26),reale(229992094,10037497LL<<27),
      -reale(136854274,43911089LL<<26),reale(9990763,2550227LL<<28),
      reale(74520212,5689801LL<<26),-reale(84057599,709549LL<<27),
      reale(46786776,54603587LL<<26),-reale(11406945,39298831LL<<26),
      reale(102545305936LL,0x27e511795bd59LL),
      // C4[12], coeff of eps^13, polynomial in n of order 16
      real(1030055LL<<30),real(829418525LL<<26),-real(19924010015LL<<27),
      reale(9050,10804695LL<<26),-reale(78488,9283787LL<<28),
      reale(459151,22444081LL<<26),-reale(1962716,17985613LL<<27),
      reale(6408338,7820523LL<<26),-reale(16395176,1626457LL<<29),
      reale(33311385,35536325LL<<26),-reale(53946341,7054843LL<<27),
      reale(69201696,61410431LL<<26),-reale(69012103,3773337LL<<28),
      reale(51544754,7834329LL<<26),-reale(26961871,12889641LL<<27),
      reale(8722958,26104467LL<<26),-reale(1300056,320360129LL<<22),
      reale(34181768645LL,0x62a1b07dc9473LL),
      // C4[12], coeff of eps^12, polynomial in n of order 17
      real(127075LL<<24),real(91195LL<<28),real(26902525LL<<24),
      real(715607165LL<<25),-real(73094160425LL<<24),
      reale(4440,35913265LL<<26),-reale(41519,978831LL<<24),
      reale(264211,112928967LL<<25),-reale(1241795,175695285LL<<24),
      reale(4515620,18853435LL<<27),-reale(13069247,261014043LL<<24),
      reale(30619380,129358865LL<<25),-reale(58537051,226171969LL<<24),
      reale(91194564,65482939LL<<26),-reale(113993206,58979239LL<<24),
      reale(109036979,115740763LL<<25),-reale(67602927,138149837LL<<24),
      reale(18850816,700509149LL<<21),reale(102545305936LL,0x27e511795bd59LL),
      // C4[13], coeff of eps^29, polynomial in n of order 0
      -real(634219LL<<23),reale(3193,0x402148867236bLL),
      // C4[13], coeff of eps^28, polynomial in n of order 1
      -real(400561LL<<32),real(1739049LL<<27),reale(66909,0xbcc54ee94d445LL),
      // C4[13], coeff of eps^27, polynomial in n of order 2
      -real(6387996953LL<<29),-real(3461245957LL<<28),-real(49206438547LL<<24),
      reale(286172946,0xcc6f5fc7e64c9LL),
      // C4[13], coeff of eps^26, polynomial in n of order 3
      real(7296571113LL<<30),reale(10661,1488313LL<<31),
      -reale(6836,2507629LL<<30),real(103233906747LL<<25),
      reale(900397808,0x384bb07b32421LL),
      // C4[13], coeff of eps^25, polynomial in n of order 4
      -reale(1030602,1434287LL<<30),reale(976249,6303335LL<<28),
      -reale(29214,7243007LL<<29),-reale(27193,4360723LL<<28),
      -reale(41363,170006437LL<<24),reale(36916310137LL,0x41f43bb0c949LL),
      // C4[13], coeff of eps^24, polynomial in n of order 5
      -reale(2597630,109963LL<<31),-reale(46366,88065LL<<33),
      real(835763379LL<<31),reale(754229,667751LL<<32),
      -reale(376195,1000319LL<<31),reale(33027,62908623LL<<26),
      reale(36916310137LL,0x41f43bb0c949LL),
      // C4[13], coeff of eps^23, polynomial in n of order 6
      reale(1907767,7512493LL<<29),-reale(1322539,10099505LL<<28),
      -reale(6889396,2784065LL<<30),reale(3566231,12064393LL<<28),
      reale(392016,1668111LL<<29),-reale(16024,9677725LL<<28),
      -reale(223530,46890859LL<<24),reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^22, polynomial in n of order 7
      reale(2768819,1533979LL<<30),reale(18682895,1051157LL<<31),
      -reale(7962826,2061455LL<<30),-reale(3008388,501585LL<<32),
      -reale(1419492,411945LL<<30),reale(4033867,1208903LL<<31),
      -reale(1511425,98131LL<<30),reale(90538,115806565LL<<25),
      reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^21, polynomial in n of order 8
      -reale(51332124,214119LL<<31),reale(7579765,3153587LL<<28),
      reale(12475441,1655707LL<<29),reale(7005177,5256105LL<<28),
      -reale(13679833,119207LL<<30),reale(3016909,4530623LL<<28),
      reale(1323928,2024201LL<<29),reale(284896,6925237LL<<28),
      -reale(424636,138679005LL<<24),reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^20, polynomial in n of order 9
      reale(47250711,14679LL<<32),-reale(18472981,62575LL<<35),
      -reale(42459575,893863LL<<32),reale(33307537,106651LL<<33),
      reale(3060800,842635LL<<32),-reale(5867540,102671LL<<34),
      -reale(6941741,849331LL<<32),reale(7324844,423881LL<<33),
      -reale(1953157,771073LL<<32),reale(46148,28524089LL<<27),
      reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^19, polynomial in n of order 10
      -reale(218954922,27801141LL<<27),reale(144947317,36456347LL<<26),
      -reale(2491117,129025LL<<29),-reale(43746541,53566187LL<<26),
      -reale(5414513,33128531LL<<27),reale(38475112,39418671LL<<26),
      -reale(19653214,3660993LL<<28),-reale(2031714,8235735LL<<26),
      reale(2517568,31068687LL<<27),reale(1433299,8158787LL<<26),
      -reale(884985,911850811LL<<22),reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^18, polynomial in n of order 11
      reale(186784429,10521821LL<<28),-reale(7066121,6171767LL<<29),
      -reale(168709085,159265LL<<28),reale(199772613,1997709LL<<31),
      -reale(91000398,5796399LL<<28),-reale(16385586,3500385LL<<29),
      reale(28845005,1198547LL<<28),reale(9523912,3994797LL<<30),
      -reale(24921512,7172347LL<<28),reale(12920997,2065141LL<<29),
      -reale(2137442,11262329LL<<28),-reale(100773,90157665LL<<23),
      reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^17, polynomial in n of order 12
      reale(153014747,10291963LL<<28),-reale(229746959,27043849LL<<26),
      reale(237324258,14238909LL<<27),-reale(127910449,9268979LL<<26),
      -reale(52661288,2811671LL<<29),reale(178449477,48064707LL<<26),
      -reale(166899831,11458293LL<<27),reale(67870692,24951769LL<<26),
      reale(7326612,206249LL<<28),-reale(16569622,39442673LL<<26),
      -reale(550002,21806887LL<<27),reale(6246699,62490405LL<<26),
      -reale(2219585,747027389LL<<22),reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^16, polynomial in n of order 13
      reale(15145062,3114639LL<<29),-reale(45473383,71569LL<<31),
      reale(104280194,4043033LL<<29),-reale(182691434,3191599LL<<30),
      reale(238813543,4302931LL<<29),-reale(215430056,686779LL<<32),
      reale(95643898,4170781LL<<29),reale(58502156,871831LL<<30),
      -reale(149008930,6169513LL<<29),reale(135928257,1117477LL<<31),
      -reale(68778720,227103LL<<29),reale(17013972,3743517LL<<30),
      -reale(171458,5381733LL<<29),-reale(594747,43724235LL<<24),
      reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^15, polynomial in n of order 14
      reale(268265,12727175LL<<27),-reale(1599130,17232215LL<<26),
      reale(6942204,4883571LL<<28),-reale(22914299,20494129LL<<26),
      reale(58880733,8011269LL<<27),-reale(118985431,7979051LL<<26),
      reale(188592645,4054545LL<<29),-reale(229762161,35295621LL<<26),
      reale(203148724,31300867LL<<27),-reale(107385077,53637247LL<<26),
      -reale(6680614,2406191LL<<28),reale(75281884,8527271LL<<26),
      -reale(77627899,32310399LL<<27),reale(42028799,34263981LL<<26),
      -reale(10160264,35032709LL<<22),reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^14, polynomial in n of order 15
      real(8350913025LL<<28),-reale(8241,2495877LL<<29),
      reale(78008,13111987LL<<28),-reale(500800,4045265LL<<30),
      reale(2364509,9424021LL<<28),-reale(8593646,4773407LL<<29),
      reale(24709323,11418567LL<<28),-reale(57114100,2066875LL<<31),
      reale(106928260,4889705LL<<28),-reale(162113251,5033977LL<<29),
      reale(197269922,8596123LL<<28),-reale(188736396,4070811LL<<30),
      reale(136566807,16720509LL<<28),-reale(69783667,938643LL<<29),
      reale(22203894,1359919LL<<28),-reale(3271109,212531129LL<<23),
      reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[13], coeff of eps^13, polynomial in n of order 16
      -real(94185LL<<30),-real(86179275LL<<26),real(2372802705LL<<27),
      -real(83726038305LL<<26),reale(12668,1555717LL<<28),
      -reale(87922,39994007LL<<26),reale(452934,19637187LL<<27),
      -reale(1815855,62281965LL<<26),reale(5835571,1574167LL<<29),
      -reale(15318374,24668899LL<<26),reale(33211265,14617205LL<<27),
      -reale(59722012,27257657LL<<26),reale(88729847,57943LL<<28),
      -reale(107054489,21433519LL<<26),reale(99917523,12239271LL<<27),
      -reale(61060708,48513541LL<<26),reale(16900731,943456205LL<<22),
      reale(110748930411LL,0xc5dcb3125bdbLL),
      // C4[14], coeff of eps^29, polynomial in n of order 0
      real(41LL<<28),real(0x3fbc634a12a6b1LL),
      // C4[14], coeff of eps^28, polynomial in n of order 1
      -real(6907093LL<<31),-real(59887787LL<<28),
      reale(5739014,0x909af11944e4bLL),
      // C4[14], coeff of eps^27, polynomial in n of order 2
      reale(3432,499601LL<<33),-real(2083199471LL<<32),real(3406572267LL<<28),
      reale(307370942,0xdb94118adae9fLL),
      // C4[14], coeff of eps^26, polynomial in n of order 3
      reale(287986,4314073LL<<29),reale(5344,3636147LL<<30),
      -reale(6205,2906637LL<<29),-reale(13964,12467885LL<<26),
      reale(13216950542LL,0xe1def252c54b5LL),
      // C4[14], coeff of eps^25, polynomial in n of order 4
      -reale(258061,515595LL<<33),-reale(74790,1657665LL<<31),
      reale(745027,493173LL<<32),-reale(337382,84843LL<<31),
      reale(26418,5099583LL<<27),reale(39650851628LL,0xa59cd6f84fe1fLL),
      // C4[14], coeff of eps^24, polynomial in n of order 5
      -reale(100052,3082133LL<<30),-reale(6991386,428305LL<<32),
      reale(2902871,3549453LL<<30),reale(514674,1320943LL<<31),
      reale(32543,4070319LL<<30),-reale(220557,2292103LL<<27),
      reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^23, polynomial in n of order 6
      reale(6249633,975799LL<<32),-reale(1750517,1286063LL<<31),
      -reale(1090661,209219LL<<33),-reale(631632,1802089LL<<31),
      reale(1285387,761149LL<<32),-reale(440347,2020899LL<<31),
      reale(22303,14762615LL<<27),reale(39650851628LL,0xa59cd6f84fe1fLL),
      // C4[14], coeff of eps^22, polynomial in n of order 7
      -reale(1155507,7367607LL<<29),reale(11090657,3295693LL<<30),
      reale(9416360,3921899LL<<29),-reale(12562252,1614097LL<<31),
      reale(1905484,7990669LL<<29),reale(1324142,2135535LL<<30),
      reale(362851,6226223LL<<29),-reale(408795,45924241LL<<26),
      reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^21, polynomial in n of order 8
      -reale(2556392,83451LL<<35),-reale(45924405,628445LL<<32),
      reale(25722566,76303LL<<33),reale(6427311,738073LL<<32),
      -reale(4460754,79355LL<<34),-reale(7474566,842481LL<<32),
      reale(6714086,421381LL<<33),-reale(1643964,835835LL<<32),
      reale(21175,2825543LL<<28),reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^20, polynomial in n of order 9
      reale(101794762,1213867LL<<31),reale(21384252,53331LL<<34),
      -reale(38294895,814203LL<<31),-reale(14666563,397319LL<<32),
      reale(37283642,1395551LL<<31),-reale(15279397,125869LL<<33),
      -reale(3174330,433863LL<<31),reale(2115734,458067LL<<32),
      reale(1510128,1624339LL<<31),-reale(831539,10478291LL<<28),
      reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^19, polynomial in n of order 10
      reale(50295468,469581LL<<33),-reale(186185623,531407LL<<32),
      reale(174713425,41641LL<<35),-reale(59412258,3489LL<<32),
      -reale(26728127,294149LL<<33),reale(23787374,31373LL<<32),
      reale(13262792,54953LL<<34),-reale(23669724,520005LL<<32),
      reale(11190603,172969LL<<33),-reale(1670792,243479LL<<32),
      -reale(115324,7271069LL<<28),reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^18, polynomial in n of order 11
      -reale(229751836,26450113LL<<27),reale(205122059,12799441LL<<28),
      -reale(76881886,7573243LL<<27),-reale(89213757,3943587LL<<30),
      reale(179505441,31406283LL<<27),-reale(144430080,11541225LL<<28),
      reale(48475411,26026641LL<<27),reale(12591449,3614557LL<<29),
      -reale(14798010,26226089LL<<27),-reale(1654995,15989667LL<<28),
      reale(6017860,18394141LL<<27),-reale(2035659,55899187LL<<24),
      reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^17, polynomial in n of order 12
      -reale(60003627,419631LL<<31),reale(122260158,890121LL<<29),
      -reale(193015194,2586489LL<<30),reale(228332901,6912147LL<<29),
      -reale(182676146,109669LL<<32),reale(57596630,2823965LL<<29),
      reale(79437172,3055697LL<<30),-reale(146103578,5035353LL<<29),
      reale(121669517,1691035LL<<31),-reale(57926620,1249999LL<<29),
      reale(13245099,2677787LL<<30),reale(250593,3348667LL<<29),
      -reale(546524,56364575LL<<25),reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^16, polynomial in n of order 13
      -reale(2910026,15317989LL<<28),reale(10671028,169493LL<<30),
      -reale(30788418,3245427LL<<28),reale(70862414,261923LL<<29),
      -reale(130581700,1010945LL<<28),reale(191189814,642567LL<<31),
      -reale(216782974,13106831LL<<28),reale(177827671,7710997LL<<29),
      -reale(82572754,6587933LL<<28),-reale(19188450,2518521LL<<30),
      reale(74652545,11169877LL<<28),-reale(71762036,443641LL<<29),
      reale(37978089,1743047LL<<28),-reale(9119456,66783679LL<<25),
      reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^15, polynomial in n of order 14
      -reale(25313,471763LL<<30),reale(176943,4508751LL<<29),
      -reale(914488,1483519LL<<31),reale(3661023,8037561LL<<29),
      -reale(11683077,3602217LL<<30),reale(30253421,7276067LL<<29),
      -reale(64215578,725077LL<<32),reale(112133608,2030221LL<<29),
      -reale(160624032,1744767LL<<30),reale(186713478,2165751LL<<29),
      -reale(172286017,2016853LL<<31),reale(121247637,6964321LL<<29),
      -reale(60696359,459733LL<<30),reale(19031909,1781195LL<<29),
      -reale(2775486,102023215LL<<25),reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[14], coeff of eps^14, polynomial in n of order 15
      -real(614557125LL<<27),real(5831464275LL<<28),
      -reale(3808,17097199LL<<27),reale(28626,5026047LL<<29),
      -reale(160361,32462873LL<<27),reale(702411,15495849LL<<28),
      -reale(2480054,13665347LL<<27),reale(7201343,703573LL<<30),
      -reale(17420896,11395693LL<<27),reale(35365729,6899711LL<<28),
      -reale(60346284,10497687LL<<27),reale(86059048,7827541LL<<29),
      -reale(100689087,8447169LL<<27),reale(91987561,3237205LL<<28),
      -reale(55509735,6799595LL<<27),reale(15265177,48513541LL<<24),
      reale(118952554885LL,0xf0d684e8efa5dLL),
      // C4[15], coeff of eps^29, polynomial in n of order 0
      -real(204761LL<<28),reale(20426,0xaa7b82b97d24fLL),
      // C4[15], coeff of eps^28, polynomial in n of order 1
      -real(34699LL<<42),real(26415501LL<<29),reale(6134808,0xac3bb24726559LL),
      // C4[15], coeff of eps^27, polynomial in n of order 2
      reale(16894,439LL<<40),-reale(3396,5539LL<<38),
      -reale(13997,7293149LL<<28),reale(14128464373LL,0x6d08ce11dbba7LL),
      // C4[15], coeff of eps^26, polynomial in n of order 3
      -reale(50643,63489LL<<36),reale(243167,8553LL<<37),
      -reale(100839,3467LL<<36),reale(7018,548741LL<<30),
      reale(14128464373LL,0x6d08ce11dbba7LL),
      // C4[15], coeff of eps^25, polynomial in n of order 4
      -reale(6907413,21379LL<<36),reale(2301071,198931LL<<34),
      reale(591806,32973LL<<35),reale(76262,38289LL<<34),
      -reale(216244,23833777LL<<27),reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^24, polynomial in n of order 5
      -reale(2869395,52521LL<<36),-reale(3255913,8819LL<<38),
      -reale(2304160,33823LL<<36),reale(3661540,28261LL<<37),
      -reale(1155441,18213LL<<36),reale(48366,13607837LL<<28),
      reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^23, polynomial in n of order 6
      reale(8754539,110435LL<<35),reale(11218727,249609LL<<34),
      -reale(11298141,31087LL<<36),reale(996220,194783LL<<34),
      reale(1275763,41633LL<<35),reale(426630,95573LL<<34),
      -reale(392368,19533817LL<<27),reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^22, polynomial in n of order 7
      -reale(46020147,1607LL<<36),reale(18483914,31121LL<<37),
      reale(8546239,40923LL<<36),-reale(2972379,4277LL<<38),
      -reale(7799822,49315LL<<36),reale(6131851,9051LL<<37),
      -reale(1385578,60289LL<<36),reale(2743,3362879LL<<30),
      reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^21, polynomial in n of order 8
      reale(36025526,1303LL<<40),-reale(30131090,26093LL<<37),
      -reale(21835156,15459LL<<38),reale(35026415,31673LL<<37),
      -reale(11464406,5705LL<<39),-reale(3907909,12145LL<<37),
      reale(1722090,1439LL<<38),reale(1557916,18869LL<<37),
      -reale(781446,6381283LL<<28),reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^20, polynomial in n of order 9
      -reale(190262221,2387LL<<40),reale(146996287,1227LL<<41),
      -reale(33573688,3541LL<<40),-reale(32294922,2067LL<<39),
      reale(18331180,2115LL<<40),reale(16022794,2331LL<<40),
      -reale(22246846,3383LL<<40),reale(9695242,4143LL<<39),
      -reale(1302304,2671LL<<40),-reale(123235,5577019LL<<29),
      reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^19, polynomial in n of order 10
      reale(169057636,9637LL<<37),-reale(31515275,17095LL<<36),
      -reale(115123722,7359LL<<39),reale(174060780,58071LL<<36),
      -reale(122862790,20125LL<<37),reale(32880337,12981LL<<36),
      reale(15824026,705LL<<38),-reale(12943852,57005LL<<36),
      -reale(2522257,22495LL<<37),reale(5771316,18225LL<<36),
      -reale(1873338,7714415LL<<28),reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^18, polynomial in n of order 11
      reale(137352006,26079LL<<36),-reale(197705648,26891LL<<37),
      reale(213128803,51077LL<<36),-reale(150461460,3135LL<<39),
      reale(25445949,34251LL<<36),reale(93962136,11667LL<<37),
      -reale(140732252,24207LL<<36),reale(108614245,6273LL<<38),
      -reale(48923563,62665LL<<36),reale(10316934,1969LL<<37),
      reale(535285,33757LL<<36),-reale(501186,3348667LL<<30),
      reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^17, polynomial in n of order 12
      reale(15155809,205049LL<<34),-reale(39104030,957115LL<<32),
      reale(81967212,139599LL<<33),-reale(139468172,993913LL<<32),
      reale(190415844,61587LL<<35),-reale(202311488,967447LL<<32),
      reale(154439958,403401LL<<33),-reale(61783996,889045LL<<32),
      -reale(28504911,66989LL<<34),reale(73132006,1030157LL<<32),
      -reale(66442314,293949LL<<33),reale(34502754,346959LL<<32),
      -reale(8240730,128798053LL<<25),reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[15], coeff of eps^16, polynomial in n of order 13
      reale(114172,142577LL<<34),-reale(499141,33119LL<<36),
      reale(1750098,174183LL<<34),-reale(5016097,114721LL<<35),
      reale(11893006,66221LL<<34),-reale(23470909,19093LL<<37),
      reale(38591591,188131LL<<34),-reale(52613301,65223LL<<35),
      reale(58753809,139305LL<<34),-reale(52512562,23925LL<<36),
      reale(36059714,158111LL<<34),-reale(17726185,93229LL<<35),
      reale(5486676,138853LL<<34),-reale(792996,14574745LL<<26),
      reale(42385393120LL,0x471a6a35932f5LL),
      // C4[15], coeff of eps^15, polynomial in n of order 14
      real(592706205LL<<33),-reale(9147,839013LL<<32),
      reale(55355,240865LL<<34),-reale(262940,644275LL<<32),
      reale(1011310,29095LL<<33),-reale(3215965,1023905LL<<32),
      reale(8575909,35467LL<<35),-reale(19352216,329839LL<<32),
      reale(37124659,455473LL<<33),-reale(60529336,778589LL<<32),
      reale(83288367,93771LL<<34),-reale(94856196,165035LL<<32),
      reale(85043486,110139LL<<33),-reale(50751757,943257LL<<32),
      reale(13877433,107462891LL<<25),reale(127156179360LL,0xd54f3ea0b98dfLL),
      // C4[16], coeff of eps^29, polynomial in n of order 0
      real(553LL<<31),real(0x292ecb9a960d27d1LL),
      // C4[16], coeff of eps^28, polynomial in n of order 1
      -real(61453LL<<36),-real(4754645LL<<34),
      reale(19591808,0x57955a5f17535LL),
      // C4[16], coeff of eps^27, polynomial in n of order 2
      reale(33770,14237LL<<36),-reale(12917,115767LL<<35),
      real(1665987897LL<<31),reale(2148568314LL,0xda506166fe05fLL),
      // C4[16], coeff of eps^26, polynomial in n of order 3
      reale(1765351,9719LL<<36),reale(634098,16193LL<<37),
      reale(114937,5021LL<<36),-reale(211035,902511LL<<32),
      reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^25, polynomial in n of order 4
      -reale(3041817,11535LL<<37),-reale(2643315,63657LL<<35),
      reale(3458443,225LL<<36),-reale(1011407,29251LL<<35),
      reale(33755,354965LL<<31),reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^24, polynomial in n of order 5
      reale(12443946,111847LL<<35),-reale(9978547,23661LL<<37),
      reale(264818,24689LL<<35),reale(1196082,9587LL<<36),
      reale(477961,17339LL<<35),-reale(375862,243659LL<<32),
      reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^23, polynomial in n of order 6
      reale(11971225,59849LL<<36),reale(9677599,56595LL<<35),
      -reale(1515717,2317LL<<37),-reale(7956047,112667LL<<35),
      reale(5585704,62147LL<<36),-reale(1169086,64041LL<<35),
      -reale(10840,1435009LL<<31),reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^22, polynomial in n of order 7
      -reale(20905609,47207LL<<36),-reale(27003899,18815LL<<37),
      reale(32128586,62715LL<<36),-reale(8207156,6437LL<<38),
      -reale(4335741,20931LL<<36),reale(1351161,14763LL<<37),
      reale(1583200,44703LL<<36),-reale(734819,355141LL<<32),
      reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^21, polynomial in n of order 8
      reale(119271221,13241LL<<38),-reale(13185134,117885LL<<35),
      -reale(34424757,12741LL<<36),reale(12971898,62105LL<<35),
      reale(17958221,23929LL<<37),-reale(20751983,45233LL<<35),
      reale(8405753,5609LL<<36),-reale(1009805,84123LL<<35),
      -reale(126669,998091LL<<31),reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^20, polynomial in n of order 9
      reale(7281953,46177LL<<36),-reale(132136826,3687LL<<39),
      reale(164419146,28463LL<<36),-reale(102943145,13301LL<<37),
      reale(20499773,15997LL<<36),reale(17609391,14489LL<<38),
      -reale(11127728,9397LL<<36),-reale(3194109,31911LL<<37),
      reale(5518515,63129LL<<36),-reale(1729623,95963LL<<34),
      reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^19, polynomial in n of order 10
      -reale(197461925,11965LL<<36),reale(194820269,1667LL<<35),
      -reale(119937281,937LL<<38),-reale(1213288,24915LL<<35),
      reale(103481944,52469LL<<36),-reale(133891976,7945LL<<35),
      reale(96822293,18071LL<<37),-reale(41434588,121951LL<<35),
      reale(8025452,27431LL<<36),reale(724406,126187LL<<35),
      -reale(459367,1295029LL<<31),reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^18, polynomial in n of order 11
      -reale(47525285,62545LL<<36),reale(91887649,22453LL<<37),
      -reale(145781941,19979LL<<36),reale(186991182,8001LL<<39),
      -reale(187151585,35749LL<<36),reale(133148350,20179LL<<37),
      -reale(44425461,13151LL<<36),-reale(35371425,9983LL<<38),
      reale(71056997,38023LL<<36),-reale(61631567,21647LL<<37),
      reale(31499493,50637LL<<36),-reale(7491423,758351LL<<32),
      reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^17, polynomial in n of order 12
      -reale(2259631,24697LL<<37),reale(7098981,78723LL<<35),
      -reale(18582556,8879LL<<36),reale(40852668,12369LL<<35),
      -reale(75692831,12691LL<<38),reale(118080654,84927LL<<35),
      -reale(154130872,52073LL<<36),reale(166118829,74381LL<<35),
      -reale(144327913,2259LL<<37),reale(96964720,98683LL<<35),
      -reale(46899246,18595LL<<36),reale(14349769,50505LL<<35),
      -reale(2057503,1465135LL<<31),reale(135359803835LL,0xb9c7f85883761LL),
      // C4[16], coeff of eps^16, polynomial in n of order 13
      -reale(18695,264305LL<<33),reale(95700,8265LL<<35),
      -reale(398136,419847LL<<33),reale(1375381,177071LL<<34),
      -reale(4004787,429789LL<<33),reale(9930000,13635LL<<36),
      -reale(21101250,231795LL<<33),reale(38532718,52073LL<<34),
      -reale(60367925,93257LL<<33),reale(80490566,118467LL<<35),
      -reale(89511061,246751LL<<33),reale(78923731,162339LL<<34),
      -reale(46636750,263349LL<<33),reale(12687939,1991833LL<<30),
      reale(135359803835LL,0xb9c7f85883761LL),
      // C4[17], coeff of eps^29, polynomial in n of order 0
      -real(280331LL<<31),reale(154847,0x4e6e7be138cdbLL),
      // C4[17], coeff of eps^28, polynomial in n of order 1
      -real(82431LL<<38),real(142069LL<<33),reale(989485,0x4511e2f2b39a3LL),
      // C4[17], coeff of eps^27, polynomial in n of order 2
      reale(30957,2723LL<<36),reale(7080,38071LL<<35),
      -reale(9773,1986585LL<<31),reale(6836353729LL,0x13b9f01928417LL),
      // C4[17], coeff of eps^26, polynomial in n of order 3
      -reale(138771,28785LL<<37),reale(154910,14439LL<<38),
      -reale(42193,29611LL<<37),real(1108797915LL<<32),
      reale(6836353729LL,0x13b9f01928417LL),
      // C4[17], coeff of eps^25, polynomial in n of order 4
      -reale(1238256,21701LL<<37),-reale(44811,81027LL<<35),
      reale(156785,14859LL<<36),reale(74079,77407LL<<35),
      -reale(51372,1082481LL<<31),reale(20509061187LL,0x3b2dd04b78c45LL),
      // C4[17], coeff of eps^24, polynomial in n of order 5
      reale(10057115,7495LL<<39),-reale(158283,1579LL<<41),
      -reale(7978477,2703LL<<39),reale(5079175,3021LL<<40),
      -reale(987192,2101LL<<39),-reale(20806,242401LL<<34),
      reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^23, polynomial in n of order 6
      -reale(30405369,16203LL<<36),reale(28904813,10839LL<<35),
      -reale(5472666,28841LL<<37),-reale(4538327,21695LL<<35),
      reale(1010309,2151LL<<36),reale(1591197,61387LL<<35),
      -reale(691600,842821LL<<31),reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^22, polynomial in n of order 7
      reale(2360974,20517LL<<37),-reale(34168343,7885LL<<38),
      reale(7988557,399LL<<37),reale(19220645,2761LL<<39),
      -reale(19251394,21847LL<<37),reale(7294617,7633LL<<38),
      -reale(776533,25709LL<<37),-reale(127091,628387LL<<32),
      reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^21, polynomial in n of order 8
      -reale(141970389,5875LL<<38),reale(152285106,31LL<<35),
      -reale(85013706,54665LL<<36),reale(10784519,74157LL<<35),
      reale(18376223,22989LL<<37),-reale(9415616,123045LL<<35),
      -reale(3707065,39939LL<<36),reale(5266887,19945LL<<35),
      -reale(1601936,974407LL<<31),reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^20, polynomial in n of order 9
      reale(174732199,7199LL<<38),-reale(91781661,1783LL<<41),
      -reale(22947906,1007LL<<38),reale(109147441,451LL<<39),
      -reale(126268040,11085LL<<38),reale(86262862,2409LL<<40),
      -reale(35185382,1435LL<<38),reale(6220582,7041LL<<39),
      reale(846498,391LL<<38),-reale(421214,84365LL<<33),
      reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^19, polynomial in n of order 10
      reale(100447726,5039LL<<36),-reale(149758021,121873LL<<35),
      reale(181554380,1939LL<<38),-reale(171878757,123903LL<<35),
      reale(113962110,29289LL<<36),-reale(29967290,80205LL<<35),
      -reale(40353928,13613LL<<37),reale(68655180,86853LL<<35),
      -reale(57285125,57949LL<<36),reale(28886745,8439LL<<35),
      -reale(6846764,2025561LL<<31),reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^18, polynomial in n of order 11
      reale(9163438,32371LL<<37),-reale(22188557,9937LL<<38),
      reale(45681407,15569LL<<37),-reale(80085759,21LL<<40),
      reale(119267529,32127LL<<37),-reale(149784698,12951LL<<38),
      reale(156408668,30941LL<<37),-reale(132494258,5045LL<<39),
      reale(87286969,10059LL<<37),-reale(41608633,10397LL<<38),
      reale(12599797,16681LL<<37),-reale(1793721,181577LL<<32),
      reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[17], coeff of eps^17, polynomial in n of order 12
      reale(152058,3531LL<<37),-reale(566838,109449LL<<35),
      reale(1788421,65069LL<<36),-reale(4828739,49907LL<<35),
      reale(11241509,10969LL<<38),-reale(22666304,107837LL<<35),
      reale(39633653,283LL<<36),-reale(59939783,113319LL<<35),
      reale(77715030,3737LL<<37),-reale(84609105,47985LL<<35),
      reale(73498818,52617LL<<36),-reale(43049308,20443LL<<35),
      reale(11659187,1311925LL<<31),reale(143563428310LL,0x9e40b2104d5e3LL),
      // C4[18], coeff of eps^29, polynomial in n of order 0
      real(35LL<<34),real(0x29845c2bcb5c10d7LL),
      // C4[18], coeff of eps^28, polynomial in n of order 1
      reale(3628,18373LL<<37),-reale(4063,232509LL<<34),
      reale(3097286791LL,0x8a812bfedbe75LL),
      // C4[18], coeff of eps^27, polynomial in n of order 2
      reale(435730,613LL<<39),-reale(110987,3811LL<<38),real(489021323LL<<34),
      reale(21681007540LL,0xc98833f803533LL),
      // C4[18], coeff of eps^26, polynomial in n of order 3
      -reale(762945,31179LL<<36),reale(988791,87LL<<37),
      reale(550009,38375LL<<36),-reale(343815,323189LL<<33),
      reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^25, polynomial in n of order 4
      reale(1063744,27LL<<41),-reale(7897635,7767LL<<39),
      reale(4613149,699LL<<40),-reale(833936,93LL<<39),
      -reale(28054,94387LL<<35),reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^24, polynomial in n of order 5
      reale(25578507,4379LL<<38),-reale(3209600,1553LL<<40),
      -reale(4577572,5923LL<<38),reale(702466,1583LL<<39),
      reale(1586031,287LL<<38),-reale(651636,122639LL<<35),
      reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^23, polynomial in n of order 6
      -reale(32324739,1815LL<<40),reale(3520775,1207LL<<39),
      reale(19946468,259LL<<41),-reale(17787966,4575LL<<39),
      reale(6336978,3235LL<<40),-reale(589727,5685LL<<39),
      -reale(125505,20667LL<<35),reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^22, polynomial in n of order 7
      reale(138884729,22203LL<<36),-reale(69168625,14473LL<<37),
      reale(3249237,4577LL<<36),reale(18436830,10301LL<<38),
      -reale(7840055,31609LL<<36),-reale(4091705,30595LL<<37),
      reale(5021146,27565LL<<36),-reale(1488082,115687LL<<33),
      reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^21, polynomial in n of order 8
      -reale(66334778,1299LL<<41),-reale(40377625,16285LL<<38),
      reale(111882749,4839LL<<39),-reale(118325119,4711LL<<38),
      reale(76858390,3693LL<<40),-reale(29952902,3569LL<<38),
      reale(4790818,4941LL<<39),reale(921311,4421LL<<38),
      -reale(386621,181821LL<<34),reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^20, polynomial in n of order 9
      -reale(151679112,16629LL<<37),reale(174648786,1667LL<<40),
      -reale(156892091,15835LL<<37),reale(96799837,4169LL<<38),
      -reale(17949188,6721LL<<37),-reale(43885384,7293LL<<39),
      reale(66080580,25305LL<<37),-reale(53357084,1853LL<<38),
      reale(26599572,17011LL<<37),-reale(6287689,169979LL<<34),
      reale(151767052785LL,0x82b96bc817465LL),
      // C4[18], coeff of eps^19, polynomial in n of order 10
      -reale(8594193,5169LL<<39),reale(16702080,5475LL<<38),
      -reale(27882498,1245LL<<41),reale(39843622,14413LL<<38),
      -reale(48340851,951LL<<39),reale(49066184,11639LL<<38),
      -reale(40627946,3165LL<<40),reale(26296855,15713LL<<38),
      -reale(12371894,1597LL<<39),reale(3711568,4235LL<<38),
      -reale(524991,147555LL<<34),reale(50589017595LL,0x2b9323ed5d177LL),
      // C4[18], coeff of eps^18, polynomial in n of order 11
      -reale(768539,29011LL<<36),reale(2243105,18035LL<<37),
      -reale(5671852,39713LL<<36),reale(12494515,7255LL<<39),
      -reale(24051943,5231LL<<36),reale(40468348,22085LL<<37),
      -reale(59307062,46653LL<<36),reale(74994737,5975LL<<38),
      -reale(80108014,59787LL<<36),reale(68664012,25623LL<<37),
      -reale(39899358,51033LL<<36),reale(10762327,20443LL<<33),
      reale(151767052785LL,0x82b96bc817465LL),
      // C4[19], coeff of eps^29, polynomial in n of order 0
      -real(69697LL<<34),reale(220556,0x6c98ea537e51fLL),
      // C4[19], coeff of eps^28, polynomial in n of order 1
      -real(1238839LL<<41),real(675087LL<<35),
      reale(141943813,0x222cc7846d81LL),
      // C4[19], coeff of eps^27, polynomial in n of order 2
      reale(876102,3999LL<<40),reale(573743,1451LL<<39),
      -reale(328615,14973LL<<34),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^26, polynomial in n of order 3
      -reale(7739083,17LL<<46),reale(4186838,53LL<<45),-reale(704448,1LL<<46),
      -reale(33249,11241LL<<37),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^25, polynomial in n of order 4
      -reale(1360864,133LL<<42),-reale(4500609,2667LL<<40),
      reale(427896,299LL<<41),reale(1570943,1191LL<<40),
      -reale(614728,45789LL<<35),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^24, polynomial in n of order 5
      -reale(379105,631LL<<42),reale(20252634,139LL<<44),
      -reale(16388211,705LL<<42),reale(5510947,339LL<<43),
      -reale(439601,699LL<<42),-reale(122601,56745LL<<36),
      reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^23, polynomial in n of order 6
      -reale(55355388,567LL<<41),-reale(2520461,2117LL<<40),
      reale(18017708,147LL<<42),-reale(6413373,771LL<<40),
      -reale(4373212,61LL<<41),reale(4784182,2079LL<<40),
      -reale(1386197,54485LL<<35),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^22, polynomial in n of order 7
      -reale(54112477,29LL<<46),reale(112419812,35LL<<45),
      -reale(110372726,9LL<<46),reale(68510282,53LL<<46),
      -reale(25556330,19LL<<46),reale(3652507,1LL<<45),reale(962676,17LL<<46),
      -reale(355362,30093LL<<37),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^21, polynomial in n of order 8
      reale(166723371,209LL<<42),-reale(142457721,7469LL<<39),
      reale(81530379,2787LL<<40),-reale(7977897,3383LL<<39),
      -reale(46298043,1775LL<<41),reale(63437092,799LL<<39),
      -reale(49803454,3807LL<<40),reale(24585849,2581LL<<39),
      -reale(5799325,105875LL<<34),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^20, polynomial in n of order 9
      reale(54095236,1729LL<<41),-reale(86448328,33LL<<44),
      reale(119042325,527LL<<41),-reale(140012701,875LL<<42),
      reale(138519104,1133LL<<41),-reale(112357061,257LL<<43),
      reale(71568963,1275LL<<41),-reale(33272498,441LL<<42),
      reale(9897515,729LL<<41),-reale(1391838,12705LL<<35),
      reale(159970677260LL,0x6732257fe12e7LL),
      // C4[19], coeff of eps^19, polynomial in n of order 10
      reale(2731650,3225LL<<40),-reale(6520331,5423LL<<39),
      reale(13678206,885LL<<42),-reale(25266687,5569LL<<39),
      reale(41073925,3215LL<<40),-reale(58519302,7091LL<<39),
      reale(72351138,181LL<<41),-reale(75968694,8133LL<<39),
      reale(64333849,3333LL<<40),-reale(37115682,4791LL<<39),
      reale(9974839,182105LL<<34),reale(159970677260LL,0x6732257fe12e7LL),
      // C4[20], coeff of eps^29, polynomial in n of order 0
      real(1LL<<39),reale(386445,0x44b61aebc827LL),
      // C4[20], coeff of eps^28, polynomial in n of order 1
      reale(3670,3431LL<<40),-real(63923791LL<<37),
      reale(1044560880,0x57ec63f8653c9LL),
      // C4[20], coeff of eps^27, polynomial in n of order 2
      reale(165149,453LL<<43),-reale(25858,471LL<<42),-real(26276299LL<<38),
      reale(7311926162LL,0x6776bbcac4a7fLL),
      // C4[20], coeff of eps^26, polynomial in n of order 3
      -reale(4343033,595LL<<42),reale(185313,303LL<<43),
      reale(1548473,271LL<<42),-reale(580654,777LL<<40),
      reale(168174301735LL,0x4baadf37ab169LL),
      // C4[20], coeff of eps^25, polynomial in n of order 4
      reale(20236427,149LL<<44),-reale(15067334,133LL<<42),
      reale(4797544,165LL<<43),-reale(318599,375LL<<42),
      -reale(118861,3875LL<<38),reale(168174301735LL,0x4baadf37ab169LL),
      // C4[20], coeff of eps^24, polynomial in n of order 5
      -reale(6870833,1979LL<<41),reale(17282399,281LL<<43),
      -reale(5135975,189LL<<41),-reale(4572111,263LL<<42),
      reale(4557653,1537LL<<41),-reale(1294702,4061LL<<38),
      reale(168174301735LL,0x4baadf37ab169LL),
      // C4[20], coeff of eps^23, polynomial in n of order 6
      reale(111332564,131LL<<43),-reale(102611836,439LL<<42),
      reale(61113705,49LL<<44),-reale(21849131,865LL<<42),
      reale(2742318,257LL<<43),reale(980372,533LL<<42),
      -reale(327159,8391LL<<38),reale(168174301735LL,0x4baadf37ab169LL),
      // C4[20], coeff of eps^22, polynomial in n of order 7
      -reale(128743521,979LL<<42),reale(67998970,481LL<<43),
      reale(279122,855LL<<42),-reale(47847734,245LL<<44),
      reale(60794248,257LL<<42),-reale(46583621,181LL<<43),
      reale(22803394,43LL<<42),-reale(5369928,2229LL<<40),
      reale(168174301735LL,0x4baadf37ab169LL),
      // C4[20], coeff of eps^21, polynomial in n of order 8
      -reale(88564699,121LL<<45),reale(117949702,533LL<<42),
      -reale(134881895,27LL<<43),reale(130376590,239LL<<42),
      -reale(103788735,57LL<<44),reale(65154071,233LL<<42),
      -reale(29963298,393LL<<43),reale(8844588,195LL<<42),
      -reale(1237189,6873LL<<38),reale(168174301735LL,0x4baadf37ab169LL),
      // C4[20], coeff of eps^20, polynomial in n of order 9
      -reale(7362630,999LL<<40),reale(14785858,137LL<<43),
      -reale(26321377,9LL<<40),reale(41483460,1083LL<<41),
      -reale(57615917,1643LL<<40),reale(69797568,521LL<<42),
      -reale(72155594,1933LL<<40),reale(60438019,617LL<<41),
      -reale(34641303,3055LL<<40),reale(9278920,21175LL<<37),
      reale(168174301735LL,0x4baadf37ab169LL),
      // C4[21], coeff of eps^29, polynomial in n of order 0
      -real(2017699LL<<39),reale(144690669,0x92d5d14b2b5b9LL),
      // C4[21], coeff of eps^28, polynomial in n of order 1
      -reale(21806,31LL<<47),-real(1751493LL<<42),
      reale(7668605487LL,0x6644548ff9f4dLL),
      // C4[21], coeff of eps^27, polynomial in n of order 2
      -real(610053LL<<43),reale(66113,223LL<<42),-reale(23877,14131LL<<38),
      reale(7668605487LL,0x6644548ff9f4dLL),
      // C4[21], coeff of eps^26, polynomial in n of order 3
      -reale(601427,223LL<<44),reale(181759,65LL<<45),-reale(9602,5LL<<44),
      -reale(4983,2721LL<<39),reale(7668605487LL,0x6644548ff9f4dLL),
      // C4[21], coeff of eps^25, polynomial in n of order 4
      reale(16348405,227LL<<44),-reale(4001511,795LL<<42),
      -reale(4705038,397LL<<43),reale(4342393,855LL<<42),
      -reale(1212256,1051LL<<38),reale(176377926210LL,0x302398ef74febLL),
      // C4[21], coeff of eps^24, polynomial in n of order 5
      -reale(95167920,19LL<<45),reale(54565817,7LL<<47),
      -reale(18712410,5LL<<45),reale(2011897,15LL<<46),reale(981374,25LL<<45),
      -reale(301721,597LL<<40),reale(176377926210LL,0x302398ef74febLL),
      // C4[21], coeff of eps^23, polynomial in n of order 6
      reale(56043535,133LL<<43),reale(7101303,759LL<<42),
      -reale(48732132,249LL<<44),reale(58197907,161LL<<42),
      -reale(43660867,425LL<<43),reale(21217809,619LL<<42),
      -reale(4990122,11039LL<<38),reale(176377926210LL,0x302398ef74febLL),
      // C4[21], coeff of eps^22, polynomial in n of order 7
      reale(38792824,189LL<<44),-reale(43241527,125LL<<45),
      reale(40920531,151LL<<44),-reale(32022608,39LL<<46),
      reale(19836099,97LL<<44),-reale(9032168,63LL<<45),
      reale(2647359,187LL<<44),-reale(368524,4161LL<<39),
      reale(58792642070LL,0x100bdda526ff9LL),
      // C4[21], coeff of eps^21, polynomial in n of order 8
      reale(15813930,121LL<<45),-reale(27228018,205LL<<42),
      reale(41726053,443LL<<43),-reale(56628215,983LL<<42),
      reale(67341662,57LL<<44),-reale(68636694,193LL<<42),
      reale(56918234,105LL<<43),-reale(32430156,715LL<<42),
      reale(8660325,15343LL<<38),reale(176377926210LL,0x302398ef74febLL),
      // C4[22], coeff of eps^29, polynomial in n of order 0
      -real(229LL<<43),reale(2018939,0x935060fc493cdLL),
      // C4[22], coeff of eps^28, polynomial in n of order 1
      reale(64733,61LL<<46),-reale(22613,493LL<<43),
      reale(8025284812LL,0x6511ed552f41bLL),
      // C4[22], coeff of eps^27, polynomial in n of order 2
      reale(158513,3LL<<48),-reale(6162,29LL<<47),-reale(4786,487LL<<43),
      reale(8025284812LL,0x6511ed552f41bLL),
      // C4[22], coeff of eps^26, polynomial in n of order 3
      -reale(130438,301LL<<43),-reale(208062,47LL<<44),reale(179942,497LL<<43),
      -reale(49466,167LL<<40),reale(8025284812LL,0x6511ed552f41bLL),
      // C4[22], coeff of eps^25, polynomial in n of order 4
      reale(2120438,3LL<<47),-reale(697803,39LL<<45),reale(61914,3LL<<46),
      reale(42203,115LL<<45),-reale(12120,543LL<<41),
      reale(8025284812LL,0x6511ed552f41bLL),
      // C4[22], coeff of eps^24, polynomial in n of order 5
      reale(12722577,33LL<<44),-reale(49104495,51LL<<46),
      reale(55677556,71LL<<44),-reale(41002422,115LL<<45),
      reale(19800840,109LL<<44),-reale(4652345,837LL<<41),
      reale(184581550685LL,0x149c52a73ee6dLL),
      // C4[22], coeff of eps^23, polynomial in n of order 6
      -reale(124610244,57LL<<46),reale(115654934,113LL<<45),
      -reale(89096506,19LL<<47),reale(54518354,119LL<<45),
      -reale(24598996,19LL<<46),reale(7163443,125LL<<45),
      -reale(992759,1841LL<<41),reale(184581550685LL,0x149c52a73ee6dLL),
      // C4[22], coeff of eps^22, polynomial in n of order 7
      -reale(27999005,155LL<<43),reale(41827085,121LL<<44),
      -reale(55581037,1LL<<43),reale(64987058,83LL<<45),
      -reale(65383321,103LL<<43),reale(53725829,211LL<<44),
      -reale(30444636,461LL<<43),reale(8107539,715LL<<40),
      reale(184581550685LL,0x149c52a73ee6dLL),
      // C4[23], coeff of eps^29, polynomial in n of order 0
      -reale(4289,21LL<<43),reale(1676392827,0x7a5fe79ee0e95LL),
      // C4[23], coeff of eps^28, polynomial in n of order 1
      -real(1351LL<<51),-real(234789LL<<44),
      reale(1676392827,0x7a5fe79ee0e95LL),
      // C4[23], coeff of eps^27, polynomial in n of order 2
      -reale(209744,1LL<<50),reale(171585,3LL<<49),-reale(46526,469LL<<43),
      reale(8381964137LL,0x63df861a648e9LL),
      // C4[23], coeff of eps^26, polynomial in n of order 3
      -reale(599194,1LL<<51),reale(41297,0),reale(41388,1LL<<51),
      -reale(11218,97LL<<45),reale(8381964137LL,0x63df861a648e9LL),
      // C4[23], coeff of eps^25, polynomial in n of order 4
      -reale(2134087,7LL<<49),reale(2315275,31LL<<47),-reale(1677358,15LL<<48),
      reale(805613,21LL<<47),-reale(189149,1213LL<<41),
      reale(8381964137LL,0x63df861a648e9LL),
      // C4[23], coeff of eps^24, polynomial in n of order 5
      reale(4740508,1LL<<49),-reale(3599518,1LL<<51),reale(2177844,7LL<<49),
      -reale(974429,1LL<<50),reale(282071,5LL<<49),-reale(38931,779LL<<42),
      reale(8381964137LL,0x63df861a648e9LL),
      // C4[23], coeff of eps^23, polynomial in n of order 6
      reale(1817763,3LL<<48),-reale(2369306,23LL<<47),reale(2727592,1LL<<49),
      -reale(2711734,1LL<<47),reale(2209561,1LL<<48),-reale(1245816,11LL<<47),
      reale(330919,1979LL<<41),reale(8381964137LL,0x63df861a648e9LL),
      // C4[24], coeff of eps^29, polynomial in n of order 0
      -real(1439LL<<46),reale(44813556,0x37a4fd885dffdLL),
      // C4[24], coeff of eps^28, polynomial in n of order 1
      reale(32742,3LL<<50),-reale(8770,21LL<<47),
      reale(1747728692,0x7a229fc651f8bLL),
      // C4[24], coeff of eps^27, polynomial in n of order 2
      reale(4928,1LL<<51),reale(8067,1LL<<50),-reale(2080,43LL<<46),
      reale(1747728692,0x7a229fc651f8bLL),
      // C4[24], coeff of eps^26, polynomial in n of order 3
      reale(2214330,0),-reale(1581120,0),reale(755790,0),
      -reale(177363,7LL<<47),reale(8738643462LL,0x62ad1edf99db7LL),
      // C4[24], coeff of eps^25, polynomial in n of order 4
      -reale(1116955,0),reale(668788,3LL<<50),-reale(296917,1LL<<51),
      reale(85476,1LL<<50),-reale(11752,63LL<<46),
      reale(2912881154LL,0x20e45f9fddf3dLL),
      // C4[24], coeff of eps^24, polynomial in n of order 5
      -reale(2320992,3LL<<48),reale(2634056,1LL<<50),-reale(2590155,5LL<<48),
      reale(2094168,1LL<<49),-reale(1175298,7LL<<48),reale(311454,11LL<<45),
      reale(8738643462LL,0x62ad1edf99db7LL),
      // C4[25], coeff of eps^29, polynomial in n of order 0
      -real(3707LL<<46),reale(12720731,0x2bd144a4925efLL),
      // C4[25], coeff of eps^28, polynomial in n of order 1
      real(301LL<<53),-real(2379LL<<48),reale(139928042,0xe1fdf3124a145LL),
      // C4[25], coeff of eps^27, polynomial in n of order 2
      -reale(298603,1LL<<51),reale(142145,1LL<<50),-reale(33346,63LL<<46),
      reale(1819064557,0x79e557edc3081LL),
      // C4[25], coeff of eps^26, polynomial in n of order 3
      reale(370617,0),-reale(163358,0),reale(46787,0),-reale(6410,23LL<<47),
      reale(1819064557,0x79e557edc3081LL),
      // C4[25], coeff of eps^25, polynomial in n of order 4
      reale(508963,0),-reale(495426,3LL<<50),reale(397689,1LL<<51),
      -reale(222238,1LL<<50),reale(58764,59LL<<46),
      reale(1819064557,0x79e557edc3081LL),
      // C4[26], coeff of eps^29, polynomial in n of order 0
      -real(1LL<<49),reale(131359,0xe834f81ee20c1LL),
      // C4[26], coeff of eps^28, polynomial in n of order 1
      reale(10305,0),-reale(2417,1LL<<49),reale(145415417,0x1d0ced8b7a293LL),
      // C4[26], coeff of eps^27, polynomial in n of order 2
      -reale(11556,0),reale(3294,0),-real(3599LL<<49),
      reale(145415417,0x1d0ced8b7a293LL),
      // C4[26], coeff of eps^26, polynomial in n of order 3
      -reale(36490,1LL<<51),reale(29097,0),-reale(16195,1LL<<51),
      reale(4273,13LL<<48),reale(145415417,0x1d0ced8b7a293LL),
      // C4[27], coeff of eps^29, polynomial in n of order 0
      -real(2029LL<<49),reale(16766976,0xd0e6a80084b19LL),
      // C4[27], coeff of eps^28, polynomial in n of order 1
      real(7LL<<56),-real(61LL<<50),reale(5588992,0x45a238002c3b3LL),
      // C4[27], coeff of eps^27, polynomial in n of order 2
      reale(3080,0),-real(427LL<<54),real(3599LL<<49),
      reale(16766976,0xd0e6a80084b19LL),
      // C4[28], coeff of eps^29, polynomial in n of order 0
      -real(1LL<<53),reale(827461,0x318a62b8e0a5bLL),
      // C4[28], coeff of eps^28, polynomial in n of order 1
      -real(29LL<<55),real(61LL<<52),reale(2482383,0x949f282aa1f11LL),
      // C4[29], coeff of eps^29, polynomial in n of order 0
      real(1LL<<53),reale(88602,0xec373d36a45dfLL),
    };  // count = 5425
#else
#error "Bad value for GEOGRAPHICLIB_GEODESICEXACT_ORDER"
#endif
    static_assert(sizeof(coeff) / sizeof(real) ==
                  (nC4_ * (nC4_ + 1) * (nC4_ + 5)) / 6,
                  "Coefficient array size mismatch in C4coeff");
    int o = 0, k = 0;
    for (int l = 0; l < nC4_; ++l) {        // l is index of C4[l]
      for (int j = nC4_ - 1; j >= l; --j) { // coeff of eps^j
        int m = nC4_ - j - 1;               // order of polynomial in n
        _cC4x[k++] = Math::polyval(m, coeff + o, _n) / coeff[o + m + 1];
        o += m + 2;
      }
    }
    // Post condition: o == sizeof(coeff) / sizeof(real) && k == nC4x_
    if  (!(o == sizeof(coeff) / sizeof(real) && k == nC4x_))
      throw GeographicErr("C4 misalignment");
  }
};

Math::real CosSeries(Math::real sinx, Math::real cosx,
                     const Math::real c[], int n) {
  // Evaluate
  // y = sum(c[i] * cos((2*i+1) * x), i, 0, n-1)
  // using Clenshaw summation.
  // Approx operation count = (n + 5) mult and (2 * n + 2) add
  c += n ;                    // Point to one beyond last element
  Math::real
    ar = 2 * (cosx - sinx) * (cosx + sinx), // 2 * cos(2 * x)
    y0 = n & 1 ? *--c : 0, y1 = 0;          // accumulators for sum
  // Now n is even
  n /= 2;
  while (n--) {
    // Unroll loop x 2, so accumulators return to their original role
    y1 = ar * y0 - y1 + *--c;
    y0 = ar * y1 - y0 + *--c;
  }
  return cosx * (y0 - y1);    // cos(x) * (y0 - y1)
}

Math::real SinSeries(Math::real sinx, Math::real cosx, const Math::real c[], int n) {
  // Evaluate
  // y = sum(c[i] * sin((2*i+1) * x), i, 0, n-1)
  // using Clenshaw summation.
  // Approx operation count = (n + 5) mult and (2 * n + 2) add
  c += n ;                    // Point to one beyond last element
  Math::real
    ar = 2 * (cosx - sinx) * (cosx + sinx), // 2 * cos(2 * x)
    y0 = n & 1 ? *--c : 0, y1 = 0;          // accumulators for sum
  // Now n is even
  n /= 2;
  while (n--) {
    // Unroll loop x 2, so accumulators return to their original role
    y1 = ar * y0 - y1 + *--c;
    y0 = ar * y1 - y0 + *--c;
  }
  return sinx * (y0 + y1);    // sin(x) * (y0 + y1)
}

Math::real fft_check(const vector<Math::real>& vals,
                     const vector<Math::real>& tx,
                     bool centerp = false) {
  Math::real maxerr = 0;
  int N = vals.size();
  for (int i = 0; i < N; ++i) {
    Math::real
      sig = (2*i + (centerp ? 1 : 2))/Math::real(4*N) * Math::pi(),
      err = fabs(vals[i] - SinSeries(sin(sig), cos(sig),
                                     tx.data(), tx.size()));
    maxerr = fmax(err, maxerr);
  }
  return maxerr;
}

// Implement DST-III (centerp = false) or DST-IV (centerp = true)
void fft_transform(const vector<Math::real>& in, vector<Math::real>& out,
                   bool centerp = false, bool check = false) {
  int N = in.size(); out.resize(N);
#if HAVE_FFTW
  fftw_r2r_kind kind = centerp ? FFTW_RODFT11 : FFTW_RODFT01;
  fftw_plan p;
#if GEOGRAPHICLIB_PRECISION == 4
  vector<__float128> temp(N);
  for (int i = 0; i < N; ++i) temp[i] = __float128(in[i]);
  p = fftw_plan_r2r_1d(N, temp.data(), temp.data(), kind, FFTW_ESTIMATE);
  fftw_execute(p);
  for (int i = 0; i < N; ++i) out[i] = Math::real(temp[i])/N;
#else
  out = in;
  p = fftw_plan_r2r_1d(N, out.data(), out.data(), kind, FFTW_ESTIMATE);
  fftw_execute(p);
  for (int i = 0; i < N; ++i) out[i] /= N;
#endif
  fftw_destroy_plan(p);
#else
  vector<Math::real> tempin(4*N);
  if (centerp) {
    for (int i = 0; i < N; ++i) {
      tempin[i] = in[i];
      tempin[N+i] = in[N-1-i];
      tempin[2*N+i] = -in[i];
      tempin[3*N+i] = -in[N-1-i];
    }
  } else {
    tempin[0] = 0;
    for (int i = 0; i < N; ++i) tempin[i+1] = in[i];
    for (int i = 1; i < N; ++i) tempin[N+i] = tempin[N-i];
    for (int i = 0; i < 2*N; ++i) tempin[2*N+i] = -tempin[i];
  }
  kissfft<Math::real> fft(2*N, false);
  vector<complex<Math::real>> tempout(2*N);
  fft.transform_real(tempin.data(), tempout.data());
  for (int i = 0, j = 1; i < N; ++i, j+=2) {
    if (centerp)
      tempout[j] *= exp(complex<Math::real>(0, -j*Math::pi()/(4*N)));
    out[i] = -tempout[j].imag() / (2*N);
  }
#endif
  if (check)
    cout << "err(" << centerp << ") = " << fft_check(in, out, centerp) << "\n";
}

void fft_transform2(const vector<Math::real>& oldin,
                    const vector<Math::real>& newin,
                    const vector<Math::real>& oldout,
                    vector<Math::real>& newout,
                    bool check = false) {
  // oldin and oldout are a transform pair with centerp = false.
  // newin at the corresponding centerp = true values

  // newout is the centerp = false transform for the combined set of values, so
  // newout.size() = 2*oldout.size()

  // N.B. the combined set of input values are order newin[0], oldin[0],
  // newin[1], oldin[1], newin[2], oldin[2], etc.

  // oldin is only touched with check = true
  int N = newin.size();
  fft_transform(newin, newout, true, false);
  newout.resize(2*N);
  for (int i = N; i < 2*N; ++i)
    newout[i] = (-oldout[2*N-1-i] + newout[2*N-1-i])/2;
  for (int i = 0; i < N; ++i)
    newout[i] = (oldout[i] + newout[i])/2;
  if (check) {
    vector<Math::real> tempin(2*N);
    for (int i = 0; i < N; ++i) {
      tempin[2*i  ] = newin[i];
      tempin[2*i+1] = oldin[i];
    }
    if (check)
      cout << "err(2) = " << fft_check(tempin, newout, false) << "\n";

  }
}

void I4f(Math::real n, Math::real alp0, int N,
         vector<Math::real>& I4, bool centerp = false, bool check = false) {
  Math::real
    ep2 = 4*n/Math::sq(1 - n),
    k2 = ep2 * Math::sq( Math::cosd(alp0) );
  I4Integrand i4(ep2, k2);
  vector<Math::real> in(N);
  for (int i = 0; i < N; ++i) {
    Math::real sig = (2*i + (centerp ? 1 : 2))/Math::real(4*N) * Math::pi();
    in[i] = i4(sig);
  }
  fft_transform(in, I4, centerp, check);
}

// Scale to do the integal -1/(2*l + 1) and to normalize term to estimate error
// A4/c2;

void C4f(Math::real n, Math::real alp0, int N,
         vector<Math::real>& C4, bool centerp = false) {
  I4f(n, alp0, N, C4, centerp);
  Math::real
    ep2 = 4*n/Math::sq(1 - n),
    e2 = 4*n/Math::sq(1 + n),
    c2 = (1 + Math::sq((1-n)/(1+n)) *
          (n == 0 ? 1 :
           (n > 0 ? asinh(sqrt(ep2)) : atan(sqrt(-e2))) /
           sqrt(fabs(e2))))/2, // authalic radius squared
    A4 = Math::cosd(alp0) * Math::sind(alp0) * e2;
  for (int l = 0; l < N; ++l)
    C4[l] *= - A4/((2*l + 1) * c2);
}

void C4falt(int prec, Math::real n, Math::real alp0, int N,
            vector<Math::real>& C4, const TaylorI4& tay) {
  if (prec)
    C4f(n, alp0, N, C4);
  else
    tay.C4f(alp0, C4);
}

Math::real maxerr(const vector<Math::real> C4a, const vector<Math::real> C4b) {
  int Na = C4a.size(), Nb = C4b.size(), N0 = min(Na, Nb), N1 = max(Na, Nb);
  vector<Math::real> diff = Nb > Na ? C4b : C4a;
  for (int i = 0; i < N0; ++i)
    diff[i] -= Nb > Na ? C4a[i] : C4b[i];
  Math::real err = 0;
  if (true) {
    // Simplest (and most conservative) estimate of error is to sum the abs
    // difference of the coefficients
    for (int i = 0; i < N1; ++i) {
      err += fabs(diff[i]);
    }
  } else {
    // Step through sig looking for the largest error
    for (int i = 0; i < 4*N0; ++i) {
      Math::real sig = Math::pi()/2 * i / Math::real(4*N0);
      err = fmax(err, fabs( CosSeries(sin(sig), cos(sig), diff.data(), N1) ));
    }
  }
  return err;
}

string bfloat(Math::real x) {
  ostringstream str;
  str << setprecision(Math::digits10()) << x;
  string s = str.str();
  string::size_type p = s.find('e', 0);
  if (p == string::npos)
    s = s + "b0";
  else
    s[p] = 'b';
  return s;
}

int main(int argc, const char* const argv[]) {
  try {
    Utility::set_digits();
    if (argc < 2)
      { cerr << "AreaEst [estN|i4table|c4diff|c4arr] ...\n"; return 1; }
    string mode(argv[1]);
    if (mode == "estN") {
      if (argc != 4) { cerr << "AreaEst estN Nmax prec\n"; return 1; }
      int Nmax, prec;
      Nmax = Utility::val<int>(string(argv[2]));
      prec = Utility::val<int>(string(argv[3]));
      vector<Math::real> C4ref, C4;
      Math::real eps;
      switch (prec) {
        // prec = 0 means use 30th order taylor series
        // prec = -1 means use N=30 DST
      case -1:
      case 0: eps = numeric_limits<Math::real>::infinity(); break; // Use TaylorI4
      case 1: eps = numeric_limits<float>::epsilon() / 2; break;
      case 2: eps = numeric_limits<double>::epsilon() / 2; break;
      case 3: eps = numeric_limits<long double>::epsilon() / 2; break;
      case 4: eps = pow(Math::real(0.5), 113); break;
#if GEOGRAPHICLIB_PRECISION > 1
        // Skip case 5 for float prec to avoid underflow to 0
      case 5: eps = pow(Math::real(0.5), 256); break;
#endif
      default: eps = numeric_limits<double>::epsilon() / 2; break;
      }
      // The range of n in [-0.91, 0.91] the includes 1/20 < b/a < 20
      //      cout << "epsilon = " << (prec > 0 ? eps :
      //       numeric_limits<double>::epsilon() / 2) << endl;
      for (int in = -99; in <= 99; ++in) {
        vector<Math::real> C4x;
        Math::real n = in/Math::real(100),
          errx = -1,
          maxalp0 = -1;
        TaylorI4 tay(n);
        int N = 0;
        // Pick N = 2^k and 3*2^k: [4, 6, 8, 12, 16, 24, 32, ...]
        for (N = (prec > 0 ? 4 : GEOGRAPHICLIB_GEODESICEXACT_ORDER);
             N <= Nmax; N = N % 3 == 0 ? 4*N/3 : 3*N/2) {
          errx = -1;
          maxalp0 = -1;
          Math::real alp0 = 10, err;
          C4f(n, alp0, 2*N, C4ref);
          C4falt(prec, n, alp0, N, C4, tay);
          err = maxerr(C4, C4ref);
          // cerr << "A " << N << " " << alp0 << " " << err << "\n";
          if (err > eps) continue;
          bool ok = true;
          for (int a = 1; a < 90; ++a) {
            alp0 = a;
            C4f(n, alp0, 2*N, C4ref);
            C4falt(prec, n, alp0, N, C4, tay);
            err = maxerr(C4, C4ref);
            // cerr << "B " << N << " " << alp0 << " " << err << "\n";
            if (err > eps) { ok = false; break; }
            if (err > errx) {
              errx = err; C4x = C4;
              maxalp0 = alp0;
            }
          }
          if (!ok) continue;
          Math::real alp00 = maxalp0;
          for (int a = -9; a < 10; ++a) {
            alp0 = alp00 + a/Math::real(10);
            C4f(n, alp0, 2*N, C4ref);
            C4falt(prec, n, alp0, N, C4, tay);
            err = maxerr(C4, C4ref);
            // cerr << "C " << N << " " << alp0 << " " << err << "\n";
            if (err > eps) { ok = false; break; }
            if (err > errx) {
              errx = err; C4x = C4;
              maxalp0 = alp0;
            }
          }
          if (!ok) continue;
          alp00 = maxalp0;
          for (int a = -9; a < 10; ++a) {
            alp0 = alp00 + a/Math::real(100);
            C4f(n, alp0, 2*N, C4ref);
            C4falt(prec, n, alp0, N, C4, tay);
            err = maxerr(C4, C4ref);
            // cerr << "D " << N << " " << alp0 << " " << err << "\n";
            if (err > eps) { ok = false; break; }
            if (err > errx) {
              errx = err; C4x = C4;
              maxalp0 = alp0;
            }
          }
          if (!ok) continue;
          alp00 = maxalp0;
          for (int a = -9; a < 10; ++a) {
            alp0 = alp00 + a/Math::real(1000);
            C4f(n, alp0, 2*N, C4ref);
            C4falt(prec, n, alp0, N, C4, tay);
            err = maxerr(C4, C4ref);
            // cerr << "E " << N << " " << alp0 << " " << err << "\n";
            if (err > eps) { ok = false; break; }
            if (err > errx) {
              errx = err; C4x = C4;
              maxalp0 = alp0;
            }
          }
          if (!ok) continue;
          alp00 = maxalp0;
          for (int a = -9; a < 10; ++a) {
            alp0 = alp00 + a/Math::real(10000);
            C4f(n, alp0, 2*N, C4ref);
            C4falt(prec, n, alp0, N, C4, tay);
            err = maxerr(C4, C4ref);
            // cerr << "F " << N << " " << alp0 << " " << err << "\n";
            if (err > eps) { ok = false; break; }
            if (err > errx) {
              errx = err; C4x = C4;
              maxalp0 = alp0;
            }
          }
          if (ok) break;
          if (prec <= 0) break;
        }
        N = C4x.size();
        /*
          Math::real erry = 0;
          // Assess summing last few scaled coefficients as a less expensive
          // error metric.  This metric is not being used -- so
          // skip calculating and printing it.
          for (int i = (31*N)/32; i < N; ++i)
          erry = erry+fabs(C4x[i]);
        */
        cout << n << " " << N << " " << maxalp0 << " " << errx << endl;
      }
    } else if (mode == "estN2") {
      if (argc != 2) { cerr << "AreaEst estN2\n"; return 1; }
      int Nmax = 9000, prec = 2;
      vector<Math::real> C4ref, C4;
      Math::real eps;
      switch (prec) {
        // prec = 0 means use 30th order taylor series
        // prec = -1 means use N=30 DST
      case -1:
      case 0: eps = numeric_limits<Math::real>::infinity(); break; // Use TaylorI4
      case 1: eps = numeric_limits<float>::epsilon() / 2; break;
      case 2: eps = numeric_limits<double>::epsilon() / 2; break;
      case 3: eps = numeric_limits<long double>::epsilon() / 2; break;
      case 4: eps = pow(Math::real(0.5), 113); break;
#if GEOGRAPHICLIB_PRECISION > 1
        // Skip case 5 for float prec to avoid underflow to 0
      case 5: eps = pow(Math::real(0.5), 256); break;
#endif
      default: eps = numeric_limits<double>::epsilon() / 2; break;
      }
      // The range of n in [-0.91, 0.91] the includes 1/20 < b/a < 20
      //      cout << "epsilon = " << (prec > 0 ? eps :
      //       numeric_limits<double>::epsilon() / 2) << endl;
      for (int s = -1; s <= 1; s += 2) {
        for (int in = 0; in <= 99; ++in) {
          vector<Math::real> C4x;
          Math::real n = s * (in ? in/Math::real(100) : 1/Math::real(1000)),
            errx = -1,
            maxalp0 = -1;
          TaylorI4 tay(n);
          int N = 4;
          // Pick N = 2^k and 3*2^k: [4, 6, 8, 12, 16, 24, 32, ...]
          Math::real alp0 = 10;
          for (; N <= Nmax; N += (1 << max(0,int(log2(N))-5))) {
            errx = -1;
            alp0 = maxalp0;
            maxalp0 = -1;
            Math::real err;
            C4f(n, alp0, 2*N, C4ref);
            C4falt(prec, n, alp0, N, C4, tay);
            err = maxerr(C4, C4ref);
            // cerr << "A " << N << " " << alp0 << " " << err << "\n";
            if (err > eps) continue;
            bool ok = true;
            for (int a = 1; a < 90; ++a) {
              alp0 = a;
              C4f(n, alp0, 2*N, C4ref);
              C4falt(prec, n, alp0, N, C4, tay);
              err = maxerr(C4, C4ref);
              // cerr << "B " << N << " " << alp0 << " " << err << "\n";
              if (err > eps) { ok = false; break; }
              if (err > errx) {
                errx = err; C4x = C4;
                maxalp0 = alp0;
              }
            }
            if (!ok) continue;
            Math::real alp00 = maxalp0;
            for (int a = -9; a < 10; ++a) {
              alp0 = alp00 + a/Math::real(10);
              C4f(n, alp0, 2*N, C4ref);
              C4falt(prec, n, alp0, N, C4, tay);
              err = maxerr(C4, C4ref);
              // cerr << "C " << N << " " << alp0 << " " << err << "\n";
              if (err > eps) { ok = false; break; }
              if (err > errx) {
                errx = err; C4x = C4;
                maxalp0 = alp0;
              }
            }
            if (!ok) continue;
            alp00 = maxalp0;
            for (int a = -9; a < 10; ++a) {
              alp0 = alp00 + a/Math::real(100);
              C4f(n, alp0, 2*N, C4ref);
              C4falt(prec, n, alp0, N, C4, tay);
              err = maxerr(C4, C4ref);
              // cerr << "D " << N << " " << alp0 << " " << err << "\n";
              if (err > eps) { ok = false; break; }
              if (err > errx) {
                errx = err; C4x = C4;
                maxalp0 = alp0;
              }
            }
            if (!ok) continue;
            alp00 = maxalp0;
            for (int a = -9; a < 10; ++a) {
              alp0 = alp00 + a/Math::real(1000);
              C4f(n, alp0, 2*N, C4ref);
              C4falt(prec, n, alp0, N, C4, tay);
              err = maxerr(C4, C4ref);
              // cerr << "E " << N << " " << alp0 << " " << err << "\n";
              if (err > eps) { ok = false; break; }
              if (err > errx) {
                errx = err; C4x = C4;
                maxalp0 = alp0;
              }
            }
            if (!ok) continue;
            alp00 = maxalp0;
            for (int a = -9; a < 10; ++a) {
              alp0 = alp00 + a/Math::real(10000);
              C4f(n, alp0, 2*N, C4ref);
              C4falt(prec, n, alp0, N, C4, tay);
              err = maxerr(C4, C4ref);
              // cerr << "F " << N << " " << alp0 << " " << err << "\n";
              if (err > eps) { ok = false; break; }
              if (err > errx) {
                errx = err; C4x = C4;
                maxalp0 = alp0;
              }
            }
            if (ok) break;
            if (prec <= 0) break;
          }
          N = C4x.size();
          /*
            Math::real erry = 0;
            // Assess summing last few scaled coefficients as a less expensive
            // error metric.  This metric is not being used -- so
            // skip calculating and printing it.
            for (int i = (31*N)/32; i < N; ++i)
            erry = erry+fabs(C4x[i]);
          */
          cout << n << " " << N << " " << maxalp0 << " " << errx << endl;
        }
      }
    } else if (mode == "i4table") {
      if (argc != 4) { cerr << "AreaEst i4table n alp0\n"; return 1; }
      Math::real n, alp0;
      n = Utility::val<Math::real>(string(argv[2]));
      alp0 = Utility::val<Math::real>(string(argv[3]));
      int N = 1024;
      int m = 256;
      vector<Math::real> C4;
      C4f(n, alp0, N, C4);
      cout << setprecision(17);
      for (int i = 0; i <= m; ++i) {
       Math::real sig = i * Math::real(90) / m, ssig, csig;
       Math::sincosd(sig, ssig, csig);
       cout << sig << " "
            << CosSeries(ssig, csig, C4.data(), N) << "\n";
     }
    } else if (mode == "c4diff") {
      if (argc != 5) { cerr << "AreaEst c4diff n alp0 N0\n"; return 1; }
      Math::real n = Utility::val<Math::real>(string(argv[2])),
        alp0 = Utility::val<Math::real>(string(argv[3]));
      int N0 = Utility::val<int>(string(argv[4]));
      vector<Math::real> C4ref, C4;
      C4f(n, alp0, 4*N0, C4ref);
      cout << setprecision(17);
      for (int N = 4; N <= N0; ++N) {
        C4f(n, alp0, N, C4);
        cout << N;
        for (int l = 0; l < N0; ++l)
          cout << " " << (l < N ? C4[l] : 0) - C4ref[l];
        cout << "\n";
      }
    } else if (mode == "c4arr") {
      if (argc != 5) { cerr << "AreaEst c4arr n alp0 N\n"; return 1; }
      Math::real n = Utility::val<Math::real>(string(argv[2])),
        alp0 = Utility::val<Math::real>(string(argv[3]));
      int N = Utility::val<int>(string(argv[4]));
      Math::real salp0, calp0;
      Math::sincosd(alp0, salp0, calp0);
      Math::real ep2 = 4*n/Math::sq(1 - n),
        e2 = 4*n/Math::sq(1 + n),
        k2 = ep2 * Math::sq(calp0),
        eps = k2 / (2 * (1 + sqrt(1 + k2)) + k2),
        c2 = (1 + Math::sq((1-n)/(1+n)) *
              (n == 0 ? 1 :
               (n > 0 ? asinh(sqrt(ep2)) : atan(sqrt(-e2))) /
               sqrt(fabs(e2))))/2, // authalic radius squared
        A4 = Math::cosd(alp0) * Math::sind(alp0) * e2;
      cout << "(fpprec:" << Math::digits10()
           << ",\nN:" << N
           << ",\nnx:" << bfloat(n)
           << ",\nalp0x:" << bfloat(alp0)
           << ",\nepsx:" << bfloat(eps)
           << ",\nnorm:" << bfloat(A4/c2);
      vector<Math::real> C4;
      C4f(n, alp0, 4*N, C4);
      for (int l = 0; l < N; ++l)
        cout << ",\nC40[" << l << "]:" << bfloat(C4[l]);
      cout << ")$\n";
    } else {
      cerr << "Unknown mode " << mode << "\n";
      return 1;
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Caught exception: " << e.what() << "\n";
    return 1;
  }
  catch (...) {
    std::cerr << "Caught unknown exception\n";
    return 1;
  }
}
