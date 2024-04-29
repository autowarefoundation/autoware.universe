#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <limits>
#include <algorithm>
#include <boost/numeric/odeint.hpp>

#include <GeographicLib/Math.hpp>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/NormalGravity.hpp>

using namespace GeographicLib;
namespace ode = boost::numeric::odeint;
typedef Math::real real;
typedef std::array<real, 2> point;

// Initialize with the end points of a line
class LineDistance {
  point _a, _b;
  real _l, _nx, _ny;
public:
  LineDistance(const point& a, const point& b);
  real Distance(const point& p) const;
  // negative on left of line, pos on right
  real Displacement(const point& p) const;
};

// Use Ramer-Douglas-Peucker to simplify a polyline
class LineSimplifier {
  // Simplify range [a,b]
  static void InternalSimplify(std::vector<point>& p, unsigned a, unsigned b,
                               real thresh);
public:
  static void Simplify(std::vector<point>& p, real thresh);
};

inline LineDistance::LineDistance(const point& a, const point& b)
  : _a(a), _b(b) {
  using std::hypot;
  real dx = _b[0] - _a[0], dy = _b[1] - _a[1];
  _l = hypot(dx, dy);
  if (_l > std::numeric_limits<real>::epsilon()) {
  _nx = dx / _l; _ny = dy / _l;
  } else {
    _l = 0; _nx = 1; _ny = 0;
  }
}

inline real LineDistance::Distance(const point& p) const {
  using std::abs; using std::hypot;
  real x = p[0] - _a[0], y = p[1] - _a[1];
  if (_l != 0) {
    real X = x * _nx + y * _ny, Y = abs(x * _ny - y * _nx);
    X = X < 0 ? -X : (X > _l ? X - _l : 0);
    return X != 0 ? hypot(X, Y) : Y;
  } else
    return hypot(x, y);
}

inline real LineDistance::Displacement(const point& p) const {
  real x = p[0] - _a[0], y = p[1] - _a[1];
  return x * _ny - y * _nx;
}

inline void LineSimplifier::Simplify(std::vector<point>& p, real thresh) {
  using std::isnan;
  unsigned n = p.size();
  InternalSimplify(p, 0, n-1, thresh);
  unsigned i = 0, j = 0;
  while (i < n) {               // Squeeze out nans
    if (!isnan(p[i][0])) { p[j] = p[i]; ++j; }
    ++i;
  }
  p.resize(j);
}

void LineSimplifier::InternalSimplify(std::vector<point>& p,
                                      unsigned a, unsigned b,
                                      real thresh) {
  if (b - a + 1 <= 2) return;
  // Assume b > a+1, thresh > 0
  real maxdist = -1;
  unsigned maxind = (a + b) / 2;         // initial value isn't used
  LineDistance dist(p[a], p[b]);
  for (unsigned i = a + 1; i <= b - 1; ++i) {
    real d = dist.Distance(p[i]);
    if (d > maxdist) { maxdist = d; maxind = i; }
  }
  if (maxdist > thresh) {
    InternalSimplify(p, a, maxind, thresh);
    InternalSimplify(p, maxind, b, thresh);
  } else {
    for (unsigned i = a+1; i <= b-1; ++i)
      p[i][0] = Math::NaN();
  }
}

class PointTest {
  real _xmin, _xmax, _ymin, _ymax;
public:
  PointTest(real xmin, real xmax, real ymin, real ymax)
    : _xmin(xmin)
    , _xmax(xmax)
    , _ymin(ymin)
    , _ymax(ymax) {}
  bool operator() (const point &p) const {
    return p[0] >= _xmin && p[0] <= _xmax && p[1] >= _ymin && p[1] <= _ymax;
  }
};

class GravityInt {
  const NormalGravity& _grav;
  unsigned _dir;
public:
  GravityInt(const NormalGravity& grav, unsigned dir)
    : _grav(grav)
    , _dir(dir & 3u)
  {}
  void operator() (const point& p, point &d, const real /*t*/) const {
    real Y = 0, gX, gY, gZ;
    _grav.U(p[0], Y, p[1], gX, gY, gZ);
    Math::norm(gX, gZ);
    switch (_dir) {
    case  0: d[0] =  gX; d[1] =  gZ; break; // parallel to g
    case  1: d[0] = -gZ; d[1] =  gX; break; // counterclockwise 90d from g
    case  2: d[0] = -gX; d[1] = -gZ; break; // antiparallel to g
    case  3:
    default: d[0] =  gZ; d[1] = -gX; break; // clockwise 90d from g
    }
  }
};

class GravityFollow {
public:
  static void follow(const GravityInt& sys, const point& p0, real t0, real ds,
                     std::vector<point>& points, const PointTest& box) {
    ode::result_of::make_dense_output
      < ode::runge_kutta_dopri5< point, real > >::type integrator =
      ode::make_dense_output(real(1.0e-8), real(0*1.0e-8),
                             ode::runge_kutta_dopri5< point, real >() );
    integrator.initialize(p0, t0, real(1e-2));
    integrator.do_step(sys);
    int n = 1, i = 1;
    point out;
    while (true) {
      real tout = i * ds;
      while (integrator.current_time() < tout) {
        integrator.do_step(sys); ++n;
      }
      integrator.calc_state(tout, out);
      points.push_back(out);
      if (!box(out)) break;
      ++i;
    }
  }
};

// Evaluate gravity potential or gradient along 1 axis
class GravityEval {
  const NormalGravity& _grav;
  bool _zaxis, _grad;
  real _disp;
public:
  GravityEval(const NormalGravity& grav, bool zaxis, bool grad, real disp)
    : _grav(grav)
    , _zaxis(zaxis)
    , _grad(grad)
    , _disp(disp) {};
  real operator() (real v) const {
    real
      X = _zaxis ? _disp : v,
      Y = 0,
      Z = _zaxis ? v : _disp,
      gX, gY, gZ,
      U = _grav.U(X, Y, Z, gX, gY, gZ);
    return _grad ? gX : U;
  }
};

class Interpolator {
private:
  int _maxit;
  real _eps;
public:
  Interpolator()
    : _maxit(20), _eps(sqrt(std::numeric_limits<real>::epsilon())) {}
  real operator() (const GravityEval& gravfun,
                   real x0, real x1, real val) {
    using std::abs;
    real y0 = gravfun(x0) - val, y1;
    for (int i = 0, trip = 0; i < _maxit; ++i) {
      y1 = gravfun(x1) - val;
      if (y1 == y0) break;
      real x2 = x1 - y1 * (x1 - x0) / (y1 - y0);
      x0 = x1; x1 = x2; y0 = y1;
      if (abs(x0 - x1) <= _eps) ++trip;
      if (trip > 2) break;
    }
    return x1;
  }
};

void dump(const std::vector<point>& points) {
  for (auto p = points.begin(); p != points.end(); ++p) {
    std::cout << (*p)[0] << "," << (*p)[1] << ";\n";
  }
}

int main() {
  Utility::set_digits();
  using std::sqrt;
  real a = 1, GM = 1, omega = 3/Math::real(10), f = 2/Math::real(10);
  real gX, gY, gZ, b = (1 - f) * a, E = a * sqrt(f * (2 - f));
  real thresh = real(0.5e-3);
  NormalGravity grav(a, GM, omega, f, true);
  real xmax = 3, ymax = 2;
  PointTest box(0, xmax, 0, ymax);
  Interpolator intpol;
  real
    X0 = a,
    U0 = grav.U(X0, 0, 0, gX, gY, gZ),
    Uc = grav.U(0, 0, 0, gX, gY, gZ);
  real Xnull, Unull;
  {
    GravityEval eval(grav, false, true, 0);
    Xnull = intpol(eval, 1, 2, 0);
    Unull = grav.U(Xnull, 0, 0, gX, gY, gZ);
  }
  int ndiv = 20;
  real del = (U0 - Unull) / 20;
  std::cout << std::setprecision(6);
  std::cout << "a=" << a << "; b=" << b << "; Rnull=" << Xnull
            << "; xmax=" << xmax << "; ymax=" << ymax << ";\n";
  std::cout << "q={}; p={}; qa={}; pa={};\n";
  std::vector<point> points;
  point p0;
  int k = 0;
  for (int i = 0; i <= 90; i += 10) {
    points.clear();
    real s, c;
    Math::sincosd(real(i), s, c);
    p0[0] = a * c; p0[1] = b * s;
    points.push_back(p0);
    if (i == 0) {
      p0[0] = xmax; p0[1] = 0;
      points.push_back(p0);
    } else if (i == 90) {
      p0[0] = 0; p0[1] = ymax;
      points.push_back(p0);
    } else {
      GravityInt sys(grav, 2u);
      GravityFollow::follow(sys, p0, real(0), 1/real(100), points, box);
    }
    LineSimplifier::Simplify(points, thresh);
    std::cout << "q{" << ++k << "}=[\n";
    dump(points);
    std::cout << "];\n";
  }
  {
    points.clear();
    p0[0] = Xnull; p0[1] = 0;
    points.push_back(p0);
    p0[0] = Xnull; p0[1] = real(0.001);
    points.push_back(p0);
    GravityInt sys(grav, 2u);
    GravityFollow::follow(sys, p0, real(0), 1/real(100), points, box);
    LineSimplifier::Simplify(points, thresh);
    std::cout << "q{" << ++k << "}=[\n";
    dump(points);
    std::cout << "];\n";
  }
  for (int i = 1; i <= 5; ++i) {
    points.clear();
    p0[0] = xmax;
    p0[1] = real(i == 1 ? 0.45 :
                 (i == 2 ? 0.9 :
                  (i == 3 ? 1.25 :
                   (i == 4 ? 1.6 : 1.9))));
    points.push_back(p0);
    if (!box(p0)) break;
    GravityInt sys(grav, 2u);
    GravityFollow::follow(sys, p0, real(0), 1/real(100), points, box);
    LineSimplifier::Simplify(points, thresh);
    std::cout << "q{" << ++k << "}=[\n";
    dump(points);
    std::cout << "];\n";
  }
  k = 0;
  for (int i = 0; i <= 90; i += 10) {
    points.clear();
    real s, c;
    Math::sincosd(real(i), s, c);
    p0[0] = a * c; p0[1] = b * s;
    points.push_back(p0);
    if (i == 0) {
      p0[0] = E; p0[1] = 0;
      points.push_back(p0);
    } else if (i == 90) {
      p0[0] = 0; p0[1] = 0;
      points.push_back(p0);
    } else {
      GravityInt sys(grav, 0u);
      GravityFollow::follow(sys, p0, real(0), 1/real(100), points, box);
    }
    LineSimplifier::Simplify(points, thresh);
    std::cout << "qa{" << ++k << "}=[\n";
    dump(points);
    std::cout << "];\n";
  }
  k = 0;
  {
    GravityEval eval(grav, false, false, 0);
    for (int i = 0; i < ndiv; ++i) {
      points.clear();
      real U = U0 - del * i;
      p0[0] = intpol(eval, X0, Xnull, U); p0[1] = 0;
      points.push_back(p0);
      GravityInt sysa(grav, 3u);
      GravityFollow::follow(sysa, p0, real(0), 1/real(100), points, box);
      LineSimplifier::Simplify(points, thresh);
      std::cout << "p{" << ++k << "}=[\n";
      dump(points);
      std::cout << "];\n";
    }
    for (int i = ndiv - 1;; --i) {
      points.clear();
      real U = U0 - del * i;
      p0[0] = intpol(eval, Xnull + 3, Xnull, U); p0[1] = 0;
      if (!box(p0)) break;
      points.push_back(p0);
      GravityInt sysa(grav, 1u);
      GravityFollow::follow(sysa, p0, real(0), 1/real(100), points, box);
      LineSimplifier::Simplify(points, thresh);
      std::cout << "p{" << ++k << "}=[\n";
      dump(points);
      std::cout << "];\n";
    }
  }
  {
    GravityEval eval(grav, true, false, 0);
    for (int i = ndiv + 1;; ++i) {
      points.clear();
      real U = U0 - del * i;
      p0[1] = intpol(eval, X0, Xnull, U); p0[0] = 0;
      if (!box(p0)) break;
      points.push_back(p0);
      GravityInt sysa(grav, 1u);
      GravityFollow::follow(sysa, p0, real(0), 1/real(100), points, box);
      LineSimplifier::Simplify(points, thresh);
      std::cout << "p{" << ++k << "}=[\n";
      dump(points);
      std::cout << "];\n";
    }
  }
  {
    real dy = real(0.02);
    GravityEval eval(grav, false, false, dy);
    points.clear();
    p0[0] = Xnull; p0[1] = 0;
    points.push_back(p0);
    p0[0] = intpol(eval, Xnull - real(0.1), Xnull, Unull); p0[1] = dy;
    points.push_back(p0);
    GravityInt sysa(grav, 3u);
    GravityFollow::follow(sysa, p0, real(0), 1/real(100), points, box);
    LineSimplifier::Simplify(points, thresh);
    std::cout << "p{" << ++k << "}=[\n";
    dump(points);
    std::cout << "];\n";
  }
  {
    real dy = real(0.02);
    GravityEval eval(grav, false, false, dy);
    points.clear();
    p0[0] = Xnull; p0[1] = 0;
    points.push_back(p0);
    p0[0] = intpol(eval, Xnull + real(0.1), Xnull, Unull); p0[1] = dy;
    points.push_back(p0);
    GravityInt sysa(grav, 1u);
    GravityFollow::follow(sysa, p0, real(0), 1/real(100), points, box);
    LineSimplifier::Simplify(points, thresh);
    std::cout << "p{" << ++k << "}=[\n";
    dump(points);
    std::cout << "];\n";
  }
  k = 0;
  {
    GravityEval eval(grav, false, false, 0);
    for (int i = 1;; ++i) {
      points.clear();
      real U = U0 + 5 * del * i;
      if (U > Uc) break;
      p0[0] = intpol(eval, 0, X0, U); p0[1] = 0;
      points.push_back(p0);
      GravityInt sysa(grav, 3u);
      GravityFollow::follow(sysa, p0, real(0), 1/real(100), points, box);
      LineSimplifier::Simplify(points, thresh);
      std::cout << "pa{" << ++k << "}=[\n";
      dump(points);
      std::cout << "];\n";
    }
  }
}
