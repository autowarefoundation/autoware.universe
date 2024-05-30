#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <random>
#include <algorithm>

#include <GeographicLib/NormalGravity.hpp>
#include <GeographicLib/Utility.hpp>

using namespace std;
using namespace GeographicLib;

void checkdiff(const NormalGravity& g,
               Math::real X, Math::real Y, Math::real Z,
               Math::real eps, Math::real tol) {
  Math::real gX, gY, gZ, t,
    V   = g.V0(X, Y, Z, gX, gY, gZ),
    VXp = g.V0(X+eps, Y, Z, t, t, t), VXm = g.V0(X-eps, Y, Z, t, t, t),
    VYp = g.V0(X, Y+eps, Z, t, t, t), VYm = g.V0(X, Y-eps, Z, t, t, t),
    VZp = g.V0(X, Y, Z+eps, t, t, t), VZm = g.V0(X, Y, Z-eps, t, t, t),
    ggX = (VXp - VXm) / (2*eps),
    ggY = (VYp - VYm) / (2*eps),
    ggZ = (VZp - VZm) / (2*eps),
    lap = (VXp + VXm + VYp + VYm + VZp + VZm - 6 * V) / Math::sq(eps);
  if (!(abs(ggX - gX) < tol &&
        abs(ggY - gY) < tol &&
        abs(ggZ - gZ) < tol &&
        abs(lap) < tol))
    cout << "Failure with f = " << g.Flattening() << " X,Y,Z = "
         << X << " " << Y << " " << Z << "\n"
         << "     " << ggX - gX << " " << ggY - gY << " " << ggZ - gZ
         << " " << lap << "\n";
}

// Copied from NormalGravity (w/o the series expansion)
Math::real atanzz(Math::real x, bool alt) {
  // This routine obeys the identity
  //   atanzz(x, alt) = atanzz(-x/(1+x), !alt)
  //
  // Require x >= -1.  Best to call with alt, s.t. x >= 0; this results in
  // a call to atan, instead of asin, or to asinh, instead of atanh.
  Math::real z = sqrt(abs(x));
  return x == 0 ? 1 :
    (alt
     ? (!(x < 0) ? asinh(z) : asin(z)) / sqrt(abs(x) / (1 + x))
     : (!(x < 0) ? atan(z) : atanh(z)) / z);
}

// Copied from NormalGravity (w/o the series expansion)
Math::real Qf(Math::real x, bool alt) {
  // Compute
  //   Q(z) = (((1 + 3/z^2) * atan(z) - 3/z)/2) / z^3
  //        = q(z)/z^3 with q(z) defined by H+M, Eq 2-57 with z = E/u
  //   z = sqrt(x)
  Math::real y = alt ? -x / (1 + x) : x;
  return ((1 + 3/y) * atanzz(x, alt) - 3/y) / (2 * y);
}

int main() {
  Utility::set_digits();
  Math::real
    eps = sqrt(sqrt(numeric_limits<Math::real>::epsilon())),
    tol = eps;
  cout << setprecision(13);
  cout << "eps = " << eps << "\n";
  unsigned s = random_device()(); // Set seed from random_device
  mt19937 r(s);                   // Initialize URNG
  if (1)  {
    Math::real GM = 1, a = 1, omega = 1;
    {
      Math::real f = 1/Math::real(5);
      NormalGravity g(a, GM, omega, f, true);
      Math::real gX, gY, gZ, U, X = Math::real(5)/10, Y = 0,
        R = hypot(X, Y),
        b = a*(1-f),  E2 = abs(a*a - b*b), E = sqrt(E2);
      U = g.V0(X, Y, 0, gX, gY, gZ);
      cout << U << " " << gX << " " << gY << " " << gZ << "\n";
      U = g.V0(X, Y, eps, gX, gY, gZ);
      cout << U << " " << gX << " " << gY << " " << gZ << "\n";
      cout << "gz = "
           << -(GM/(E*sqrt(E2-R*R)) +
                Math::sq(a*b*omega)*b /
                (E2*E2*E*Qf(Math::sq(E/b), false)) *
                (2*E2/3 - R*R)/sqrt(E2-R*R)) << "\n";
    }
    {
      Math::real f = -1/Math::real(5);
      NormalGravity g(a, GM, omega, f, true);
      Math::real gX, gY, gZ, U, Z = Math::real(4)/10, Y = 0,
        b = a*(1-f),  E2 = abs(a*a - b*b), E = sqrt(E2);
      U = g.V0(0, Y, Z, gX, gY, gZ);
      cout << U << " " << gX << " " << gY << " " << gZ << "\n";
      U = g.V0(eps, Y, Z, gX, gY, gZ);
      cout << U << " " << gX << " " << gY << " " << gZ << "\n";
      cout << "-R*gR = " << eps*hypot(gX, gY) << " "
           << 2*(GM/(2*E) + Math::sq(a*b*omega)*b /
                 (E2*E2*E*Qf(Math::sq(E/a), true)) *
                 (3*Z*Z - E2)/12) << "\n";
    }
    {
      Math::real f = 0;
      NormalGravity g(a, GM, omega, f, true);
      Math::real gX, gY, gZ, U,
        X = Math::real(1)/10, Z = Math::real(4)/10, Y = 0,
        b = a*(1-f), R = hypot(X, Z), beta = atan2(Z, X);
      U = g.V0(X, Y, Z, gX, gY, gZ);
      cout << U << " " << gX << " " << gY << " " << gZ << "\n";
      cout << "U = " << GM/R + Math::sq(a*b*omega)*b/(R*R*R) *
        (3*Math::sq(sin(beta)) - 1)/6 << "\n";
      cout << "gR = " << gX*cos(beta) + gZ*sin(beta) << " "
           << -GM/(R*R) - (Math::sq(a*b*omega)*b/(R*R*R*R) *
                           (3*Math::sq(sin(beta)) - 1)/2)
           << " gbeta = " << -gX*sin(beta) + gZ*cos(beta) << " "
           << Math::sq(a*b*omega)*b/(R*R*R*R) * sin(beta)*cos(beta) << "\n";
    }
    return 0;
  }
  uniform_real_distribution<double> dis;
  vector<Math::real> f = {-99, -0.1, 0, 0.1, 0.99};
  Math::real GM = 1, omega = 1;
  int num = 1000;
  for (size_t i = 0; i < f.size(); ++i) {
    Math::real a = 1 / sqrt(1 - f[i]);
    cout << a << " " << a*(1-f[i]) << "\n";
    NormalGravity g(a, GM, omega, f[i], true);
    for (int j = 0; j < num; ++j) {
      Math::real
        X = Math::real(dis(r)), Y = Math::real(dis(r)), Z = Math::real(dis(r));
      checkdiff(g, X, Y, Z, eps, tol);
    }
  }
}
