#include <iostream>
#include <iomanip>
#include <GeographicLib/EllipticFunction.hpp>
#include <GeographicLib/Ellipsoid.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (disable: 4127)
#endif

using namespace GeographicLib;
using namespace std;
int main() {
  typedef GeographicLib::Math::real real;
  try {
    if (true) {
      for (int i = -7; i <= 7; ++i) {
        if (i == 0)
          continue;
        real a = 1, b = a * real(pow(real(2), i)), f = (a-b)/a;
        Ellipsoid e(a, f);
        real m = e.QuarterMeridian();
        real na = a * real(1e7)/m, nb = na * (1-f);
        Ellipsoid ne(na, f);
        cout << b/a << " " << na << " " << nb << " "
             << ne.QuarterMeridian() << "\n";
      }
      for (int i = -1; i <= 1; ++i) {
        if (i == 0)
          continue;
        real a = 1, b = a * real(pow(real(3), i)), f = (a-b)/a;
        Ellipsoid e(a, f);
        real m = e.QuarterMeridian();
        real na = a * real(1e7)/m, nb = na * (1-f);
        Ellipsoid ne(na, f);
        cout << b/a << " " << na << " " << nb << " "
             << ne.QuarterMeridian() << "\n";
      }
      return 0;
    }
    if (false) {
      // Longitude check
      real a = real(1.5), b = 1, f = (a - b)/a,
        e2 = (a*a - b*b)/(a*a), ep2 = (a*a - b*b)/(b*b),
        alp0 = real(0.85), calp0 = cos(alp0), salp0 = sin(alp0),
        k2 = ep2 * Math::sq(calp0), kp2 = k2/(1 + k2),
        sigma = real(1.2), theta = atan(sqrt(1+k2) * tan(sigma)),
        kap2 = Math::sq(calp0)/(1-Math::sq(salp0)*e2),
        omg = atan(salp0 * tan(sigma)),
        ups = atan(sqrt((1+ep2)/(1+k2*Math::sq(sin(sigma)))) * tan(omg)),
        // psi = atan(sqrt((1+k2*Math::sq(sin(sigma)))/(1+ep2)) * tan(omg)),
        psi = atan(sqrt((1-e2)/(1+k2*Math::sq(cos(theta)))) *
                   salp0*tan(theta));
      EllipticFunction ella1(-k2, Math::sq(calp0));
      EllipticFunction ella2(-k2, -ep2);
      EllipticFunction ellb1(kp2, kap2);
      EllipticFunction ellb2(kp2, e2);
      EllipticFunction ellc(kp2, kp2);
      cout << setprecision(15);
      cout << (1 - f) * salp0 * ella1.G(sigma) << " "
           << ups - ep2/sqrt(1+ep2) * salp0 * ella2.H(sigma) << " "
           << (1-f) * sqrt(1-kp2) * salp0 * ellb1.Pi(theta) << " "
           << psi + (1-f) * sqrt(1-kp2) * salp0 *
        (ellb2.F(theta) - ellb2.Pi(theta)) << "\n";
      cout << b*ella1.E(sigma) << " "
           << b * sqrt(1-kp2) * ellc.Pi(theta) << " "
           << b / sqrt(1-kp2) *
        (ellc.E(theta) - kp2 * cos(theta) * sin(theta)/
         sqrt(1 - kp2*Math::sq(sin(theta)))) << "\n";
      return 0;
    }
    if (false) {
      real b = 1, a = 10,
        e2 = (a*a - b*b)/(a*a), ep2 = (a*a - b*b)/(b*b);
      EllipticFunction elle(e2, e2);
      EllipticFunction ellep(-ep2, -ep2);
      cout << fixed << setprecision(8);
      for (int i = 0; i <= 90; i += 5) {
        real
          beta = i * Math::degree(),
          phi = atan(sqrt(1 + ep2) * tan(beta)),
          u = elle.F(phi),
          y = b * ellep.E(beta),
          M = a*elle.E();
        cout << (y / M) * 90 << " "
                  << i << " "
                  << phi / Math::degree() << " "
                  << (u / elle.K()) * 90 << "\n";
      }
      /* Create plot with
t=load('data.txt');
plot(t(:,1),t(:,1),'-k',...
     t(:,1),t(:,2),'-kx',...
     t(:,1),t(:,4),'-ko',...
     t(:,1),t(:,3),'-k+')
axis equal;axis([0,90,0,90]);
title('meridian measures for a/b = 10');
xlabel('meridian distance (\circ)');
ylabel('measure (\circ)');
legend('distance',...
       'parametric latitude',...
       'elliptic variable',...
       'geographic latitude',...
       'Location','SouthEast');
set(gcf,'PaperSize',[4.5,4.5]);
set(gcf,'PaperPosition',[0,0,4.5,4.5]);
print meridian-measures.png -dpng

       */
      return 0;
    }

    if (false) {
      real alpha2 = real(0.8), k2 = real(-0.4);
      EllipticFunction ellG(k2,alpha2);
      EllipticFunction ellH(k2,k2/alpha2);

      cout << setprecision(10);
      for (int i = -179; i <= 180; i += 10) {
        real
          phi = i * Math::degree(),
          sn = sin(phi), cn = cos(phi), dn = ellG.Delta(sn, cn),
          g = ellG.G(phi),
          h = (k2/alpha2)*ellH.H(phi) + sqrt(1-k2/alpha2)/sqrt(1-alpha2)*
          atan2(sqrt(1-alpha2)*sqrt(1-k2/alpha2)*sn, dn*cn);

        cout << i << " " << g << " " << h << " " << h-g << "\n";
      }
      return 0;
    }
    if (false) {
      // For tabulated values in A+S
      real
        ASalpha = 30*Math::degree<real>(),
        k2 = Math::sq(sin(ASalpha)),
        alpha2 = real(0.3);

      EllipticFunction ell(k2, alpha2);
      real dphi = Math::degree();
      cout << fixed << setprecision(10);
      for (int i = 0; i <= 90; i += 15) {
        real phi = i * dphi;
        cout << i << " "
                  << ell.F(phi) << " "
                  << ell.E(phi) << " "
                  << ell.D(phi) << " "
                  << ell.Pi(phi) << " "
                  << ell.G(phi) << " "
                  << ell.H(phi) << "\n";
      }
      return 0;
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
