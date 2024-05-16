#include <iostream>
#include <GeographicLib/Utility.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/GeodesicLineExact.hpp>

int main(int argc, const char* const argv[]) {
  try {
    using namespace GeographicLib;
    typedef Math::real real;
    Utility::set_digits();
    real a = 2 / Math::pi();
    if (argc != 2) {
      std::cout << "Usage: M12zero f\n";
      return 1;
    }
    real f = Utility::fract<real>(std::string(argv[1]));

    GeodesicExact g(a, f);
    GeodesicLineExact l(g, 90, 0, 0);
    real s12 = a * Math::pi()/2;
    real lat2, lon2, azi2, m12, M12, M21, ds12;
    for (int i = 0; i < 20; ++i) {
      using std::abs;
      l.Position(s12, lat2, lon2, azi2, m12, M12, M21);
      ds12 = m12 * M12 / (1 - M12 * M21);
      s12 = abs(s12 + ds12);
    }
    real q;
    g.Inverse(0,0,90,0,q);
    int p = 16 + Math::extra_digits();
    p = 20;
    std::cout << std::fixed << std::setprecision(p) << s12 << " " << q << "\n";
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
