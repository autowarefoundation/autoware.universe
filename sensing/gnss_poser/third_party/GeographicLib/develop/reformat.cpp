#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <cmath>

int cosind(int N, int /*M*/, int n, int m) {
  return m * N - m * (m - 1) / 2 + n;
}

int sinind(int N, int M, int n, int m) {
  return cosind(N, M, n, m) - (N + 1);
}

void storecos(std::vector<double>& C, int N, int M,
              double v, int n, int m) {
  if (v == 0)
    return;
  if (n < 0 || n > N || m < 0 || m > M)
    throw std::runtime_error("Invalid coefficient");
  C[cosind(N, M, n, m)] = v;
}

void storesin(std::vector<double>& S, int N, int M,
              double v, int n, int m) {
  if (v == 0)
    return;
  if (n < 0 || n > N || m <= 0 || m > M)
    throw std::runtime_error("Invalid coefficient");
  S[sinind(N, M, n, m)] = v;
}

void storecoeff(std::ostream& str,
                const std::vector<double>& C, const std::vector<double>& S,
                int M, int N) {
  int K = (M + 1) * (2*N - M + 2) / 2;
  int ind[2] = {N, M};
  str.write(reinterpret_cast<const char*>(ind), 2 * sizeof(int));
  str.write(reinterpret_cast<const char*>(&C[0]), K * sizeof(double));
  str.write(reinterpret_cast<const char*>(&S[0]), (K-N-1) * sizeof(double));
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: reformat model outfile\n";
    return 1;
  }
  std::string model = argv[1];
  std::string file = argv[2];
  try {
    // model = wmm2010
    //http://ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2010_Sph_Windows_Linux.zip
    // unpack
    // coefficients are in EMM_Sph_Windows_Linux/WMM2010.COF
    // N=739, M=718
    std::ofstream fout(file.c_str(), std::ios::binary);
    if (model == "emm2010") {
      // download
      // http://ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2010_Sph_Windows_Linux.zip
      // unpack
      // coefficients are in EMM_Sph_Windows_Linux/EMM2010.COF
      // and EMM_Sph_Windows_Linux/EMM2010SV.COF
      std::string id = "EMM2010A";
      fout.write(id.c_str(), 8);
      for (int i = 0; i < 2; ++i) {
        std::string filename = i == 0 ? "EMM_Sph_Windows_Linux/EMM2010.COF" :
          "EMM_Sph_Windows_Linux/EMM2010SV.COF";
        int
          N = i == 0 ? 739 : 16,
          M = i == 0 ? 718 : 16,
          K = (M + 1) * (2*N - M + 2) / 2;
        std::vector<double> C(K, 0.0);
        std::vector<double> S(K - (N + 1), 0.0);
        std::ifstream fin(filename.c_str());
        std::string ss;
        if (i == 0) std::getline(fin, ss); // Skip first line
        while (std::getline(fin, ss)) {
          int n, m;
          double c, s;
          std::istringstream is(ss);
          if (!(is >> n >> m >> c >> s))
            throw std::runtime_error("Short read");
          storecos(C, N, M, c, n, m);
          storesin(S, N, M, s, n, m);
        }
        storecoeff(fout, C, S, M, N);
      }
    } else if (model == "emm2015") {
      // download
      //http://www.ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2015_Sph_Linux.zip
      // unpack
      //
      // * The only coefficients needed are EMM20{00-15}.COF and EMM2015SV.COF.
      //
      // * The other SV files can be ignored because the time dependence can be
      //   obtained using linear interpolation for dates < 2015 (or
      //   extrapolation for dates < 2000).
      //
      // * The time varying part of the field is of degree 15.  This
      //   constitutes all the coefficients in EMM20{00-14}.COF and
      //   EMM2015SV.COF and a subset of the coefficients in EMM2015.COF.
      //
      // * To this should be added a time independent short wavelength field
      //   which is given by the degree > 15 terms in EMM2015.COF.
      //
      // * These time independent terms are of degree 729 and order 718.  There
      //   are higher degree and order coefficients listed in the file, but
      //   these are zero.
      //
      // * The EMM2015 coefficients as used by GeographicLib compress much
      //   better than those for EMM2010 (660 kB instead of 3700 kB).
      //   Presumably this is because the EMM2015 are only given with 4 decimal
      //   digits.
      //
      // * The GeographicLib implementation produces the same results as listed
      //   in EMM2015_TEST_VALUES.txt

      std::string id = "EMM2015A";
      fout.write(id.c_str(), 8);
      for (int i = 0; i <= 17; ++i) {
        std::string filename;
        {
          std::ostringstream os;
          os << "EMM2015_linux/EMM" << (2000 + std::min(15, i))
             << (i == 16 ? "SV" : "") << ".COF";
          filename = os.str();
        }
        int
          N = i == 17 ? 729 : 15,
          M = i == 17 ? 718 : 15,
          K = (M + 1) * (2*N - M + 2) / 2;
        std::vector<double> C(K, 0.0);
        std::vector<double> S(K - (N + 1), 0.0);
        std::ifstream fin(filename.c_str());
        std::string ss;
        if (i != 16) std::getline(fin, ss); // Skip first line
        while (std::getline(fin, ss)) {
          int n, m;
          double c, s;
          std::istringstream is(ss);
          if (!(is >> n >> m >> c >> s))
            throw std::runtime_error("Short read " + filename + ": " + ss);
          if (i == 15 && n > 15)
            continue;
          if (i == 17 && n <= 15)
            continue;
          storecos(C, N, M, c, n, m);
          storesin(S, N, M, s, n, m);
        }
        storecoeff(fout, C, S, M, N);
      }
    } else if (model == "emm2017") {
      // download
      //https://www.ngdc.noaa.gov/geomag/EMM/data/geomag/EMM2017_Sph_Linux.zip
      // unpack
      //
      // * The only coefficients needed are EMM20{00-17}.COF and EMM2017SV.COF.
      //
      // * The other SV files can be ignored because the time dependence can be
      //   obtained using linear interpolation for dates < 2017 (or
      //   extrapolation for dates < 2000).
      //
      // * The time varying part of the field is of degree 15.  This
      //   constitutes all the coefficients in EMM20{00-16}.COF and
      //   EMM2017SV.COF and a subset of the coefficients in EMM2017.COF.
      //
      // * To this should be added a time independent short wavelength field
      //   which is given by the degree > 15 terms in EMM2017.COF.
      //
      // * These time independent terms are of degree 790 and order 790.
      //
      // * The GeographicLib implementation produces the same results as listed
      //   in EMM2017TestValues.txt

      std::string id = "EMM2017A";
      fout.write(id.c_str(), 8);
      int maxy = 17;
      for (int i = 0; i <= 19; ++i) {
        // i = maxy for low res components for 2000+maxy
        // i = maxy+1 for SV at 2000+maxy
        // i = maxy+2 for high res components for 2000+maxy
        std::string filename;
        {
          std::ostringstream os;
          os << "EMM2017_Linux/EMM" << (2000 + std::min(maxy, i))
             << (i == maxy+1 ? "SV" : "") << ".COF";
          filename = os.str();
        }
        int
          N = i == maxy+2 ? 790 : 15,
          M = i == maxy+2 ? 790 : 15,
          K = (M + 1) * (2*N - M + 2) / 2;
        std::vector<double> C(K, 0.0);
        std::vector<double> S(K - (N + 1), 0.0);
        std::ifstream fin(filename.c_str());
        std::string ss;
        if (i != maxy+1) std::getline(fin, ss); // Skip first line
        while (std::getline(fin, ss)) {
          int n, m;
          double c, s;
          std::istringstream is(ss);
          if (!(is >> n >> m >> c >> s))
            throw std::runtime_error("Short read " + filename + ": " + ss);
          if (i == maxy && n > 15)
            continue;
          if (i == maxy+2 && n <= 15)
            continue;
          storecos(C, N, M, c, n, m);
          storesin(S, N, M, s, n, m);
        }
        storecoeff(fout, C, S, M, N);
      }
    } else if (model == "wmm2010" || model == "igrf11") {
      // Download
      // http://ngdc.noaa.gov/IAGA/vmod/geomag70_linux.tar.gz
      // unpack
      // wmm2010 coefficients are in geomag70_linux/WMM2010.COF
      // igrf11 coefficients are in geomag70_linux/IGRF11.COF
      std::string id = model == "wmm2010" ? "WMM2010A" : "IGRF11-A";
      fout.write(id.c_str(), 8);
      std::string filename = model == "wmm2010" ? "geomag70_linux/WMM2010.COF"
        : "geomag70_linux-2010/IGRF11.COF";
      std::ifstream fin(filename.c_str());
      std::string ss;
      bool start = true;
      std::getline(fin, ss);
      std::vector<double> C;
      std::vector<double> S;
      std::vector<double> C1;
      std::vector<double> S1;
      int N = 0, M = 0, N1 = 0, M1 = 0;
      while (true) {
        if (ss.size() == 0)
          break;
        std::istringstream is(ss);
        int n, m;
        double c, s, c1, s1;
        if (start) {
          std::string mm;
          if (!(is >> mm >> mm >> N >> N1))
            throw std::runtime_error("Short read on header");
          M = N; M1 = N1;
          int
            K = (M + 1) * (2*N - M + 2) / 2,
            K1 = (M1 + 1) * (2*N1 - M1 + 2) / 2;
          C.resize(K);
          S.resize(K - (N + 1));
          C1.resize(K1);
          S1.resize(K1 - (N1 + 1));
          std::fill(C.begin(), C.end(), 0.0);
          std::fill(S.begin(), S.end(), 0.0);
          std::fill(C1.begin(), C1.end(), 0.0);
          std::fill(S1.begin(), S1.end(), 0.0);
          start = false;
        } else {
          if (!(is >> n >> m >> c >> s >> c1 >> s1))
            throw std::runtime_error("Short read on data");
          storecos(C, N, M, c, n, m);
          storesin(S, N, M, s, n, m);
          storecos(C1, N1, M1, c1, n, m);
          storesin(S1, N1, M1, s1, n, m);
        }
        if (!std::getline(fin, ss))
          ss = "";
        if (ss.size() == 0 || (ss.size() >= 3 && ss.substr(0, 3) == "   ")) {
          if (ss.size() > 0 && N1 != 0)
            throw std::runtime_error("Secular coeffs not last");
          if (ss.size() == 0 && N1 == 0)
            throw std::runtime_error("No secular coeffs");
          storecoeff(fout, C, S, M, N);
          if (ss.size() == 0)
            storecoeff(fout, C1, S1, M1, N1);
          start = true;
        }
      }
    } else if (model == "wmm2015" || model == "wmm2015v2" ||
               model == "wmm2020" || model == "igrf12" ||
               model == "igrf13") {
      // Download WMM2015COF.zip
      // http://ngdc.noaa.gov/geomag/WMM/WMM_coeff.shtml
      // wmm2015 coefficients are in WMM2015COF/WMM.COF
      //
      // Download WMM2015v2COF.zip
      // https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015v2COF.zip
      // wmm2015v2 coefficients are in WMM2015v2COF/WMM.COF
      //
      // Download WMM2020COF.zip
      // http://ngdc.noaa.gov/geomag/WMM/WMM_coeff.shtml
      // wmm2020 coefficients are in WMM2020COF/WMM.COF
      //
      // igrf12 coefficients
      // http://ngdc.noaa.gov/IAGA/vmod/geomag70_linux.tar.gz
      // igrf12 coefficients are in geomag70_linux/IGRF12.COF
      //
      // igrf13 coefficients
      // https://www.ngdc.noaa.gov/IAGA/vmod/geomag70_linux.tar.gz
      // igrf13 coefficients are in geomag70_linux/IGRF13.COF
      std::string id = model == "wmm2015" ? "WMM2015A" :
        (model == "wmm2015v2" ?  "WMM2015B" :
         (model == "wmm2020" ? "WMM2020A" :
          (model == "igrf12" ? "IGRF12-A" : "IGRF13-A")));
      fout.write(id.c_str(), 8);
      std::string filename = model == "wmm2015" ? "WMM2015COF/WMM.COF"
        : (model == "wmm2015v2" ? "WMM2015v2COF/WMM.COF"
           : (model == "wmm2020" ? "WMM2020COF/WMM.COF"
              : (model == "igrf12" ? "geomag70_linux-2015/IGRF12.COF"
                 : "geomag70_linux-2020/IGRF13.COF")));
      std::ifstream fin(filename.c_str());
      std::string ss;
      bool start = true;
      std::getline(fin, ss);
      std::vector<double> C;
      std::vector<double> S;
      std::vector<double> C1;
      std::vector<double> S1;
      int N = 0, M = 0, N1 = 0, M1 = 0;
      while (true) {
        if (ss.size() == 0)
          break;
        std::istringstream is(ss);
        int n, m;
        double c, s, c1, s1;
        if (start) {
          if (model == "wmm2015" || model == "wmm2015v2" ||
              model == "wmm2020") {
            N = 12; N1 = 12;
          } else {
            std::string mm;
            if (!(is >> mm >> mm >> N >> N1))
              throw std::runtime_error("Short read on header");
          }
          M = N; M1 = N1;
          int
            K = (M + 1) * (2*N - M + 2) / 2,
            K1 = (M1 + 1) * (2*N1 - M1 + 2) / 2;
          C.resize(K);
          S.resize(K - (N + 1));
          C1.resize(K1);
          S1.resize(K1 - (N1 + 1));
          std::fill(C.begin(), C.end(), 0.0);
          std::fill(S.begin(), S.end(), 0.0);
          std::fill(C1.begin(), C1.end(), 0.0);
          std::fill(S1.begin(), S1.end(), 0.0);
          start = false;
        } else {
          if (!(is >> n >> m >> c >> s >> c1 >> s1)) {
            std::cerr << ss << "\n";
            throw std::runtime_error("Short read on data");
          }
          storecos(C, N, M, c, n, m);
          storesin(S, N, M, s, n, m);
          storecos(C1, N1, M1, c1, n, m);
          storesin(S1, N1, M1, s1, n, m);
        }
        if (!std::getline(fin, ss))
          ss = "";
        if (model == "wmm2015" || model == "wmm2015v2" ||
            model == "wmm2020") {
          if (ss.size() && ss[0] == '9')
            ss = "";
          if (ss.size() == 0) {
            storecoeff(fout, C, S, M, N);
            storecoeff(fout, C1, S1, M1, N1);
          }
        } else {
          if (ss.size() == 0 || (ss.size() >= 3 && ss.substr(0, 3) == "   ")) {
            if (ss.size() > 0 && N1 != 0)
              throw std::runtime_error("Secular coeffs not last");
            if (ss.size() == 0 && N1 == 0)
              throw std::runtime_error("No secular coeffs");
            storecoeff(fout, C, S, M, N);
            if (ss.size() == 0)
              storecoeff(fout, C1, S1, M1, N1);
            start = true;
          }
        }
      }
    } else if (model == "egm2008" || model == "egm96" || model == "egm84"
               || model == "wgs84") {
      std::string id = model == "egm2008" ? "EGM2008A" :
        (model == "egm96" ? "EGM1996A" :
         (model == "egm84" ? "EGM1984A" : "WGS1984A"));
      fout.write(id.c_str(), 8);
      for (int i = 0; i < 2; ++i) {
        std::string filename = "../gravity/";
        filename +=
          model == "egm2008" ?
          (i == 0 ? "EGM2008_to2190_TideFree" : "Zeta-to-N_to2160_egm2008") :
          (model == "egm96" ?
           (i == 0 ? "EGM96" : "CORRCOEF") :
           (model == "egm84" ?
            (i == 0 ? "egm180.nor" : "zeta84") :
            (i == 0 ? "wgs84.cof" : "zeta84")));
        std::ifstream fin(filename.c_str());
        if (!fin.good())
            throw std::runtime_error("File not found");
        int
          N = model == "egm2008" ? (i == 0 ? 2190 : 2160) :
          (model == "egm96" ? 360 :
           (model == "egm84" ? (i == 0 ? 180 : -1) :
            (i == 0 ? 20 : -1))),
          M = model == "egm2008" ? (i == 0 ? 2159 : 2160) :
          (model == "egm96" ? 360 :
           (model == "egm84" ? (i == 0 ? 180 : -1) :
            (i == 0 ? 0 : -1))),
          K = (M + 1) * (2*N - M + 2) / 2;
        std::vector<double> C(K, 0.0);
        std::vector<double> S(K - (N + 1), 0.0);
        std::string ss;
        while (std::getline(fin, ss)) {
          std::string::size_type p = 0;
          while (true) {
            p = ss.find_first_of('D', p);
            if (p < ss.size())
              ss[p] = 'E';
            else
              break;
            ++p;
          }
          std::istringstream is(ss);
          int n, m;
          double c, s;
          if (!(is >> n >> m >> c >> s))
            throw std::runtime_error("Short read");
          storecos(C, N, M, c, n, m);
          storesin(S, N, M, s, n, m);
        }
        storecoeff(fout, C, S, M, N);
      }
    } else if (model == "dtm2006") {
      std::string id = "DTM2006A";
      fout.write(id.c_str(), 8);
      int N = 2190, M = N;
      int K = (M + 1) * (2*N - M + 2) / 2;
      std::string filename
        = "../gravity/Coeff_Height_and_Depth_to2190_DTM2006.0";
      std::ifstream fin(filename.c_str());
      std::vector<double> C(K, 0.0);
      std::vector<double> S(K - (N + 1), 0.0);
      std::string ss;
      while (std::getline(fin, ss)) {
        std::string::size_type p = 0;
        while (true) {
          p = ss.find_first_of('D', p);
          if (p < ss.size())
            ss[p] = 'E';
          else
            break;
          ++p;
        }
        std::istringstream is(ss);
        int n, m;
        double c, s;
        if (!(is >> n >> m >> c >> s))
          throw std::runtime_error("Short read");
        storecos(C, N, M, c, n, m);
        storesin(S, N, M, s, n, m);
      }
      storecoeff(fout, C, S, M, N);
    } else {
      std::cerr << "UNKNOWN MODEL " << model << "\n";
      return 1;
    }
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
