//
// Created by ali on 14/07/22.
//


#include "utils_act/writetopath.hpp"
#include "utils_act/act_utils.hpp"

int main()
{
  // Create a continuous signal for an angle series.
  auto log_path = getOutputPath() / "helper_funcs";

  double Nx = 512;
  std::vector<double> xvec = ns_utils::linspace<double>(0.0, Nx, Nx);

  // Generate y = 6*sin(2*pi*n/N).

  std::vector<double> yvec;
  std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [&](auto const &x)
  {
      return 6 * sin(2 * M_PI * x / Nx);
  });

  writeToFile(log_path, yvec, "xc");

  /**
   * Wrap the signal into [-pi, pi]
   * */

  std::vector<double> xw;
  std::transform(yvec.cbegin(), yvec.cend(), std::back_inserter(xw), [&](auto const &x)
  {
      return std::atan2(sin(x), cos(x));
  });

  writeToFile(log_path, xw, "xw");

  /**
   * unWrap the wrapped signal xw.
   * */

  ns_utils::unWrap(xw);
  writeToFile(log_path, xw, "uw_x");

  return 0;
}