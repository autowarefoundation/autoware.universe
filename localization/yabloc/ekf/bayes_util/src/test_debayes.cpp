#include "bayes_util/bayes_util.hpp"

#include <gtest/gtest.h>

void test(double a, double b, double c, double d)
{
  Eigen::Matrix2d S;
  S << a, b, c, d;

  Eigen::Matrix2d approx = pcdless::bayes_util::approximate_by_spd(S, true);
  std::cout << "target:\n " << S << std::endl;
  std::cout << "opt:\n " << approx << std::endl;
  std::cout << std::endl;
}

TEST(BayesUtilTestSuite, debayes)
{
  test(2, 1, 1, 1);

  test(2, 2, 2, 1);

  test(4.1, 2, 2, 1);
}
