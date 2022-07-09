#include "imgproc/orientation_optimizer.hpp"

#include <ceres/ceres.h>

#include <cstdio>
#include <vector>

namespace imgproc::opt
{
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DistanceFromCircleCost
{
public:
  DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
  template <typename T>
  bool operator()(
    const T * const x, const T * const y,
    const T * const m,  // r = m^2
    T * residual) const
  {
    T r = *m * *m;

    T xp = xx_ - *x;
    T yp = yy_ - *y;
    residual[0] = r * r - xp * xp - yp * yp;
    return true;
  }

private:
  double xx_, yy_;
};

void sample()
{
  double x = 1, y = 1, r = 3;
  double m = std::sqrt(r);

  Problem problem;
  LossFunction * loss = nullptr;

  for (int i = 0; i < 4; i++) {
    double xx = 4 * std::cos(i * 6.28 / 4);
    double yy = 4 * std::sin(i * 6.28 / 4);
    CostFunction * cost = new AutoDiffCostFunction<DistanceFromCircleCost, 1, 1, 1, 1>(
      new DistanceFromCircleCost(xx, yy));
    problem.AddResidualBlock(cost, loss, &x, &y, &m);
  }

  Solver::Options options;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::DENSE_QR;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  r = m * m;
  std::cout << summary.BriefReport() << std::endl;
}
}  // namespace imgproc::opt