#include <gtest/gtest.h>
#include "kalman_filter/kalman_filter.hpp"


TEST(kalman_filter, kf){
  KalmanFilter kf_;

  Eigen::MatrixXd x_t (2, 1);
  x_t << 1, 2;

  Eigen::MatrixXd P_t(2, 2);
  P_t << 1, 0, 0, 1; 

  Eigen::MatrixXd Q_t(2, 2);
  Q_t << 0.01, 0, 0, 0.01;

  Eigen::MatrixXd R_t(2, 2);
  R_t << 0.09, 0, 0, 0.09;

  Eigen::MatrixXd C_t(2, 2);
  C_t << 1, 0, 0, 1;

  Eigen::MatrixXd A_t(2, 2);
  A_t << 1, 0, 0, 1;

  Eigen::MatrixXd B_t(2, 2);
  B_t << 1, 0, 0, 1;

  //init
  EXPECT_TRUE(kf_.init(x_t, A_t, B_t, C_t, Q_t, R_t, P_t));

  // predict
  Eigen::MatrixXd u_t(2,1); 
  u_t << 0.1, 0.1;
  EXPECT_TRUE(kf_.predict(u_t));

  // check
  Eigen::MatrixXd x_predict_expected = A_t * x_t + B_t * u_t;

  Eigen::MatrixXd x_predict;
  kf_.getX(x_predict);

  EXPECT_NEAR(x_predict(0,0), x_predict_expected(0,0), 1e-3);
  EXPECT_NEAR(x_predict(1,0), x_predict_expected(1,0), 1e-3);

  // update
  Eigen::MatrixXd y_t(2,1);
  y_t << 1.05, 2.05;
  EXPECT_TRUE(kf_.update(y_t));

  // check
  const Eigen::MatrixXd PCT_t = P_t * C_t.transpose();
  const Eigen::MatrixXd K_t = PCT_t * ((R_t + C_t * PCT_t).inverse());
  const Eigen::MatrixXd y_pred = C_t * x_predict_expected;
  Eigen::MatrixXd x_update_expected = x_predict_expected + K_t * (y_t - y_pred);

  Eigen::MatrixXd x_update;
  kf_.getX(x_update);

  EXPECT_NEAR(x_update(0,0), x_update_expected(0,0), 1e-3); 
  EXPECT_NEAR(x_update(1,0), x_update_expected(1,0), 1e-3);

}


int main(int argc, char * argv[]) 
{
  testing::InitGoogleTest(&argc, argv);
  bool result = RUN_ALL_TESTS();
  return result;
}