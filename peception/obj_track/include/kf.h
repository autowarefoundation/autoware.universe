#ifndef	_KF_H_
#define	_KF_H_

#include<iostream>
#include<vector>
#include<Eigen/Dense>

using namespace Eigen;
using namespace std;

typedef struct target_info{
    double x_pos;  //目标x坐标
    double y_pos;  //目标y坐标
    double heading;//航向
    double vx;     //目标x方向速度，m/s
    double vy;     //目标y方向速度,m/s
    double l;      //长
    double w;      //宽
    double h;      //高
    target_info():x_pos(0),y_pos(0),heading(0),vx(0),vy(0),l(0),w(0),h(0){};
} Target_info;

class kalman_filter{
private:
    MatrixXd A;    //状态转移矩阵
    MatrixXd B;    //控制量
    MatrixXd H;    //观测矩阵
    MatrixXd Q;    //预测过程噪声协方差
    MatrixXd R;    //测量过程噪声协方差

    MatrixXd X_evlt; //最优估计
    MatrixXd X_pdct; //先验估计
    MatrixXd Z_meas; //观量值
    MatrixXd P;      //估计误差协方差
    MatrixXd K;      //卡尔曼增益

    bool isfirst;

public:
    kalman_filter();
    ~kalman_filter();
    std::vector<Target_info> kalman_process(target_info measure);
};

#endif