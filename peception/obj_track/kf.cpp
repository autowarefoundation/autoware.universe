#include"kf.h"

/**
 * @brief 构造函数
 * @param 
 */
kalman_filter::kalman_filter(){
     isfirst = true;
     A = MatrixXd(8, 8);    //状态转移矩阵
     B = MatrixXd(8, 8);    //控制量
     H = MatrixXd(6, 8);    //观测矩阵
     Q = MatrixXd(8, 8);    //预测过程噪声协方差
     R = MatrixXd(6, 6);    //测量过程噪声协方差

     X_evlt = MatrixXd(8, 1); //最优估计
     X_pdct = MatrixXd(8, 1); //先验估计
     Z_meas = MatrixXd(6, 1); //观量值
     P = MatrixXd(8, 8);      //估计误差协方差
     K = MatrixXd(8, 6);      //卡尔曼增益

     //std::cout << "构造目标" << std::endl;


    A << 1, 0, 0.1, 0, 0, 0, 0, 0,
         0, 1, 0, 0.1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 1;
    //std::cout << "A:" << std::endl << A << std::endl;

    H << 1, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 1;
    //std::cout << "H:" << std::endl << H << std::endl;

    Q << 0.11, 0, 0, 0, 0, 0, 0, 0,
         0, 0.11, 0, 0, 0, 0, 0, 0,
         0, 0, 0.001, 0, 0, 0, 0, 0,
         0, 0, 0, 0.001, 0, 0, 0, 0,
         0, 0, 0, 0, 0.001, 0, 0, 0,
         0, 0, 0, 0, 0, 0.001, 0, 0,
         0, 0, 0, 0, 0, 0, 0.001, 0,
         0, 0, 0, 0, 0, 0, 0, 0.0001;
    //std::cout << "Q:" << std::endl << Q << std::endl;

    R << 0.15, 0, 0, 0, 0, 0, 
         0, 0.15, 0, 0, 0, 0, 
         0, 0, 0.22, 0, 0, 0, 
         0, 0, 0, 0.22, 0, 0, 
         0, 0, 0, 0, 0.22, 0,
         0, 0, 0, 0, 0, 0.84;
    //std::cout << "R:" << std::endl << R << std::endl;

    P << MatrixXd::Identity(8,8)*10;
    //std::cout << "P:" << std::endl << P << std::endl;
    X_evlt.fill(0.0);
    //std::cout << "X_evlt:" << std::endl << X_evlt << std::endl;

    
}

/**
 * @brief 析构函数
 * @param 
 */
kalman_filter::~kalman_filter(){
    
}

/**
 * @brief 卡尔曼迭代过程--预测--更新
 * @param 
 */
std::vector<Target_info> kalman_filter::kalman_process(target_info measure){
     if(isfirst){
          X_evlt(0,0) =  measure.x_pos;
          X_evlt(1,0) =  measure.y_pos;
          isfirst = false;
     }
    //1.状态估计
    X_pdct = A * X_evlt;
    //2.预测状态与真实状态的协方差矩阵，P
    P = A * P * A.transpose() + Q;
    //3.计算卡尔曼增益
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    //4.最优估计，状态更新
    Z_meas << measure.x_pos, measure.y_pos, measure.l, measure.w, measure.h, measure.heading;
    X_evlt = X_pdct + K * (Z_meas - H * X_pdct);
    //5.估计状态和真实状态的协方差矩阵更新
    P = (MatrixXd::Identity(8,8) - K * H) * P;

    std::vector<Target_info> ret;
    Target_info pdct,evlt;
    pdct.x_pos = X_pdct(0,0);
    pdct.y_pos = X_pdct(1,0);
    pdct.vx    = X_pdct(2,0);
    pdct.vy    = X_pdct(3,0);
    //pdct.heading  = atan2(pdct.vy,pdct.vx);
    pdct.l     = X_pdct(4,0);
    pdct.w     = X_pdct(5,0);
    pdct.h     = X_pdct(6,0);
    pdct.heading  = X_pdct(7,0);
    ret.push_back(pdct);

    evlt.x_pos = X_evlt(0,0);
    evlt.y_pos = X_evlt(1,0);
    evlt.vx    = X_evlt(2,0);
    evlt.vy    = X_evlt(3,0);
    //evlt.heading   = atan2(evlt.vy,evlt.vx);
    evlt.l     = X_evlt(4,0);
    evlt.w     = X_evlt(5,0);
    evlt.h     = X_evlt(6,0);
    evlt.heading  = X_evlt(7,0);
    ret.push_back(evlt);
    return ret;

}