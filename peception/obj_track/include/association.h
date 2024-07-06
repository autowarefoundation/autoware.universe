#ifndef _ASSOCIATION_H
#define _ASSOCIATION_H
#include <iostream>
#include <iomanip>
#include <vector>
#include <list>
#include <string.h>
#include <fstream>
#include <cmath>
#include "kf.h"
#include "LidarRawObjectArray.h"
#include "LidarRawObject.h"
#include "ObjectArray.h"
#include "Object.h"
#include "LaneLineArray.h"
#include "LaneLine.h"

#define PI 3.14159265

/*----------------------------------------------------------------------
@ describe : 已跟踪目标类
----------------------------------------------------------------------*/
class tracked_target:public kalman_filter{
public:
    int id;
    Target_info info_evlt;  //目标当前时刻最优估计状态
    Target_info info_pdct;  //目标预测状态
    int time_match;
    int time_disapper;
    double point[4][2];

    //目标特征模型
    std::vector<double> position;
    std::vector<double> shape;
    std::vector<double> topology;


public:
    tracked_target();
    // ~tracked_target();
    void set_tar_features(int index, const std::list<tracked_target> &tar_list ,const MatrixXd &targets_dis);
    void updata(Target_info target_info_meas);
};

/*----------------------------------------------------------------------
@ describe : 测量目标类
----------------------------------------------------------------------*/
class measure_target{
public:
    Target_info info_meas;  //目标当前时刻测量状态
    double point[4][2];

    //目标特征模型
    std::vector<double> position;
    std::vector<double> shape;
    std::vector<double> topology;


public:
    measure_target();
    //~measure_target();
    void set_tar_features(int index, const std::list<measure_target> &tar_list ,const MatrixXd &targets_dis);
};

/*----------------------------------------------------------------------
@ describe : 关联匹配类
----------------------------------------------------------------------*/
class association{
public:
    int id_pool;
    std::list<tracked_target> targets_tracked;
    std::list<measure_target> targets_measure;

    MatrixXd dis_trac_matrix;
    MatrixXd dis_meas_matrix;
    MatrixXd pos_similar;
    MatrixXd sha_similar;
    MatrixXd top_similar;
    MatrixXd tar_similar;
    MatrixXd incidence_matrix; //关联矩阵

public:
    association();
    //~association();
    void get_measure_tar_and_work(LidarRawObjectArray_h *msg);
    void compute_dis_trac();
    void compute_dis_meas();
    double Euclidean_Distance(const std::vector<double> &v1,const std::vector<double> &v2,double para);
    double Cosine_Similarity(const std::vector<double> &v1,const std::vector<double> &v2);
    void compute_similar();
    void association_process();

};



#endif //_ASSOCIATION_H
