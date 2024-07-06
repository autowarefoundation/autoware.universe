#include"association.h"
//--------------------------------------------已跟踪目标类------------------------------------------------------------
/**
 * @brief 构造函数
 * @param 
 */
tracked_target::tracked_target(){
    id = 0;
    time_match = 0;
    time_disapper = 0;
    position.resize(2);
    shape.resize(3);
    topology.resize(8);
}
/**
 * @brief 设置已跟踪目标特征
 * @param 
 */
void tracked_target::set_tar_features(int index, const std::list<tracked_target> &tar_list ,const MatrixXd &targets_dis){
    position[0] = info_evlt.x_pos;
    position[1] = info_evlt.y_pos;
    shape[0] = info_evlt.l; 
    shape[1] = info_evlt.w; 
    shape[2] = info_evlt.h;
    
    double angle=0;
    auto it = tar_list.begin();
    for(int i=0 ; it != tar_list.end(); it++,i++){
        angle = atan2(it->info_evlt.y_pos - info_evlt.y_pos ,
                        it->info_evlt.x_pos - info_evlt.x_pos);
        if(angle>=0 && angle<PI/4)
            topology[0] = topology[0] + targets_dis(index,i);
        if(angle>=PI/4 && angle<PI/2)
            topology[1] = topology[1] + targets_dis(index,i);
        if(angle>=PI/2 && angle<PI*3/4)
            topology[2] = topology[2] + targets_dis(index,i);
        if(angle>=PI*3/4 && angle<PI)
            topology[3] = topology[3] + targets_dis(index,i);
        if(angle>=-PI && angle<-PI*3/4)
            topology[4] = topology[4] + targets_dis(index,i);
        if(angle>=-PI*3/4 && angle<-PI/2)
            topology[5] = topology[5] + targets_dis(index,i);
        if(angle>=-PI/2 && angle<-PI/4)
            topology[6] = topology[6] + targets_dis(index,i);
        if(angle>=-PI/4 && angle<0)
            topology[7] = topology[7] + targets_dis(index,i);
    }

}

/**
 * @brief 更新，包含滤波过程
 * @param 
 */
void tracked_target::updata(Target_info target_info_meas){
    std::vector<Target_info> temp = kalman_process(target_info_meas);
    info_pdct = temp[0];
    info_evlt = temp[1];
}

//---------------------------------------测量目标类-----------------------------------------------------------------

/**
 * @brief 构造函数
 * @param 
 */
measure_target::measure_target(){
    position.resize(2);
    shape.resize(3);
    topology.resize(8);
}

/**
 * @brief 设置测量目标特征
 * @param 
 */
void measure_target::set_tar_features(int index, const std::list<measure_target> &tar_list ,const MatrixXd &targets_dis){
    position[0] = info_meas.x_pos;
    position[1] = info_meas.y_pos;
    shape[0] = info_meas.l; 
    shape[1] = info_meas.w; 
    shape[2] = info_meas.h;
    
    double angle=0;
    auto it = tar_list.begin();
    for(int i=0 ; it != tar_list.end(); it++,i++){
        angle = atan2(it->info_meas.y_pos - info_meas.y_pos ,
                        it->info_meas.x_pos - info_meas.x_pos);
        if(angle>=0 && angle<PI/4)
            topology[0] = topology[0] + targets_dis(index,i);
        if(angle>=PI/4 && angle<PI/2)
            topology[1] = topology[1] + targets_dis(index,i);
        if(angle>=PI/2 && angle<PI*3/4)
            topology[2] = topology[2] + targets_dis(index,i);
        if(angle>=PI*3/4 && angle<PI)
            topology[3] = topology[3] + targets_dis(index,i);
        if(angle>=-PI && angle<-PI*3/4)
            topology[4] = topology[4] + targets_dis(index,i);
        if(angle>=-PI*3/4 && angle<-PI/2)
            topology[5] = topology[5] + targets_dis(index,i);
        if(angle>=-PI/2 && angle<-PI/4)
            topology[6] = topology[6] + targets_dis(index,i);
        if(angle>=-PI/4 && angle<0)
            topology[7] = topology[7] + targets_dis(index,i);
    }

}

//--------------------------------------关联核类------------------------------------------------------------------

/**
 * @brief 构造函数
 * @param 
 */
association::association(){
    id_pool = 0;
    targets_tracked.clear();
    targets_measure.clear();
}

/**
 * @brief 计算已跟踪目标间距离
 * @param 
 */
void association::compute_dis_trac(){
    dis_trac_matrix = MatrixXd(targets_tracked.size(),targets_tracked.size());
    if(targets_tracked.size() == 0)
        return;
    double distance=0;
    int i=0,j=0;
    for(auto it1 = targets_tracked.begin(); it1 != targets_tracked.end() ; i++,it1++){
        j=0;
        for(auto it2 = targets_tracked.begin(); it2 != targets_tracked.end() ;j++,it2++){
            distance = (it1->info_evlt.x_pos - it2->info_evlt.x_pos) * 
                        (it1->info_evlt.x_pos - it2->info_evlt.x_pos) +
                        (it1->info_evlt.y_pos - it2->info_evlt.y_pos) * 
                        (it1->info_evlt.y_pos - it2->info_evlt.y_pos);
            dis_trac_matrix(i,j) = sqrt(distance);
        }
    }
}

/**
 * @brief 计算测量目标间距离
 * @param 
 */
void association::compute_dis_meas(){
    dis_meas_matrix = MatrixXd(targets_measure.size(),targets_measure.size());
    if(targets_measure.size() == 0)
        return;
    double distance=0;
    
    int i=0,j=0;
    for(auto it1 = targets_measure.begin(); it1 != targets_measure.end() ; i++,it1++){
        j=0;
        for(auto it2 = targets_measure.begin(); it2 != targets_measure.end() ;j++,it2++){
            distance = (it1->info_meas.x_pos - it2->info_meas.x_pos) * 
                        (it1->info_meas.x_pos - it2->info_meas.x_pos) +
                        (it1->info_meas.y_pos - it2->info_meas.y_pos) * 
                        (it1->info_meas.y_pos - it2->info_meas.y_pos);
            dis_meas_matrix(i,j) = sqrt(distance);
        }
    }    
}

/**
 * @brief 欧几里得距离（改进版）
 * @param para:
 * @return 相似度[0,1]
 */
double association::Euclidean_Distance(const std::vector<double> &v1,const std::vector<double> &v2,double para){
    int v1_size = v1.size();
    int v2_size = v2.size();
    if(v1_size != v2_size){
        std::cout<<"Euclidean_Distance:vector size not equal!! "<< v1_size<< " "<<v2_size <<std::endl;
        exit(0);
    }
    double dis=0;
    for(int i=0 ; i<v1_size ; i++){
        dis += (v1[i] - v2[i])*(v1[i] - v2[i]);
    }
    dis = sqrt(dis);
    dis = dis>para ? para : dis;  
    return (1-dis/para);
}

/**
 * @brief 余弦相识度
 * @param 
 * @return 相似度[0,1]
 */
double association::Cosine_Similarity(const std::vector<double> &v1,const std::vector<double> &v2){
    int v1_size = v1.size();
    int v2_size = v2.size();
    if(v1_size != v2_size){
        std::cout<<"Cosine_Similarity:vector size not equal!! "<< v1_size<< " "<<v2_size <<std::endl;
        exit(0);
    }
    double dis_1=0,dis_2=0,dot=0;
    for(int i=0 ; i<v1_size ; i++){
        dis_1 += v1[i]*v1[i];
        dis_2 += v2[i]*v2[i];
        dot += v1[i]*v2[i];
    }
    dis_1 = sqrt(dis_1);
    dis_2 = sqrt(dis_2);
    return fabs(dot/(dis_1*dis_2));
}

/**
 * @brief 计算目标相似度
 * @param 
 */
void association::compute_similar(){
    if(targets_tracked.size() == 0 && targets_measure.size() != 0){
        pos_similar.resize(1, targets_measure.size());
        sha_similar.resize(1, targets_measure.size());
        top_similar.resize(1, targets_measure.size());
        tar_similar.resize(1, targets_measure.size());
        pos_similar.fill(0);
        sha_similar.fill(0);
        top_similar.fill(0);
        tar_similar.fill(0);
        return;
    }
    if(targets_measure.size() == 0 && targets_tracked.size() != 0){
        pos_similar.resize(targets_tracked.size(), 1);
        sha_similar.resize(targets_tracked.size(), 1);
        top_similar.resize(targets_tracked.size(), 1);
        tar_similar.resize(targets_tracked.size(), 1);
        pos_similar.fill(0);
        sha_similar.fill(0);
        top_similar.fill(0);
        tar_similar.fill(0);
        return;
    }
    if(targets_measure.size() == 0 && targets_tracked.size() == 0){
        pos_similar.resize(1, 1);
        sha_similar.resize(1, 1);
        top_similar.resize(1, 1);
        tar_similar.resize(1, 1);
        pos_similar.fill(0);
        sha_similar.fill(0);
        top_similar.fill(0);
        tar_similar.fill(0);
        return;
    }

    pos_similar.resize(targets_tracked.size(), targets_measure.size());
    sha_similar.resize(targets_tracked.size(), targets_measure.size());
    top_similar.resize(targets_tracked.size(), targets_measure.size());
    tar_similar.resize(targets_tracked.size(), targets_measure.size());
    pos_similar.fill(0);
    sha_similar.fill(0);
    top_similar.fill(0);
    tar_similar.fill(0);
    auto it1 = targets_tracked.begin();
    auto it2 = targets_measure.begin();
    for(int i=0 ;i<targets_tracked.size();i++,it1++){
        it2 = targets_measure.begin();
        for(int j=0 ;j<targets_measure.size();j++,it2++){
            pos_similar(i,j) = Euclidean_Distance(it1->position , it2->position ,25);
            sha_similar(i,j) = Cosine_Similarity(it1->shape , it2->shape);
            top_similar(i,j) = Euclidean_Distance(it1->topology , it2->topology ,1000000);
            tar_similar(i,j) = 0.8*pos_similar(i,j) + 0.2*sha_similar(i,j) + 0.0*top_similar(i,j);
        }
    }
}

/**
 * @brief 关联过程
 * @param 
 */
void association::association_process(){
    //cout<<"---"<<endl;
    //cout<<"pos_similar:"<<endl<< pos_similar<<endl;
    //cout<<"sha_similar:"<<endl<< sha_similar<<endl;
    //cout<<"top_similar:"<<endl<< top_similar<<endl;
    //cout<<"tar_similar:"<<endl<< tar_similar<<endl;
    incidence_matrix.resize(tar_similar.rows(), tar_similar.cols());
    incidence_matrix.fill(0);
    MatrixXd::Index maxRow, maxCol;
    double rowMax,colMax,colSum;


    //求解关联矩阵
    for(int i=0 ; i<tar_similar.rows() ; i++){
        rowMax = tar_similar.row(i).maxCoeff(&maxRow,&maxCol);
        if(rowMax >= 0.6){
            incidence_matrix(i,maxCol) = 1;
        }
    }
    for(int i=0 ; i<incidence_matrix.cols() ; i++){
        colSum = incidence_matrix.col(i).sum();
        if(colSum > 1){
            tar_similar.col(i).maxCoeff(&maxRow,&maxCol);
            incidence_matrix.col(i).fill(0);
            incidence_matrix(maxRow,i) = 1;
        }
    }
    //cout<<"incidence_matrix:"<<endl<< incidence_matrix<<endl;

    //遍历已跟踪目标(找出关联对、找出未匹配目标)
    auto it1 = targets_tracked.begin();
    auto it2 = targets_measure.begin();
    for(int i=0 ; i<incidence_matrix.rows() ; i++,it1++){
        rowMax = incidence_matrix.row(i).maxCoeff(&maxRow,&maxCol);
        if(rowMax == 1 && targets_tracked.size() !=0){
            it2 = targets_measure.begin();
            while(maxCol>0){
                it2++;
                maxCol--;    
            }
            it1->updata(it2->info_meas);
            for (uint m = 0;m < 4; m++){
                it1->point[m][0]=it2->point[m][0];
                it1->point[m][1]=it2->point[m][1];
            }
            it1->time_match = it1->time_match + 1;
            it1->time_disapper = 0;
        }
        if(rowMax == 0 && targets_tracked.size() !=0){
            it1->updata(it1->info_pdct);
            it1->time_disapper = it1->time_disapper + 1;
        }
    }

    //遍历测量目标（找出未匹配目标）
    auto it3 = targets_measure.begin();
    for(int i=0 ; i<incidence_matrix.cols() ; i++,it3++){
        colMax = incidence_matrix.col(i).maxCoeff(&maxRow,&maxCol);
        if(colMax == 0 && targets_measure.size() !=0){
            tracked_target temp;
            temp.updata(it3->info_meas);
            for (uint m = 0;m < 4; m++){
                temp.point[m][0]=it3->point[m][0];
                temp.point[m][1]=it3->point[m][1];
            }
            targets_tracked.push_back(temp);
        }
    }

}

/**
 * @brief 获取测量目标，启动跟踪流程
 * @param 
 */
void association::get_measure_tar_and_work(LidarRawObjectArray_h *msg){
    targets_measure.clear();
    for(uint i = 0;i < msg->objs.size();i++){
        measure_target temp;
        temp.info_meas.l = msg->objs[i].lwh.x();
        temp.info_meas.w = msg->objs[i].lwh.y();
        temp.info_meas.h = msg->objs[i].lwh.z();
        temp.info_meas.x_pos = msg->objs[i].x_pos;
        temp.info_meas.y_pos = msg->objs[i].y_pos;
        for (uint m = 0;m < 4; m++){
            temp.point[m][0]=msg->objs[i].bbox_point[m].x;
            temp.point[m][1]=msg->objs[i].bbox_point[m].y;
        }
        double distan_1 = (temp.point[0][0] - temp.point[1][0]) * (temp.point[0][0] - temp.point[1][0]) + 
                        (temp.point[0][1] - temp.point[1][1]) * (temp.point[0][1] - temp.point[1][1]);
        double distan_2 = (temp.point[1][0] - temp.point[2][0]) * (temp.point[1][0] - temp.point[2][0]) + 
                        (temp.point[1][1] - temp.point[2][1]) * (temp.point[1][1] - temp.point[2][1]);
        if(distan_1>distan_2){
            temp.info_meas.heading = atan2(temp.point[0][1] - temp.point[1][1],temp.point[0][0] - temp.point[1][0]);
        }else{
            temp.info_meas.heading = atan2(temp.point[1][1] - temp.point[2][1],temp.point[1][0] - temp.point[2][0]);
        }
        
        targets_measure.push_back(temp);
    }
    compute_dis_meas();
    int i = 0;
    for(auto it = targets_measure.begin();it != targets_measure.end();it++,i++){
       it->set_tar_features(i,targets_measure,dis_meas_matrix);
    }
    //cout << "----------------1.接收测量目标，更新测量目标特征--------------" <<targets_measure.size()<< endl;

    compute_similar();
    //cout << "----------------2.计算相似度----------------------------------" << endl;

    association_process();
    //cout << "----------------3.关联---------------------------------------" << endl;

    for(auto it = targets_tracked.begin();it != targets_tracked.end();it++){
       if(it->time_match >= 3){
          if(it->id == 0){
              it->id = id_pool++;
          }
          it->time_match = it->time_match > 40 ? 40 :it->time_match;
       }
       if(it->time_disapper >= 9){
          it = targets_tracked.erase(it);
          it--;
       }
    }
    //cout << "----------------4.判断跟踪起始与终止--------------------------" << endl;

    compute_dis_trac();
    i = 0;
    for(auto it = targets_tracked.begin();it != targets_tracked.end();it++,i++){
       it->set_tar_features(i,targets_tracked,dis_trac_matrix);
    }
    //cout << "----------------5.更新已跟踪目标特征-------------------------" <<targets_tracked.size()<< endl;
}