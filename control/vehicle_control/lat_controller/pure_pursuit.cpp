#include "pure_pursuit.h"

//-->>静态全局变量
static struct VehPara veh_para = {2.5, 0.8, 1.850, 500, 1.37, 1.37, 1.95, 34.2, 1024, 24, 0.6};
static std::vector<Eigen:: Vector2f> ref_path_veh;
static float veh_speed = 0;


/**
 * @brief pure_veh_para_init 初始化车辆参数
 * @param arg 参数服务器加载的参数
 */
void pure_veh_para_init(const struct VehPara &arg){
    veh_para = arg;
    return ;
}

/**
 * @brief pure_set_veh_speed
 * @param msg_speed
 */
void pure_set_veh_speed(const float arg){
   veh_speed = arg;
   return ;
}

void pure_set_ref_path(const std::vector<float> &x,const std::vector<float> &y){
    ref_path_veh.clear();
    Eigen:: Vector2f temp_point;
    for (uint32_t i = 0;i < x.size();i++){
        temp_point << x[i],y[i];
        ref_path_veh.push_back(temp_point);
    }
    // for (uint32_t i = 0;i < ref_path_veh.size();i++){
    //     std::cout<<"row_path X Y:"<<ref_path_veh[i].transpose()<<std::endl;
    //     // std::cout<<"row_path Y:"<<ref_path_veh[1]<<std::endl;
    // }
    // std::cout<<"row_path size :"<<ref_path_veh.size()<<std::endl;
    return;
}

//-->>功能函数
//判断是否为参考路段 :输入量为： alpha1、alpha2 、beta 、theta  Segment_ind
static bool is_relevant(const double alpha1,const double alpha2,const double theta, const uint32_t Segment_ind){
  bool ret = true;
   if ((fabs(alpha1) < M_PI_2)||(fabs(fabs(alpha1) - M_PI_2) < 0.001))
     {
       if ((alpha2 < 0 && theta/2 > 0)||(alpha2 > 0 && theta/2 < 0))
        {
          ret = fabs(alpha2) < fabs(theta/2) ? true : false;
        }
       else
        {
          if ( (fabs(alpha2) < M_PI_2) || (fabs(fabs(alpha2) - M_PI_2) < 0.001) )
            ret = true;
          else
           {
             ret = fabs(alpha2) < (3 *M_PI_2 - fabs(theta)) ?  true : false;
           }
        }
     }
   else  //是否为第一段路径
     {
      ret =  Segment_ind == 1 ? true : false;
     }

   return ret;
}

//在参考路径中根据车辆锚定点坐标查找关联路径段，返回路径段标号
static uint32_t search_releve_seg_ind(const std::vector<Eigen:: Vector2f> &ref_path, const uint32_t s_node){
    Eigen:: Vector2f  Segment_1;
    Eigen:: Vector2f  Segment_2;
    Eigen:: Vector2f  Q_1;
    Eigen:: Vector2f  Q_2;

    uint64_t path_len = ref_path.size();     //计算路径点数
    uint32_t node_index = 0;                 //获取起始路段号

    float alpha_1 = 0; float alpha_2 = 0; float theta   = 0;   //beta = theta / 2

    for(node_index = s_node;node_index < path_len;node_index++)
    {
        Segment_1 = ref_path.at(node_index) - ref_path.at(node_index-1);
        Q_1       = -ref_path.at(node_index-1);
        Q_2       = ref_path.at(node_index);
        alpha_1   = atan2(Segment_1[1],Segment_1[0]) - atan2(Q_1[1],Q_1[0]);
        alpha_2   = atan2(Segment_1[1],Segment_1[0]) - atan2(Q_2[1],Q_2[0]);

        if(node_index < path_len -1)
          {
            Segment_2 =  ref_path.at(node_index + 1) - ref_path.at(node_index);
            Eigen:: Vector2f temp = - Segment_1;
            theta =  acos(temp.dot(Segment_2) / temp.norm() * Segment_2.norm());
          }
        else
          {
            theta = static_cast<float>(M_PI);
            break;
          }

        if(is_relevant(static_cast<double>(alpha_1),static_cast<double>(alpha_2),static_cast<double>(theta),node_index))
          {
            break;
          }
    }
    return node_index;
}

//--->>>算法函数
//预瞄点选取- Q1的norm Q2的norm  La      alpha1   vecLine1.norm
static Eigen:: Vector2f search_goal_point(const float La, const std::vector<Eigen:: Vector2f> &ref_path,const uint32_t Segment_ind){
  //计算初始参数
    Eigen:: Vector2f  Segment_1 = ref_path.at(Segment_ind) - ref_path.at(Segment_ind - 1);    //相关路段
    Eigen:: Vector2f  Q_1 = - ref_path.at(Segment_ind - 1);
    Eigen:: Vector2f  Q_2 = ref_path.at(Segment_ind);
    float  alpha_1 = atan2(Segment_1[1],Segment_1[0]) - atan2(Q_1[1],Q_1[0]);

    Eigen:: Vector2f  map_point = ref_path.at(Segment_ind - 1) + Q_1.norm() * cos(alpha_1) * Segment_1 /Segment_1.norm();
    Eigen:: Vector2f  temp = ref_path.at(Segment_ind - 1) - map_point;
    float Ld = temp.norm();
    float ed = Q_1.norm() * sin(alpha_1);



    if(Ld > Segment_1.norm()){
      Ld        = Segment_1.norm();
      map_point = ref_path.at(Segment_ind);
      ed        = map_point.norm();
    }


    Eigen:: Vector2f  goal_point;                //预瞄点坐标
    if( Segment_ind == 1 && (Q_1.norm() >= La) && (Q_1.norm() >= La) && fabs(alpha_1) > static_cast<float>(M_PI_2) ) //在第一个点前，且超过预瞄距离，选择第一个点
    {
      goal_point = ref_path.at(Segment_ind - 1);
      return goal_point;
    }

    uint32_t index = 0;
    for(index = Segment_ind; index < ref_path.size();index++)
    {

        Segment_1 = ref_path.at(index) - ref_path.at(index -1);
        Q_1       = - ref_path.at(index -1);
        Q_2       = ref_path.at(index);

        if(Q_1.norm() <= La && Q_2.norm() >= La)
          {
           float cosgam = ( powf(Segment_1.norm(),2) + powf(Q_1.norm(),2) - powf(Q_2.norm(),2) ) /
                          ( 2 * powf(Segment_1.norm(),2) );
           float Plen  = Q_1.norm() * cosgam + sqrtf(powf(Q_1.norm(),2)*(powf(cosgam,2) - 1) + powf(La,2));
            goal_point = Plen * Segment_1 / Segment_1.norm() + ref_path.at(index -1);
            return goal_point;
          }
        else {
            if(index == Segment_ind && Q_1.norm() > La && Q_1.norm() > La)
              {

                if (fabs(ed)<= La)
                 {
                    float Plen =  sqrtf(powf(La,2) - powf(ed,2));
                    goal_point = Plen * Segment_1 / Segment_1.norm() + map_point;
                    return goal_point;
                 }
                else
                 {
                    goal_point = Ld * Segment_1 / Segment_1.norm() + ref_path.at(index -1);
                    return goal_point;
                 }

              }
          }
    }
    return  goal_point = ref_path.at(index - 1);
}

/**
 * @brief set_La_f 设置La预瞄距离选择函数
 * @param speed
 * @return
 */
static float set_La_f(const float speed){
#if 0
    if(speed < 1.34f * 3.6f)                                 return 4;
    else if(1.34f * 3.6f <= speed && speed < 5.36f * 3.6f)   return 2.98f * speed/3.6f;
    else                                                     return 16;
#endif
    // cout << "set_La_f_speed = " << speed <<endl;
    float speed_km = speed * 3.6;
    if(speed_km < 1.34f * 3.6f)                                    return 2.5;
    else if(1.34f * 3.6f <= speed_km && speed_km < 5.36f * 3.6f)   return 1.5 * speed_km/3.6f;
    else                                                           return 5.5;
    // std::cout<<"---------------------------------:"<<speed<<std::endl;
}

/**
 * @brief ackermann_steering
 * @param goal_point
 * @param La
 * @return
 */
static float ackermann_steering(const Eigen:: Vector2f &goal_point,const float La){
    //->计算预瞄向量相对航向的角度
    float tgangle = atan2(goal_point[1],goal_point[0]) -static_cast<float>(M_PI_2);
    // std::cout << "tgangle is: " << tgangle << std::endl;

    //->由当前位姿所得平均车轮转角，此处以车辆后轴中心为参考点计算得到的自行车模型车轮转向角 参考公式 4.2
    float delta = atan2(2 * veh_para.wheelbase * sin(tgangle),La);
    if(fabsf(delta)  > veh_para.max_wheel_angle * M_PI / 180){
        delta = delta/fabs(delta) * veh_para.max_wheel_angle * M_PI / 180;
    }
    // std::cout << "delta is(rad): " << delta << std::endl;

    //->由平均角delta计算得到内轮转角 推导公式
    float deltai = atan2(veh_para.wheelbase * tan(delta),veh_para.wheelbase - (veh_para.f_tread/2)*tan(fabs(delta)));
    if(fabsf(deltai)  > veh_para.max_wheel_angle * M_PI /180){
        deltai =deltai/fabs(deltai) * veh_para.max_wheel_angle * M_PI /180;
    }

    //->内轮转角弧度转化为度
    deltai = deltai*180/static_cast<float>(M_PI);
    // std::cout << "deltai is: " << deltai << std::endl;

    //->转换车轮转角至方向盘转角（CAN控制命令）
    if(fabs(deltai)<=25)                           return deltai*19.2f;
    else if (fabs(deltai)>25 && fabs(deltai)<=30)  return deltai*18.5f;
    else                                           return deltai*17.2f;
}


/**
 * @brief pure_pursuit
 * @return
 */
float pure_pursuit(){
    if(ref_path_veh.size() <20) return 0;

    //->搜索相关路径段
    uint32_t Segment_index = search_releve_seg_ind(ref_path_veh,1);
    //->设置预瞄距离
    float aim_La = set_La_f(veh_speed) + 2;
    // std::cout << "aim_La = " << aim_La << std::endl;
    // ROS_INFO_THROTTLE(1,"aim_La = %f", aim_La);
    //->搜索预瞄点
    Eigen::Vector2f goal_aim_point = search_goal_point(aim_La,ref_path_veh,Segment_index);
    // std::cout << "Vector is : " << goal_aim_point[0] << goal_aim_point[1] << std::endl;
    //-->>阿克曼转向计算控制转角
    return  ackermann_steering(goal_aim_point,aim_La);
}

uint8_t set_turn_light(){

  if(ref_path_veh.size() <20) return 0x0;
  Eigen:: Vector2f  first_point = ref_path_veh.at(8);
  Eigen:: Vector2f  next_point;
  float K = 0;

  for (unsigned int i = 9;i < 15;i++){
    float K_temp = 0;
    next_point = ref_path_veh.at(i);
    K_temp = atan2(next_point[1] - first_point[1],next_point[0] - first_point[0]);

    if(fabs(K_temp) >= fabs(K)){
      K = K_temp;
    }
  }
  K = K*180/M_PI;
  std::cout<<K<<std::endl;
  if(K <= 75){
    std::cout<<"turn right light!"<<std::endl;
     return 0x1;  //uint8 TurnRight=1
  }

  if(K >= 97){
    std::cout<<"turn left light!"<<std::endl;
     return 0x2;     //uint8 TurnLeft=2
  }

  return 0x0;  //  none
}

