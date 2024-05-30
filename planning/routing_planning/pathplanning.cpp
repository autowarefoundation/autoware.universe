#include "pathplanning.h"
#include <iostream>

//获取参考路径     
PathPlanning::PathPlanning()
{
    x_ref.resize(30);
    y_ref.resize(30);
}

//析构函数
PathPlanning::~PathPlanning()
{
}


//计算规划长度
void PathPlanning::get_plan_dis(float vel_speed_ref)
{
    plan_distance = 15 + vel_speed_ref * 0.277; //单位m
    cout<<"plan_dis_temp: "<<plan_distance<<endl;
    return;
}


//转存参考路径信息
void PathPlanning::Get_Path_Ref(LaneLine &laneline)
{
    map_waypoints_x.clear();
    map_waypoints_y.clear();
    map_waypoints_s.clear();
    for(int i = 0; i < laneline.points.size();i++)
    {
        map_waypoints_x.push_back(laneline.points[i].x);
        map_waypoints_y.push_back(laneline.points[i].y);
        map_waypoints_s.push_back(laneline.points[i].s);
        // std::cout << "map X: " << map_waypoints_x[i] << " map Y: " << map_waypoints_y[i] 
        // << " map S: " << map_waypoints_s[i] << std::endl;
    }

    return;
}

//获取主车当前状态（位姿和速度）
void PathPlanning::Get_Curr_Sta(CurrentPose &curr_pose_temp, /*CurrentState &curr_state*/ float velocity)  
{

    this->curr_car_s = curr_pose_temp.s;
    this->curr_car_d = curr_pose_temp.d;
    this->curr_car_x = curr_pose_temp.x;
    this->curr_car_y = curr_pose_temp.y;

    this->vel_speed_self = velocity; /*curr_state.velocity;*/
    // std::cout << "speed: " << vel_speed_self << std::endl;
    return;
}



//采样拟合路径  
void PathPlanning::simple_from_spline()
{
    static vector<double> previous_path_x;
    static vector<double> previous_path_y;
    previous_path_x.resize(30);
    previous_path_y.resize(30);
    static double last_point_d = 0;
    static double last_point_s = 0;
    
    // is_changing_lane = false;   
    // if ( is_changing_lane==true && fabs(last_point_d-curr_car_d)>0.5 && fabs(last_point_s-curr_car_s)>1  )  //处于变道中，且未行驶完上次规划的路径，就不重新规划，继续使用上次的路径   && first_time>1
    // {
    //     //cout<< "I am here"<<endl;

    //     double last_to_dis = distance(curr_car_x, curr_car_y, previous_path_x[30], previous_path_y[30]);
    //     if (last_to_dis > 1)
    //     {
    //         int j = 0;
    //         int passed_points_num = NextWaypoint(curr_car_x, curr_car_y, previous_path_x, previous_path_y);
    //         for (int i = passed_points_num; i < 30; i++)     
    //         {
    //             x_ref[j] = previous_path_x[passed_points_num + j];
    //             y_ref[j] = previous_path_y[passed_points_num + j];
    //             j++;
    //         }

    //         for(int i=j; i<30; i++)          //确保路径点数量，下位机需要30个点
    //         {
    //             x_ref[j] = previous_path_x[30-1];
    //             y_ref[j] = previous_path_y[30-1];
    //         }

    //         return;
    //     }
    //     else  is_changing_lane = false;   //变道完成
    // }

    
    for (int i = 0; i < 30; i++)
    {
        double target_s;
        double target_d;
        target_s = curr_car_s + i*(plan_distance/50);                 //取plan_distance/30为一间隔   (i+1)
        target_d = s_path(target_s);
        // std::cout << "target S: " << target_s << " target D: " <<target_d<<std::endl;

        //if(i == 0)
        //cout<<"target_s   "<<target_s<<"   "<<"target_d:   "<<target_d<<endl;

        if (target_s > map_waypoints_s[map_waypoints_s.size()-1] - 0.5)   //判断是否到达地图的终点
        {
            x_ref[i] = map_waypoints_x[map_waypoints_x.size()-1];
            y_ref[i] = map_waypoints_y[map_waypoints_y.size()-1];
        }
        else 
        {
            vector<double> planned_point = getXY(target_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
         //   if(i == 0)
        //cout<<"x   "<<planned_point[0]<<"   "<<"x:   "<<planned_point[1]<<endl;
            x_ref[i] = planned_point[0];
            y_ref[i] = planned_point[1];

            previous_path_x[i] = x_ref[i];
            previous_path_y[i] = y_ref[i];

            last_point_s = target_s;
            last_point_d = target_d;
        }
        // std::cout << "x_ref: " << x_ref[i] << " y_ref: " << y_ref[i] << std::endl;
    }

    return;
}


void PathPlanning::push_sd(vector<double> &pts_s,vector<double> &pts_d)
{
    if ((curr_car_s+plan_distance) > map_waypoints_s[map_waypoints_s.size()-1])     //规划距离超出本路段终点,不能规划变道。如需终点变道应延长地图
    {
        double temp_to_last_point = map_waypoints_s[map_waypoints_s.size()-1] - curr_car_s;
        
        pts_s.push_back(curr_car_s);
        pts_s.push_back(curr_car_s + temp_to_last_point*1/10);
        pts_s.push_back(curr_car_s + temp_to_last_point*1/5);
        pts_s.push_back(map_waypoints_s[map_waypoints_s.size()-2]);
        pts_s.push_back(map_waypoints_s[map_waypoints_s.size()-1]);

        pts_d.push_back(curr_car_d);
        pts_d.push_back(curr_car_d + (offset-curr_car_d));
        pts_d.push_back(curr_car_d + (offset-curr_car_d));
        pts_d.push_back(curr_car_d + (offset-curr_car_d));  
        pts_d.push_back(curr_car_d + (offset-curr_car_d));  
  
    }
     else{
        pts_s.push_back(curr_car_s);
        pts_s.push_back(curr_car_s + first_s + 0.5);
        pts_s.push_back(curr_car_s + first_s + 1);
        pts_s.push_back(curr_car_s + first_s + 2);
        pts_s.push_back(curr_car_s + first_s + 3);
        pts_s.push_back(curr_car_s + first_s + 5);
        pts_s.push_back(curr_car_s + first_s + 5+(plan_distance - first_s - 5)*2/5);
        pts_s.push_back(curr_car_s + first_s + 5+(plan_distance - first_s - 5)*1/2);
        pts_s.push_back(curr_car_s + first_s + 5+(plan_distance - first_s - 5)*4/5);
        pts_s.push_back(curr_car_s + plan_distance);

        // pts_d.push_back(curr_car_d);
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));  
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));  
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));  
        // pts_d.push_back(curr_car_d + (offset-curr_car_d));  
        // pts_d.push_back(curr_car_d + (offset-curr_car_d)); 


        double decay_factor = 0.3; 
        int sequence_length = 10; 
        double step_size = std::abs(curr_car_d) * decay_factor; 

        for(int i = 0; i < sequence_length; i++) {
            double new_d;
            if(i<3){
                if (curr_car_d > 0) {
                    new_d = curr_car_d - step_size * i;
                } else {
                    new_d = curr_car_d + step_size * i;
                }
            }
            if (i > 3 || i == 3) {
                new_d = 0.0;
            }
            pts_d.push_back(new_d);
        }
    }

     
     return;  

}

//轨迹计算
void PathPlanning::calculate_trajectory()
{
    vector<double> pts_s;            //临时变量 保存s点集 
    vector<double> pts_d;            //临时变量 保存d点集  
    
    push_sd(pts_s,pts_d); 

    //cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
    //for(int i=0;i<pts_d.size();i++)
    //cout<<pts_d[i]<<" ";
    // std::cout<< "****************************"<<std::endl;
    for(int i = 0; i < pts_d.size(); i++){
        std::cout << "s: " << pts_s[i] << " d: " << pts_d[i] << std::endl;
    }

    this->s_path.set_points(pts_s,pts_d,true);                                //路径曲线拟合
    


    // double Z_s = curr_car_s;
    // double Z_d = s_path(Z_s);
    // std::cout << " Z D: " <<Z_d<<std::endl;

    simple_from_spline();                                   //采样拟合曲线并转为笛卡尔坐标

    return;
}

 //路径规划主函数       
void PathPlanning::generate_path(CurrentPose &curr_pose, LaneLine &lanelines, /*CurrentState &curr_state*/ float velocity) 
{
    first_time++;                                                            //解决起点不在参考线上的问题


    Get_Path_Ref(lanelines);                                                 //获取参考路径点                                 

    Get_Curr_Sta(curr_pose, velocity);                                    //获取主车当前状态（位姿和速度）

    get_plan_dis(/*this->vel_speed_self*/velocity);                                     //计算规划距离    

    calculate_trajectory();                                                  //计算轨迹



    return;
 }
