extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}


#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "node_routing_core.h"
#include"data_type.h"



int len;
LaneLine line_ref;            //参考路径信息
PathPlanning paths;                  //实现路径规划的对象


void toPath(Path_h &path_msg, CurrentPose &current_pose_temp)  
{
    path_msg.x_ref.resize(paths.x_ref.size());
    path_msg.y_ref.resize(paths.y_ref.size());
    int con = 0;
    for (int i = 0; i < paths.x_ref.size(); i++)
    {
        double shift_x = paths.x_ref[i] - current_pose_temp.x;
        double shift_y = paths.y_ref[i] - current_pose_temp.y;
        con++;
        std::cout << "****************" << con << "***************"<< std::endl;
        std::cout << "paths X: " << paths.x_ref[i] << " paths Y: " << paths.y_ref[i] << std::endl;
        std::cout << "current X: " << current_pose_temp.x << " current Y: " << current_pose_temp.y << std::endl;
        std::cout << "shift X: " << shift_x << " shift Y: " << shift_y << std::endl;
        
        //将路径点转换到车辆坐标系（后轮中心）
        path_msg.x_ref[i]=shift_x * sin(current_pose_temp.yaw) - shift_y * cos(current_pose_temp.yaw);
        path_msg.y_ref[i]=shift_y * sin(current_pose_temp.yaw) + shift_x * cos(current_pose_temp.yaw);

        std::cout << "path: X: " << path_msg.x_ref[i] << " path Y: " << path_msg.y_ref[i] << std::endl;
    }
    return;
}


void Current_Pose_callback(void *dora_context, char *msg)
{
    CurrentPose pose;
    CurrentPose current_pose;     //主车当前位姿
    Path_h path;
    CurPose_h *cur_pose = reinterpret_cast<CurPose_h *>(msg);


    // pose.yaw = j["theta"]
    current_pose.yaw = deg2rad(cur_pose->theta);
    current_pose.x = cur_pose->x - trans_para_back * cos(pose.yaw); 
    current_pose.y = cur_pose->y - trans_para_back * sin(pose.yaw);

    vector<double> map_waypoints_x_temp;    
    vector<double> map_waypoints_y_temp;
    for (int j = 0; j < ( line_ref.points.size() ); j++)
    {
        map_waypoints_x_temp.push_back( ( line_ref.points.at(j).x ) ) ;
        map_waypoints_y_temp.push_back( ( line_ref.points.at(j).y ) ) ;
    } 

    vector<double> current_sd_para = getFrenet2(current_pose.x, current_pose.y, map_waypoints_x_temp, map_waypoints_y_temp, 0);
     current_pose.s = current_sd_para[0];
     current_pose.d = current_sd_para[1];
    std::cout << "x: " << current_pose.x << " y: " << current_pose.y 
    << " yaw: " << current_pose.yaw << " s: " << current_pose.s << " d: " << current_pose.d <<std::endl;

    paths.generate_path(current_pose, line_ref, 3.6); 
    toPath(path,current_pose);         //传出规划路径，传出path      current_pose_temp

    std::string out_id = "raw_path";
    std::vector<char> output_data;
    for(const auto & x : path.x_ref){
        float v = x;
        // std::cout << "x: " << v << std::endl;
        char *temp = reinterpret_cast<char*>(&v);
        output_data.insert(output_data.end(), temp, temp + sizeof(float));
    }
    for(const auto & y :path.y_ref){
        float v = y;
        // std::cout << "y: " << v << std::endl;
        char *temp = reinterpret_cast<char*>(&v);
        output_data.insert(output_data.end(), temp, temp + sizeof(float));
    }

    size_t output_data_len = output_data.size();
    std::cout << "output_data_len: " << output_data_len << std::endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data.data(), output_data_len);
    if(result != 0){
        std::cerr << "failed to send output" << std::endl;
    }

    return;
}


void Line_Recv_callback(char *msg)
{
    // WayPoint *points = reinterpret_cast<WayPoint *>(msg);
    // std::vector<float> x_v;
    // std::vector<float> y_v;
    // int count = 0;
    // for(int i = 0; i < points->x_ref.size(); i++){
    //     count++;
    //     int x = points->x_ref[i];
    //     int y = points->y_ref[i];
    //     std::cout << "x: " << x << "y: " << y << count <<std::endl;
    //     x_v.push_back(x);
    //     y_v.push_back(y);
    // }


    int num_points = len / sizeof(float); 
    float* float_array = reinterpret_cast<float*>(msg); 

    int num_xy_points = num_points / 2; 

    std::vector<float> x_v(float_array, float_array + num_xy_points);
    std::vector<float> y_v(float_array + num_xy_points, float_array + num_points); 
    line_ref.points.clear();
    Point point_ref;
    std::vector<double> x_v_double(x_v.begin(), x_v.end());
    std::vector<double> y_v_double(y_v.begin(), y_v.end());
    int num = 0;
    for(int i = 0; i < x_v.size(); i++){
        num++;
        point_ref.x = x_v[i];
        point_ref.y = y_v[i];
        point_ref.s = getFrenet2(x_v[i], y_v[i], x_v_double, y_v_double, 0)[0];
        // std::cout << "x: " << point_ref.x << " y: " << point_ref.y << " s: " << point_ref.s << " count: " << num <<std::endl;
        line_ref.points.push_back(point_ref);
    }

    // int num_1 = 0;
    // int num_2 = 0;
    // for(const auto &x : x_v){
    //     num_1++;
    //     std::cout << "x: " << x << " " << num_1 << std::endl;
    // }
    // for(const auto &y : y_v){
    //     num_2++;
    //     std::cout << "y: " << y << "  " << num_2 << std::endl;
    // }
    return;
}


int run(void *dora_context)
{

    while (true)
    {

        void * event = dora_next_event(dora_context);

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            len = data_len;
            std::cout << "Input Data length: " << data_len << std::endl;
            if (strcmp("road_lane", data_id) == 0)
            {
                Line_Recv_callback(data);
            }
            else if (strcmp("cur_pose_all", data_id) == 0)
            {
                Current_Pose_callback(dora_context, data);
            }


        }


        
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "node_routing_core" << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END node_routing_core" << std::endl;

    return ret;
}


