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

#include "road_lane.h"
#include "Localization.h"

int len;

struct Pose2D
{
    double x;
    double y;
    double theta;
};

std::vector<double> x_v_double;
std::vector<double> y_v_double;


void Map_Point_Callback(char *msg){
    std::cout << "-------------------" << std::endl;


    int num_points = len / sizeof(float); 
    float* float_array = reinterpret_cast<float*>(msg); 

    int num_xy_points = num_points / 2; 

    std::vector<float> x_v(float_array, float_array + num_xy_points);
    std::vector<float> y_v(float_array + num_xy_points, float_array + num_points); 

    x_v_double.assign(x_v.begin(), x_v.end());
    y_v_double.assign(y_v.begin(), y_v.end());
}



void Current_Pose_callback(void *dora_context, char *msg)
{
    std::cout << "++++++++++++++++" << std::endl;
    Pose2D pose;
    //current_pose.yaw = deg2rad(msg->theta)
    std::string data_str(msg, len);
    json j = json::parse(data_str);
    // std::cout << j << std::endl;

    pose.x = j["position"]["x"].get<double>();
    // std::cout << "x"<< pose.x << std::endl;
    pose.y = j["position"]["y"].get<double>();
    // std::cout << "y"<< pose.y << std::endl;
    double theta_raw = j["orientation"]["Heading"].get<double>();
    // std::cout << "theta_raw"<< theta_raw << std::endl;
    double theta = theta_raw*57.3;
    pose.theta = (theta >= 0) && (theta < 90) ?   //-->>真北方向夹角转地图坐标系
                                90 - theta : 450 - theta;   
    std::cout << "x: " << pose.x << " y: " << pose.y << " theta: "<< theta << std::endl;
    // pose.theta = j["theta"]


    std::vector<double> start_frenet = getFrenet2(0, 0, x_v_double, y_v_double, 0);
    std::vector<double> end_frenet = getFrenet2(-0.19927235, -0.38909271, x_v_double, y_v_double, 0); 
    std::vector<double> cur_frenet = getFrenet2(pose.x,pose.y,x_v_double, y_v_double, 0);

    CurPose_h cur_pose_all;
    cur_pose_all.x = pose.x;
    cur_pose_all.y = pose.y;
    cur_pose_all.theta = pose.theta;
    cur_pose_all.s = cur_frenet[0];
    cur_pose_all.d = cur_frenet[1];

    std::string out_id = "cur_pose_all";
    CurPose_h *pose_all_ptr = &cur_pose_all;
    char *output_data = (char*)pose_all_ptr;
    size_t output_data_len = sizeof(cur_pose_all);
    std::cout << "output_data_len: " << output_data_len << std::endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data, output_data_len);
    if(result != 0){
        std::cerr << "failed to send output" << std::endl;
    }

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
            std::string id(data_id, data_id_len);
            std::cout << "Input Data length: " << data_len << std::endl;
            std::cout << id.c_str() << std::endl;

            if (strcmp("DoraGnssPose", data_id) == 0)
            {
                Current_Pose_callback(dora_context, data);
            }
            else if (strcmp("road_lane", data_id) == 0)
            {
                Map_Point_Callback(data);
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
    std::cout << "road_lane " << std::endl;

    auto dora_context = init_dora_context_from_env();

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END road_lane" << std::endl;

    return ret;
}


