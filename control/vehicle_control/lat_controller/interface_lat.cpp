extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include "pure_pursuit.h"
#include "interface_lat.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "Controller.h"
#include "VehicleStat.h"
#include "Planning.h"

int len;

typedef struct
{
    std::vector<float> x_ref;
    std::vector<float> y_ref;
} WayPoint;
// uint8_t turn_light_enable = false;

/**
 * @brief ros_veh_para_init
 */
void ros_veh_para_init()
{
    struct VehPara temp;

    temp.len = 2.5;
    temp.width = 0.8;
    temp.hight = 1.85;
    temp.mass = 500;
    temp.f_tread = 0.540;
    temp.r_tread = 0.540;
    temp.wheelbase = 0.840;
    temp.steering_ratio = 34.2;
    temp.max_steer_angle = 27.76;
    temp.max_wheel_angle = 315;
    temp.wheel_diam = 0.6;
    pure_veh_para_init(temp); // 纯跟踪算法获取车辆参数
    return;
}

/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(char *msg)
{
    VehicleStat_h *vehiclestat_data = reinterpret_cast<VehicleStat_h *>(msg);
    pure_set_veh_speed(vehiclestat_data->veh_speed);
    // std::cout<<"veh_speed : "<<msg->veh_speed<<std::endl;
    return;
}

/**
 * @brief ref_path_callback
 * @param msg
 */
void ref_path_callback(char *msg)
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



    // int count = 0;
    // std::string data_str(msg, len);
    // std::cout << len << std::endl;
    // std::vector<float> x_ref;
    // std::vector<float> y_ref;
    // // std::cout<<"222222222222"<<std::endl;
    // try{
    //     json j = json::parse(data_str);
    //     float x = j["x"];
    //     float y = j["y"];
    //     x_ref.push_back(x);
    //     y_ref.push_back(y);
    // }

    // catch(const json::parse_error& e){
    //     std::cerr << "JSON 解析错误 " << e.what() << std::endl;
    // }
    // int num_1 = 0;
    // int num_2 = 0;
    // for(const auto &x : x_ref){
    //     num_1++;
    //     std::cout << "x: " << x << " " << num_1 << std::endl;
    // }
    // for(const auto &y : y_ref){
    //     num_2++;
    //     std::cout << "y: " << y << "  " << num_2 << std::endl;
    // }


    pure_set_ref_path(x_v, y_v);

    return;
}



void calculate_angle(void *dora_context){
    struct SteeringCmd_h steer_msg;

    const int rate = 200;   // 设定频率为200HZ
    const chrono::milliseconds interval((int)(1000/rate));

    steer_msg.SteeringAngle = -pure_pursuit(); 
    std::cout << "steer_msg: " << steer_msg.SteeringAngle << std::endl;

    SteeringCmd_h* Steerptr = &steer_msg;
    char *output_data = (char *)Steerptr;
    size_t output_data_len = sizeof(steer_msg);


    std::string out_id = "SteeringCmd";
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

    this_thread::sleep_for(interval);
    return;
}

void thread_func(void * dora_context){
    while (true)
    {
        calculate_angle(dora_context);
    }
    
}

// void turn_light_callback(const custom_msgs::Request::ConstPtr &msg){

//    turn_light_enable = msg->is_converge;
//    return;
//}
int run(void *dora_context)
{
    //std::cout << "......................................" <<std::endl;
    while (true)
    {
        //std::cout << "????????????????????????" <<std::endl;

        void * event = dora_next_event(dora_context);


        //std::cout << "&&&&&&&&&&&&&&&&&&&&" <<std::endl;

        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }
        //std::cout << "!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;

        enum DoraEventType ty = read_dora_event_type(event);
        //std::cout << "::::::::::::::::::::::::::::::::" <<std::endl;

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            //std::cout << "--------------------" <<std::endl;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            //std::cout << "+++++++++++++++++++++++++++++++" <<std::endl;
            len = data_len;
            std::cout << "Input Data length: " << data_len << std::endl;
            if (strcmp("VehicleStat", data_id) == 0)
            {
                veh_status_callback(data);
            }
            else if (strcmp("raw_path", data_id) == 0)
            {
                std::cout << " " <<std::endl;
                ref_path_callback(data);
            }
            
            std::thread t(thread_func, dora_context);

            t.join();


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
    std::cout << "lat_control" << std::endl;
    //std::cout << "***************************************" << std::endl;

    auto dora_context = init_dora_context_from_env();
    //std::cout << "////////////////////////////" << std::endl;

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "END lat_control" << std::endl;

    return ret;
}
