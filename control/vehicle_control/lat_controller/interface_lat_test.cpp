extern "C"
{
// #include "node_api.h"
// #include "operator_api.h"
// #include "operator_types.h"
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
#include "dora-node-api.h"
#include "dora-ros2-bindings.h"

int len;

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
    int count = 0;
    std::string data_str(msg, len);
    std::cout << len << std::endl;
    std::vector<float> x_ref;
    std::vector<float> y_ref;
    // std::cout<<"222222222222"<<std::endl;
    try{
        json j = json::parse(data_str);
        float x = j["x"];
        float y = j["y"];
        x_ref.push_back(x);
        y_ref.push_back(y);
    }

    catch(const json::parse_error& e){
        std::cerr << "JSON 解析错误 " << e.what() << std::endl;
    }

    for(const auto &x : x_ref){
        std::cout << "x: " << x << " " ;
    }
    for(const auto &y : y_ref){
        count++;
        std::cout << "y: " << y << "  " << count ;
    }
    std::cout << " "  << std::endl;

    pure_set_ref_path(x_ref, y_ref);

    return;
}

// void turn_light_callback(const custom_msgs::Request::ConstPtr &msg){

//    turn_light_enable = msg->is_converge;
//    return;
//}
// int run(void *dora_context)

int main()
{
    std::cout << "lat_control" << std::endl;
    //std::cout << "***************************************" << std::endl;

    // auto dora_context = init_dora_context_from_env();
    //std::cout << "////////////////////////////" << std::endl;
    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    // auto ret = run(dora_context);
    // free_dora_context(dora_context);
    while (true)
    {
        char *data;
        size_t data_len;
        // char *data_id;
        // size_t data_id_len;
        //std::cout << "--------------------" <<std::endl;
        auto event = merged_events.next();
        auto dora_event = downcast_dora(std::move(event));
        auto DoraInput = event_as_input(std::move(dora_event));
        std::memcpy(data, DoraInput.data.data(), DoraInput.data.size());
        // read_dora_input_data(event, &data, &data_len);
        // read_dora_input_id(event, &data_id, &data_id_len);
        std::cout << "Input ID: " << std::string(DoraInput.id) <<std::endl;
        len = DoraInput.data.size();
        std::cout << "Input len: " << len <<std::endl;
        if (strcmp("VehicleStat", DoraInput.id.c_str()) == 0)
        {
            veh_status_callback(data);
        }
        else if (strcmp("DoraNavSatFix", DoraInput.id.c_str()) == 0)
        {
            std::cout << " " <<std::endl;
            ref_path_callback(data);
        }

        struct SteeringCmd_h steer_msg;

        const int rate = 200;   // 设定频率为200HZ
        const chrono::milliseconds interval((int)(1000/rate));

        while(true)
        {
            steer_msg.SteeringAngle = -pure_pursuit(); 

            SteeringCmd_h* Steerptr = &steer_msg;
            char *output_data = (char *)Steerptr;
            size_t output_data_len = sizeof(steer_msg);


            std::string out_id = "SteeringCmd";
            // int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

            this_thread::sleep_for(interval);
        }
    

    }


    std::cout << "END lat_control" << std::endl;

    return 0;
}
