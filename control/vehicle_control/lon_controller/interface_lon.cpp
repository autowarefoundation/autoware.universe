extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include "interface_lon.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(char *msg){
    struct VehicleStat temp;
    VehicleStat_h *vehiclestat_data = reinterpret_cast<VehicleStat_h *>(msg);
    temp.VehicleSpeed = vehiclestat_data->veh_speed;
    set_vehicle_stat(temp);
    return;
}

/**
 * @brief veh_pose_callback
 * @param msg
 */
void veh_pose_callback(char *msg){
    struct RtkImuStat temp;
    NaviData_h *navi_data = reinterpret_cast<NaviData_h *>(msg);
    temp.pitch   = navi_data->pitch;          //俯仰角
    temp.roll    = navi_data->roll;           //翻滚角
    temp.heading = navi_data->heading;        // 航向角
    temp.speed2d = navi_data->speed2d;        //车辆速度
    set_rtk_imu_stat(temp);
    return;
}

/**
 * @brief run_req_callback
 * @param msg
 */
void run_req_callback(char *msg)
{
    struct Request temp;
    Request_h *req_data = reinterpret_cast<Request_h *>(msg);
    temp.request_type  = req_data->reques_type;
    temp.run_speed     = req_data->run_speed;
    temp.aeb_distance  = req_data->aeb_distance;
    temp.stop_distance = req_data->stop_distance;
    set_requre(temp);
    return;
}


int run(void *dora_context)
{
    unsigned char counter = 0;

    while (true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            struct TrqBreCmd_h trq_bre_msg;

            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            if (strcmp("VehicleStat", data_id) == 0)
            {
                veh_status_callback(data);
            }
            else if (strcmp("Request", data_id) == 0)
            {
                run_req_callback(data);
            }
            else if (strcmp("navi_msg", data_id) == 0)
            {
                veh_pose_callback(data);
            }

            struct Trq_Bre_Cmd temp_cmd = {0, 0, 0, 0};

            const int rate = 200;   // 设定频率为200HZ
            const chrono::milliseconds interval((int)(1000/rate));

            while(true)
            {
                //请求行驶模式处理
                switch (request_value.request_type) {
                    case AEB_ENABLE:
                            temp_cmd = aeb_solve();
                            break;
                    case STOP_ENABLE:
                            temp_cmd = stop_solve();
                            break;
                    case FORWARD_ENABLE:
                            temp_cmd = run_solve();
                            break;
                    case BACK_ENABLE:
                            temp_cmd = back_solve();
                            break;
                }
                //发布命令        
                trq_bre_msg.bre_enable = temp_cmd.bre_enable;
                trq_bre_msg.bre_value = temp_cmd.bre_value;
                trq_bre_msg.trq_enable = temp_cmd.trq_enable;
                trq_bre_msg.trq_value_3 = temp_cmd.trq_value;

                TrqBreCmd_h* TrqBretpr = &trq_bre_msg;
                char *output_data = (char *)TrqBretpr;
                size_t output_data_len = sizeof(trq_bre_msg);


                std::string out_id = "TrqBreCmd";
                int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

                this_thread::sleep_for(interval);
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
    std::cout << "HELLO FROM C++ (using C API)" << std::endl;

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "GOODBYE FROM C++ node (using C API)" << std::endl;

    return ret;
}





