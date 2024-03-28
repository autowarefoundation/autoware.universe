extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <string.h>
#include <nlohmann/json.hpp>
#include "yaml-cpp/yaml.h"
// void ImuCorrector::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
// {
//   sensor_msgs::msg::Imu imu_msg;
//   imu_msg = *imu_msg_ptr;

//   imu_msg.angular_velocity.x -= angular_velocity_offset_x_;
//   imu_msg.angular_velocity.y -= angular_velocity_offset_y_;
//   imu_msg.angular_velocity.z -= angular_velocity_offset_z_;

//   using IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
//   imu_msg.angular_velocity_covariance[IDX::X_X] =
//     angular_velocity_stddev_xx_ * angular_velocity_stddev_xx_;
//   imu_msg.angular_velocity_covariance[IDX::Y_Y] =
//     angular_velocity_stddev_yy_ * angular_velocity_stddev_yy_;
//   imu_msg.angular_velocity_covariance[IDX::Z_Z] =
//     angular_velocity_stddev_zz_ * angular_velocity_stddev_zz_;

//   imu_pub_->publish(imu_msg);
// }
using json = nlohmann::json;
double orientation_x ;
double orientation_y ;
double orientation_z ;
double orientation_w ;

double linear_acceleration_x ;
double linear_acceleration_y ;
double linear_acceleration_z ;

double angular_velocity_x ;
double angular_velocity_y ;
double angular_velocity_z ; 
double angular_velocity_offset_x_ = 0;
double angular_velocity_offset_y_ = 0;
double angular_velocity_offset_z_ = 0;
int run(void *dora_context)
{
    
    for (int i = 0; ; i++)
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
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);
            if (strcmp(id, "imu100D4") == 0)
            {
                char *data;
                size_t data_len;

                printf("imu100D4 event\n");
                read_dora_input_data(event, &data, &data_len);
                std::string data_str(data, data_len);
                try 
                {
                    // 处理解析成功的情况
                    json j = json::parse(data_str);




                    orientation_x = j["orientation"]["x"];
                    orientation_y = j["orientation"]["y"];
                    orientation_z = j["orientation"]["z"];
                    orientation_w = j["orientation"]["w"];

                    linear_acceleration_x = j["linear_acceleration"]["x"];
                    linear_acceleration_y = j["linear_acceleration"]["y"];
                    linear_acceleration_z = j["linear_acceleration"]["z"];

                    j["angular_velocity"]["x"] = (double)j["angular_velocity"]["x"]-angular_velocity_offset_x_;
                    j["angular_velocity"]["y"] = (double)j["angular_velocity"]["y"]-angular_velocity_offset_y_;
                    j["angular_velocity"]["z"] = (double)j["angular_velocity"]["z"]-angular_velocity_offset_z_;
                    // 将 JSON 对象序列化为字符串
                    std::string json_string = j.dump(4); // 参数 4 表示缩进宽度

                    // 将字符串转换为 char* 类型
                    char *c_json_string = new char[json_string.length() + 1];
                    strcpy(c_json_string, json_string.c_str());

                    // 输出提取的数据
                    // std::cout << "Orientation: (" << orientation_x << ", " << orientation_y << ", " << orientation_z << ", " << orientation_w << ")" << std::endl;
                    // std::cout << "Linear Acceleration: (" << linear_acceleration_x << ", " << linear_acceleration_y << ", " << linear_acceleration_z << ")" << std::endl;
                    // std::cout << "Angular Velocity: (" << j["angular_velocity"]["x"] << ", " << j["angular_velocity"]["y"] << ", " <<  j["angular_velocity"]["z"] << ")" << std::endl;
                    printf(c_json_string);
                    
                    std::string out_id = "imu100D4";

                    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
                   
                    if (result != 0)
                    {
                        std::cerr << "failed to send output" << std::endl;
                        return 1;
                    }
                } 
                catch (const json::parse_error& e) 
                {
                    std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
               
                }
            }
            // if (resultend != 0)
            // {
            //     std::cerr << "failed to send output" << std::endl;
            //     return 1;
            // }
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
    std::cout << "ring_outlier_filter node" << std::endl;
    auto dora_context = init_dora_context_from_env();

    std::string configfile_name = "config/imu_corrector.param.yaml";
    YAML::Node config = YAML::LoadFile(configfile_name);
    angular_velocity_offset_x_ = config["angular_velocity_offset_x"].as<float>();
    angular_velocity_offset_y_ = config["angular_velocity_offset_y"].as<float>();
    angular_velocity_offset_z_ = config["angular_velocity_offset_z"].as<float>();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    return ret;
}
