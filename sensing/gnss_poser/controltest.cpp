extern "C" {
#include "/home/nvidia/dora_project/dora/apis/c/node/node_api.h"
#include "/home/nvidia/dora_project/dora/apis/c/operator/operator_api.h"
#include "/home/nvidia/dora_project/dora/apis/c/operator/operator_types.h"
}
#include <nlohmann/json.hpp>

#include <string.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>
#include <unistd.h>
using json = nlohmann::json;
#include <stdio.h>

// int sumnum;

typedef struct
{
  float x;
  float y;
  int z;
  double latitude;
  double longitude;
  double altitude;
} Data;

typedef struct
{
    std::vector<float> x_ref;
    std::vector<float> y_ref;
} WayPoint;

int run(void * dora_context)
{
    for(int i = 0; i < 1; i++){
    usleep(1000);
    // sumnum++;
    // std::cout<<sumnum<<std::endl;
    void * event = dora_next_event(dora_context);

    if (event == NULL) {
        printf("[c node] ERROR: unexpected end of event\n");
        return -1;
    }

    enum DoraEventType ty = read_dora_event_type(event);

    if (ty == DoraEventType_Input) {
        FILE * file = fopen("zuobiao.txt", "r");
        if (file == NULL) {
            printf("Failed to open file\n");
            return -1;
        }
        // printf("------------------------");

        Data data;
        //json j;
        WayPoint waypoint;
        int count = 0;

        while (fscanf(file, "%f %f", &data.x, &data.y) == 2) 
        {
            float x = data.x;
            float y = data.y;
            // int z = data.z;
            // double latitude = data.latitude;
            // double longitude = data.longitude;
            // double altitude = data.altitude;
            waypoint.x_ref.push_back(x);
            waypoint.y_ref.push_back(y);

            // printf("X: %lf, Y: %lf, Z: %d, Latitude: %lf, Longitude: %lf, Altitude: %lf\n", data.x, data.y,
            // data.z, data.latitude, data.longitude, data.altitude);
            // j["x"] = data.x;
            // j["y"] = data.y;
            // j["z"] = data.z;
            // j["latitude"] = data.latitude;
            // j["longitude"] = data.longitude;
            // j["altitude"] = data.altitude;
            // count++;
            // std::string json_string = j.dump(4); 
            // char *c_json_string = new char[json_string.length() + 1];
            // strcpy(c_json_string, json_string.c_str());
            // size_t json_length = std::strlen(c_json_string);
            // std::cout << "Length of the C string: " << json_length << "  " << count <<std::endl;
            // std::string out_id = "DoraNavSatFix";
            // int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, json_length);
            // if (result != 0)
            // {
            //     std::cerr << "failed to send output" << std::endl;
            // }
            // std::cout << " x " << j["x"] << " y " << j["y"] << " z " << j["z"] << /*" " << count << */std::endl;
        }

        fclose(file);

        // int num = 0;
        // for(const auto &x : waypoint.x_ref){
        //     num++;
        //     std::cout << "x: " << x << " " << num << std::endl;
        // }
        // for(const auto &y : waypoint.y_ref){
        //     count++;
        //     std::cout << "y: " << y << "  " << count << std::endl;
        // }


        std::string out_id = "road_lane"; 
        std::vector<char> output_data; 
        for(const auto & x: waypoint.x_ref)
        { 
            float v=x; 
            char* temp = reinterpret_cast<char*>(&v); 
            output_data.insert(output_data.end(), temp, temp + sizeof(float)); 
        } 

        for(const auto & y: waypoint.y_ref)
        { 
            float v=y; 
            char* temp = reinterpret_cast<char*>(&v); 
            output_data.insert(output_data.end(), temp, temp + sizeof(float));  
        } 

        size_t output_data_len = output_data.size();
        std::cout << "output_data_len: " << output_data_len << std::endl;
        int result = dora_send_output(dora_context, &out_id[0], out_id.size(), output_data.data(), output_data_len);

        // WayPoint* waypoint_ptr = &waypoint;
        // char *output_data = (char *)waypoint_ptr;
        // size_t size_x_ref = sizeof(float) * waypoint.x_ref.size();
        // size_t size_y_ref = sizeof(float) * waypoint.y_ref.size();
        // size_t output_data_len = size_x_ref + size_y_ref;
        // std::string out_id = "DoraNavSatFix";
        // std::cout << "output_data_len: " << output_data_len << std::endl;
        // int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

        if (result != 0)
        {
            std::cerr << "failed to send output" << std::endl;
        }
    } 
    
    else if (ty == DoraEventType_Stop) {
        printf("[c node] received stop event\n");
    } 
    else {
        printf("[c node] received unexpected event: %d\n", ty);
    }
    free_dora_event(event);
  }
  return 0;
}

int main()
{
    std::cout << "test gnss poser for dora " << std::endl;
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "exit test gnss poser ..." << std::endl;
    return ret;
}
