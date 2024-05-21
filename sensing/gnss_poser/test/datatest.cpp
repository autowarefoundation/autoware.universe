extern "C" {
#include "/home/zxd/dora/apis/c/node/node_api.h"
#include "/home/zxd/dora/apis/c/operator/operator_api.h"
#include "/home/zxd/dora/apis/c/operator/operator_types.h"
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

typedef struct
{
  double x;
  double y;
  int z;
  double latitude;
  double longitude;
  double altitude;
} Data;

int run(void * dora_context)
{
  while(true){
    usleep(1000);
    void * event = dora_next_event(dora_context);

    if (event == NULL) {
        printf("[c node] ERROR: unexpected end of event\n");
        return -1;
    }

    enum DoraEventType ty = read_dora_event_type(event);

    if (ty == DoraEventType_Input) {
        FILE * file = fopen("out_0428_3.txt", "r");
        if (file == NULL) {
            printf("Failed to open file\n");
            return -1;
        }
        printf("------------------------");

        Data data;
        json j;
        int count = 1;
        while (fscanf(file, "%lf %lf %d %lf %lf %lf", &data.x, &data.y, &data.z, &data.latitude,
                &data.longitude, &data.altitude) == 6) 
        {
            // printf("X: %lf, Y: %lf, Z: %d, Latitude: %lf, Longitude: %lf, Altitude: %lf\n", data.x, data.y,
            // data.z, data.latitude, data.longitude, data.altitude);
            j["x"] = data.x;
            j["y"] = data.y;
            j["z"] = data.z;
            j["latitude"] = data.latitude;
            j["longitude"] = data.longitude;
            j["altitude"] = data.altitude;

            std::string json_string = j.dump(4); 
            char *c_json_string = new char[json_string.length() + 1];
            strcpy(c_json_string, json_string.c_str());
            std::string out_id = "DoraNavSatFix";
            int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
            if (result != 0)
            {
            std::cerr << "failed to send output" << std::endl;
            }

        }

        fclose(file);

 

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
