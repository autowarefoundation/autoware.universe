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
using json = nlohmann::json;
 

int run(void *dora_context)
{
    
    while(true)
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

            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);
            json j;
            // 将数据转化为字符串
            std::string data_str(data, data_len);
            try 
            {
              j = json::parse(data_str); // 解析 JSON 字符串               
            } 
            catch (const json::parse_error& e) 
            {
              std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
              //free_dora_event(event);
            }

            if (strcmp(id, "DoraNavSatFix") == 0)
            {
              printf("DoraNavSatFix event\n");
              std::cout << "<----print---->" <<j<<"   "<< j["frame_id"] << std::endl;
            }
            else if(strcmp(id, "DoraQuaternionStamped") == 0)
            {
              std::cerr << " DoraQuaternionStamped"<<std::endl; 
              std::cout << "<----print---->" <<j<<"   "<< j["frame_id"] << std::endl; 
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
  std::cout << "gnss poser for dora " << std::endl;
  auto dora_context = init_dora_context_from_env();
  auto ret = run(dora_context);
  free_dora_context(dora_context);

  std::cout << "exit gnss poser ..." << std::endl;
  return ret;
}