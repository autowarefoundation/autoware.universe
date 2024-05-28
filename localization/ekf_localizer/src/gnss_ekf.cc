extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <mutex>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <string.h>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std;


// 创建一个空的 JSON 对象
json j_pose;
uint32_t count_1=0,count_2;

void* EKF_fusion_pthread(void *dora_context)
{
  uint32_t count=0;
  const int rate = 50;   // 设定频率为 xx HZ
  const chrono::milliseconds interval((int)(1000/rate));
  std::mutex mtx; // 
  while(true)
  {
      count++;
      struct timeval tv;
      gettimeofday(&tv, NULL);

      cout << "The EKF_fusion pub time is: "  << tv.tv_sec <<","<< tv.tv_usec/1000.0f<<" ms " 
       << "count:" <<count << std::endl;
      //cout<<j_pose<<endl;

      // std::cout << "x: " << j["x"] << std::endl;
      // std::cout << "y: " << j["y"] << std::endl;
      // std::cout << "z: " << j["z"] << std::endl;
      // 将 JSON 对象序列化为字符串
      mtx.lock();
      std::string json_string = j_pose.dump(4); // 参数 4 表示缩进宽度
      mtx.unlock();
      // 将字符串转换为 char* 类型
      char *c_json_string = new char[json_string.length() + 1];
      strcpy(c_json_string, json_string.c_str());
      std::string out_id = "DoraGnssPose";
      //std::cout<<json_string<<endl;
      int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
      if (result != 0)
      {
          std::cerr << "failed to send output" << std::endl;
      }
      //std::cout << "dora_send_output()" << std::endl;
      this_thread::sleep_for(interval);
  }

}
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
              count_1++;
              printf("NavSatFix event: cnt: %d\n",count_1);
              //std::cout << "<----print---->" <<j<< std::endl;
              j_pose["position"]["x"] = j["x"];
              j_pose["position"]["y"] = j["y"];
              j_pose["position"]["z"] = j["z"];
            }
            else if(strcmp(id, "DoraQuaternionStamped") == 0)
            {
              count_2 ++;
              printf("QuaternionStamped event: cnt: %d\n",count_2);
               
              //std::cout << "<----print---->" <<j << std::endl; 

              j_pose["header"]["frame_id"] = j["frame_id"];
              j_pose["header"]["stamp"]["sec"] = j["sec"];
              j_pose["header"]["stamp"]["nanosec"] = j["nanosec"];

              j_pose["orientation"]["x"] = j["x"];
              j_pose["orientation"]["y"] = j["y"];
              j_pose["orientation"]["z"] = j["z"];
              j_pose["orientation"]["w"] = j["w"];
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
  std::cout << "ekf fusion for dora " << std::endl;
  auto dora_context = init_dora_context_from_env();

  // 从 JSON 数据中提取关键字
  j_pose["header"]["frame_id"] = "gnss";
  j_pose["header"]["stamp"]["sec"] = 0;
  j_pose["header"]["stamp"]["nanosec"] = 0.0;
  j_pose["position"]["x"] = 0.0;
  j_pose["position"]["y"] = 0.0;
  j_pose["position"]["z"] = 0.0;
  j_pose["position"]["vx"] = 0.0;
  j_pose["position"]["vy"] = 0.0;
  j_pose["position"]["vz"] = 0.0;

  j_pose["orientation"]["x"] = 0.0;
  j_pose["orientation"]["y"] = 0.0;
  j_pose["orientation"]["z"] = 0.0;
  j_pose["orientation"]["w"] = 0.0;
  j_pose["orientation"]["Roll"] = 0.0;
  j_pose["orientation"]["Pitch"] = 0.0;
  j_pose["orientation"]["Heading"] = 0.0;

  pthread_t id = 0;
  // 开启车辆状态线程
  if (pthread_create(&id, nullptr, EKF_fusion_pthread, dora_context) != 0)
  {
      std::cerr << "create EKF_fusion_pthread thread fail!" << std::endl;
      exit(-1);
  }

  auto ret = run(dora_context);
  free_dora_context(dora_context);

  std::cout << "exit ekf fusion node ..." << std::endl;
  return ret;
}