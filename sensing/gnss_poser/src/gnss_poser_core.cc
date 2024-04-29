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
#include <msgpack.hpp> //sudo apt-get install libmsgpack-dev
#include "gnss_poser/gnss_poser_core.hpp"
DoraNavSatFix nav_sat_fix_origin_;

namespace gnss_poser
{
  GNSSPoser::GNSSPoser(void *dora_context,CoordinateSystem coordinate_system)
  {
    int buff_epoch = 1;
    position_buffer_.set_capacity(buff_epoch);
    GNSSPoser_dora_context = dora_context;
    coordinate_system_ = coordinate_system;
    // nav_sat_fix_origin_.latitude = 29.746722;
    // nav_sat_fix_origin_.longitude = 106.553791;
    // nav_sat_fix_origin_.altitude = 239.360000;
  }
  void GNSSPoser::callbackNavSatFix(const DoraNavSatFix nav_sat_fix_msg_ptr)
    {
      // check fixed topic
      static unsigned int count = 0;
      const bool is_fixed = isFixed(nav_sat_fix_msg_ptr.status);
      // publish is_fixed topic
      int coordinate_system = (int)CoordinateSystem::MGRS;
      if(count==0)
      {
        nav_sat_fix_origin_.latitude = nav_sat_fix_msg_ptr.latitude;
        nav_sat_fix_origin_.longitude = nav_sat_fix_msg_ptr.longitude;
        nav_sat_fix_origin_.altitude = nav_sat_fix_msg_ptr.altitude;
      }
      if (!is_fixed) {
        
          std::cerr << "Not Fixed Topic. Skipping Calculate." << std::endl;
        // RCLCPP_WARN_STREAM_THROTTLE(
        //   this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        //   "Not Fixed Topic. Skipping Calculate.");
        return;
      }
      // get position in coordinate_system
      
 
      const auto gnss_stat = convert(nav_sat_fix_msg_ptr, coordinate_system_);
      // std::cout << "latitude: " << gnss_stat.latitude << std::endl;
      // std::cout << "longitude: " << gnss_stat.longitude << std::endl;
      // std::cout << "altitude: " << gnss_stat.altitude << std::endl;
      // std::cerr << "[2]" << std::endl;
      const auto position = getPosition(gnss_stat);
      count++;
      std::cerr << "<-------------->"<< std::endl;
      std::cerr << "count:" <<count << " [x]:" <<position.x << " [y]:" 
            <<position.y<< " [z]:" <<position.z<< std::endl;
      std::cout << "latitude: " << nav_sat_fix_msg_ptr.latitude  
                << " longitude: " << nav_sat_fix_msg_ptr.longitude 
                 << " altitude: " << nav_sat_fix_msg_ptr.altitude << std::endl;
      

      // calc median position
      // 缓存定位信息，计算中值位置
      position_buffer_.push_front(position);
      if (!position_buffer_.full()) {
        // RCLCPP_WARN_STREAM_THROTTLE(
        //   this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        //   "Buffering Position. Output Skipped.");
        return;
      }
      // std::cerr << "[4]" << std::endl;
      // 计算中值位置
      const auto median_position = getMedianPosition(position_buffer_);
      // std::cerr << "[5]" << std::endl;


      // 创建一个空的 JSON 对象
      json j;
      // 添加数据到 JSON 对象
      j["x"] = median_position.x;
      j["y"] = median_position.y;
      j["z"] = median_position.z;
      j["latitude"] = nav_sat_fix_msg_ptr.latitude;
      j["longitude"] = nav_sat_fix_msg_ptr.longitude;
      j["altitude"] = nav_sat_fix_msg_ptr.altitude;

      // std::cout << "x: " << j["x"] << std::endl;
      // std::cout << "y: " << j["y"] << std::endl;
      // std::cout << "z: " << j["z"] << std::endl;
      // 将 JSON 对象序列化为字符串
      std::string json_string = j.dump(4); // 参数 4 表示缩进宽度
      // 将字符串转换为 char* 类型
      char *c_json_string = new char[json_string.length() + 1];
      strcpy(c_json_string, json_string.c_str());
      std::string out_id = "DoraNavSatFix";
      // std::cout<<json_string;
      int result = dora_send_output(GNSSPoser_dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
      if (result != 0)
      {
          std::cerr << "failed to send output" << std::endl;
      }

    }

  bool GNSSPoser::isFixed(const NavSatStatus & nav_sat_status_msg)
  {
    return nav_sat_status_msg.status >= NavSatStatus::STATUS_FIX;
  }
    
  Point GNSSPoser::getPosition(const GNSSStat & gnss_stat)
  {
    Point point;
    point.x = gnss_stat.x;
    point.y = gnss_stat.y;
    point.z = gnss_stat.z;
    return point;
  }
  
  Point GNSSPoser::getMedianPosition(
    const boost::circular_buffer<Point> & position_buffer)
  {
    if (position_buffer.empty()) {
      std::cerr << "position_buffer is empty" << std::endl;
        // 如果 position_buffer 为空，可以返回一个默认的 Point，或者采取其他适当的处理方法
        return Point{0.0, 0.0, 0.0}; // 临时返回一个默认值
    }
    auto getMedian = [](std::vector<double> array) {
      std::sort(std::begin(array), std::end(array));
      const size_t median_index = array.size() / 2;
      double median = (array.size() % 2)
                        ? (array.at(median_index))
                        : ((array.at(median_index) + array.at(median_index - 1)) / 2);
      return median;
    };

    std::vector<double> array_x;
    std::vector<double> array_y;
    std::vector<double> array_z;
    for (const auto & position : position_buffer) {
      array_x.push_back(position.x);
      array_y.push_back(position.y);
      array_z.push_back(position.z);
    }

    Point median_point;
    median_point.x = getMedian(array_x);
    median_point.y = getMedian(array_y);
    median_point.z = getMedian(array_z);
    return median_point;
  }

  GNSSStat GNSSPoser::convert(
    const DoraNavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system)
  {
    GNSSStat gnss_stat;
    
    // std::cerr << "[5]convert" << std::endl;
    // printf("nav_sat_fix_msg.latitude: %f\n", nav_sat_fix_msg.latitude);
    // printf("nav_sat_fix_msg.longitude: %f\n", nav_sat_fix_msg.longitude);
    // printf("nav_sat_fix_msg.altitudes: %f\n", nav_sat_fix_msg.altitude);
    // printf("nav_sat_fix_origin_.latitude: %f\n", nav_sat_fix_origin_.latitude);
    // printf("nav_sat_fix_origin_.longitude: %f\n", nav_sat_fix_origin_.longitude);
    // printf("nav_sat_fix_origin_.altitudes: %f\n", nav_sat_fix_origin_.altitude);
    if (coordinate_system == CoordinateSystem::UTM) {
      gnss_stat = NavSatFix2UTM(nav_sat_fix_msg);
    } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_UTM) {
      gnss_stat =
        NavSatFix2LocalCartesianUTM(nav_sat_fix_msg, nav_sat_fix_origin_);
    } else if (coordinate_system == CoordinateSystem::MGRS) {
      gnss_stat = NavSatFix2MGRS(nav_sat_fix_msg, MGRSPrecision::_100MICRO_METER);
    } else if (coordinate_system == CoordinateSystem::PLANE) {
      gnss_stat = NavSatFix2PLANE(nav_sat_fix_msg, plane_zone_);
    } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_WGS84) {
      gnss_stat = NavSatFix2LocalCartesianWGS84(nav_sat_fix_msg, nav_sat_fix_origin_);
    } else {
      std::cout << "Unknown Coordinate System" << std::endl;
      
      // RCLCPP_ERROR_STREAM_THROTTLE(
      //   this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      //   "Unknown Coordinate System");
    }
    return gnss_stat;
  }


}


int run(void *dora_context)
{
    gnss_poser::GNSSPoser GnssPoser(dora_context,gnss_poser::CoordinateSystem::LOCAL_CARTESIAN_WGS84);
    while(true)
    {
        usleep(1000);
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
            if (strcmp(id, "DoraNavSatFix") == 0)
            {
              char *data;
              size_t data_len;

              printf("DoraNavSatFix event\n");
              read_dora_input_data(event, &data, &data_len);
              // 将数据转化为字符串
              std::string data_str(data, data_len);
              try 
              {
                 // 解析 JSON 字符串
                json j = json::parse(data_str);
                DoraNavSatFix navSatFix;
                
                // 获取数据
                navSatFix.header.frame_id = j["frame_id"];
                navSatFix.header.sec = j["sec"];
                navSatFix.header.nanosec = j["nanosec"];
                navSatFix.latitude = j["latitude"].get<double>();
                navSatFix.longitude = j["longitude"].get<double>();
                navSatFix.altitude = j["altitude"].get<double>();
                // navSatFix.position_covariance = j["position_covariance"];
                navSatFix.position_covariance_type = j["position_covariance_type"];
                navSatFix.status.status = j["status"];
                navSatFix.status.service = j["service"];
                // 将 position_covariance 中的元素逐个添加到 position_covariance 向量中
                for (auto& elem : j["position_covariance"]) {
                    navSatFix.position_covariance.push_back(elem);
                }
                GnssPoser.callbackNavSatFix(navSatFix);
                
                // 打印结果
                // std::cout << "<----print---->" << navSatFix.header.frame_id << std::endl;
                // std::cout << "frame_id: " << navSatFix.header.frame_id << std::endl;
                // std::cout << "sec: " << navSatFix.header.sec << std::endl;
                // std::cout << "nanosec: " << navSatFix.header.nanosec << std::endl;
              
                // std::cout << "longitude: " << j["longitude"] << std::endl;
                // std::cout << "altitude: " <<  j["altitude"] << std::endl;
                // std::cout.width(12); //设置输出宽度
                // std::cout << "latitude: " << navSatFix.latitude << std::endl;
                // printf("navSatFix.latitude: %f\n", navSatFix.latitude);
                // printf("navSatFix.longitude: %f\n", navSatFix.longitude);
                // printf("navSatFix.altitudes: %f\n", navSatFix.altitude);
                // std::cout << "longitude: " << navSatFix.longitude << std::endl;
                // std::cout << "altitude: " <<  navSatFix.altitude << std::endl;
                // std::cout << "position_covariance: ";
                // for (const auto& cov : navSatFix.position_covariance) {
                // std::cout << cov << " ";
                // }
                // std::cout << std::endl;
                // std::cout << "position_covariance_type: " << navSatFix.position_covariance_type << std::endl;
                // std::cout << "status: " << navSatFix.status.status << std::endl;
                // std::cout << "service: " << navSatFix.status.service << std::endl;
                //free_dora_event(event);
              } 
              catch (const json::parse_error& e) 
              {
                std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
                //free_dora_event(event);
              }
            }
            // else if(strcmp(id, "DoraNavSatFix") == 0)
            // {

            // }
            // else if(strcmp(id, "DoraQuaternionStamped") == 0)
            // {

            // }
            // else if(strcmp(id, "DoraTwistStamped") == 0)
            // {
              
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
  std::cout << "gnss poser for dora " << std::endl;
  auto dora_context = init_dora_context_from_env();
  auto ret = run(dora_context);
  free_dora_context(dora_context);

  std::cout << "exit gnss poser ..." << std::endl;
  return ret;
}