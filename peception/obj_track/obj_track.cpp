extern "C"
{
#include "node_api.h"   
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <vector>
#include <string>
#include "association.h"
#include <ctime>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "frenet.h"
#include "Object.h"
#include "ObjectArray.h"
#include "LidarRawObjectArray.h"
#include "LaneLine.h"
#include "LaneLineArray.h"
#include "Controlsrv.h"

using namespace std;

LaneLine_h lane_line;
ObjectArray_h obj_result;
association ass_core;
clock_t start_time, end_time;

typedef struct BOX_POINT{
	double x;
	double y;
} BOX_POINT;

vector<BOX_POINT> trans_angle(const double &center_x,const double &center_y,
                                const double &l,const double &w,const double &heading){
	vector<BOX_POINT> box_point;
	double angle = heading - PI/2;//heading - PI/2;

    BOX_POINT temp;
    double x,y;
    x = center_x - w/2;
    y = center_y - l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

    x = center_x + w/2;
    y = center_y - l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

    x = center_x + w/2;
    y = center_y + l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

    x = center_x - w/2;
    y = center_y + l/2;
    temp.x =  (x - center_x)*cos(angle) - (y-center_y)*sin(angle) + center_x;
    temp.y =  (y - center_y)*cos(angle) + (x-center_x)*sin(angle) + center_y;
    box_point.push_back(temp);

	return box_point;

}




void OnlidarRecv(char *OnlidarRecv_data, int len, void* dora_context){

    struct Object_h temp;
    // LidarRawObjectArray_h *object_data = reinterpret_cast<LidarRawObjectArray_h *>(OnlidarRecv_data);
    LidarRawObjectArray_h object_data;
    LidarRawObject_h lidar_object;

    std::string data_str(OnlidarRecv_data, len);
    try 
    {
        // 解析 JSON 字符串
        json j = json::parse(data_str);

        for (const auto& obj : j["LidarRawObjects"]["objs"]) 
        {
            for (int i = 0; i < obj["bbox_point"]["x"].size(); ++i) 
            {
                lidar_object.bbox_point[i].x = obj["bbox_point"]["x"][i];
                lidar_object.bbox_point[i].y = obj["bbox_point"]["y"][i];
                lidar_object.bbox_point[i].z = obj["bbox_point"]["z"][i];
                std::cout << "Point " << i << ": (" << lidar_object.bbox_point[i].x << ", " 
                          << lidar_object.bbox_point[i].y << ", " 
                          << lidar_object.bbox_point[i].z << ")" << std::endl;
            }

            lidar_object.lwh.x() = obj["lwh"]["x"];
            lidar_object.lwh.y() = obj["lwh"]["y"];
            lidar_object.lwh.z() = obj["lwh"]["z"];
            std::cout << "Dimensions (lwh): (" << lidar_object.lwh.x() << ", " 
                      << lidar_object.lwh.y() << ", "  
                      << lidar_object.lwh.z() << ")" << std::endl;
            
            lidar_object.x_pos = obj["x_pos"];
            lidar_object.y_pos = obj["y_pos"];
            lidar_object.z_pos = obj["z_pos"];
            std::cout << "Position: (" << obj["x_pos"] << ", " << obj["y_pos"] << ", " << obj["z_pos"] << ")" << std::endl;
            object_data.objs.push_back(lidar_object);
        }
      
        object_data.header.frame_id = j["header"]["frame_id"];
        std::cout << "Frame ID: " << object_data.header.frame_id << std::endl;
        object_data.header.seq = j["header"]["cnt"];
        std::cout << "Sequence Number: " << object_data.header.seq << std::endl;
        object_data.header.stamp = j["header"]["stamp"];
        std::cout << "Timestamp: " << object_data.header.stamp << std::endl;

    } 
    catch (const json::parse_error& e) 
    {
        std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
    }

    LidarRawObjectArray_h *object_data_ptr = &object_data;

    start_time = clock();
    ass_core.get_measure_tar_and_work(object_data_ptr);
    obj_result.header.stamp = object_data_ptr->header.stamp;
    obj_result.header.frame_id = object_data_ptr->header.frame_id;
    obj_result.objs.clear();
    for(auto it = ass_core.targets_tracked.begin();it != ass_core.targets_tracked.end();it++){
        Object_h temp;
        temp.id = it->id;
        if(temp.id == 0)
            continue;
        temp.type = -1;
        temp.vx = it->info_evlt.vx;
        temp.vy = it->info_evlt.vy;
        temp.v = sqrt(temp.vx*temp.vx + temp.vy*temp.vy );
        temp.lwh.x() = it->info_evlt.l;
        temp.lwh.y() = it->info_evlt.w;
        temp.lwh.z() = it->info_evlt.h;
        temp.x_pos = it->info_evlt.x_pos;
        temp.y_pos = it->info_evlt.y_pos;
        if(!lane_line.s.empty()){
            std::vector<double> object_s_d = getFrenet2(temp.x_pos,temp.y_pos,lane_line.x,lane_line.y,lane_line.s[0]); 
            temp.s_pos = object_s_d[0];
            temp.d_pos = object_s_d[1];
        }
        //if(it->time_match > 20){
        if(1){
            vector<BOX_POINT> temp_point;
            temp_point = trans_angle(temp.x_pos,temp.y_pos,temp.lwh.x(),temp.lwh.y(),it->info_evlt.heading);
            for (uint m = 0;m < 4; m++){
                temp.bbox_point[m].x = temp_point[m].x;
                temp.bbox_point[m].y = temp_point[m].y;
                temp.bbox_point[m].z = 0;
                temp.bbox_point[m+4].x = temp_point[m].x;
                temp.bbox_point[m+4].y = temp_point[m].y;
                temp.bbox_point[m+4].z = temp.lwh.z();
            }
        }else{
            for (uint m = 0;m < 4; m++){
                temp.bbox_point[m].x = it->point[m][0];
                temp.bbox_point[m].y = it->point[m][1];
                temp.bbox_point[m].z = 0;
                temp.bbox_point[m+4].x = it->point[m][0];
                temp.bbox_point[m+4].y = it->point[m][1];
                temp.bbox_point[m+4].z = temp.lwh.z();
            }
        }
        
        
        obj_result.objs.push_back(temp);
    }

    // custom_msgs::Object temp;
    // temp.id = 9999;
    // temp.type = -1;
    // temp.vx = 5;
    // temp.vy = 5;
    // temp.v = sqrt(temp.vx*temp.vx + temp.vy*temp.vy );
    // temp.lwh.x = 10;
    // temp.lwh.y = 5;
    // temp.lwh.z = 2;
    // temp.x_pos = 20;
    // temp.y_pos = 2;
    // if(!lane_line.s.empty()){
    //     // std::vector<double> object_s_d = getFrenet2(temp.x_pos,temp.y_pos,lane_line.x,lane_line.y,lane_line.s[0]); 
    //     // temp.s_pos = object_s_d[0];
    //     // temp.d_pos = object_s_d[1];
    // }
    
    // vector<BOX_POINT> temp_point;
    // temp_point = trans_angle(temp.x_pos,temp.y_pos,temp.lwh.x,temp.lwh.y,90);
    // for (uint m = 0;m < 4; m++){
    //     temp.bbox_point[m].x = temp_point[m].x;
    //     temp.bbox_point[m].y = temp_point[m].y;
    //     temp.bbox_point[m].z = 0;
    //     temp.bbox_point[m+4].x = temp_point[m].x;
    //     temp.bbox_point[m+4].y = temp_point[m].y;
    //     temp.bbox_point[m+4].z = temp.lwh.z;
    // }
    
    // obj_result.objs.push_back(temp);
    ObjectArray_h * obj = &obj_result;
    char * output_data = (char*)obj;
    std::string out_id = "ass_object";
    //std::cout<<json_string<<endl;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, sizeof(Controlsrv_h));
    if (result != 0)
    {
        std::cerr << "failed to send output" << std::endl;
    }

    end_time = clock();
    //cout<<"the track_process time is : "<<(double)(end_time - start_time) / CLOCKS_PER_SEC<<"s."<<endl<<endl;

}



void OnlaneRecv(char *OnlaneRecv_data)
{
    LaneLineArray_h *laneline_data = reinterpret_cast<LaneLineArray_h *>(OnlaneRecv_data); 
    lane_line = laneline_data->lines[0];
}


int run(void *dora_context)
{
    unsigned char counter = 0;
    
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
            char *data;
            size_t data_len;
            char *data_id;
            size_t data_id_len;
            read_dora_input_data(event, &data, &data_len);
            read_dora_input_id(event, &data_id, &data_id_len);
            if (strcmp("LidarRawObject", data_id) == 0)
            {
                OnlidarRecv(data, data_len, dora_context);
            }
            // else if (strcmp("road_lane", data_id) == 0)
            // {
            //     OnlaneRecv(data);
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
    std::cout << "obj_track for dora " << std::endl;

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    std::cout << "exit obj_track  ..." << std::endl;

    return ret;
}
