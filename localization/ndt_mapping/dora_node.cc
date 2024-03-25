extern "C"
{
#include "node_api.h"   
#include "operator_api.h"
#include "operator_types.h"
}
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "dora_ndt_mapper.h"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

int max_save = 1000;

int mapping(void *dora_context, ndt_mapping::NDTMapper &mapper){
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");
    while(true){
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);
            // std::cout << "This is a dora ndt mapping node!" << std::endl;
            int32_t point_len = (data_len-16)/4 ;
            // std::cout << "data_len:" << data_len 
            //             << "  Point_len:" << point_len <<std::endl;

            pcl::PointCloud<pcl::PointXYZI> pointcloud;
            pointcloud.header.seq = *(std::uint32_t*)data;
            pointcloud.header.stamp = *(std::uint64_t*)(data+8);
            pointcloud.header.frame_id = "lidar";
            pointcloud.width = point_len;
            pointcloud.height = 1;
            pointcloud.is_dense = true;
            // pcl::PointXYZI* tem_point =  reinterpret_cast<pcl::PointXYZI*>(data.ptr + 16)
            // pointcloud.points = tem_point;
            for(int i = 0; i < (data_len-16)/16; i++){
                pcl::PointXYZI tem_point;
                
                tem_point.x = *(float*)(data + 16 + 16 * i);
                tem_point.y = *(float*)(data + 16 + 4 + 16 * i);
                tem_point.z = *(float*)(data + 16 + 8 + 16 * i);
                tem_point.intensity = *(float*)(data + 16 + 12 + 16 * i);
                pointcloud.points.push_back(tem_point);
                //   pointcloud.points.push_back(pcl::PointXYZI(*(float*)(data + 16 + 16 * i), *(float*)(data + 16 + 4 + 16 * i),*(float*)(data + 16 + 8 + 16 * i), *(float*)(data + 16 + 12 + 16 * i)));
            }

            pointcloud_ptr = pointcloud.makeShared();
            mapper.points_callback(pointcloud_ptr);
            
            if(*(std::uint32_t*)data >= max_save){  // if(seq == 1000)
                if(*(std::uint32_t*)data == max_save)
                {
                    std::cout << "1000 scan map" << std::endl;
                    mapper.saveMap();
                }
                free_dora_event(event);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            // mapper.saveMap();
            printf("[c node] received stop event\n");
            free_dora_event(event);
        }
        else
        {
            // mapper.saveMap();
            printf("[c node] received unexpected event: %d\n", ty);
            free_dora_event(event);
        }
    }

}

int main(int argc, char* argv[])
{
    printf("dora ndt mapping node\n");

    void *dora_context = init_dora_context_from_env();

    ndt_mapping::NDTMapper mapper;
    mapping(dora_context, mapper);


    printf("[c node] received 10 events\n");

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
