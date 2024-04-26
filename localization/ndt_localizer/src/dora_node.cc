extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "map_loader.h"
#include "points_downsampler.h"
#include "ndt_localizer.h"
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

int ndt_ld(void *dora_context, MapLoader &maploader, Points_downsampler &p_d, NdtLocalizer &ndt_l)
{
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");
    std::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> initial_pose_msg_ptr = std::make_shared<const geometry_msgs::PoseWithCovarianceStamped>(
        geometry_msgs::PoseWithCovarianceStamped{.header = {0, "map"}, .pose{{{0, 0, 0}, {0, 0, 0, 0}}, .covariance{0.0}}});
    ndt_l.callback_init_pose(initial_pose_msg_ptr);
    auto pc_msg = maploader.CreatePcd();
    auto out_msg = maploader.TransformMap(pc_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_msg_ptr = out_msg;
    ndt_l.callback_pointsmap(out_msg_ptr);
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
            read_dora_input_data(event, &data, &data_len);
            std::cout << "This is a dora ndt_localizer node!" << std::endl;
            int32_t point_len = (data_len - 16) / 4;
            std::cout << "data_len:" << data_len
                      << "  Point_len:" << point_len << std::endl;

            pcl::PointCloud<pcl::PointXYZI> voxel_grid_filter;
            voxel_grid_filter.header.seq = *(std::uint32_t *)data;
            voxel_grid_filter.header.stamp = *(std::uint64_t *)(data + 8);
            voxel_grid_filter.header.frame_id = "lidar";
            voxel_grid_filter.width = point_len;
            voxel_grid_filter.height = 1;
            voxel_grid_filter.is_dense = true;

            std::cout << "*******************************************" << std::endl;
            std::cout << "lidar scan seq:" << voxel_grid_filter.header.seq << std::endl;
            std::cout << "*******************************************" << std::endl;

            for (int i = 0; i < (data_len - 16) / 16; i++)
            {
                pcl::PointXYZI tem_point;
                tem_point.x = *(float *)(data + 16 + 16 * i);
                tem_point.y = *(float *)(data + 16 + 4 + 16 * i);
                tem_point.z = *(float *)(data + 16 + 8 + 16 * i);
                tem_point.intensity = *(float *)(data + 16 + 12 + 16 * i);
                voxel_grid_filter.points.push_back(tem_point);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(voxel_grid_filter, *voxel_grid_filter_ptr);

            if (out_msg_ptr->width != 0)
            {
                out_msg_ptr->header.frame_id = "map";
                auto filtered_points = p_d.scan_callback(voxel_grid_filter_ptr);
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points));
                // ndt_l.callback_pointsmap(out_msg_ptr);
                ndt_l.callback_pointcloud(filtered_points_ptr);
                if(ndt_l.is_converged)
                {
                    std::string out = "resultpose";
                    std::string out_id = "resultpose";
                    float o_w = ndt_l.result_pose_stamped_msg.pose.orientation.w;
                    float o_x = ndt_l.result_pose_stamped_msg.pose.orientation.x;
                    float o_y = ndt_l.result_pose_stamped_msg.pose.orientation.y;
                    float o_z = ndt_l.result_pose_stamped_msg.pose.orientation.z;
                    float p_x = ndt_l.result_pose_stamped_msg.pose.position.x;
                    float p_y = ndt_l.result_pose_stamped_msg.pose.position.y;
                    float p_z = ndt_l.result_pose_stamped_msg.pose.position.z;

                    json path_data_dict;
                    json pose_position = {
                        {"x", p_x},
                        {"y", p_y},
                        {"z", p_z}
                    };
                    json pose_orientation = {
                        {"x", o_x},
                        {"y", o_y},
                        {"z", o_z},
                        {"w", o_w}
                    };
                    json pose = {
                        {"position", pose_position},
                        {"orientation", pose_orientation}
                    };
                    path_data_dict["pose"] = pose;
                    // 填充header数据
                    json header_stamp = {
                        {"sec", 111},
                        {"nanosec", 222}
                    };
                    json header = {
                        {"seq", 333},
                        {"stamp", 123},
                        {"frame_id", "map"}
                    };
                    path_data_dict["header"] = header;
                    // 将 JSON 对象序列化为字符串
                    std::string json_string = path_data_dict.dump(4); // 参数 4 表示缩进宽度
                    // 将字符串转换为 char* 类型
                    char *c_json_string = new char[json_string.length() + 1];
                    strcpy(c_json_string, json_string.c_str());
                    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
                    if (result != 0)
                    {
                         printf("failed to send output\n");
                        // std::cerr << "failed to send output" << std::endl;
                        free_dora_event(event);
                        return 1;
                    }
                }
                else printf("ndt_l.is_converged = 0\n");
            }
            else  printf("out_msg_ptr->width = 0\n");
            free_dora_event(event);
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
            free_dora_event(event);
        }
        else
        {

            printf("[c node] received unexpected event: %d\n", ty);
            free_dora_event(event);
        }
    }
}

int main(int argc, char *argv[])
{
    printf("dora ndt_localizer node\n");

    void *dora_context = init_dora_context_from_env();

    MapLoader maploader;
    Points_downsampler p_d;
    NdtLocalizer ndt_l;
    ndt_ld(dora_context, maploader, p_d, ndt_l);

    printf("[c node] received 10 events\n");

    free_dora_context(dora_context);

    printf("[c node] finished successfully\n");

    return 0;
}
