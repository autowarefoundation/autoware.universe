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

int ndt_ld(void *dora_context, MapLoader &maploader, Points_downsampler &p_d, NdtLocalizer &ndt_l)
{
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");
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

            for (int i = 0; i < (data_len - 16) / 16; i++)
            {
                // voxel_grid_filter.points.push_back(pcl::PointXYZI(*(float*)(data + 16 + 16 * i), *(float*)(data + 16 + 4 + 16 * i),
                //                                         *(float*)(data + 16 + 8 + 16 * i), *(float*)(data + 16 + 12 + 16 * i)));
                pcl::PointXYZI tem_point;
                tem_point.x = *(float *)(data + 16 + 16 * i);
                tem_point.y = *(float *)(data + 16 + 4 + 16 * i);
                tem_point.z = *(float *)(data + 16 + 8 + 16 * i);
                tem_point.intensity = *(float *)(data + 16 + 12 + 16 * i);
                voxel_grid_filter.points.push_back(tem_point);
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_grid_filter_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(voxel_grid_filter, *voxel_grid_filter_ptr);

            auto pc_msg = maploader.CreatePcd();
            auto out_msg = maploader.TransformMap(pc_msg);
            pcl::PointCloud<pcl::PointXYZ>::Ptr out_msg_ptr = out_msg;
            if (out_msg_ptr->width != 0)
            {
                out_msg_ptr->header.frame_id = "map";
                auto filtered_points = p_d.scan_callback(voxel_grid_filter_ptr);

                std::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> initial_pose_msg_ptr = std::make_shared<const geometry_msgs::PoseWithCovarianceStamped>(
                    geometry_msgs::PoseWithCovarianceStamped{.header = {0, "map"}, .pose{{{0, 0, 0}, {0, 0, 0, 0}}, .covariance{0.0}}});

                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_points));

                ndt_l.callback_init_pose(initial_pose_msg_ptr);
                ndt_l.callback_pointsmap(out_msg_ptr);
                ndt_l.callback_pointcloud(filtered_points_ptr);
            }

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
