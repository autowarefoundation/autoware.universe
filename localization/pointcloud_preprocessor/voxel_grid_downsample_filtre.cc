// clang++ voxel_grid_downsample_filtre.cc -std=c++14 -o voxel_grid_downsample_filtre -I/usr/include/eigen3 -I/usr/include/pcl-1.12  -lpcap -ldora_node_api_c -L /home/zxd/dora-rs/dora/target/release -lpthread -ldl -lrt -I /home/zxd/rs_driver/src -I /home/zxd/dora-rs/dora/apis/c -l dora_operator_api_c -L /home/zxd/dora-rs/dora/target/debug -lpcl_common -lpcl_filters

// clang++ voxel_grid_downsample_filtre.cc -std=c++14 -o voxel_grid_downsample_filtre -I/usr/include/eigen3 -I/usr/include/pcl-1.12  -lpcap -ldora_node_api_c -L /home/zxd/dora-rs/dora/target/release -lpthread -ldl -lrt -I /home/zxd/rs_driver/src -I /home/zxd/dora-rs/dora/apis/c -l dora_operator_api_c -L /home/zxd/dora-rs/dora/target/debug -lpcl_common -lpcl_filters

extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <pcl/point_types.h>

static Vec_uint8_t point_data;

namespace pointcloud_preprocessor
{
  class DownsampleFilter
  {
  public:
    DownsampleFilter(double voxel_size_x, double voxel_size_y, double voxel_size_z)
        : voxel_size_x_(voxel_size_x), voxel_size_y_(voxel_size_y), voxel_size_z_(voxel_size_z)
    {
    }

    void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
    {
      pcl::VoxelGrid<pcl::PointXYZI> filter;
      filter.setInputCloud(input);
      filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
      filter.filter(*output);
    }

    void randomFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &output, size_t sample_num)
    {
      pcl::RandomSample<pcl::PointXYZI> filter;
      filter.setInputCloud(input);
      filter.setSample(sample_num);
      filter.filter(*output);
    }

  private:
    double voxel_size_x_;
    double voxel_size_y_;
    double voxel_size_z_;
  };

}

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
      char *data;
      size_t data_len;
      read_dora_input_data(event, &data, &data_len);
      std::cout << "length: " << data_len << std::endl;
      size_t point_mum = data_len / 16 - 1;
      std::cout << "point_mum : " << point_mum << std::endl;

      pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      // pcl::PointXYZI *input_points = reinterpret_cast<pcl::PointXYZI *>(data + 16);
      // for (size_t i = 0; i < data_len / 16 - 1; i++)
      // {
      //   // std::cout << count << std::endl;
      //   input_cloud->push_back(input_points[i]);
      //   // std::cout << "x: " << input_cloud->points[i].x << std::endl;
      // }

      pcl::PointCloud<pcl::PointXYZI> pointcloud_tran;
      for (int i = 0; i < point_mum; i++)
      {
        pcl::PointXYZI pointcloud_tran_1;
        pointcloud_tran_1.x = *(float *)(data + 16 + 16 * i);
        pointcloud_tran_1.y = *(float *)(data + 16 + 4 + 16 * i);
        pointcloud_tran_1.z = *(float *)(data + 16 + 8 + 16 * i);
        pointcloud_tran_1.intensity = *(float *)(data + 16 + 12 + 16 * i);
        pointcloud_tran.push_back(pointcloud_tran_1);
      }

      // for(const auto& point : pointcloud_tran.points){
      //   std::cout << "x: " << point.x << "y: " << point.y << std::endl;
      // }

      input_cloud = pointcloud_tran.makeShared();

      pointcloud_preprocessor::DownsampleFilter filter(0.5, 0.5, 0.2);
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_voxelgrid(new pcl::PointCloud<pcl::PointXYZI>);
      filter.voxelGridFilter(input_cloud, output_cloud_voxelgrid);
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_random(new pcl::PointCloud<pcl::PointXYZI>);
      filter.randomFilter(output_cloud_voxelgrid, output_cloud_random, 5000);

      std::cout << "outpoint size: " << output_cloud_random->points.size() << std::endl;

      size_t all_size = 16 + output_cloud_random->points.size() * 16;
      point_data.ptr = new uint8_t[all_size];

      memcpy(point_data.ptr, data, 16);
      memcpy(point_data.ptr + 16, output_cloud_random->points.data(), output_cloud_random->points.size() * 16);

      char *output_data = (char *)point_data.ptr;
      size_t output_data_len = ((output_cloud_random->points.size() + 1) * 16);
      std::string out_id = "pointcloud";
      int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
      delete[] point_data.ptr;

      if (resultend != 0)
      {
        std::cerr << "failed to send output" << std::endl;
        return 1;
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
  std::cout << "voxel_grid_downsample_fliter node" << std::endl;
  auto dora_context = init_dora_context_from_env();
  auto ret = run(dora_context);
  free_dora_context(dora_context);
  return ret;
}
