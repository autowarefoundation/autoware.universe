#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "yaml-cpp/yaml.h"
#include <fstream>

class MapLoader
{
public:
    std::vector<std::string> file_list_;

    MapLoader();

    float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_;

    void init_tf_params();
    pcl::PointCloud<pcl::PointXYZ> CreatePcd();
    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformMap(pcl::PointCloud<pcl::PointXYZ> &in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);
};

#endif