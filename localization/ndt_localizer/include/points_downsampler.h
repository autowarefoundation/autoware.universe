#pragma once
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include "yaml-cpp/yaml.h"
#define MAX_MEASUREMENT_RANGE 100.0

class Points_downsampler
{
public:
    static pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range);
    pcl::PointCloud<pcl::PointXYZ> scan_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input);
};