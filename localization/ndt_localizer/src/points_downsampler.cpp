#include "points_downsampler.h"
// YAML::Node config = YAML::Load("/home/char/ndt_localizer_new/ndt_localizer_config.yaml");
//  voxel_leaf_size = config["voxel_leaf_size"].as<double>();
static double voxel_leaf_size = 1.00;

pcl::PointCloud<pcl::PointXYZ> Points_downsampler::removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
  narrowed_scan.header = scan.header;

  if (min_range >= max_range)
  {
    std::cout << ("min_range>=max_range @(%lf, %lf)", min_range, max_range);
    return scan;
  }

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZ &p = *iter;
    double square_distance = p.x * p.x + p.y * p.y;

    if (square_min_range <= square_distance && square_distance <= square_max_range)
    {
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}

pcl::PointCloud<pcl::PointXYZ> Points_downsampler::scan_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &input)
{
  pcl::PointCloud<pcl::PointXYZ> scan = *input;
  scan = removePointsByRange(scan, 0, MAX_MEASUREMENT_RANGE);

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PointCloud<pcl::PointXYZ> filtered_msg;

  // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (voxel_leaf_size >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
    filtered_msg = *filtered_scan_ptr;
  }
  else
  {
    filtered_msg = *scan_ptr;
  }

  filtered_msg.header = input->header;

  return filtered_msg;
}
