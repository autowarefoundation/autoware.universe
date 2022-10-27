#include <pclomp/ndt_omp_impl.hpp>

#include <pclomp/ndt_omp.h>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
