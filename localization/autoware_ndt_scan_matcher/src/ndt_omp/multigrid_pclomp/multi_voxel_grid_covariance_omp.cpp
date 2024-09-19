#include "autoware/ndt_omp/multigrid_pclomp/multi_voxel_grid_covariance_omp.h"

#include "autoware/ndt_omp/multigrid_pclomp/multi_voxel_grid_covariance_omp_impl.hpp"

template class pclomp::MultiVoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::MultiVoxelGridCovariance<pcl::PointXYZI>;
