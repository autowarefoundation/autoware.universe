#include "autoware/ndt_omp/multigrid_pclomp/multigrid_ndt_omp.h"

#include "autoware/ndt_omp/multigrid_pclomp/multigrid_ndt_omp_impl.hpp"

template class pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::MultiGridNormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
