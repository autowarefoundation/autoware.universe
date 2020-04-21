/*
 * BSD 2-Clause License
 * 
 * Copyright (c) 2019, k.koide
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
*/
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <iostream>

#include <ndt_omp/ndt_omp.h>

// align point clouds and measure processing time
pcl::PointCloud<pcl::PointXYZ>::Ptr align(
  pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud)
{
  registration->setInputTarget(target_cloud);
  registration->setInputSource(source_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

  auto t1 = ros::WallTime::now();
  registration->align(*aligned);
  auto t2 = ros::WallTime::now();
  std::cout << "single : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  for (int i = 0; i < 10; i++) {
    registration->align(*aligned);
  }
  auto t3 = ros::WallTime::now();
  std::cout << "10times: " << (t3 - t2).toSec() * 1000 << "[msec]" << std::endl;
  std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;

  return aligned;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cout << "usage: align target.pcd source.pcd" << std::endl;
    return 0;
  }

  std::string target_pcd = argv[1];
  std::string source_pcd = argv[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile(target_pcd, *target_cloud)) {
    std::cerr << "failed to load " << target_pcd << std::endl;
    return 0;
  }
  if (pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
    std::cerr << "failed to load " << source_pcd << std::endl;
    return 0;
  }

  // downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  voxelgrid.setInputCloud(target_cloud);
  voxelgrid.filter(*downsampled);
  *target_cloud = *downsampled;

  voxelgrid.setInputCloud(source_cloud);
  voxelgrid.filter(*downsampled);
  source_cloud = downsampled;

  ros::Time::init();

  // benchmark
  std::cout << "--- pcl::NDT ---" << std::endl;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
    new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt->setResolution(1.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned = align(ndt, target_cloud, source_cloud);

  std::vector<int> num_threads = {1, omp_get_max_threads()};
  std::vector<std::pair<std::string, ndt_omp::NeighborSearchMethod>> search_methods = {
    {"KDTREE", ndt_omp::KDTREE}, {"DIRECT7", ndt_omp::DIRECT7}, {"DIRECT1", ndt_omp::DIRECT1}};

  ndt_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(
    new ndt_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1.0);

  for (int n : num_threads) {
    for (const auto & search_method : search_methods) {
      std::cout << "--- ndt_omp::NDT (" << search_method.first << ", " << n << " threads) ---"
                << std::endl;
      ndt_omp->setNumThreads(n);
      ndt_omp->setNeighborhoodSearchMethod(search_method.second);
      aligned = align(ndt_omp, target_cloud, source_cloud);
    }
  }

  // visulization
  pcl::visualization::PCLVisualizer vis("vis");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_handler(
    target_cloud, 255.0, 0.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_handler(
    source_cloud, 0.0, 255.0, 0.0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_handler(
    aligned, 0.0, 0.0, 255.0);
  vis.addPointCloud(target_cloud, target_handler, "target");
  vis.addPointCloud(source_cloud, source_handler, "source");
  vis.addPointCloud(aligned, aligned_handler, "aligned");
  vis.spin();

  return 0;
}
