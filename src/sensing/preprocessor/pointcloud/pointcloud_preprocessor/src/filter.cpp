/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pointcloud_preprocessor/filter.h"
#include <pcl/io/io.h>
#include "pcl_ros/transforms.h"

/*//#include <pcl/filters/pixel_grid.h>
//#include <pcl/filters/filter_dimension.h>
*/

/*//typedef pcl::PixelGrid PixelGrid;
//typedef pcl::FilterDimension FilterDimension;
*/

// Include the implementations instead of compiling them separately to speed up compile time
//#include "extract_indices.cpp"
//#include "passthrough.cpp"
//#include "project_inliers.cpp"
//#include "radius_outlier_removal.cpp"
//#include "statistical_outlier_removal.cpp"
//#include "voxel_grid.cpp"

/*//PLUGINLIB_EXPORT_CLASS(PixelGrid,nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(FilterDimension,nodelet::Nodelet);
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::computePublish(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices)
{
  PointCloud2 output;
  // Call the virtual method in the child
  filter(input, indices, output);

  PointCloud2::Ptr cloud_tf(new PointCloud2(output));  // set the output by default
  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && output.header.frame_id != tf_output_frame_) {
    NODELET_DEBUG(
      "[%s::computePublish] Transforming output dataset from %s to %s.", getName().c_str(),
      output.header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(tf_output_frame_, output, cloud_transformed, tf_listener_)) {
      NODELET_ERROR(
        "[%s::computePublish] Error converting output dataset from %s to %s.", getName().c_str(),
        output.header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }
  if (tf_output_frame_.empty() && output.header.frame_id != tf_input_orig_frame_)
  // no tf_output_frame given, transform the dataset to its original frame
  {
    NODELET_DEBUG(
      "[%s::computePublish] Transforming output dataset from %s back to %s.", getName().c_str(),
      output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!pcl_ros::transformPointCloud(
          tf_input_orig_frame_, output, cloud_transformed, tf_listener_)) {
      NODELET_ERROR(
        "[%s::computePublish] Error converting output dataset from %s back to %s.",
        getName().c_str(), output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }

  // Copy timestamp to keep it
  cloud_tf->header.stamp = input->header.stamp;

  // Publish a boost shared ptr
  pub_output_.publish(cloud_tf);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
    sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

    if (approximate_sync_) {
      sync_input_indices_a_ = boost::make_shared<message_filters::Synchronizer<
        sync_policies::ApproximateTime<PointCloud2, pcl_msgs::PointIndices> > >(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(bind(&Filter::input_indices_callback, this, _1, _2));
    } else {
      sync_input_indices_e_ = boost::make_shared<message_filters::Synchronizer<
        sync_policies::ExactTime<PointCloud2, pcl_msgs::PointIndices> > >(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(bind(&Filter::input_indices_callback, this, _1, _2));
    }
  } else
    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = pnh_->subscribe<sensor_msgs::PointCloud2>(
      "input", max_queue_size_,
      bind(&Filter::input_indices_callback, this, _1, pcl_msgs::PointIndicesConstPtr()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else
    sub_input_.shutdown();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::onInit()
{
  // Call the super onInit ()
  PCLNodelet::onInit();

  // Call the child's local init
  bool has_service = false;
  if (!child_init(*pnh_, has_service)) {
    NODELET_ERROR("[%s::onInit] Initialization failed.", getName().c_str());
    return;
  }

  pub_output_ = advertise<PointCloud2>(*pnh_, "output", max_queue_size_);

  // Enable the dynamic reconfigure service
  if (!has_service) {
    srv_ = boost::make_shared<dynamic_reconfigure::Server<pcl_ros::FilterConfig> >(*pnh_);
    dynamic_reconfigure::Server<pcl_ros::FilterConfig>::CallbackType f =
      boost::bind(&Filter::config_callback, this, _1, _2);
    srv_->setCallback(f);
  }

  NODELET_DEBUG("[%s::onInit] Nodelet successfully created.", getName().c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::config_callback(
  pcl_ros::FilterConfig & config, uint32_t level)
{
  // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are inexistent in PCL
  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG(
      "[%s::config_callback] Setting the input TF frame to: %s.", getName().c_str(),
      tf_input_frame_.c_str());
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG(
      "[%s::config_callback] Setting the output TF frame to: %s.", getName().c_str(),
      tf_output_frame_.c_str());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::input_indices_callback(
  const PointCloud2::ConstPtr & cloud, const PointIndicesConstPtr & indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    NODELET_ERROR("[%s::input_indices_callback] Invalid input!", getName().c_str());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    NODELET_ERROR("[%s::input_indices_callback] Invalid indices!", getName().c_str());
    return;
  }

  /// DEBUG
  if (indices)
    NODELET_DEBUG(
      "[%s::input_indices_callback]\n"
      "                                 - PointCloud with %d data points (%s), stamp %f, and frame "
      "%s on topic %s "
      "received.\n"
      "                                 - PointIndices with %zu values, stamp %f, and frame %s on "
      "topic %s received.",
      getName().c_str(), cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      cloud->header.stamp.toSec(), cloud->header.frame_id.c_str(),
      pnh_->resolveName("input").c_str(), indices->indices.size(), indices->header.stamp.toSec(),
      indices->header.frame_id.c_str(), pnh_->resolveName("indices").c_str());
  else
    NODELET_DEBUG(
      "[%s::input_indices_callback] PointCloud with %d data points and frame %s on topic %s "
      "received.",
      getName().c_str(), cloud->width * cloud->height, cloud->header.frame_id.c_str(),
      pnh_->resolveName("input").c_str());
  ///

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2::ConstPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    NODELET_DEBUG(
      "[%s::input_indices_callback] Transforming input dataset from %s to %s.", getName().c_str(),
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    if (!tf_listener_.waitForTransform(
          tf_input_frame_, cloud->header.frame_id, cloud->header.stamp, ros::Duration(1.0))) {
      NODELET_ERROR("[%s::input_indices_callback] timeout tf", getName().c_str());
      return;
    }
    if (!pcl_ros::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, tf_listener_)) {
      NODELET_ERROR(
        "[%s::input_indices_callback] Error converting input dataset from %s to %s.",
        getName().c_str(), cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = boost::make_shared<PointCloud2>(cloud_transformed);
  } else
    cloud_tf = cloud;

  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) vindices.reset(new std::vector<int>(indices->indices));

  computePublish(cloud_tf, vindices);
}
