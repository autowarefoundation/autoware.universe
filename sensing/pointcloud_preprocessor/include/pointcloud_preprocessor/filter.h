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
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef POINTS_PREPROCESSOR_PCL_ROS_FILTER_H_
#define POINTS_PREPROCESSOR_PCL_ROS_FILTER_H_

// PCL includes
#include <pcl/filters/filter.h>
#include "pcl_ros/pcl_nodelet.h"

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "pcl_ros/FilterConfig.h"

namespace pointcloud_preprocessor
{
namespace sync_policies = message_filters::sync_policies;

/** \brief @b Filter represents the base filter class. Some generic 3D operations that are applicable to all filters
 * are defined here as static methods.
 * \author Radu Bogdan Rusu
 */
class Filter : public pcl_ros::PCLNodelet
{
public:
  typedef sensor_msgs::PointCloud2 PointCloud2;

  typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
  typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

  Filter() {}

protected:
  /** \brief The input PointCloud subscriber. */
  ros::Subscriber sub_input_;

  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default:
   * false. */
  bool filter_limit_negative_;

  /** \brief The input TF frame the data should be transformed into, if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into, if input.header.frame_id is different. */
  std::string tf_output_frame_;

  /** \brief Internal mutex. */
  boost::mutex mutex_;

  /** \brief Child initialization routine.
   * \param nh ROS node handle
   * \param has_service set to true if the child has a Dynamic Reconfigure service
   */
  virtual bool child_init(ros::NodeHandle & nh, bool & has_service)
  {
    has_service = false;
    return (true);
  }

  /** \brief Virtual abstract filter method. To be implemented by every child.
   * \param input the input point cloud dataset.
   * \param indices a pointer to the vector of point indices to use.
   * \param output the resultant filtered PointCloud2
   */
  virtual void filter(
    const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) = 0;

  /** \brief Lazy transport subscribe routine. */
  virtual void subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  virtual void unsubscribe();

  /** \brief Nodelet initialization routine. */
  virtual void onInit();

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input the input point cloud dataset.
   * \param indices a pointer to the vector of point indices to use.
   */
  void computePublish(const PointCloud2::ConstPtr & input, const IndicesPtr & indices);

private:
  /** \brief Pointer to a dynamic reconfigure service. */
  boost::shared_ptr<dynamic_reconfigure::Server<pcl_ros::FilterConfig> > srv_;

  /** \brief Synchronized input, and indices.*/
  boost::shared_ptr<
    message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices> > >
    sync_input_indices_e_;
  boost::shared_ptr<
    message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices> > >
    sync_input_indices_a_;

  /** \brief Dynamic reconfigure service callback. */
  virtual void config_callback(pcl_ros::FilterConfig & config, uint32_t level);

  /** \brief PointCloud2 + Indices data callback. */
  void input_indices_callback(
    const PointCloud2::ConstPtr & cloud, const PointIndicesConstPtr & indices);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pointcloud_preprocessor

#endif  //#ifndef POINTS_PREPROCESSOR_PCL_ROS_FILTER_H_
