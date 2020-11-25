/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pointcloud_preprocessor/passthrough_filter/passthrough_filter_uint16_nodelet.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
PassThroughFilterUInt16Component::PassThroughFilterUInt16Component(
  const rclcpp::NodeOptions & options)
: Filter("PassThroughFilterUInt16", options)
{
  // set initial parameters
  {
    int filter_min = static_cast<int>(declare_parameter("filter_limit_min", 0));
    int filter_max = static_cast<int>(declare_parameter("filter_limit_max", 127));
    impl_.setFilterLimits(filter_min, filter_max);

    impl_.setFilterFieldName(
      static_cast<std::string>(declare_parameter("filter_field_name", "ring")));
    impl_.setKeepOrganized(static_cast<bool>(declare_parameter("keep_organized", false)));
    impl_.setFilterLimitsNegative(
      static_cast<bool>(declare_parameter("filter_limit_negative", false)));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PassThroughFilterUInt16Component::paramCallback, this, _1));
}

void PassThroughFilterUInt16Component::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);

  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*(input), *(pcl_input));
  impl_.setInputCloud(pcl_input);
  impl_.setIndices(indices);
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter(pcl_output);
  pcl_conversions::moveFromPCL(pcl_output, output);
}

rcl_interfaces::msg::SetParametersResult PassThroughFilterUInt16Component::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  boost::mutex::scoped_lock lock(mutex_);

  std::uint16_t filter_min, filter_max;
  impl_.getFilterLimits(filter_min, filter_max);

  // Check the current values for filter min-max
  if (get_param(p, "filter_limit_min", filter_min)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the minimum filtering value a point will be considered from to: %d.",
      filter_min);
    // Set the filter min-max if different
    impl_.setFilterLimits(filter_min, filter_max);
  }
  // Check the current values for filter min-max
  if (get_param(p, "filter_limit_max", filter_max)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the maximum filtering value a point will be considered from to: %d.",
      filter_max);
    // Set the filter min-max if different
    impl_.setFilterLimits(filter_min, filter_max);
  }

  // Check the current value for the filter field
  std::string filter_field_name;
  get_param(p, "filter_field_name", filter_field_name);
  if (impl_.getFilterFieldName() != filter_field_name) {
    // Set the filter field if different
    impl_.setFilterFieldName(filter_field_name);
    RCLCPP_DEBUG(get_logger(), "Setting the filter field name to: %s.", filter_field_name.c_str());
  }

  // Check the current value for keep_organized
  bool keep_organized;
  get_param(p, "keep_organized", keep_organized);
  if (impl_.getKeepOrganized() != keep_organized) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the filter keep_organized value to: %s.",
      keep_organized ? "true" : "false");
    // Call the virtual method in the child
    impl_.setKeepOrganized(keep_organized);
  }

  // Check the current value for the negative flag
  bool filter_limit_negative;
  get_param(p, "filter_limit_negative", filter_limit_negative);
  if (impl_.getFilterLimitsNegative() != filter_limit_negative) {
    RCLCPP_DEBUG(
      get_logger(), "Setting the filter negative flag to: %s.",
      filter_limit_negative ? "true" : "false");
    // Call the virtual method in the child
    impl_.setFilterLimitsNegative(filter_limit_negative);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PassThroughFilterUInt16Component)
