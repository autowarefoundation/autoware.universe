#pragma once

#include <pcl/search/pcl_search.h>
#include "pointcloud_preprocessor/PassThroughFilterUInt16Config.h"
#include "pointcloud_preprocessor/passthrough_filter/passthrough_uint16.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor {
class PassThroughFilterUInt16Nodelet : public pointcloud_preprocessor::Filter {
 protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::PassThroughFilterUInt16Config> > srv_;
  virtual void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle& nh, bool& has_service);
  void config_callback(pointcloud_preprocessor::PassThroughFilterUInt16Config& config, uint32_t level);

 private:
  pcl::PassThroughUInt16<pcl::PCLPointCloud2> impl_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pointcloud_preprocessor
