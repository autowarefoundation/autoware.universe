#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
class BoundingBoxCalculator{

public:
struct BoundingBox
{
  BoundingBox(){}
  cv::Point3f corners[8];/**< 8 corners of boundingbox */
  cv::Point3f size;/**< x:length, y:width, z:height, notice that length >= width*/
  cv::Point3f center; /**<box geometry center*/
//  cv::Point3f anchor;/**< the nearest corner point of box to the lidar*/
  float angle; /**< 2D (x-y plane) box main-direction(parallel to the longer edge of box) angle relative to x-axis of lidar coordinate*/
};

BoundingBox calcBoundingBox(const pcl::PointCloud<pcl::PointXYZI> &segment);
void init();

private:
BoundingBox calcSimpleBox(const pcl::PointCloud<pcl::PointXYZI> &segment);


float theta_ ;
std::vector<float> tabCos_,tabSin_;


};