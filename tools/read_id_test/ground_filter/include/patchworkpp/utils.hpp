#ifndef COMMON_H
#define COMMON_H

#include "math.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <map>
#include <sstream>
#include <vector>
// #include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <fstream>
// CLASSES
#define SENSOR_HEIGHT 1.73

#define UNLABELED 0
#define OUTLIER 1
#define NUM_ALL_CLASSES 34
#define ROAD 40
#define PARKING 44
#define SIDEWALKR 48
#define OTHER_GROUND 49
#define BUILDING 50
#define FENSE 51
#define LANE_MARKING 60
#define VEGETATION 70
#define TERRAIN 72

#define TRUEPOSITIVE 3
#define TRUENEGATIVE 2
#define FALSEPOSITIVE 1
#define FALSENEGATIVE 0

int NUM_ZEROS = 5;

using namespace std;

double VEGETATION_THR = -SENSOR_HEIGHT * 3 / 4;
/** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct EIGEN_ALIGN16 PointXYZILID
{
  PCL_ADD_POINT4D;  // quad-word XYZ
  float intensity;  ///< laser intensity reading
  uint16_t label;   ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
};

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZILID, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                  std::uint16_t, label, label)(std::uint16_t, id, id))

void PointXYZILID2XYZI(
  pcl::PointCloud<PointXYZILID> & src, pcl::PointCloud<pcl::PointXYZI>::Ptr dst)
{
  dst->points.clear();
  for (const auto & pt : src.points) {
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x = pt.x;
    pt_xyzi.y = pt.y;
    pt_xyzi.z = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }
}
std::vector<int> outlier_classes = {UNLABELED, OUTLIER};
std::vector<int> ground_classes = {ROAD,         PARKING,    SIDEWALKR, OTHER_GROUND,
                                   LANE_MARKING, VEGETATION, TERRAIN};
std::vector<int> ground_classes_except_terrain = {
  ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING};
std::vector<int> traversable_ground_classes = {ROAD, PARKING, LANE_MARKING, OTHER_GROUND};

int count_num_ground(const pcl::PointCloud<PointXYZILID> & pc)
{
  int num_ground = 0;

  std::vector<int>::iterator iter;

  for (auto const & pt : pc.points) {
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()) {  // corresponding class is in ground classes
      if (pt.label == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          num_ground++;
        }
      } else
        num_ground++;
    }
  }
  return num_ground;
}

int count_num_ground_without_vegetation(const pcl::PointCloud<PointXYZILID> & pc)
{
  int num_ground = 0;

  std::vector<int>::iterator iter;

  std::vector<int> classes = {ROAD, PARKING, SIDEWALKR, OTHER_GROUND, LANE_MARKING, TERRAIN};

  for (auto const & pt : pc.points) {
    iter = std::find(classes.begin(), classes.end(), pt.label);
    if (iter != classes.end()) {  // corresponding class is in ground classes
      num_ground++;
    }
  }
  return num_ground;
}

std::map<int, int> set_initial_gt_counts(std::vector<int> & gt_classes)
{
  map<int, int> gt_counts;
  for (int i = 0; i < gt_classes.size(); ++i) {
    gt_counts.insert(pair<int, int>(gt_classes.at(i), 0));
  }
  return gt_counts;
}

std::map<int, int> count_num_each_class(const pcl::PointCloud<PointXYZILID> & pc)
{
  int num_ground = 0;
  auto gt_counts = set_initial_gt_counts(ground_classes);
  std::vector<int>::iterator iter;

  for (auto const & pt : pc.points) {
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()) {  // corresponding class is in ground classes
      if (pt.label == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          gt_counts.find(pt.label)->second++;
        }
      } else
        gt_counts.find(pt.label)->second++;
    }
  }
  return gt_counts;
}

int count_num_outliers(const pcl::PointCloud<PointXYZILID> & pc)
{
  int num_outliers = 0;

  std::vector<int>::iterator iter;

  for (auto const & pt : pc.points) {
    iter = std::find(outlier_classes.begin(), outlier_classes.end(), pt.label);
    if (iter != outlier_classes.end()) {  // corresponding class is in ground classes
      num_outliers++;
    }
  }
  return num_outliers;
}

void discern_ground(
  const pcl::PointCloud<PointXYZILID> & src, pcl::PointCloud<PointXYZILID> & ground,
  pcl::PointCloud<PointXYZILID> & non_ground)
{
  ground.clear();
  non_ground.clear();
  std::vector<int>::iterator iter;
  for (auto const & pt : src.points) {
    if (pt.label == UNLABELED || pt.label == OUTLIER) continue;
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()) {  // corresponding class is in ground classes
      if (pt.label == VEGETATION) {
        if (pt.z < VEGETATION_THR) {
          ground.push_back(pt);
        } else
          non_ground.push_back(pt);
      } else
        ground.push_back(pt);
    } else {
      non_ground.push_back(pt);
    }
  }
}

void discern_ground_without_vegetation(
  const pcl::PointCloud<PointXYZILID> & src, pcl::PointCloud<PointXYZILID> & ground,
  pcl::PointCloud<PointXYZILID> & non_ground)
{
  ground.clear();
  non_ground.clear();
  std::vector<int>::iterator iter;
  for (auto const & pt : src.points) {
    if (pt.label == UNLABELED || pt.label == OUTLIER) continue;
    iter = std::find(ground_classes.begin(), ground_classes.end(), pt.label);
    if (iter != ground_classes.end()) {  // corresponding class is in ground classes
      if (pt.label != VEGETATION) ground.push_back(pt);
    } else {
      non_ground.push_back(pt);
    }
  }
}

void calculate_precision_recall(
  const pcl::PointCloud<PointXYZILID> & pc_curr, pcl::PointCloud<PointXYZILID> & ground_estimated,
  double & precision, double & recall, bool consider_outliers = true)
{
  int num_ground_est = ground_estimated.points.size();
  int num_ground_gt = count_num_ground(pc_curr);
  int num_TP = count_num_ground(ground_estimated);
  if (consider_outliers) {
    int num_outliers_est = count_num_outliers(ground_estimated);
    precision = (double)(num_TP) / (num_ground_est - num_outliers_est) * 100;
    recall = (double)(num_TP) / num_ground_gt * 100;
  } else {
    precision = (double)(num_TP) / num_ground_est * 100;
    recall = (double)(num_TP) / num_ground_gt * 100;
  }
}

void calculate_precision_recall_without_vegetation(
  const pcl::PointCloud<PointXYZILID> & pc_curr, pcl::PointCloud<PointXYZILID> & ground_estimated,
  double & precision, double & recall, bool consider_outliers = true)
{
  int num_veg = 0;
  for (auto const & pt : ground_estimated.points) {
    if (pt.label == VEGETATION) num_veg++;
  }

  int num_ground_est = ground_estimated.size() - num_veg;
  int num_ground_gt = count_num_ground_without_vegetation(pc_curr);
  int num_TP = count_num_ground_without_vegetation(ground_estimated);
  if (consider_outliers) {
    int num_outliers_est = count_num_outliers(ground_estimated);
    precision = (double)(num_TP) / (num_ground_est - num_outliers_est) * 100;
    recall = (double)(num_TP) / num_ground_gt * 100;
  } else {
    precision = (double)(num_TP) / num_ground_est * 100;
    recall = (double)(num_TP) / num_ground_gt * 100;
  }
}

void save_all_labels(
  const pcl::PointCloud<PointXYZILID> & pc, string ABS_DIR, string seq, int count)
{
  std::string count_str = std::to_string(count);
  std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
  std::string output_filename = ABS_DIR + "/" + seq + "/" + count_str_padded + ".csv";
  ofstream sc_output(output_filename);

  vector<int> labels(NUM_ALL_CLASSES, 0);
  for (auto const & pt : pc.points) {
    if (pt.label == 0)
      labels[0]++;
    else if (pt.label == 1)
      labels[1]++;
    else if (pt.label == 10)
      labels[2]++;
    else if (pt.label == 11)
      labels[3]++;
    else if (pt.label == 13)
      labels[4]++;
    else if (pt.label == 15)
      labels[5]++;
    else if (pt.label == 16)
      labels[6]++;
    else if (pt.label == 18)
      labels[7]++;
    else if (pt.label == 20)
      labels[8]++;
    else if (pt.label == 30)
      labels[9]++;
    else if (pt.label == 31)
      labels[10]++;
    else if (pt.label == 32)
      labels[11]++;
    else if (pt.label == 40)
      labels[12]++;
    else if (pt.label == 44)
      labels[13]++;
    else if (pt.label == 48)
      labels[14]++;
    else if (pt.label == 49)
      labels[15]++;
    else if (pt.label == 50)
      labels[16]++;
    else if (pt.label == 51)
      labels[17]++;
    else if (pt.label == 52)
      labels[18]++;
    else if (pt.label == 60)
      labels[19]++;
    else if (pt.label == 70)
      labels[20]++;
    else if (pt.label == 71)
      labels[21]++;
    else if (pt.label == 72)
      labels[22]++;
    else if (pt.label == 80)
      labels[23]++;
    else if (pt.label == 81)
      labels[24]++;
    else if (pt.label == 99)
      labels[25]++;
    else if (pt.label == 252)
      labels[26]++;
    else if (pt.label == 253)
      labels[27]++;
    else if (pt.label == 254)
      labels[28]++;
    else if (pt.label == 255)
      labels[29]++;
    else if (pt.label == 256)
      labels[30]++;
    else if (pt.label == 257)
      labels[31]++;
    else if (pt.label == 258)
      labels[32]++;
    else if (pt.label == 259)
      labels[33]++;
  }

  for (uint8_t i = 0; i < NUM_ALL_CLASSES; ++i) {
    if (i != 33) {
      sc_output << labels[i] << ",";
    } else {
      sc_output << labels[i] << endl;
    }
  }
  sc_output.close();
}

void save_all_accuracy(
  const pcl::PointCloud<PointXYZILID> & pc_curr, pcl::PointCloud<PointXYZILID> & ground_estimated,
  string acc_filename, double & accuracy, map<int, int> & pc_curr_gt_counts,
  map<int, int> & g_est_gt_counts)
{
  //  std::cout<<"debug: "<<acc_filename<<std::endl;
  ofstream sc_output2(acc_filename, ios::app);

  int num_True = count_num_ground(pc_curr);
  int num_outliers_gt = count_num_outliers(pc_curr);
  int num_outliers_est = count_num_outliers(ground_estimated);

  int num_total_est = ground_estimated.points.size() - num_outliers_est;
  int num_total_gt = pc_curr.points.size() - num_outliers_gt;

  int num_False = num_total_gt - num_True;
  int num_TP = count_num_ground(ground_estimated);
  int num_FP = num_total_est - num_TP;
  accuracy = static_cast<double>(num_TP + (num_False - num_FP)) / num_total_gt * 100.0;

  pc_curr_gt_counts = count_num_each_class(pc_curr);
  g_est_gt_counts = count_num_each_class(ground_estimated);

  // save output
  for (auto const & class_id : ground_classes) {
    sc_output2 << g_est_gt_counts.find(class_id)->second << ","
               << pc_curr_gt_counts.find(class_id)->second << ",";
  }
  sc_output2 << accuracy << endl;

  sc_output2.close();
}

void pc2pcdfile(
  const pcl::PointCloud<PointXYZILID> & TP, const pcl::PointCloud<PointXYZILID> & FP,
  const pcl::PointCloud<PointXYZILID> & FN, const pcl::PointCloud<PointXYZILID> & TN,
  std::string pcd_filename)
{
  pcl::PointCloud<pcl::PointXYZI> pc_out;

  for (auto const pt : TP.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = TRUEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt : FP.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = FALSEPOSITIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt : FN.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = FALSENEGATIVE;
    pc_out.points.push_back(pt_est);
  }
  for (auto const pt : TN.points) {
    pcl::PointXYZI pt_est;
    pt_est.x = pt.x;
    pt_est.y = pt.y;
    pt_est.z = pt.z;
    pt_est.intensity = TRUENEGATIVE;
    pc_out.points.push_back(pt_est);
  }
  pc_out.width = pc_out.points.size();
  pc_out.height = 1;
  pcl::io::savePCDFileASCII(pcd_filename, pc_out);
}

#endif
