// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"

#include <rclcpp/qos.hpp>

#include "std_msgs/msg/string.hpp"

#include <memory>
//#include <pcl_ros/transforms.hpp>
// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
//#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
using std::placeholders::_1;
// using SyncPolicy = message_filters::sync_policies::ExactTime<PointCloud2>;
// using Sync = message_filters::Synchronizer<SyncPolicy>;

class MinimalSubscriber : public rclcpp::Node
{
public:
  struct PointXYZIR
  {
    pcl::PointXYZ point;
    float intensity;
    unsigned short ring;  // ring number if available
  };
  // typedef PointXYZIRPtr *PointXYZIR;
  typedef std::vector<PointXYZIR> PointCloudXYZIR;

  // int call_count;

  MinimalSubscriber() : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/pointcloud_raw", rclcpp::QoS(1).best_effort(),
      std::bind(&MinimalSubscriber::topic_callback, this, _1));
    //  obstacle_pointcloud_sub_.subscribe(
    //    this, "/sensng/lidar/top/pointcloud_raw",
    //    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
    gnuplot_fp = popen("/usr/bin/gnuplot", "w");
    fprintf(gnuplot_fp, "set term x11\n");
    //    fprintf(gnuplot_fp, "set xrange [-6:6]\n");
    fprintf(gnuplot_fp, "set yrange [-10:110]\n");

    pgm_fp = fopen("intensity.pgm", "w");
    fprintf(pgm_fp, "P2\n# feep.pgm\n%d\n%d\n100\n", 1000, 100);

    //    call_count = 0;
    // hoe
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    printf("hoge\n");
    static int call_count = 0;
    call_count++;
    //rclcpp::Time stamp_ros = msg->header.stamp;
    //double stamp = stamp_ros.seconds();
    //   RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->fields.size());
    for (size_t i = 0; i < msg->fields.size(); i++) {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'",);
      printf(
        "%s %d %d\n", msg->fields[i].name.c_str(), msg->fields[i].datatype, msg->fields[i].offset);
    }
    /*
      //msgからダイレクトにアクセスすれば多分ココらへんいらない
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

      pcl::PointCloud<pcl::PointXYZI>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZI> );
      //pcl::fromPCLPointCloud2( mesh.cloud, *vertices );
      //pcl::PointCloud<PointXYZI> cloud;
      //pcl::fromROSMsg (*msg, cloud);
      pcl_conversions::toPCL(*msg, *cloud);
      pcl::fromPCLPointCloud2( *cloud, *vertices );

      // access each vertex
      // テスト用・消す
      for( int idx = 0; idx < vertices->size(); idx++ )
      {
        pcl::PointXYZI v = vertices->points[ idx ];

        float x = v._PointXYZI::data[ 0 ];
        float y = v._PointXYZI::data[ 1 ];
        float z = v._PointXYZI::data[ 2 ];
        float intensity = v._PointXYZI::data[ 3 ];//出てなかった
        double angle=atan2(x*x+y*y,z*z);
        //printf("%f %f %f %f %lf\n",x,y,z,intensity,angle);
      }
*/
    PointCloudXYZIR points[128];

    // for(int i=0;cloud->points.size();i++){

    for (size_t i = 0; i < msg->data.size() / msg->point_step; i++) {
      // PointXYZIR  point;
      // pcl::PointCloud<PointXYZIR> points = static_cast<pcl::PointCloud<PointCloudXYZIR> >(msg);
      // Direct access
      PointXYZIR point;
      point.point.x = *(float *)&msg->data[i * msg->point_step + msg->fields[0].offset];
      point.point.y = *(float *)&msg->data[i * msg->point_step + msg->fields[1].offset];
      point.point.z = *(float *)&msg->data[i * msg->point_step + msg->fields[2].offset];
      point.intensity = *(float *)&msg->data[i * msg->point_step + msg->fields[3].offset];
      point.ring = *(unsigned short *)&msg->data[i * msg->point_step + msg->fields[4].offset];

      points[point.ring].push_back(point);
      // unsigned short ring=*ring_;
      // point=static_cast<PointXYZIR*>(&msg->data[i*sizeof(PointXYZIR)]);
      // printf("%d \n",msg->data[i*sizeof(PointXYZIR)]);
    }

    /*
      fprintf(gnuplot_fp,"plot ");
      for(int j=0;j<128;j++){
        FILE *fp;
        char name[100];
        sprintf(name,"intensity%03d",j);
        fp=fopen(name,"w");
        //fpritf(pgm_fp,"P2\n# feep.pgm\n%d\n%d\n100\n",1000,100);
        for(int i=0; i<points[j].size();i++){
          PointXYZIR  point;
          point=points[j][i];
          //if(point.point.x >0)
          fprintf(fp,"%f %f %f %f
      %d\n",point.point.x,point.point.y,point.point.z,point.intensity,point.ring);
        }
        fclose(fp);
        if(j>8 &&j<10)fprintf(gnuplot_fp,"'%s' u 1:4 w l,",name);
      }
      fprintf(gnuplot_fp,"'intensity' u 1:4 w l\n");
      fflush(gnuplot_fp);
*/
    // pgm_output
    for (size_t i = 0; i < 1000; i++) {
      PointXYZIR point;
      point = points[6][i];
      // if(point.point.x >0)
      if (i < points[6].size())
        fprintf(pgm_fp, "%d ", (int)point.intensity);
      else
        fprintf(pgm_fp, "0 ");
    }
    fprintf(pgm_fp, "\n");
    fflush(pgm_fp);

    int vote[400];
    for (int i = 0; i < 400; i++) vote[i] = 0;
    FILE * ring_intensity_fp[128];
    FILE * ring_filter_x_fp[128];
    FILE * ring_intensity_x_fp[128];

    for (int target_ring = 30; target_ring < 80; target_ring++) {
      char name[100];
      sprintf(name, "ring_intensity_%03d.csv", target_ring);
      ring_intensity_fp[target_ring] = fopen(name, "w");
      sprintf(name, "ring_intensity_x_%03d.csv", target_ring);
      ring_intensity_x_fp[target_ring] = fopen(name, "w");
      sprintf(name, "ring_filter_x_%03d.csv", target_ring);
      ring_filter_x_fp[target_ring] = fopen(name, "w");
    }
    for (int target_ring = 30; target_ring < 80; target_ring++) {
      // initialize intensity line image
      double intensity_line_image[400];
      for (int i = 0; i < 400; i++) intensity_line_image[i] = -1;
      // set intensity line image
      // fprintf(ring_intensity_fp[target_ring],"%lf,",stamp);
      for (size_t i = 0; i < points[target_ring].size(); i++) {
        PointXYZIR point;
        point = points[target_ring][i];
        if (point.point.x > 0) {
          int ix;
          ix = point.point.y / 0.05 + 200;
          if (ix >= 0 && ix < 400) {
            intensity_line_image[ix] = point.intensity;
          }
        }
        fprintf(ring_intensity_fp[target_ring], "%.2lf\n", point.intensity);
      }
      fprintf(ring_intensity_fp[target_ring], "\n");

      // filter
      // fprintf(ring_intensity_x_fp[target_ring],"%lf,",stamp);
      // fprintf(ring_filter_x_fp[target_ring],"%lf,",stamp);
      for (int i = 10; i < 400 - 10; i++) {
        double pos = 0;
        double neg = 0;
        for (int j = -2; j <= 2; j++) pos += intensity_line_image[i + j];
        for (int j = -5; j <= -4; j++) neg += intensity_line_image[i + j];
        for (int j = 4; j <= 5; j++) neg += intensity_line_image[i + j];
        double feature = pos / 5 - neg / 4;
        //int mark;
        if (feature > 40) {
        //  mark = 100;
          vote[i]++;
        } else {
        //  mark = 0;
        }
        fprintf(ring_filter_x_fp[target_ring], "%.2f \n", feature);
        fprintf(ring_intensity_x_fp[target_ring], "%.2f \n", intensity_line_image[i]);
      }
      fprintf(ring_filter_x_fp[target_ring], "\n");
      fprintf(ring_intensity_x_fp[target_ring], "\n");
      // fprintf(fp, "\n");
    }

    FILE * fp;
    fp = fopen("filter", "w");
    //    fprintf(fp, "%lf ", stamp);
    for (int i = 10; i < 400 - 10; i++) {
      fprintf(fp, "%d %d\n", i, vote[i]);
    }
    fprintf(fp, "\n");
    fclose(fp);

    for (int target_ring = 30; target_ring < 80; target_ring++) {
      fclose(ring_intensity_fp[target_ring]);
      fclose(ring_intensity_x_fp[target_ring]);
      fclose(ring_filter_x_fp[target_ring]);
    }

    char name[100];
    sprintf(name, "results%03d.png", call_count);
    fprintf(
      gnuplot_fp,
      "plot 'filter' u 2 w l,'ring_intensity_x_050.csv' u 1 w l\n");  //,'ring_intensity_x_050.csv'
                                                                      // u 1 w
                                                                      // l,'ring_intensity_x_079.csv'
                                                                      // u 1 w l\n");
    fprintf(gnuplot_fp, "set term png\n set output '%s'\n replot\n set term x11\n", name);
    fflush(gnuplot_fp);
  }

  FILE * gnuplot_fp;
  FILE * pgm_fp;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  //  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> obstacle_pointcloud_sub_;
  // std::shared_ptr<Sync> sync_ptr_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
