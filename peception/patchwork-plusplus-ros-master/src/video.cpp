#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "patchworkpp/patchworkpp.hpp"

#include "tools/kitti_loader.hpp"

using PointType = PointXYZILID;
using namespace std;

void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    // Terminate program
    exit(signum);
}

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg) {
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg, cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}


int main(int argc, char**argv) {

    ros::Publisher CloudPublisher;
    ros::Publisher GroundPublisher;
    ros::Publisher NongroundPublisher;
    // ros::Publisher TPPublisher;
    // ros::Publisher FPPublisher;
    // ros::Publisher FNPublisher;

    boost::shared_ptr<PatchWorkpp<PointType> > PatchworkppGroundSeg;
    std::string output_csvpath;

    std::string acc_filename;
    std::string pcd_savepath;
    std::string data_path;
    string      algorithm;
    string      seq;

    bool        save_flag;

    int         init_idx;
    bool        save_csv_file;
    bool        stop_per_each_frame;

    ros::init(argc, argv, "Video");

    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchworkpp");
    nh.param<string>("/sequence", seq, "00");
    nh.param<string>("/data_path", data_path, "/");
    nh.param<string>("/output_csvpath", output_csvpath, "/data/");

    nh.param<int>("/init_idx", init_idx, 0);
    nh.param<bool>("/save_csv_file", save_csv_file, false);
    nh.param<bool>("/stop_per_each_frame", stop_per_each_frame, false);

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    string HOME = std::getenv("HOME");
    data_path = HOME + data_path;
    output_csvpath = HOME + output_csvpath;

    CloudPublisher      = nh.advertise<sensor_msgs::PointCloud2>("/demo/cloud", 100, true);
    GroundPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/demo/ground", 100, true);
    NongroundPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/demo/nonground", 100, true);
    // TPPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TP", 100, true);
    // FPPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FP", 100, true);
    // FNPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FN", 100, true);

    signal(SIGINT, signal_callback_handler);

    PatchworkppGroundSeg.reset(new PatchWorkpp<PointXYZILID>(&nh));
    data_path = data_path + "/" + seq;
    KittiLoader loader(data_path);

    int      N = loader.size();
    for (int n = init_idx; n < N; ++n) {     

        cout << n << "th node come" << endl;
        pcl::PointCloud<PointType> pc_curr;
        loader.get_cloud(n, pc_curr);
        pcl::PointCloud<PointType> pc_ground;
        pcl::PointCloud<PointType> pc_non_ground;

        static double time_taken;
        cout << "Operating patchwork++..." << endl;

        PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

        // Estimation
        // double precision, recall, precision_wo_veg, recall_wo_veg;
        // calculate_precision_recall(pc_curr, pc_ground, precision, recall);
        // calculate_precision_recall_without_vegetation(pc_curr, pc_ground, precision_wo_veg, recall_wo_veg);

        cout << "\033[1;32m" << n << "th, " << " takes : " << time_taken << " | " 
             << pc_curr.size() << " -> " << pc_ground.size() << "\033[0m" << endl;

        // cout << "\033[1;32m P: " << precision_wo_veg << " | R: " << recall_wo_veg << "\033[0m" << endl;

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save precision/recall in a text file, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        // if (save_csv_file)
        // {
        //     ofstream sc_output(output_csvpath + seq + ".csv", ios::app);
        //     sc_output << n << "," << time_taken << "," << precision << "," << recall << "," << precision_wo_veg << "," << recall_wo_veg;
        //     sc_output << std::endl;
        //     sc_output.close();
        // }
        
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        // Publish msg
        // pcl::PointCloud<PointType> TP;
        // pcl::PointCloud<PointType> FP;
        // pcl::PointCloud<PointType> FN;
        // pcl::PointCloud<PointType> TN;
        // discern_ground_without_vegetation(pc_ground, TP, FP);
        // discern_ground_without_vegetation(pc_non_ground, FN, TN);
        
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save the output of pcd, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        if (save_flag) {
//            std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
//            double             accuracy;
//            save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);
//
//            std::string count_str        = std::to_string(n);
//            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
//            std::string pcd_filename     = pcd_savepath + "/" + count_str_padded + ".pcd";
//            pc2pcdfile(TP, FP, FN, TN, pcd_filename);
//        }
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        CloudPublisher.publish(cloud2msg(pc_curr));
        GroundPublisher.publish(cloud2msg(pc_ground));
        NongroundPublisher.publish(cloud2msg(pc_non_ground));
        // TPPublisher.publish(cloud2msg(TP));
        // FPPublisher.publish(cloud2msg(FP));
        // FNPublisher.publish(cloud2msg(FN));
        ros::spinOnce();

        if (stop_per_each_frame) cin.ignore();
    }

    return 0;
}
