#include "map_loader.h"

void MapLoader::init_tf_params()
{

    // YAML::Node config = YAML::Load("/home/char/ndt_localizer_new/ndt_localizer_config.yaml");
    //  tf_roll_ = config["roll"].as<float>();
    //  tf_x_ = config["x"].as<float>();
    //  tf_y_ = config["y"].as<float>();
    //  tf_yaw_ = config["yaw"].as<float>();
    //  tf_z_ = config["z"].as<float>();
    tf_pitch_ = 0.0;
    tf_roll_ = 0.0;
    tf_x_ = 0.0;
    tf_y_ = 0.0;
    tf_z_ = 0.0;
    tf_yaw_ = 0.0;
    std::cout << "x" << tf_x_ << "y: " << tf_y_ << "z: " << tf_z_ << "roll: "
              << tf_roll_ << " pitch: " << tf_pitch_ << "yaw: " << tf_yaw_;
}

pcl::PointCloud<pcl::PointXYZ> MapLoader::CreatePcd()
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_list;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const std::string &path : file_list_)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.c_str(), *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file.");
        }
        else
        {
            cloud_list.push_back(cloud);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr concatenated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &raw_cloud : cloud_list)
    {
        *concatenated_cloud += *raw_cloud;
    }

    return *concatenated_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapLoader::SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr)
{

    // YAML::Node config = YAML::Load("/home/char/ndt_localizer_new/ndt_localizer_config.yaml");
    // std::string pcd_save_path = config["pcd_save_path"].as<std::string>();
    std::string pcd_save_path = "./transformed_map/transformed_map.pcd";
    pcl::io::savePCDFile(pcd_save_path, *map_pc_ptr);
    return map_pc_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MapLoader::TransformMap(pcl::PointCloud<pcl::PointXYZ> &in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc(new pcl::PointCloud<pcl::PointXYZ>(in));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);
    // tl: translation
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX()); // rot: rotation
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

    pcl::transformPointCloud(*in_pc, *transformed_pc_ptr, tf_m2w);
    SaveMap(transformed_pc_ptr);
    in_pc.reset();

    return transformed_pc_ptr;
}

MapLoader::MapLoader()
{
    std::string pcd_file_path;

    // YAML::Node config = YAML::Load("/home/char/ndt_localizer_new/ndt_localizer_config.yaml");
    //  pcd_file_path = config["pcd_path"].as<std::string>();
    pcd_file_path = "./map_pcd/lidar2_pcap.pcd";

    init_tf_params(/*nh*/);

    file_list_.push_back(pcd_file_path);
}
