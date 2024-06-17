//
// Created by shapelim on 6/23/21.
//

#ifndef PATCHWORK_PCD_LOADER_HPP
#define PATCHWORK_PCD_LOADER_HPP

template <typename T>
class PcdLoader {
public:
    PcdLoader(const std::string &pcd_path) : pcd_path_(pcd_path){
        for (num_frames_ = 0;; num_frames_++) {
            std::string filename = (boost::format("%s/%06d.pcd") % pcd_path % num_frames_).str();
            if (!boost::filesystem::exists(filename)) {
                break;
            }
        }

        if (num_frames_ == 0) {
            std::cerr << "error: no files in " << pcd_path << std::endl;
        }
    }

    ~PcdLoader() {}

    size_t size() const { return num_frames_; }

    pcl::PointCloud<T>::ConstPtr cloud(size_t i) const {
        std::string filename = (boost::format("%s/%06d.bin") % pcd_path_ % i).str();
        FILE *file = fopen(filename.c_str(), "rb");
        if (!file) {
            std::cerr << "error: failed to load " << filename << std::endl;
            return nullptr;
        }

        std::vector<float> buffer(1000000);
        size_t num_points = fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
        fclose(file);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->resize(num_points);

        for (int i = 0; i < num_points; i++) {
            auto &pt = cloud->at(i);
            pt.x = buffer[i * 4];
            pt.y = buffer[i * 4 + 1];
            pt.z = buffer[i * 4 + 2];
            // pt.intensity = buffer[i * 4 + 3];
        }

        return cloud;
    }

private:
    int num_frames_;
    std::string pcd_path_;
};
#endif //PATCHWORK_PCD_LOADER_HPP
