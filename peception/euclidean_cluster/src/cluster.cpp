#include "cluster.h"
#include <stack>

PolarGridBase::PolarGridBase(const float &polar_angle, const float &polar_range_size,
                                  const float &max_range, const float &max_z_height)
{
    polar_angle_ = polar_angle;
    polar_range_size_ = polar_range_size;
    max_range_ = max_range;
    max_z_height_ = max_z_height;

    polar_min_rad_ = (polar_angle_ / 180.) * M_PI;
    MSegs_ = (2 * M_PI + 0.001) / polar_min_rad_;
    NBins_ = (max_range_ + 0.001) / polar_range_size_;

    polar_leave_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    polar_save_grid_pts_.resize(NBins_);
    for (int j = 0; j < NBins_; ++j)
    {
        polar_save_grid_pts_[j].resize(MSegs_);
    }
    air_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    dilation_size_ = 1;
    polar_has_input_ = false;

}

bool PolarGridBase::polarGridCluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
                                     std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters)
{
   
    if (!genPolarGrowthGrid(in_cloud_ptr))
    {
        std::cout << "genPolarGrowthGrid!" << std::endl;
        return false;
    }
 
    cv::Mat ground_label_mat;
    polarGroundCluster(ground_label_mat);
  
    polarUpGrowthCluster(ground_label_mat, clusters);
 
    return true;
}

bool polarComputeZ(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
{
    return pt1.z < pt2.z;
}

bool PolarGridBase::genPolarGrowthGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr)
{
    if (in_cloud_ptr->empty())
    {
        std::cout << "the input cloud is empty!!" << std::endl;
        return false;
    }
 
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            polar_save_grid_pts_[j][i].clear();
        }
    }
    
    polar_leave_cloud_ptr_->clear();
    air_cloud_ptr_->clear();
    pcl::PointXYZI tmp_pt;
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        tmp_pt = in_cloud_ptr->points[i];
        if (std::isnan(tmp_pt.x) || std::isnan(tmp_pt.y) || std::isnan(tmp_pt.z) || std::isnan(tmp_pt.intensity))
            continue;
        float tmp_range = computeXYDis(tmp_pt);
        if (tmp_range > max_range_)
        {
            polar_leave_cloud_ptr_->push_back(tmp_pt);
            continue;
        }
        if (tmp_pt.z > max_z_height_)
        {
            air_cloud_ptr_->push_back(tmp_pt);
            continue;
        }
        
        float tmp_beta = computBeta(tmp_pt);
        
        int x_index = tmp_beta / polar_min_rad_;
        int y_index = tmp_range / polar_range_size_;

        if(x_index==MSegs_)
            x_index--;
        if(y_index==NBins_)
            y_index--;
        polar_save_grid_pts_[y_index][x_index].push_back(tmp_pt);
    
    }

    polar_has_input_ = true;
    return true;
}

bool point_cmp(const pcl::PointXYZI &a, const pcl::PointXYZI &b) { return a.z < b.z; }

void PolarGridBase::polarGroundCluster(cv::Mat &label_mat)
{
    cv::Mat bin_mat;
    bin_mat = cv::Mat::zeros(NBins_, MSegs_, CV_8U);
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            if (polar_save_grid_pts_[j][i].empty())
                continue;
            // sort(polar_save_grid_pts_[j][i].begin(), polar_save_grid_pts_[j][i].end(), point_cmp);

            // std::vector<pcl::PointXYZI> ::iterator max = polar_save_grid_pts_[j][i].end() - 1;
            // std::vector<pcl::PointXYZI> ::iterator min = polar_save_grid_pts_[j][i].begin();

            // float Height_dif = max->z - min->z;
            // if (Height_dif < GRID_HIGHTTHRESHOLD)
            //     continue;

            bin_mat.at<uchar>(j, i) = 255;
        }
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * dilation_size_ + 1, 2 * dilation_size_ + 1),
                                                cv::Point(dilation_size_, dilation_size_));
    //cv::erode(bin_mat, bin_mat, element);
    // cv::dilate(bin_mat, bin_mat, element);

    label_mat.release();
    label_mat = cv::Mat::zeros(NBins_, MSegs_, CV_32S);

    int label = 0; // start by 1
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            if (bin_mat.at<uchar>(j, i) == 0)
                continue;
            if (label_mat.at<int>(j, i) != 0)
                continue;
            std::stack<cv::Point2i> neighborPixels;
            neighborPixels.push(cv::Point2i(i, j));
            label++;
            label_mat.at<int>(j, i) = label;
            while (!neighborPixels.empty())
            {
                cv::Point2i cur_index = neighborPixels.top();
                neighborPixels.pop();
                for (int jj = cur_index.y - 1; jj <= cur_index.y + 1; ++jj)
                {
                    if (jj >= NBins_ || jj < 0)
                        continue;
                    for (int ii = cur_index.x - 1; ii <= cur_index.x + 1; ++ii)
                    {
                        int ii_index = ii;
                        if (ii_index >= MSegs_)
                            ii_index -= MSegs_;
                        if (ii_index < 0)
                            ii_index += MSegs_;
                        if (jj == cur_index.y || ii_index == cur_index.x)
                        {
                            if (bin_mat.at<uchar>(jj, ii_index) == 0)
                                continue;
                            if (label_mat.at<int>(jj, ii_index) != 0)
                                continue;
                            label_mat.at<int>(jj, ii_index) = label;
                            neighborPixels.push(cv::Point2i(ii_index, jj));
                        }
                    }
                }
            }
        }
    }
    max_label_ = label;
    // cv::namedWindow("Range Image", cv::WINDOW_NORMAL);
    // cv::imshow("Range Image",bin_mat);
    // cv::waitKey(20);
}

void PolarGridBase::polarUpGrowthCluster(const cv::Mat &ground_mat, std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI> > tmp_clusters;
    tmp_clusters.clear();
    clusters.clear();
    tmp_clusters.resize(max_label_ + 1);
    for (int j = 0; j < NBins_; ++j)
    {
        for (int i = 0; i < MSegs_; ++i)
        {
            if (polar_save_grid_pts_[j][i].empty())
                continue;
            for (int k = 0; k < polar_save_grid_pts_[j][i].size(); ++k)
            {
                tmp_clusters[ground_mat.at<int>(j, i)].push_back(polar_save_grid_pts_[j][i][k]);
            }
        }
    }
    for (int i = 0; i < tmp_clusters.size(); ++i)
    {
        if (tmp_clusters[i].points.size() <11)  //每个聚类最少点数
            continue;

        clusters.push_back(tmp_clusters[i]);
    }
    std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(tmp_clusters);
}

void PolarGridBase::clusterstoColor(std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters,pcl::PointCloud<pcl::PointXYZRGB>& color_cloud)
{
    for (int i = 0; i < clusters.size(); ++i)
    {
        int red = rand() % 256;
        int green = rand() % 256;
        int blue = rand() % 256;
        for (int j = 0; j < clusters[i].points.size(); ++j)
        {
            pcl::PointXYZRGB tmp_point;
            tmp_point.x = clusters[i].points[j].x;
            tmp_point.y = clusters[i].points[j].y;
            tmp_point.z = clusters[i].points[j].z;
            tmp_point.r = red;
            tmp_point.g = green;
            tmp_point.b = blue;
            color_cloud.push_back(tmp_point);
        }
    }
}

float PolarGridBase::computBeta(const pcl::PointXYZI &pt)
{
    float beta = atan2(pt.y, pt.x);
    beta = beta < 0. ? beta += 2 * M_PI : beta;
    beta = beta > (2 * M_PI) ? beta -= 2 * M_PI : beta;
    return beta;
}

float PolarGridBase::computeXYDis(const pcl::PointXYZI &pt)
{
    return sqrtf(powf(pt.x, 2.) + powf(pt.y, 2.));
}
