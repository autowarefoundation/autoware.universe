#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class PolarGridBase
{
  public:
    /** @brief Init of PolarGridBase.
        @param[in] polar_angle,polar_range_size the polar grid size param
        @param[in] max_range: points in max_range will be process
        @param[in] max_z_height: points below max_z_height will be process
       */
    PolarGridBase(const float &polar_angle = 1., const float &polar_range_size = 0.25,
                  const float &max_range = 50., const float &max_z_height = 0.7);
    ~PolarGridBase() {}

    /** @brief cluster the points.
        @param[in] in_cloud_ptr: the no ground points
        @param[out] clusters: the clusted points
       */
    bool polarGridCluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
                          std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters);

    /** @brief generate the points.
        @param[in] in_cloud_ptr: the no ground points
        @param[out] clusters: the clusted points
       */
    bool genPolarGrowthGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);

    /**@brief Generate polar grid
       @param[in] in_cloud_ptr input cloud data
       @ingroup common/geo_tool
       */
    bool genPolarGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr);

    void clusterstoColor(std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters,
                            pcl::PointCloud<pcl::PointXYZRGB>& color_cloud);



  protected:
    float max_range_, max_z_height_, polar_angle_, polar_range_size_;
    int MSegs_, NBins_;
    float polar_min_rad_;
    int dilation_size_;
    bool polar_has_input_;
    bool is_sort_grids_;
    float GRID_HIGHTTHRESHOLD = 0.03;

    pcl::PointCloud<pcl::PointXYZI>::Ptr polar_leave_cloud_ptr_;
    std::vector<std::vector<std::vector<pcl::PointXYZI> > > polar_save_grid_pts_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr air_cloud_ptr_;

  private:
    float computeXYDis(const pcl::PointXYZI &pt);
    float computBeta(const pcl::PointXYZI &pt);
    void polarGroundCluster(cv::Mat &label_mat);

    void polarUpGrowthCluster(const cv::Mat &ground_mat, 
                                std::vector<pcl::PointCloud<pcl::PointXYZI> > &clusters);
    int max_label_;
};
