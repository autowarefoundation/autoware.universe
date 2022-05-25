#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <costmap_generator/points_to_costmap.hpp>

using pointcloud = pcl::PointCloud<pcl::PointXYZ>;
class PointsToCostmapTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }

    ~PointsToCostmapTest() override { rclcpp::shutdown(); }

    grid_map::GridMap constrcut_gridmap();

public:
    double grid_resolution_ = 1;
    double grid_length_x_ = 20;
    double grid_length_y_ = 20;
    double grid_position_x_ = 0;
    double grid_position_y_ = 0;
};


grid_map::GridMap PointsToCostmapTest::constrcut_gridmap()
{
    grid_map::GridMap gm;

    //set gridmap size
    gm.setFrameId("map");
    gm.setGeometry(
    grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
    grid_map::Position(grid_position_x_, grid_position_y_));

    //set initial value
    gm.add("points",0);

    //set car postion in map frame to center of grid
    grid_map::Position p;
    p.x() = 0;
    p.y() = 0;
    gm.setPosition(p);

    return gm;
}


// gridy
// |  mapx-------
// |            |
// |            |
// |            | 
// |            mapy
// |__________________gridx
TEST_F(PointsToCostmapTest, TestmakeCostmapFromPoints_validPoints)
{
    //construct pointcloud in map frame
    pointcloud in_sensor_points;

    in_sensor_points.width    = 3;
    in_sensor_points.height   = 1;
    in_sensor_points.is_dense = false;
    in_sensor_points.resize (in_sensor_points.width * in_sensor_points.height);

    in_sensor_points.points[0].x = 0.7;
    in_sensor_points.points[0].y = 1;
    in_sensor_points.points[0].z = 1;

    in_sensor_points.points[1].x = 1.1;
    in_sensor_points.points[1].y = 1;
    in_sensor_points.points[1].z = 4;

    in_sensor_points.points[2].x = 1.4;
    in_sensor_points.points[2].y = 2;
    in_sensor_points.points[2].z = 2.7;

    grid_map::GridMap gridmap = constrcut_gridmap();

    PointsToCostmap point2costmap;
    double maximum_height_thres = 5;
    double minimum_lidar_height_thres = 0;
    double grid_min_value = 0;
    double grid_max_value = 1;
    
    std::string gridmap_layer_name = "points";
    grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(maximum_height_thres,
                                        minimum_lidar_height_thres,
                                        grid_min_value, 
                                        grid_max_value,
                                        gridmap,
                                        gridmap_layer_name, 
                                        in_sensor_points);
 
    int nonempty_gridcell_num = 0;
    for(int i = 0; i < costmap_data.rows();i++)
    {
        for(int j = 0; j < costmap_data.cols();j++)
        {
            if(costmap_data(i,j) == grid_max_value)
            {
                std::cout << "i:"<< i <<",j:"<<j<< std::endl;
                nonempty_gridcell_num += 1;
            }
        }
    }

    EXPECT_EQ(nonempty_gridcell_num,3);
}

TEST_F(PointsToCostmapTest, TestmakeCostmapFromPoints_invalidPoints_biggerThanMaximumHeightThres)
{
    //construct pointcloud in map frame
    pointcloud in_sensor_points;
    in_sensor_points.width    = 1;
    in_sensor_points.height   = 1;
    in_sensor_points.is_dense = false;
    in_sensor_points.resize (in_sensor_points.width * in_sensor_points.height);

    in_sensor_points.points[0].x = 0.7;
    in_sensor_points.points[0].y = 1;
    in_sensor_points.points[0].z = 1; //out of [maximum_height_thres,minimum_lidar_height_thres]

    grid_map::GridMap gridmap = constrcut_gridmap();

    PointsToCostmap point2costmap;
    double maximum_height_thres = 0.99;
    double minimum_lidar_height_thres = 0;
    double grid_min_value = 0;
    double grid_max_value = 1;
    
    std::string gridmap_layer_name = "points";
    grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(maximum_height_thres,
                                        minimum_lidar_height_thres,
                                        grid_min_value, 
                                        grid_max_value,
                                        gridmap,
                                        gridmap_layer_name, 
                                        in_sensor_points);
 
    int nonempty_gridcell_num = 0;
    for(int i = 0; i < costmap_data.rows();i++)
    {
        for(int j = 0; j < costmap_data.cols();j++)
        {
            if(costmap_data(i,j) == grid_max_value)
            {
                nonempty_gridcell_num += 1;
            }
        }
    }

    EXPECT_EQ(nonempty_gridcell_num,0);
}

TEST_F(PointsToCostmapTest, TestmakeCostmapFromPoints_invalidPoints_lessThanMinimumHeightThres)
{
    //construct pointcloud in map frame
    pointcloud in_sensor_points;
    in_sensor_points.width    = 1;
    in_sensor_points.height   = 1;
    in_sensor_points.is_dense = false;
    in_sensor_points.resize (in_sensor_points.width * in_sensor_points.height);

    in_sensor_points.points[0].x = 0.7;
    in_sensor_points.points[0].y = 1;
    in_sensor_points.points[0].z = -0.1; //out of [maximum_height_thres,minimum_lidar_height_thres]

    grid_map::GridMap gridmap = constrcut_gridmap();

    PointsToCostmap point2costmap;
    double maximum_height_thres = 0.99;
    double minimum_lidar_height_thres = 0;
    double grid_min_value = 0;
    double grid_max_value = 1;
    
    std::string gridmap_layer_name = "points";
    grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(maximum_height_thres,
                                        minimum_lidar_height_thres,
                                        grid_min_value, 
                                        grid_max_value,
                                        gridmap,
                                        gridmap_layer_name, 
                                        in_sensor_points);
 
    int nonempty_gridcell_num = 0;
    for(int i = 0; i < costmap_data.rows();i++)
    {
        for(int j = 0; j < costmap_data.cols();j++)
        {
            if(costmap_data(i,j) == grid_max_value)
            {
                nonempty_gridcell_num += 1;
            }
        }
    }

    EXPECT_EQ(nonempty_gridcell_num,0);
}

TEST_F(PointsToCostmapTest, TestmakeCostmapFromPoints_invalidPoints_outOfGrid)
{
    //construct pointcloud in map frame
    pointcloud in_sensor_points;
    in_sensor_points.width    = 1;
    in_sensor_points.height   = 1;
    in_sensor_points.is_dense = false;
    in_sensor_points.resize (in_sensor_points.width * in_sensor_points.height);

    in_sensor_points.points[0].x = 1 + grid_length_x_ / 2.0;
    std::cout<<"in_sensor_points.points[0].x:"<<in_sensor_points.points[0].x<<std::endl;
    in_sensor_points.points[0].y = 1 + grid_length_y_ / 2.0;
    in_sensor_points.points[0].z = 0.5; 

    grid_map::GridMap gridmap = constrcut_gridmap();

    PointsToCostmap point2costmap;
    double maximum_height_thres = 0.99;
    double minimum_lidar_height_thres = 0;
    double grid_min_value = 0;
    double grid_max_value = 1;
    
    std::string gridmap_layer_name = "points";
    grid_map::Matrix costmap_data = point2costmap.makeCostmapFromPoints(maximum_height_thres,
                                        minimum_lidar_height_thres,
                                        grid_min_value, 
                                        grid_max_value,
                                        gridmap,
                                        gridmap_layer_name, 
                                        in_sensor_points);
 
    int nonempty_gridcell_num = 0;
    for(int i = 0; i < costmap_data.rows();i++)
    {
        for(int j = 0; j < costmap_data.cols();j++)
        {
            if(costmap_data(i,j) == grid_max_value)
            {
                nonempty_gridcell_num += 1;
            }
        }
    }

    EXPECT_EQ(nonempty_gridcell_num,0);
}