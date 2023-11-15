#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"
#include <gtest/gtest.h>


TEST(BehaviorPathPlanningFootprint, Polygon2dTranslation)
{
    
    drivable_area_expansion::Polygon2d polygon;

    polygon.outer() = {{0.0, 0.0},{1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}, {0.0, 0.0}};
    drivable_area_expansion::Polygon2d translated_polygon= drivable_area_expansion::translate_polygon(polygon, 0.0, 0.0);
    EXPECT_EQ(translated_polygon.outer()[0].x(), 0.0);
    EXPECT_EQ(translated_polygon.outer()[0].y(), 0.0);
    // printf("translated once: %f\n", translated_polygon.outer()[3].x());

    translated_polygon = drivable_area_expansion::translate_polygon(polygon, 1.0, 2.0);
    // printf("translated 2nd: %f\n", translated_polygon.outer()[0].x());
    EXPECT_EQ(translated_polygon.outer()[0].x(), 1.0);
    EXPECT_EQ(translated_polygon.outer()[0].y(), 2.0);

    EXPECT_EQ(translated_polygon.outer()[1].x(), 2.0);
    EXPECT_EQ(translated_polygon.outer()[1].y(), 5.0);
    
    EXPECT_EQ(translated_polygon.outer()[2].x(), 4.0);
    EXPECT_EQ(translated_polygon.outer()[2].y(), 5.0);
    
    EXPECT_EQ(translated_polygon.outer()[3].x(), 4.0);
    EXPECT_EQ(translated_polygon.outer()[3].y(), 3.0);
    
    EXPECT_EQ(translated_polygon.outer()[4].x(), 1.0);
    EXPECT_EQ(translated_polygon.outer()[4].y(), 2.0);
    
    //changing the polygon
    polygon.outer() = {{2.0, 4.0},{2.0, 7.0}, {5.0, 7.0}, {3.0, 4.0}, {2.0, 4.0}};
    translated_polygon = drivable_area_expansion::translate_polygon(polygon, 3.0, 3.0);

    EXPECT_EQ(translated_polygon.outer()[0].x(), polygon.outer()[0].x()+3.0);
    EXPECT_EQ(translated_polygon.outer()[0].y(), polygon.outer()[0].y()+3.0);

    EXPECT_EQ(translated_polygon.outer()[1].x(), polygon.outer()[1].x()+3.0);
    EXPECT_EQ(translated_polygon.outer()[1].y(), polygon.outer()[1].y()+3.0);
    
    EXPECT_EQ(translated_polygon.outer()[2].x(), polygon.outer()[2].x()+3.0);
    EXPECT_EQ(translated_polygon.outer()[2].y(), polygon.outer()[2].y()+3.0);
    
    EXPECT_EQ(translated_polygon.outer()[3].x(), polygon.outer()[3].x()+3.0);
    EXPECT_EQ(translated_polygon.outer()[3].y(), polygon.outer()[3].y()+3.0);
    
    EXPECT_EQ(translated_polygon.outer()[4].x(), polygon.outer()[4].x()+3.0);
    EXPECT_EQ(translated_polygon.outer()[4].y(), polygon.outer()[4].y()+3.0);
    
    
    } 

    TEST(BehaviorPathPlanningPolygon2d, Polygon2dCreateFootprint)
    {
        drivable_area_expansion::Polygon2d base_footprint;
        geometry_msgs::msg::Pose pose;

        base_footprint.outer()= {{0.0, 0.0},{1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}, {0.0, 0.0}};

        //considering it in 2D
        pose.position.x = 1.0;
        pose.position.y = 1.0;
        pose.position.z = 0.0;
        pose.orientation.w = 0.0;
        
        drivable_area_expansion::Polygon2d footprint = drivable_area_expansion::create_footprint(pose, base_footprint);
        EXPECT_EQ(footprint.outer()[0].x(), base_footprint.outer()[0].x()+1.0);
        EXPECT_EQ(footprint.outer()[0].y(), base_footprint.outer()[0].y()+1.0);

        EXPECT_EQ(footprint.outer()[1].x(), base_footprint.outer()[1].x()+1.0);
        EXPECT_EQ(footprint.outer()[1].y(), base_footprint.outer()[1].y()+1.0);

        EXPECT_EQ(footprint.outer()[2].x(), base_footprint.outer()[2].x()+1.0);
        EXPECT_EQ(footprint.outer()[2].y(), base_footprint.outer()[2].y()+1.0);

        EXPECT_EQ(footprint.outer()[3].x(), base_footprint.outer()[3].x()+1.0);
        EXPECT_EQ(footprint.outer()[3].y(), base_footprint.outer()[3].y()+1.0);

        EXPECT_EQ(footprint.outer()[4].x(), base_footprint.outer()[4].x()+1.0);
        EXPECT_EQ(footprint.outer()[4].y(), base_footprint.outer()[4].y()+1.0);


        //new position
        pose.position.x = 1.0;
        pose.position.y = 2.0;
        pose.position.z = 1.0;
        pose.orientation.w = 30.0;
        
        footprint = drivable_area_expansion::create_footprint(pose, base_footprint);
        EXPECT_EQ(footprint.outer()[0].x(), base_footprint.outer()[0].x()+1.0);
        EXPECT_EQ(footprint.outer()[0].y(), base_footprint.outer()[0].y()+2.0);

        EXPECT_EQ(footprint.outer()[1].x(), base_footprint.outer()[1].x()+1.0);
        EXPECT_EQ(footprint.outer()[1].y(), base_footprint.outer()[1].y()+2.0);

        EXPECT_EQ(footprint.outer()[2].x(), base_footprint.outer()[2].x()+1.0);
        EXPECT_EQ(footprint.outer()[2].y(), base_footprint.outer()[2].y()+2.0);

        EXPECT_EQ(footprint.outer()[3].x(), base_footprint.outer()[3].x()+1.0);
        EXPECT_EQ(footprint.outer()[3].y(), base_footprint.outer()[3].y()+2.0);

        EXPECT_EQ(footprint.outer()[4].x(), base_footprint.outer()[4].x()+1.0);
        EXPECT_EQ(footprint.outer()[4].y(), base_footprint.outer()[4].y()+2.0);


        //changing base_footprint
        base_footprint.outer() = {{2.0, 4.0},{2.0, 7.0}, {5.0, 7.0}, {3.0, 4.0}, {2.0, 4.0}};
        
        //new pose
        pose.position.x = 3.0;
        pose.position.y = 3.0;
        pose.position.z = 0.0;
        pose.orientation.w = 0.0;

        footprint = drivable_area_expansion::create_footprint(pose, base_footprint);
        EXPECT_EQ(footprint.outer()[0].x(), base_footprint.outer()[0].x()+3.0);
        EXPECT_EQ(footprint.outer()[0].y(), base_footprint.outer()[0].y()+3.0);

        EXPECT_EQ(footprint.outer()[1].x(), base_footprint.outer()[1].x()+3.0);
        EXPECT_EQ(footprint.outer()[1].y(), base_footprint.outer()[1].y()+3.0);
        
        EXPECT_EQ(footprint.outer()[2].x(), base_footprint.outer()[2].x()+3.0);
        EXPECT_EQ(footprint.outer()[2].y(), base_footprint.outer()[2].y()+3.0);
        
        EXPECT_EQ(footprint.outer()[3].x(), base_footprint.outer()[3].x()+3.0);
        EXPECT_EQ(footprint.outer()[3].y(), base_footprint.outer()[3].y()+3.0);
        
        EXPECT_EQ(footprint.outer()[4].x(), base_footprint.outer()[4].x()+3.0);
        EXPECT_EQ(footprint.outer()[4].y(), base_footprint.outer()[4].y()+3.0);

    }

    