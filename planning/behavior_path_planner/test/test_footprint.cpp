#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"
#include <gtest/gtest.h>



TEST(BehaviorPathPlanningFootprint, Polygon2dTranslation)
{
    
    drivable_area_expansion::Polygon2d polygon;

    polygon.outer() = {{0.0, 0.0},{1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}, {0.0, 0.0}};
    drivable_area_expansion::Polygon2d translated_polygon= drivable_area_expansion::translate_polygon(polygon, 0.0, 0.0);
    EXPECT_EQ(translated_polygon.outer()[0].x(), 0.0);
    EXPECT_EQ(translated_polygon.outer()[0].y(), 0.0);

    translated_polygon = drivable_area_expansion::translate_polygon(polygon, 1.0, 2.0);
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

    TEST(BehaviorPathPlanningPolygon2d, Polygon2dCreateObjectFootprint)
    {
        drivable_area_expansion::MultiPolygon2d object_footprint;// params;
        drivable_area_expansion::MultiPolygon2d footprints;
        drivable_area_expansion::MultiPolygon2d footprints1;
        drivable_area_expansion::Polygon2d base_footprint;
        drivable_area_expansion::DrivableAreaExpansionParameters params;
        autoware_auto_perception_msgs::msg::PredictedObjects objects;
        autoware_auto_perception_msgs::msg::PredictedObject object;
        autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
        autoware_auto_perception_msgs::msg::PredictedObjectKinematics kinematics;
        geometry_msgs::msg::Pose pose;


        //considering it in 2D
        pose.position.x = 1.0;
        pose.position.y = 2.0;
        pose.position.z = 0.0;
        pose.orientation.w = 0.0;

        params.avoid_dynamic_objects = true;
        object.shape.dimensions.x = 1.0;
        object.shape.dimensions.y = 1.0;
             
        params.dynamic_objects_extra_front_offset = 0.1;
        params.dynamic_objects_extra_rear_offset = 0.1;
        params.dynamic_objects_extra_left_offset = 0.1;
        params.dynamic_objects_extra_right_offset = 0.1;  

        const auto front = object.shape.dimensions.x / 2 + params.dynamic_objects_extra_front_offset;
        const auto rear = -object.shape.dimensions.x / 2 - params.dynamic_objects_extra_rear_offset;
        const auto left = object.shape.dimensions.y / 2 + params.dynamic_objects_extra_left_offset;
        const auto right = -object.shape.dimensions.y / 2 - params.dynamic_objects_extra_right_offset;   

        base_footprint.outer() = {{front, left}, {front, right}, {rear, right}, {rear, left}, {front, left}}; 
        footprints.push_back(drivable_area_expansion::create_footprint(pose, base_footprint));
        
        ASSERT_EQ(footprints.size(), 1UL);

        drivable_area_expansion::Polygon2d expected_footprint;
        expected_footprint.outer() = {{0.6, 0.6}, {0.6, -0.6}, {-0.6, -0.6}, {-0.6, 0.6}, {0.6, 0.6}}; 

        EXPECT_TRUE(boost::geometry::equals(footprints[0], expected_footprint));

        //new objects
        object.shape.dimensions.x = 2.0;
        object.shape.dimensions.y = 2.0; 
        
        footprints.push_back(drivable_area_expansion::create_footprint(pose, base_footprint));
        expected_footprint.outer() = {{0.6, 1.1}, {0.6, -1.1}, {-0.6, -1.1}, {-0.6, 1.1}, {0.6, 1.1}};
        
        EXPECT_TRUE(boost::geometry::equals(footprints[0], expected_footprint));


    }

