#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"
#include <gtest/gtest.h>


TEST(BehaviorPathPlanningPolygon2d, Polygon2D)
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