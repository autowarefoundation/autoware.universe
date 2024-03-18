
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/section.hpp>

using namespace robosense::lidar;

TEST(TestAzimuthSection, ctorFull)
{
  AzimuthSection sec(0, 36000);
  ASSERT_TRUE(sec.in(0));
  ASSERT_TRUE(sec.in(10));
  ASSERT_TRUE(sec.in(36000));
}

TEST(TestAzimuthSection, ctor)
{
  AzimuthSection sec(10, 20);
  ASSERT_EQ(sec.start_, 10);
  ASSERT_EQ(sec.end_, 20);

  ASSERT_FALSE(sec.in(5));
  ASSERT_TRUE(sec.in(10));
  ASSERT_TRUE(sec.in(15));
  ASSERT_FALSE(sec.in(20));
  ASSERT_FALSE(sec.in(25));
}

TEST(TestAzimuthSection, ctorCrossZero)
{
  AzimuthSection sec(35000, 10);
  ASSERT_EQ(sec.start_, 35000);
  ASSERT_EQ(sec.end_, 10);

  ASSERT_FALSE(sec.in(34999));
  ASSERT_TRUE(sec.in(35000));
  ASSERT_TRUE(sec.in(0));
  ASSERT_FALSE(sec.in(10));
  ASSERT_FALSE(sec.in(15));
}

TEST(TestAzimuthSection, ctorBeyondRound)
{
  AzimuthSection sec(36100, 36200);
  ASSERT_EQ(sec.start_, 100);
  ASSERT_EQ(sec.end_, 200);
}

TEST(TestDistanceSection, ctor)
{
  DistanceSection sec(0.5, 200, 0.75, 150);
  ASSERT_EQ(sec.min_, 0.75);
  ASSERT_EQ(sec.max_, 150);

  ASSERT_FALSE(sec.in(0.45));
  ASSERT_TRUE(sec.in(0.75));
  ASSERT_TRUE(sec.in(0.8));
  ASSERT_TRUE(sec.in(150));
  ASSERT_FALSE(sec.in(150.5));
}

TEST(TestDistanceSection, ctorZeroUserDistance)
{
  DistanceSection sec(0.5, 200, 0.0, 0.0);
  ASSERT_EQ(sec.min_, 0.5);
  ASSERT_EQ(sec.max_, 200);
}

TEST(TestDistanceSection, ctorNegtiveUserDistance)
{
  DistanceSection sec(0.5, 200, -0.1, -0.2);
  ASSERT_EQ(sec.min_, 0.5);
  ASSERT_EQ(sec.max_, 200);
}

