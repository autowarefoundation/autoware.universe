
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/trigon.hpp>

using namespace robosense::lidar;

TEST(TestTrigon, ctor)
{
  Trigon trigon;
#if 0
  trigon.print();
#endif

  ASSERT_EQ(trigon.sin(-9000), -1.0f);
  ASSERT_LT(trigon.cos(-9000), 0.0001f);

  ASSERT_EQ(trigon.sin(0), 0.0f);
  ASSERT_EQ(trigon.cos(0), 1.0f);

  ASSERT_EQ(trigon.sin(3000), 0.5f);
  ASSERT_EQ(trigon.cos(6000), 0.5f);

  trigon.sin(44999);
  trigon.cos(44999);

#if 0
  trigon.sin(45000);
  trigon.cos(45000);
#endif
}

