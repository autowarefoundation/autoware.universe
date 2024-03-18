
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_RS16.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/utility/dbg.hpp>

using namespace robosense::lidar;

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloud;

TEST(TestDecoderRS16, getEchoMode)
{
  ASSERT_TRUE(DecoderRS16<PointCloud>::getEchoMode(0) == RSEchoMode::ECHO_DUAL);
  ASSERT_TRUE(DecoderRS16<PointCloud>::getEchoMode(1) == RSEchoMode::ECHO_SINGLE);
  ASSERT_TRUE(DecoderRS16<PointCloud>::getEchoMode(2) == RSEchoMode::ECHO_SINGLE);
}

TEST(TestDecoderRS16, RS16DifopPkt2Adapter)
{
  uint8_t pitch_cali[48] = 
  {
    0x00, 0x3a, 0x98, // 15.000
    0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 
    0x00, 0x3a, 0x98, // 15.000
  };
  
  RS16DifopPkt src;
  src.rpm = 0;
  src.fov = {0};
  src.return_mode = 0;
  memcpy (src.pitch_cali, pitch_cali, 48);

  AdapterDifopPkt dst;
  RS16DifopPkt2Adapter(src, dst);

  ASSERT_EQ(dst.vert_angle_cali[0].sign, 1);
  ASSERT_EQ(ntohs(dst.vert_angle_cali[0].value), 150);
  ASSERT_EQ(dst.vert_angle_cali[8].sign, 0);
  ASSERT_EQ(ntohs(dst.vert_angle_cali[8].value), 150);

  ASSERT_EQ(dst.horiz_angle_cali[0].sign, 0);
  ASSERT_EQ(ntohs(dst.horiz_angle_cali[0].value), 0);
}

