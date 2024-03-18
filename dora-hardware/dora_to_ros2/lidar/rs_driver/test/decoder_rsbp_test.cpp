
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_RSBP.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/utility/dbg.hpp>

using namespace robosense::lidar;

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloud;

static ErrCode errCode = ERRCODE_SUCCESS;
static void errCallback(const Error& err)
{
  errCode = err.error_code;
}

TEST(TestDecoderRSBP, getEchoMode)
{
  ASSERT_TRUE(DecoderRSBP<PointCloud>::getEchoMode(0) == RSEchoMode::ECHO_DUAL);
  ASSERT_TRUE(DecoderRSBP<PointCloud>::getEchoMode(1) == RSEchoMode::ECHO_SINGLE);
  ASSERT_TRUE(DecoderRSBP<PointCloud>::getEchoMode(2) == RSEchoMode::ECHO_SINGLE);
}

TEST(TestDecoderRSBP, decodeDifopPkt)
{
  // const_param
  RSDecoderParam param;
  DecoderRSBP<PointCloud> decoder(param);
  decoder.regCallback(errCallback, nullptr);

  ASSERT_EQ(decoder.blks_per_frame_, 1801);
  ASSERT_EQ(decoder.split_blks_per_frame_, 1801);

  // rpm = 600, dual return
  RSBPDifopPkt pkt;
  pkt.rpm = htons(600);
  pkt.return_mode = 0;
  decoder.decodeDifopPkt((uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(decoder.rps_, 10);
  ASSERT_EQ(decoder.echo_mode_, RSEchoMode::ECHO_DUAL);
  ASSERT_EQ(decoder.blks_per_frame_, 1801);
  ASSERT_EQ(decoder.split_blks_per_frame_, 3602);

  // rpm = 1200, single return
  pkt.rpm = htons(1200);
  pkt.return_mode = 1; 
  decoder.decodeDifopPkt((uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(decoder.rps_, 20);
  ASSERT_EQ(decoder.echo_mode_, RSEchoMode::ECHO_SINGLE);
  ASSERT_EQ(decoder.blks_per_frame_, 900);
  ASSERT_EQ(decoder.split_blks_per_frame_, 900);
}

static void splitFrame(uint16_t height, double ts)
{
}

TEST(TestDecoderRSBP, decodeMsopPkt)
{
  uint8_t pkt[] = 
  {
    //
    // header
    //
    0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0, // msop id
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved_1
    0x15, 0x0a, 0x01, 0x01, 0x02, 0x03, 0x11, 0x22, 0x33, 0x44, // ts_YMD
    0x00, // lidar type
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved_2
    0x18, 0x01, // temprature
    0x00, 0x00, // reserved_3

    //
    // block_01
    //
    0xFF, 0xEE, // block id
    0x00, 0x00, // azimuth
    0x03, 0xE8, // chan_00, distance
    0x01,       // chan_00, intensity
    0x00, 0x00, // chan_01, distance
    0x00,       // chan_01, intensity
    0x00, 0x00, // chan_02, distance
    0x00,       // chan_02, intensity
    0x00, 0x00, // chan_03, distance
    0x00,       // chan_03, intensity
    0x00, 0x00, // chan_04, distance
    0x00,       // chan_04, intensity
    0x00, 0x00, // chan_05, distance
    0x00,       // chan_05, intensity
    0x00, 0x00, // chan_06, distance
    0x00,       // chan_06, intensity
    0x00, 0x00, // chan_07, distance
    0x00,       // chan_07, intensity
    0x00, 0x00, // chan_08, distance
    0x00,       // chan_08, intensity
    0x00, 0x00, // chan_09, distance
    0x00,       // chan_09, intensity
    0x00, 0x00, // chan_10, distance
    0x00,       // chan_10, intensity
    0x00, 0x00, // chan_11, distance
    0x00,       // chan_11, intensity
    0x00, 0x00, // chan_12, distance
    0x00,       // chan_12, intensity
    0x00, 0x00, // chan_13, distance
    0x00,       // chan_13, intensity
    0x00, 0x00, // chan_14, distance
    0x00,       // chan_14, intensity
    0x00, 0x00, // chan_15, distance
    0x00,       // chan_15, intensity
    0x00, 0x00, // chan_16, distance
    0x00,       // chan_16, intensity
    0x00, 0x00, // chan_17, distance
    0x00,       // chan_17, intensity
    0x00, 0x00, // chan_18, distance
    0x00,       // chan_18, intensity
    0x00, 0x00, // chan_19, distance
    0x00,       // chan_19, intensity
    0x00, 0x00, // chan_20, distance
    0x00,       // chan_20, intensity
    0x00, 0x00, // chan_21, distance
    0x00,       // chan_21, intensity
    0x00, 0x00, // chan_22, distance
    0x00,       // chan_22, intensity
    0x00, 0x00, // chan_23, distance
    0x00,       // chan_23, intensity
    0x00, 0x00, // chan_24, distance
    0x00,       // chan_24, intensity
    0x00, 0x00, // chan_25, distance
    0x00,       // chan_25, intensity
    0x00, 0x00, // chan_26, distance
    0x00,       // chan_26, intensity
    0x00, 0x00, // chan_27, distance
    0x00,       // chan_27, intensity
    0x00, 0x00, // chan_28, distance
    0x00,       // chan_28, intensity
    0x00, 0x00, // chan_29, distance
    0x00,       // chan_29, intensity
    0x00, 0x00, // chan_30, distance
    0x00,       // chan_30, intensity
    0x00, 0x00, // chan_31, distance
    0x00,       // chan_31, intensity

    //
    // block_02
    //
    0x00, 0x00, // block id
  };

  // dense_points = false, use_lidar_clock = true
  RSDecoderParam param;
  DecoderRSBP<PointCloud> decoder(param);
  decoder.regCallback(errCallback, splitFrame);

  ASSERT_EQ(decoder.chan_angles_.user_chans_.size(), 32);
  decoder.chan_angles_.user_chans_[0] = 2;
  decoder.chan_angles_.user_chans_[1] = 1;
  decoder.param_.dense_points = false;
  decoder.param_.use_lidar_clock = true;

  decoder.point_cloud_ = std::make_shared<PointCloud>();

  decoder.decodeMsopPkt(pkt, sizeof(pkt));
  ASSERT_EQ(decoder.getTemperature(), 2.1875);
  ASSERT_EQ(decoder.point_cloud_->points.size(), 32);

  PointT& point = decoder.point_cloud_->points[0];
  ASSERT_EQ(point.intensity, 1);
  ASSERT_NE(point.timestamp, 0);
  ASSERT_EQ(point.ring, 2);
}

