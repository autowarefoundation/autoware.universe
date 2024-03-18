
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/driver/decoder/block_iterator.hpp>

using namespace robosense::lidar;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} MyChannel;

typedef struct
{
  uint16_t azimuth;
  MyChannel channels[2];
} MyBlock;

typedef struct
{
  MyBlock blocks[3];
} MyPacket;

TEST(TestABDualPacketTraverser, ctor)
{
  {
    // AAB
    MyPacket pkt = 
    {
           htons(1), 0x00, 0x00, 0x00, 0x00 
        ,  htons(1), 0x00, 0x00, 0x00, 0x00 
        ,  htons(21), 0x00, 0x00, 0x00, 0x00
    };

    ABDualReturnBlockIterator<MyPacket> iter(pkt, 
        3,     // blocks per packet
        0.5f,  // block_duration
        25,    // block_az_duraton
        2.0f); // fov_blind_duration

    int32_t az_diff;
    double ts;

    // first block
    iter.get (0, az_diff, ts);
    ASSERT_EQ(az_diff, 20);
    ASSERT_EQ(ts, 0.0f);

    // second block
    iter.get (1, az_diff, ts);
    ASSERT_EQ(az_diff, 20);
    ASSERT_EQ(ts, 0.0f);

    // last block
    iter.get (2, az_diff, ts);
    ASSERT_EQ(az_diff, 25);
    ASSERT_EQ(ts, 0.5f);
  }

  {
    // ABB
    MyPacket pkt = 
    {
           htons(1), 0x00, 0x00, 0x00, 0x00 
        ,  htons(21), 0x00, 0x00, 0x00, 0x00
        ,  htons(21), 0x00, 0x00, 0x00, 0x00
    };

    ABDualReturnBlockIterator<MyPacket> iter(pkt, 
        3,     // blocks per packet
        0.5f,  // block_duration
        25,    // block_az_duraton
        2.0f); // fov_blind_duration

    int32_t az_diff;
    double ts;

    // first block
    iter.get (0, az_diff, ts);
    ASSERT_EQ(az_diff, 20);
    ASSERT_EQ(ts, 0.0f);

    // second block
    iter.get (1, az_diff, ts);
    ASSERT_EQ(az_diff, 25);
    ASSERT_EQ(ts, 0.5f);

    // last block
    iter.get (2, az_diff, ts);
    ASSERT_EQ(az_diff, 25);
    ASSERT_EQ(ts, 0.5f);
  }

}

TEST(TestABDualPacketTraverser, ctor_fov)
{
  {
    // AAB
    MyPacket pkt = 
    {
           htons(1), 0x00, 0x00, 0x00, 0x00 
        ,  htons(1), 0x00, 0x00, 0x00, 0x00 
        ,  htons(121), 0x00, 0x00, 0x00, 0x00
    };

    ABDualReturnBlockIterator<MyPacket> iter(pkt, 
        3,     // blocks per packet
        0.5f,  // block_duration
        25,    // block_az_duraton
        2.0f); // fov_blind_duration

    int32_t az_diff;
    double ts;

    // first block
    iter.get (0, az_diff, ts);
    ASSERT_EQ(az_diff, 25);
    ASSERT_EQ(ts, 0.0f);

    // second block
    iter.get (1, az_diff, ts);
    ASSERT_EQ(az_diff, 25);
    ASSERT_EQ(ts, 0.0f);

    // last block
    iter.get (2, az_diff, ts);
    ASSERT_EQ(az_diff, 25);
    ASSERT_EQ(ts, 2.0f);
  }

}


