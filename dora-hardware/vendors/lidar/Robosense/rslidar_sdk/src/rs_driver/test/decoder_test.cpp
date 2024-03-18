
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/decoder_mech.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/utility/dbg.hpp>

using namespace robosense::lidar;

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloud;

#pragma pack(push, 1)
struct MyMsopPkt
{
  uint8_t id[8];
};

struct MyDifopPkt
{
  uint8_t id[8];
  uint16_t rpm;
  RSFOV fov;
  RSCalibrationAngle vert_angle_cali[2];
  RSCalibrationAngle horiz_angle_cali[2];
};
#pragma pack(pop)

class MyDecoder : public DecoderMech<PointCloud>
{
public:
  MyDecoder(const RSDecoderMechConstParam& const_param,
      const RSDecoderParam& param)
  : DecoderMech<PointCloud>(const_param, param)
  {
  }

  virtual void decodeDifopPkt(const uint8_t* packet, size_t size)
  {
    const MyDifopPkt& pkt = *(const MyDifopPkt*)(packet);
    this->template decodeDifopCommon<MyDifopPkt>(pkt);
  }

  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size)
  {
    return false;
  }

};

static ErrCode errCode = ERRCODE_SUCCESS;

static void errCallback(const Error& err)
{
  errCode = err.error_code;
}

TEST(TestDecoder, angles_from_file)
{
  RSDecoderMechConstParam const_param;
  const_param.base.LASER_NUM = 4;

  RSDecoderParam param;
  param.config_from_file = true;
  param.angle_path = "../rs_driver/test/res/angle.csv";

  errCode = ERRCODE_SUCCESS;
  MyDecoder decoder(const_param, param);
  decoder.regCallback(errCallback, nullptr);
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);

  ASSERT_TRUE(decoder.angles_ready_);
}

TEST(TestDecoder, angles_from_file_fail)
{
  RSDecoderMechConstParam const_param;
  const_param.base.LASER_NUM = 4;

  RSDecoderParam param;
  param.config_from_file = true;
  param.angle_path = "../rs_driver/test/res/non_exist.csv";

  MyDecoder decoder(const_param, param);
  decoder.regCallback(errCallback, nullptr);
  ASSERT_FALSE(decoder.angles_ready_);
}

TEST(TestDecoder, processDifopPkt_fail)
{
  RSDecoderMechConstParam const_param = 
  {
    sizeof(MyMsopPkt) // msop len
    , sizeof(MyDifopPkt) // difop len
    , 8 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
  };

  RSDecoderParam param;
  MyDecoder decoder(const_param, param);
  decoder.regCallback(errCallback, nullptr);

  // wrong difop length
  MyDifopPkt pkt = {0};
  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt((const uint8_t*)&pkt, 10);
  ASSERT_EQ(errCode, ERRCODE_WRONGDIFOPLEN);

  // wrong difop id
  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt((const uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(errCode, ERRCODE_WRONGDIFOPID);
}

TEST(TestDecoder, processDifopPkt)
{
  RSDecoderMechConstParam const_param = 
  {
    sizeof(MyMsopPkt) // msop len
    , sizeof(MyDifopPkt) // difop len
    , 8 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 2 // laser number
    , 1000 // blocks per packet
    , 2 // channels per block
  };

  const_param.BLOCK_DURATION = 55.52f / 1000000;

  RSDecoderParam param;
  param.config_from_file = false;
  MyDecoder decoder(const_param, param);
  decoder.regCallback(errCallback, nullptr);
  ASSERT_FALSE(decoder.angles_ready_);

  //
  // angles from difop. no angles in difop
  //

  uint8_t pkt_no_angles[] = 
  {
    0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 // difop id
    , 0x02, 0x58 // rpm
    , 0x23, 0x28 // start angle = 9000
    , 0x46, 0x50 // end angle = 18000 
    , 0xFF, 0xFF, 0xFF // vert angles
    , 0xFF, 0xFF, 0xFF
    , 0xFF, 0xFF, 0xFF // horiz angles
    , 0xFF, 0xFF, 0xFF
  };

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt(pkt_no_angles, sizeof(MyDifopPkt));
  errCode = ERRCODE_SUCCESS;

  ASSERT_EQ(decoder.rps_, 10);
  ASSERT_EQ(decoder.blks_per_frame_, 1801);
  ASSERT_EQ(decoder.block_az_diff_, 20);
  ASSERT_EQ(decoder.split_blks_per_frame_, 1801);
  ASSERT_EQ(decoder.fov_blind_ts_diff_, 0.075); // 0.1 * 3/4
  ASSERT_FALSE(decoder.angles_ready_);
  ASSERT_EQ(decoder.chan_angles_.vert_angles_.size(), 2);
  ASSERT_EQ(decoder.chan_angles_.vert_angles_[0], 0);
  ASSERT_EQ(decoder.chan_angles_.horiz_angles_.size(), 2);
  ASSERT_EQ(decoder.chan_angles_.horiz_angles_[0], 0);

  //
  // angles from difop. valid angels in difop.
  //
  uint8_t pkt[] = 
  {
    0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 // difop id
    , 0x02, 0x58 // rpm
    , 0x00, 0x00 // start angle = 0
    , 0x8C, 0xA0 // end angle = 36000
    , 0x00, 0x00, 0x10 // vert angles
    , 0x01, 0x00, 0x20
    , 0x00, 0x00, 0x01 // horiz angles
    , 0x01, 0x00, 0x02
  };

  ASSERT_LT(decoder.getPacketDuration() - 55.52/1000, 0.00001);

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt(pkt, sizeof(MyDifopPkt));
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);

  ASSERT_EQ(decoder.rps_, 10);
  ASSERT_EQ(decoder.fov_blind_ts_diff_, 0.0f); // 0.1 * 3/4
  ASSERT_TRUE(decoder.angles_ready_);
  ASSERT_EQ(decoder.chan_angles_.vert_angles_.size(), 2);
  ASSERT_EQ(decoder.chan_angles_.vert_angles_[0], 16);
  ASSERT_EQ(decoder.chan_angles_.horiz_angles_.size(), 2);
  ASSERT_EQ(decoder.chan_angles_.horiz_angles_[0], 1);
}

TEST(TestDecoder, processDifopPkt_invalid_rpm)
{
  RSDecoderMechConstParam const_param = 
  {
    sizeof(MyMsopPkt) // msop len
    , sizeof(MyDifopPkt) // difop len
    , 8 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 32 // laser number
    , 12 // blocks per packet
    , 32 // channels per block
  };

  RSDecoderParam param;
  MyDecoder decoder(const_param, param);
  decoder.regCallback(errCallback, nullptr);

  uint8_t pkt[] = 
  {
     0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55 // difop len
    , 0x00, 0x00 // rpm = 0
  };

  errCode = ERRCODE_SUCCESS;
  decoder.processDifopPkt(pkt, sizeof(MyDifopPkt));
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);
  ASSERT_EQ(decoder.rps_, 10);
}

TEST(TestDecoder, processMsopPkt)
{
  RSDecoderMechConstParam const_param = 
  {
    sizeof(MyMsopPkt) // msop len
    , sizeof(MyDifopPkt) // difop len
    , 8 // msop id len
    , 8 // difop id len
    , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
  };

  MyMsopPkt pkt;
  RSDecoderParam param;
  MyDecoder decoder(const_param, param);
  decoder.regCallback(errCallback, nullptr);

  // wait_for_difop = true, angles not ready
  decoder.param_.wait_for_difop = true;
  decoder.angles_ready_ = false;
  errCode = ERRCODE_SUCCESS;
  decoder.processMsopPkt((const uint8_t*)&pkt, 2);
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);

#if 0
  sleep(2);
  errCode = ERRCODE_SUCCESS;
  decoder.processMsopPkt((const uint8_t*)&pkt, 2);
  ASSERT_EQ(errCode, ERRCODE_NODIFOPRECV);
#endif

  decoder.param_.wait_for_difop = true;
  decoder.angles_ready_ = true;

  // wrong msop len
  errCode = ERRCODE_SUCCESS;
  decoder.processMsopPkt((const uint8_t*)&pkt, 2);
  ASSERT_EQ(errCode, ERRCODE_WRONGMSOPLEN);

  decoder.param_.wait_for_difop = false;

  // wrong msop header
  errCode = ERRCODE_SUCCESS;
  decoder.processMsopPkt((const uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(errCode, ERRCODE_WRONGMSOPID);

  // valid msop
  uint8_t id[] = {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0};
  memcpy (pkt.id, id, 8);
  errCode = ERRCODE_SUCCESS;
  decoder.processMsopPkt((const uint8_t*)&pkt, sizeof(pkt));
  ASSERT_EQ(errCode, ERRCODE_SUCCESS);
}

