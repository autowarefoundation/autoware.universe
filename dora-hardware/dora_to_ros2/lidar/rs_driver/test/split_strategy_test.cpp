
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/split_strategy.hpp>

using namespace robosense::lidar;

TEST(TestSplitStrategyByAngle, newBlock)
{
  {
    SplitStrategyByAngle sa(10);
    ASSERT_FALSE(sa.newBlock(5));
    ASSERT_TRUE(sa.newBlock(15));
  }

  {
    SplitStrategyByAngle sa(10);
    ASSERT_FALSE(sa.newBlock(5));
    ASSERT_TRUE(sa.newBlock(10));
    ASSERT_FALSE(sa.newBlock(15));
  }

  {
    SplitStrategyByAngle sa(10);
    ASSERT_FALSE(sa.newBlock(10));
    ASSERT_FALSE(sa.newBlock(15));
  }
}

TEST(TestSplitStrategyByAngle, newBlock_Zero)
{
  {
    SplitStrategyByAngle sa(0);
    ASSERT_FALSE(sa.newBlock(35999));
    ASSERT_TRUE(sa.newBlock(1));
    ASSERT_FALSE(sa.newBlock(2));
  }

  {
    SplitStrategyByAngle sa(0);
    ASSERT_FALSE(sa.newBlock(35999));
    ASSERT_TRUE(sa.newBlock(0));
    ASSERT_FALSE(sa.newBlock(2));
  }

  {
    SplitStrategyByAngle sa(0);
    ASSERT_FALSE(sa.newBlock(0));
    ASSERT_FALSE(sa.newBlock(2));
  }
}

TEST(TestSplitStrategyByNum, newBlock)
{
  uint16_t max_blks = 2;
  SplitStrategyByNum sn(&max_blks);
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_TRUE(sn.newBlock(0));
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_TRUE(sn.newBlock(0));

  max_blks = 3;
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_TRUE(sn.newBlock(0));
}

TEST(TestSplitStrategyBySeq, newPacket_by_seq)
{
  SplitStrategyBySeq sn;
  ASSERT_EQ(sn.prev_seq_, 0);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 10);

  // init value
  ASSERT_FALSE(sn.newPacket(1));
  ASSERT_EQ(sn.prev_seq_, 1);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 11);

  // too big value
  ASSERT_FALSE(sn.newPacket(12));
  ASSERT_EQ(sn.prev_seq_, 1);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 11);

  ASSERT_FALSE(sn.newPacket(9));

  ASSERT_FALSE(sn.newPacket(12));
  ASSERT_EQ(sn.prev_seq_, 12);
  ASSERT_EQ(sn.safe_seq_min_, 2);
  ASSERT_EQ(sn.safe_seq_max_, 22);

  ASSERT_FALSE(sn.newPacket(11));
  ASSERT_EQ(sn.prev_seq_, 12);
  ASSERT_EQ(sn.safe_seq_min_, 2);
  ASSERT_EQ(sn.safe_seq_max_, 22);
}

TEST(TestSplitStrategyBySeq, newPacket_prev_seq)
{
  SplitStrategyBySeq sn;
  ASSERT_EQ(sn.prev_seq_, 0);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 10);

  // init value
  ASSERT_FALSE(sn.newPacket(15));
  ASSERT_EQ(sn.prev_seq_, 15);
  ASSERT_EQ(sn.safe_seq_min_, 5);
  ASSERT_EQ(sn.safe_seq_max_, 25);
}

TEST(TestSplitStrategyBySeq, newPacket_rewind)
{
  SplitStrategyBySeq sn;
  ASSERT_EQ(sn.prev_seq_, 0);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 10);

  // init value
  ASSERT_FALSE(sn.newPacket(2));
  ASSERT_EQ(sn.prev_seq_, 2);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 12);

  ASSERT_FALSE(sn.newPacket(10));
  ASSERT_FALSE(sn.newPacket(14));
  ASSERT_EQ(sn.prev_seq_, 14);
  ASSERT_EQ(sn.safe_seq_min_, 4);
  ASSERT_EQ(sn.safe_seq_max_, 24);

  ASSERT_TRUE(sn.newPacket(1));
  ASSERT_EQ(sn.prev_seq_, 1);
  ASSERT_EQ(sn.safe_seq_min_, 0);
  ASSERT_EQ(sn.safe_seq_max_, 11);
}
