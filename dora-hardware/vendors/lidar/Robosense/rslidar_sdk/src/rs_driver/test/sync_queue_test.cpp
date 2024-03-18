
#include <gtest/gtest.h>

#include <rs_driver/utility/sync_queue.hpp>

using namespace robosense::lidar;

TEST(TestSyncQueue, emptyPop)
{
  SyncQueue<std::shared_ptr<int>> queue;

  ASSERT_TRUE(queue.pop().get() == NULL);
  ASSERT_TRUE(queue.popWait(1000).get() == NULL);
}

TEST(TestSyncQueue, nulPtrPop)
{
  SyncQueue<std::shared_ptr<int>> queue;

  {
    std::shared_ptr<int> n_ptr;
    ASSERT_EQ(queue.push(n_ptr), 1);
    ASSERT_TRUE(queue.pop().get() == NULL);
  }

  {
    std::shared_ptr<int> n_ptr;
    ASSERT_EQ(queue.push(n_ptr), 1);
    ASSERT_TRUE(queue.popWait(1000).get() == NULL);
  }
}

TEST(TestSyncQueue, valPtrPop)
{
  SyncQueue<std::shared_ptr<int>> queue;

  {
    std::shared_ptr<int> v_ptr = std::make_shared<int>(100);
    ASSERT_EQ(queue.push(v_ptr), 1);
    ASSERT_TRUE(queue.pop().get() != NULL);
  }

  {
    std::shared_ptr<int> v_ptr = std::make_shared<int>(100);
    ASSERT_EQ(queue.push(v_ptr), 1);
    ASSERT_TRUE(queue.popWait(1000).get() != NULL);
  }
}

TEST(TestSyncQueue, clear)
{
  SyncQueue<std::shared_ptr<int>> queue;

  std::shared_ptr<int> v_ptr = std::make_shared<int>(100);
  ASSERT_EQ(queue.push(v_ptr), 1);
  ASSERT_EQ(queue.push(v_ptr), 2);
  queue.clear();
  ASSERT_EQ(queue.push(v_ptr), 1);
}
