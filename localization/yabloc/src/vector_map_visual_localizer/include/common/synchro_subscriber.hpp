#pragma once
#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

template <typename Msg1, typename Msg2>
class SyncroSubscriber
{
public:
  using Sub1 = message_filters::Subscriber<Msg1>;
  using Sub2 = message_filters::Subscriber<Msg2>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Msg1, Msg2>;
  using UserCallback = std::function<void(const Msg1 &, const Msg2 &)>;

  SyncroSubscriber(
    rclcpp::Node::SharedPtr n, const std::string & topic1, const std::string & topic2)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    sub1_ = std::make_shared<Sub1>(n, topic1);
    sub2_ = std::make_shared<Sub2>(n, topic2);
    syncronizer_ = std::make_shared<Synchronizer>(SyncPolicy(80), *sub1_, *sub2_);
    syncronizer_->registerCallback(std::bind(&SyncroSubscriber::rawCallback, this, _1, _2));
  }

  void setCallback(const UserCallback & callback) { user_callback_ = callback; }

private:
  std::shared_ptr<Sub1> sub1_;
  std::shared_ptr<Sub2> sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> syncronizer_;
  std::optional<UserCallback> user_callback_{std::nullopt};

  void rawCallback(const typename Msg1::ConstPtr & msg1, const typename Msg2::ConstPtr & msg2)
  {
    if (user_callback_.has_value()) user_callback_.value()(*msg1, *msg2);
  }
};
