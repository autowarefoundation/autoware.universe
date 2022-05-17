#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

class SyncroSubscriber
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using CloudSub = message_filters::Subscriber<Cloud>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Cloud, Cloud>;

  SyncroSubscriber(
    rclcpp::Node::SharedPtr n, const std::string & topic1, const std::string & topic2)
  {
    cloud1_sub = std::make_shared<CloudSub>(n, topic1);
    cloud2_sub = std::make_shared<CloudSub>(n, topic2);

    using std::placeholders::_1;
    using std::placeholders::_2;

    syncronizer = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(50), *cloud1_sub, *cloud2_sub);
    syncronizer->registerCallback(std::bind(&SyncroSubscriber::rawCallback, this, _1, _2));
    user_callback = std::nullopt;
  }

  using UserCallback = std::function<void(
    const sensor_msgs::msg::PointCloud2 &, const sensor_msgs::msg::PointCloud2 &)>;
  void setCallback(const UserCallback & callback) { user_callback = callback; }

private:
  std::shared_ptr<CloudSub> cloud1_sub;
  std::shared_ptr<CloudSub> cloud2_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> syncronizer;
  std::optional<UserCallback> user_callback;

  void rawCallback(const Cloud::ConstSharedPtr & msg1, const Cloud::ConstSharedPtr & msg2)
  {
    if (user_callback.has_value()) user_callback.value()(*msg1, *msg2);
  };
};
