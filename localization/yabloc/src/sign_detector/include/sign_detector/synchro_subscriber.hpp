#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>

class SyncroSubscriber
{
public:
  using ImageSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  SyncroSubscriber(rclcpp::Node::SharedPtr n, const std::string& topic1, const std::string& topic2)
  {
    image1_sub = std::make_shared<ImageSub>(n, topic1);
    image2_sub = std::make_shared<ImageSub>(n, topic2);

    syncronizer = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), *image1_sub, *image2_sub);
    syncronizer->registerCallback(std::bind(&SyncroSubscriber::rawCallback, this, std::placeholders::_1, std::placeholders::_2));
    user_callback = std::nullopt;
  }

  using UserCallback = std::function<void(const sensor_msgs::msg::Image&, const sensor_msgs::msg::Image&)>;
  void setCallback(const UserCallback& callback) { user_callback = callback; }

private:
  std::shared_ptr<ImageSub> image1_sub;
  std::shared_ptr<ImageSub> image2_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> syncronizer;
  std::optional<UserCallback> user_callback;

  void rawCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg1,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg2)
  {
    if (user_callback.has_value()) user_callback.value()(*msg1, *msg2);
  };
};