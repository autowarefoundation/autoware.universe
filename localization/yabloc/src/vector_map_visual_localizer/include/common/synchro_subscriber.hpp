#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

class SyncroSubscriber
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using PoseSub = message_filters::Subscriber<PoseStamped>;
  using ImageSub = message_filters::Subscriber<CompressedImage>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PoseStamped, CompressedImage>;

  SyncroSubscriber(
    rclcpp::Node::SharedPtr n, const std::string & topic1, const std::string & topic2)
  {
    pose_sub = std::make_shared<PoseSub>(n, topic1);
    image_sub = std::make_shared<ImageSub>(n, topic2);

    using std::placeholders::_1;
    using std::placeholders::_2;

    syncronizer = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(80), *pose_sub, *image_sub);

    syncronizer->registerCallback(std::bind(&SyncroSubscriber::rawCallback, this, _1, _2));
    user_callback = std::nullopt;
  }

  using UserCallback = std::function<void(
    const geometry_msgs::msg::PoseStamped &, const sensor_msgs::msg::CompressedImage &)>;
  void setCallback(const UserCallback & callback) { user_callback = callback; }

private:
  std::shared_ptr<PoseSub> pose_sub;
  std::shared_ptr<ImageSub> image_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> syncronizer;
  std::optional<UserCallback> user_callback;

  void rawCallback(
    const PoseStamped::ConstSharedPtr & msg1, const CompressedImage::ConstSharedPtr & msg2)
  {
    if (user_callback.has_value()) user_callback.value()(*msg1, *msg2);
  };
};
