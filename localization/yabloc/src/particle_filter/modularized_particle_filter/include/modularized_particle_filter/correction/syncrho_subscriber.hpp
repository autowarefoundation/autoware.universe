#pragma once
#include <modularized_particle_filter_msgs/msg/cloud_with_pose.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

class SyncroSubscriber
{
public:
  using CloudPose = modularized_particle_filter_msgs::msg::CloudWithPose;
  using CloudPoseSub = message_filters::Subscriber<CloudPose>;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using CloudSub = message_filters::Subscriber<Cloud>;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using ParticlesSub = message_filters::Subscriber<ParticleArray>;

  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<Cloud, CloudPose, ParticleArray>;

  SyncroSubscriber(
    rclcpp::Node::SharedPtr n, const std::string & topic1, const std::string & topic2,
    const std::string & topic3)
  {
    cloud_sub = std::make_shared<CloudSub>(n, topic1);
    cloud_pose_sub = std::make_shared<CloudPoseSub>(n, topic2);
    particles_sub = std::make_shared<ParticlesSub>(n, topic3);

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    syncronizer = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(50), *cloud_sub, *cloud_pose_sub, *particles_sub);

    syncronizer->registerCallback(std::bind(&SyncroSubscriber::rawCallback, this, _1, _2, _3));
    user_callback = std::nullopt;
  }

  using UserCallback = std::function<void(
    const sensor_msgs::msg::PointCloud2 &,
    const modularized_particle_filter_msgs::msg::CloudWithPose &,
    const modularized_particle_filter_msgs::msg::ParticleArray &)>;
  void setCallback(const UserCallback & callback) { user_callback = callback; }

private:
  std::shared_ptr<CloudSub> cloud_sub;
  std::shared_ptr<CloudPoseSub> cloud_pose_sub;
  std::shared_ptr<ParticlesSub> particles_sub;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> syncronizer;
  std::optional<UserCallback> user_callback;

  void rawCallback(
    const Cloud::ConstSharedPtr & msg1, const CloudPose::ConstSharedPtr & msg2,
    const ParticleArray::ConstSharedPtr & msg3)
  {
    if (user_callback.has_value()) user_callback.value()(*msg1, *msg2, *msg3);
  };
};
