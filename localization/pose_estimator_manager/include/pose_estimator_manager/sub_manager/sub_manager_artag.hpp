#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace multi_pose_estimator
{
class SubManagerArTag : public BasePoseEstimatorSubManager
{
public:
  using Image = sensor_msgs::msg::Image;
  using SetBool = std_srvs::srv::SetBool;

  SubManagerArTag(rclcpp::Node * node) : BasePoseEstimatorSubManager(node)
  {
    ar_tag_is_enabled_ = true;

    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    using std::placeholders::_1;
    auto on_image = std::bind(&SubManagerArTag::on_image, this, _1);
    sub_image_ = node->create_subscription<Image>("~/input/artag/image", qos, on_image);
    pub_image_ = node->create_publisher<Image>("~/output/artag/image", qos);
  }

  void set_enable(bool enabled) override { ar_tag_is_enabled_ = enabled; }

protected:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

private:
  bool ar_tag_is_enabled_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  void on_image(Image::ConstSharedPtr msg)
  {
    if (ar_tag_is_enabled_) {
      pub_image_->publish(*msg);
    }
  }
};
}  // namespace multi_pose_estimator
