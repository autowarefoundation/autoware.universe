#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace multi_pose_estimator
{
class YabLocSubManager : public rclcpp::Node, BasePoseEstimatorSubManager
{
public:
  using Image = sensor_msgs::msg::Image;
  using NavPVT = ublox_msgs::msg::NavPVT;

  YabLocSubManager() : Node("yabloc_sub_manager"), BasePoseEstimatorSubManager(this)
  {
    pcdless_is_enabled_ = true;

    using std::placeholders::_1;
    auto on_image = std::bind(&YabLocSubManager::on_image, this, _1);
    auto on_navpvt = std::bind(&YabLocSubManager::on_navpvt, this, _1);

    sub_image_ = create_subscription<Image>("~/input/image", 5, on_image);
    sub_navpvt_ = create_subscription<NavPVT>("~/input/navpvt", 5, on_navpvt);
    pub_image_ = create_publisher<Image>("~/output/image", 5);
    pub_navpvt_ = create_publisher<NavPVT>("~/output/navpvt", 5);
  }

protected:
  void on_service(
    SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response) override
  {
    pcdless_is_enabled_ = request->data;
    response->success = true;
  }

private:
  bool pcdless_is_enabled_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<NavPVT>::SharedPtr pub_navpvt_;

  void on_navpvt(NavPVT::ConstSharedPtr msg)
  {
    if (pcdless_is_enabled_) {
      pub_navpvt_->publish(*msg);
    }
  }

  void on_image(Image::ConstSharedPtr msg)
  {
    if (pcdless_is_enabled_) {
      pub_image_->publish(*msg);
    }
  }
};
}  // namespace multi_pose_estimator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_pose_estimator::YabLocSubManager>());
  rclcpp::shutdown();
  return 0;
}
