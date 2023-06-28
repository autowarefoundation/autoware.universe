#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace multi_pose_estimator
{
class NdtSubManager : public rclcpp::Node, BasePoseEstimatorSubManager
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  NdtSubManager() : Node("ndt_sub_manager"), BasePoseEstimatorSubManager(this)
  {
    using std::placeholders::_1;
    auto on_pointcloud = std::bind(&NdtSubManager::on_pointcloud, this, _1);

    sub_pointcloud_ = create_subscription<PointCloud2>(
      "~/input/pointcloud", rclcpp::SensorDataQoS(), on_pointcloud);
    pub_pointcloud_ =
      create_publisher<PointCloud2>("~/output/pointcloud", rclcpp::SensorDataQoS().keep_last(10));

    ndt_is_enabled = true;
  }

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  bool ndt_is_enabled;

  void on_service(
    SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response) override
  {
    ndt_is_enabled = request->data;
    response->success = true;
  }

  void on_pointcloud(PointCloud2::ConstSharedPtr msg)
  {
    if (ndt_is_enabled) pub_pointcloud_->publish(*msg);
  }
};
}  // namespace multi_pose_estimator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_pose_estimator::NdtSubManager>());
  rclcpp::shutdown();
  return 0;
}