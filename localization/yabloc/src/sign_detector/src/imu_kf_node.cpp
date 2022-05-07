#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/geometry.hpp>

class ImuKalmanFilter : public rclcpp::Node
{
public:
  ImuKalmanFilter() : Node("imu_kalman_filter")
  {
    std::string imu_topic = "/sensing/imu/tamagawa/imu_raw";
    std::string pose_topic = "/kf_pose";

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&ImuKalmanFilter::imuCallback, this, std::placeholders::_1));
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu& msg)
  {
    Sophus::SO3f R;
    std::cout << R.matrix() << std::endl;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ImuKalmanFilter>());
  rclcpp::shutdown();
  return 0;
}
