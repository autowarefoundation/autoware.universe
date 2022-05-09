#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/geometry.hpp>


class ImuKalmanFilter : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuKalmanFilter() : Node("imu_kalman_filter"), predict_cov(1.0f), measure_cov(400.0f)
  {
    std::string imu_topic = "/sensing/imu/tamagawa/imu_raw";
    std::string pose_topic = "/kf_pose";

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, std::bind(&ImuKalmanFilter::imuCallback, this, std::placeholders::_1));
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    R_ = Sophus::SO3f();
    cov_ = 3 * Eigen::Matrix3f::Identity();
  }

private:
  Sophus::SO3f R_;
  Eigen::Matrix3f cov_;
  std::optional<rclcpp::Time> last_stamp_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;
  const float predict_cov;
  const float measure_cov;

  void publishPoseStamped()
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = last_stamp_.value();

    ps.pose.position.x = 0;
    ps.pose.position.y = 0;
    ps.pose.position.z = 0;

    Eigen::Quaternionf q = R_.unit_quaternion();
    ps.pose.orientation.w = q.w();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    pub_pose_stamped_->publish(ps);
  }

  void imuCallback(const sensor_msgs::msg::Imu& msg)
  {
    Eigen::Vector3f a, w;
    auto acc = msg.linear_acceleration;
    auto avel = msg.angular_velocity;
    a << acc.x, acc.y, acc.z;
    w << avel.x, avel.y, avel.z;

    Eigen::Matrix3f ext = Eigen::Matrix3f::Identity();
    ext(0, 0) = ext(2, 2) = -1;
    a = ext * a;
    w = ext * w;

    rclcpp::Time stamp = msg.header.stamp;
    if (!last_stamp_.has_value()) {
      last_stamp_ = stamp;
      return;
    }

    const float dt = (stamp - last_stamp_.value()).seconds();
    // RCLCPP_INFO_STREAM(this->get_logger(), dt << " " << a.transpose() << " " << w.transpose());

    predictUpdate(dt, w);
    measureUpdate(a);
    publishPoseStamped();
    last_stamp_ = stamp;
  }

  void predictUpdate(const float dt, const Eigen::Vector3f& w)
  {
    const Eigen::Matrix3f Q = predict_cov * dt * dt * Eigen::Matrix3f::Identity();

    // std::cout << "wt: " << (w * dt).transpose() << std::endl;
    Sophus::SO3f dR = Sophus::SO3f::exp(w * dt);
    R_ = R_ * dR;
    cov_ = dR.inverse().matrix() * cov_ * dR.matrix() + Q;
  }

  void measureUpdate(const Eigen::Vector3f& a)
  {
    const Eigen::Vector3f g(0, 0, 9.8);
    Eigen::Matrix3f hat_g = Sophus::SO3f::hat(g);
    Eigen::Vector3f e = hat_g * (R_ * a);
    std::cout << "before: " << (R_ * a).transpose() << "   " << g.transpose() << std::endl;

    const Eigen::Matrix3f Q = measure_cov * Eigen::Matrix3f::Identity();
    Eigen::Matrix3f H = hat_g * R_.matrix() * Sophus::SO3f::hat(a);

    Eigen::Matrix3f S = H * cov_ * H.transpose() + Q;
    Eigen::Matrix3f K = cov_ * H.transpose() * S.inverse();
    R_ = R_ * Sophus::SO3f::exp(K * e);
    cov_ = (Eigen::Matrix3f::Identity() - K * H) * cov_;

    std::cout << "after: " << (R_ * a).transpose() << "   " << g.transpose() << std::endl;
    std::cout << "innovation: " << (K * a).transpose() << std::endl;
    // std::cout << cov_ << std::endl;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ImuKalmanFilter>());
  rclcpp::shutdown();
  return 0;
}
