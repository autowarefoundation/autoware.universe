#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

#include <lsd/lsd.hpp>
#include <opencv4/opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
  cv::Mat image = cv::imread("image.jpg");
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd = cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_ADV);
  cv::Mat lines;
  lsd->detect(image, lines);
  lsd->drawSegments(image, lines);
  cv::imshow("show", image);
  cv::waitKey(0);
  cv::destroyAllWindows();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

  return 0;
}