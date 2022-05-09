#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class Pose2Path : public rclcpp::Node
{
public:
  Pose2Path() : Node("pose_to_path")
  {
    declare_parameter("sub_topics", std::vector<std::string>());
    declare_parameter("pub_topics", std::vector<std::string>());
    std::vector<std::string> pub_topics = this->get_parameter("pub_topics").as_string_array();
    std::vector<std::string> sub_topics = this->get_parameter("sub_topics").as_string_array();


    if (sub_topics.size() != pub_topics.size()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "pub_topics size does not match with sub_topics size. " << sub_topics.size() << " " << pub_topics.size());
      exit(EXIT_FAILURE);
    }

    const int N = sub_topics.size();
    for (int i = 0; i < N; i++) {
      RCLCPP_INFO_STREAM(this->get_logger(), "subscribe: " << sub_topics.at(i) << " publish: " << pub_topics.at(i));
      std::function<void(const geometry_msgs::msg::PoseStamped&)> func = std::bind(&Pose2Path::poseCallback, this, std::placeholders::_1, i);
      auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(sub_topics.at(i), 10, func);
      auto pub = this->create_publisher<nav_msgs::msg::Path>(pub_topics.at(i), 10);
      pub_sub_msg_.emplace_back(pub, sub, nav_msgs::msg::Path{});
    }
  }


private:
  struct PubSubMsg {
    PubSubMsg(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_,
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_,
        nav_msgs::msg::Path msg_) : pub_(pub_), sub_(sub_), msg_(msg_) {}

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    nav_msgs::msg::Path msg_;
    void publish() { pub_->publish(msg_); }
  };
  std::vector<PubSubMsg> pub_sub_msg_;

  void poseCallback(const geometry_msgs::msg::PoseStamped& msg, int index)
  {
    auto& psm = pub_sub_msg_.at(index);
    psm.msg_.header = msg.header;
    psm.msg_.poses.push_back(msg);
    psm.publish();
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);


  rclcpp::spin(std::make_shared<Pose2Path>());
  rclcpp::shutdown();
  return 0;
}
