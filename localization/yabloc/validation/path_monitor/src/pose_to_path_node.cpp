// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <boost/circular_buffer.hpp>

namespace pcdless::path_monitor
{
class Pose2Path : public rclcpp::Node
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;

  Pose2Path() : Node("pose_to_path"), min_interval_(declare_parameter<double>("min_interval", 0.5))
  {
    declare_parameter("sub_topics", std::vector<std::string>());
    declare_parameter("pub_topics", std::vector<std::string>());
    std::vector<std::string> pub_topics = get_parameter("pub_topics").as_string_array();
    std::vector<std::string> sub_topics = get_parameter("sub_topics").as_string_array();

    if (sub_topics.size() != pub_topics.size()) {
      RCLCPP_FATAL_STREAM(
        this->get_logger(), "pub_topics size does not match with sub_topics size. "
                              << sub_topics.size() << " " << pub_topics.size());
      exit(EXIT_FAILURE);
    }

    const int N = sub_topics.size();
    for (int i = 0; i < N; i++) {
      RCLCPP_INFO_STREAM(
        this->get_logger(), "subscribe: " << sub_topics.at(i) << " publish: " << pub_topics.at(i));
      std::function<void(const PoseStamped &)> func =
        std::bind(&Pose2Path::on_pose, this, std::placeholders::_1, i);
      auto sub = this->create_subscription<PoseStamped>(sub_topics.at(i), 10, func);
      auto pub = this->create_publisher<Path>(pub_topics.at(i), 10);
      pub_sub_msg_.emplace_back(pub, sub);
    }
  }

private:
  const float min_interval_;
  struct PubSubMsg
  {
    PubSubMsg(
      rclcpp::Publisher<Path>::SharedPtr pub_, rclcpp::Subscription<PoseStamped>::SharedPtr sub_)
    : pub_(pub_), sub_(sub_), buffer_(1000)
    {
    }
    rclcpp::Publisher<Path>::SharedPtr pub_;
    rclcpp::Subscription<PoseStamped>::SharedPtr sub_;
    boost::circular_buffer<PoseStamped> buffer_;
    void publish(const std_msgs::msg::Header & header)
    {
      nav_msgs::msg::Path msg;
      msg.header = header;
      for (const auto & p : buffer_) msg.poses.push_back(p);
      pub_->publish(msg);
    }
  };
  std::vector<PubSubMsg> pub_sub_msg_;

  void on_pose(const geometry_msgs::msg::PoseStamped & msg, int index)
  {
    auto & psm = pub_sub_msg_.at(index);
    if (!psm.buffer_.empty()) {
      rclcpp::Time t1(psm.buffer_.back().header.stamp);
      rclcpp::Time t2(msg.header.stamp);
      if (std::abs((t1 - t2).seconds()) < min_interval_) return;
    }
    psm.buffer_.push_back(msg);
    psm.publish(msg.header);
  }
};
}  // namespace pcdless::path_monitor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::path_monitor::Pose2Path>());
  rclcpp::shutdown();
  return 0;
}
