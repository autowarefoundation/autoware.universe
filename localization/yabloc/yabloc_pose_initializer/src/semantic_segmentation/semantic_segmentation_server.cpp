#include "yabloc_pose_initializer/semantic_segmentation/semantic_segmentation_core.hpp"  // Assuming you have translated the core class to C++

#include <rclcpp/rclcpp.hpp>
#include <yabloc_pose_initializer/srv/semantic_segmentation.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <iostream>

const std::string ERROR_MESSAGE =
  R"(The yabloc_pose_initializer is not working correctly because the DNN model has not been downloaded correctly.
To download models, "-DDOWNLOAD_ARTIFACTS=ON" is required at build time.
Please see the README of yabloc_pose_initializer for more information.)";

class SemanticSegmentationServer : public rclcpp::Node
{
public:
  SemanticSegmentationServer() : Node("segmentation_server_node")
  {
    this->declare_parameter<std::string>("model_path", "");
    this->get_parameter("model_path", model_path_);

    RCLCPP_INFO_STREAM(this->get_logger(), "model path: " + model_path_);

    if (std::filesystem::exists(model_path_)) {
      dnn_ = std::make_unique<SemanticSegmentationCore>(model_path_);
    } else {
      dnn_ = nullptr;
      print_error_message();
    }

    srv_ = this->create_service<yabloc_pose_initializer::srv::SemanticSegmentation>(
      "semantic_segmentation_srv", std::bind(
                                     &SemanticSegmentationServer::on_service, this,
                                     std::placeholders::_1, std::placeholders::_2));
  }

private:
  void print_error_message()
  {
    std::istringstream stream(ERROR_MESSAGE);
    std::string line;
    while (std::getline(stream, line)) {
      RCLCPP_ERROR_STREAM(this->get_logger(), line);
    }
  }

  void on_service(
    const std::shared_ptr<yabloc_pose_initializer::srv::SemanticSegmentation::Request> request,
    std::shared_ptr<yabloc_pose_initializer::srv::SemanticSegmentation::Response> response)
  {
    if (dnn_) {
      response->dst_image = inference(request->src_image);
      response->success = true;
    } else {
      print_error_message();
      response->success = false;
      response->dst_image = request->src_image;
    }
  }

  sensor_msgs::msg::Image inference(const sensor_msgs::msg::Image & msg)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Subscribed image: " << std::to_string(msg.header.stamp.sec));
    cv::Mat src_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    cv::Mat mask = dnn_->inference(src_image);
    sensor_msgs::msg::Image dst_msg = *cv_bridge::CvImage(msg.header, "bgr8", mask).toImageMsg();

    return dst_msg;
  }

  std::unique_ptr<SemanticSegmentationCore> dnn_;
  std::string model_path_;
  rclcpp::Service<yabloc_pose_initializer::srv::SemanticSegmentation>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticSegmentationServer>());
  rclcpp::shutdown();
  return 0;
}