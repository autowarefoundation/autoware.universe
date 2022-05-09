#pragma once

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <lsd/lsd.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <optional>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/core/eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>

class LineSegmentDetector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LineSegmentDetector(const std::string& image_topic, const std::string& info_topic) : Node("line_detector"), info_(std::nullopt)
  {
    sub_image_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(image_topic, 10, std::bind(&LineSegmentDetector::imageCallback, this, std::placeholders::_1));
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 10, std::bind(&LineSegmentDetector::infoCallback, this, std::placeholders::_1));
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/projected", 10);
    pub_image_lsd_ = this->create_publisher<sensor_msgs::msg::Image>("/lsd", 10);
    lsd = cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_ADV);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string config_path = "/";
    this->declare_parameter<std::string>("config_path", config_path);
    this->get_parameter("config_path", config_path);
    if (config_path != "/") {
      YAML::Node node = YAML::LoadFile(config_path)["vmvl"];
      image_size_ = node["image_size"].as<int>();
      max_range_ = node["max_range"].as<float>();
    } else {
      image_size_ = 800;
      max_range_ = 20;
    }
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};

  cv::Ptr<cv::lsd::LineSegmentDetector> lsd;
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_, pub_image_lsd_;

  int image_size_;
  float max_range_;

  void projectEdgeOnPlane(const cv::Mat& lines, const cv::Mat& K_cv, const rclcpp::Time& stamp) const
  {
    if (!camera_extrinsic_.has_value()) {
      RCLCPP_WARN_STREAM(this->get_logger(), "camera_extrinsic_ has not been initialized");
      return;
    }
    Eigen::Vector3f t = camera_extrinsic_->translation();
    Eigen::Quaternionf q(camera_extrinsic_->rotation());
    RCLCPP_INFO_STREAM(this->get_logger(), "transform: " << t.transpose() << " " << q.coeffs().transpose());

    struct Edge {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Edge(const Eigen::Vector3f& p, const Eigen::Vector3f& q) : p(p), q(q) {}
      Eigen::Vector3f p, q;
    };

    // Convert to projected coordinate
    Eigen::Matrix3f K, K_inv;
    cv::cv2eigen(K_cv, K);
    K_inv = K.inverse();

    // NOTE: reference capture is better?
    auto conv = [t, q, K_inv](const Eigen::Vector2f& u) -> Eigen::Vector3f {
      Eigen::Vector3f u3(u.x(), u.y(), 1);
      Eigen::Vector3f bearing = (q * K_inv * u3).normalized();
      if (bearing.z() > -0.1) return Eigen::Vector3f::Zero();

      float l = -t.z() / bearing.z();
      float projected_x = t.x() + bearing.x() * l;
      float projected_y = t.y() + bearing.y() * l;
      Eigen::Vector3f tmp(projected_x, projected_y, 0);
      return tmp;
    };

    std::vector<Edge> edges;
    const int N = lines.rows;
    for (int i = 0; i < N; i++) {
      cv::Mat xyxy = lines.row(i);
      Eigen::Vector2f xy1, xy2;
      xy1 << xyxy.at<float>(0), xyxy.at<float>(1);
      xy2 << xyxy.at<float>(2), xyxy.at<float>(3);
      edges.emplace_back(conv(xy1), conv(xy2));
    }

    // Draw projected edge image
    cv::Mat image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
    {
      const cv::Size center(image.cols / 2, image.rows / 2);
      auto toCvPoint = [center, this](const Eigen::Vector3f& v) -> cv::Point {
        cv::Point pt;
        pt.x = -v.y() / this->max_range_ * center.width + center.width;
        pt.y = -v.x() / this->max_range_ * center.height + 2 * center.height;
        return pt;
      };
      for (const auto e : edges) {
        if (e.p.isZero() || e.q.isZero()) continue;
        cv::line(image, toCvPoint(e.p), toCvPoint(e.q), cv::Scalar(0, 255, 255), 2, cv::LineTypes::LINE_8);
      }
    }

    // Publish image
    {
      cv_bridge::CvImage raw_image;
      raw_image.header.stamp = stamp;
      raw_image.header.frame_id = "map";
      raw_image.encoding = "bgr8";
      raw_image.image = image;
      pub_image_->publish(*raw_image.toImageMsg());
    }
  }

  void listenExtrinsicTf(const std::string& frame_id)
  {
    try {
      geometry_msgs::msg::TransformStamped ts = tf_buffer_->lookupTransform("base_link", frame_id, tf2::TimePointZero);
      Eigen::Vector3f p;
      p.x() = ts.transform.translation.x;
      p.y() = ts.transform.translation.y;
      p.z() = ts.transform.translation.z;

      Eigen::Quaternionf q;
      q.w() = ts.transform.rotation.w;
      q.x() = ts.transform.rotation.x;
      q.y() = ts.transform.rotation.y;
      q.z() = ts.transform.rotation.z;
      camera_extrinsic_ = Eigen::Affine3f::Identity();
      camera_extrinsic_->translation() = p;
      camera_extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
    } catch (tf2::TransformException& ex) {
    }
  }

  void infoCallback(const sensor_msgs::msg::CameraInfo& msg)
  {
    info_ = msg;
    listenExtrinsicTf(info_->header.frame_id);
  }

  void imageCallback(const sensor_msgs::msg::CompressedImage& msg) const
  {
    sensor_msgs::msg::Image::ConstSharedPtr image_ptr = decompressImage(msg);
    cv::Mat image = cv_bridge::toCvCopy(*image_ptr, "rgb8")->image;
    cv::Size size = image.size();

    if (!info_.has_value()) return;
    cv::Mat K = cv::Mat(cv::Size(3, 3), CV_64FC1, (void*)(info_->k.data()));
    cv::Mat D = cv::Mat(cv::Size(5, 1), CV_64FC1, (void*)(info_->d.data()));
    cv::Mat undistorted;
    cv::undistort(image, undistorted, K, D, K);
    image = undistorted;


    const int WIDTH = 800;
    const float SCALE = 1.0f * WIDTH / size.width;
    int HEIGHT = SCALE * size.height;
    cv::resize(image, image, cv::Size(WIDTH, HEIGHT));
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    std::chrono::time_point start = std::chrono::system_clock::now();
    cv::Mat lines;
    lsd->detect(image, lines);
    lsd->drawSegments(image, lines);

    auto dur = std::chrono::system_clock::now() - start;
    long ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    RCLCPP_INFO_STREAM(this->get_logger(), cv::Size(WIDTH, HEIGHT) << " " << ms);

    {
      cv_bridge::CvImage raw_image;
      raw_image.header.stamp = msg.header.stamp;
      raw_image.header.frame_id = "map";
      raw_image.encoding = "bgr8";
      raw_image.image = image;
      pub_image_lsd_->publish(*raw_image.toImageMsg());
    }

    cv::Mat scaled_K = SCALE * K;
    scaled_K.at<double>(2, 2) = 1;
    projectEdgeOnPlane(lines, scaled_K, msg.header.stamp);
  }


  sensor_msgs::msg::Image::ConstSharedPtr decompressImage(const sensor_msgs::msg::CompressedImage& compressed_img) const
  {
    cv_bridge::CvImage raw_image;
    raw_image.header = compressed_img.header;

    const std::string& format = compressed_img.format;
    const std::string encoding = format.substr(0, format.find(";"));
    raw_image.encoding = encoding;

    constexpr int DECODE_GRAY = 0;
    constexpr int DECODE_RGB = 1;

    bool encoding_is_bayer = encoding.find("bayer") != std::string::npos;
    if (encoding_is_bayer) {
      raw_image.image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_GRAY);
      if (encoding == "bayer_rggb8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerBG2BGR);
      else if (encoding == "bayer_bggr8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerRG2BGR);
      else if (encoding == "bayer_grbg8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerGB2BGR);
      else if (encoding == "bayer_gbrg8")
        cv::cvtColor(raw_image.image, raw_image.image, cv::COLOR_BayerGR2BGR);
      else {
        std::cerr << encoding << " is not supported encoding" << std::endl;
        std::cerr << "Please implement additional decoding in " << __FUNCTION__ << std::endl;
        exit(4);
      }
      raw_image.encoding = "bgr8";
      return raw_image.toImageMsg();
    }

    raw_image.image = cv::imdecode(cv::Mat(compressed_img.data), DECODE_RGB);
    return raw_image.toImageMsg();
  }
};