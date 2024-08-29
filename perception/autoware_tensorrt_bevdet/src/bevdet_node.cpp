// Copyright 2024 AutoCore, Inc.
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

// cspell:ignore BEVDET, thre, TRTBEV, bevdet, caminfo, intrin, Ncams, bevfeat, dlongterm, RGBHWC,
// BGRCHW
#include "autoware/tensorrt_bevdet/bevdet_node.hpp"

#include "autoware/tensorrt_bevdet/preprocess.hpp"

using Label = autoware_perception_msgs::msg::ObjectClassification;
std::map<int, std::vector<int>> colormap{
  {0, {0, 0, 255}},  // dodger blue
  {1, {0, 201, 87}}, {2, {0, 201, 87}}, {3, {160, 32, 240}}, {4, {3, 168, 158}}, {5, {255, 0, 0}},
  {6, {255, 97, 0}}, {7, {30, 0, 255}}, {8, {255, 0, 0}},    {9, {0, 0, 255}},   {10, {0, 0, 0}}};

uint8_t getSemanticType(const std::string & class_name)
{
  if (class_name == "car") {
    return Label::CAR;
  } else if (class_name == "truck") {
    return Label::TRUCK;
  } else if (class_name == "bus") {
    return Label::BUS;
  } else if (class_name == "trailer") {
    return Label::TRAILER;
  } else if (class_name == "bicycle") {
    return Label::BICYCLE;
  } else if (class_name == "motorcycle") {
    return Label::MOTORCYCLE;
  } else if (class_name == "pedestrian") {
    return Label::PEDESTRIAN;
  } else {
    return Label::UNKNOWN;
  }
}

void box3DToDetectedObjects(
  const std::vector<Box> & boxes, autoware_perception_msgs::msg::DetectedObjects & bevdet_objects,
  const std::vector<std::string> & class_names, float score_thre, const bool has_twist = true)
{
  for (auto b : boxes) {
    if (b.score < score_thre) continue;
    autoware_perception_msgs::msg::DetectedObject obj;

    Eigen::Vector3f center(b.x, b.y, b.z + b.h / 2.);

    obj.existence_probability = b.score;
    // classification
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.probability = 1.0f;
    if (b.label >= 0 && static_cast<size_t>(b.label) < class_names.size()) {
      classification.label = getSemanticType(class_names[b.label]);
    } else {
      classification.label = Label::UNKNOWN;
    }
    obj.classification.emplace_back(classification);

    // pose and shape
    geometry_msgs::msg::Point p;
    p.x = center.x();
    p.y = center.y();
    p.z = center.z();
    obj.kinematics.pose_with_covariance.pose.position = p;

    tf2::Quaternion q;
    q.setRPY(0, 0, b.r);
    obj.kinematics.pose_with_covariance.pose.orientation = tf2::toMsg(q);

    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

    geometry_msgs::msg::Vector3 v;
    v.x = b.l;
    v.y = b.w;
    v.z = b.h;
    obj.shape.dimensions = v;
    if (has_twist) {
      float vel_x = b.vx;
      float vel_y = b.vy;
      geometry_msgs::msg::Twist twist;
      twist.linear.x = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
      twist.angular.z = 2 * (std::atan2(vel_y, vel_x) - b.r);
      obj.kinematics.twist_with_covariance.twist = twist;
      obj.kinematics.has_twist = has_twist;
    }

    bevdet_objects.objects.emplace_back(obj);
  }
}

void getTransform(
  const geometry_msgs::msg::TransformStamped & transform, Eigen::Quaternion<float> & rot,
  Eigen::Translation3f & translation)
{
  rot = Eigen::Quaternion<float>(
    transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y,
    transform.transform.rotation.z);
  translation = Eigen::Translation3f(
    transform.transform.translation.x, transform.transform.translation.y,
    transform.transform.translation.z);
}

void getCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, Eigen::Matrix3f & intrinsics)
{
  intrinsics << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6],
    msg->k[7], msg->k[8];
}

TRTBEVDetNode::TRTBEVDetNode(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{  // Only start camera info subscription and tf listener at the beginning
  img_N_ = this->declare_parameter<int>("data_params.N", 6);     // camera num 6
  img_w_ = this->declare_parameter<int>("data_params.W", 1600);  // W: 1600
  img_h_ = this->declare_parameter<int>("data_params.H", 900);   // H: 900

  caminfo_received_ = std::vector<bool>(img_N_, false);
  cams_intrin = std::vector<Eigen::Matrix3f>(img_N_);
  cams2ego_rot = std::vector<Eigen::Quaternion<float>>(img_N_);
  cams2ego_trans = std::vector<Eigen::Translation3f>(img_N_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  startCameraInfoSubscription();

  // Wait for camera info and tf transform initialization
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TRTBEVDetNode::checkInitialization, this));
}

void TRTBEVDetNode::initModel()
{
  score_thre_ = this->declare_parameter<float>("post_process_params.score_threshold", 0.2);

  model_config_ = this->declare_parameter("model_config", "bevdet_r50_4dlongterm_depth.yaml");

  onnx_file_ = this->declare_parameter<std::string>("onnx_path", "bevdet_one_lt_d.onnx");
  engine_file_ = this->declare_parameter<std::string>("engine_path", "bevdet_one_lt_d.engine");

  imgs_name_ = this->declare_parameter<std::vector<std::string>>("data_params.cams");
  class_names_ =
    this->declare_parameter<std::vector<std::string>>("post_process_params.class_names");

  RCLCPP_INFO_STREAM(this->get_logger(), "Successful load config!");

  sampleData_.param = camParams(cams_intrin, cams2ego_rot, cams2ego_trans);

  RCLCPP_INFO_STREAM(this->get_logger(), "Successful load image params!");

  bevdet_ = std::make_shared<BEVDet>(
    model_config_, img_N_, sampleData_.param.cams_intrin, sampleData_.param.cams2ego_rot,
    sampleData_.param.cams2ego_trans, onnx_file_, engine_file_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Successful create bevdet!");

  CHECK_CUDA(cudaMalloc(
    reinterpret_cast<void **>(&imgs_dev_), img_N_ * 3 * img_w_ * img_h_ * sizeof(uchar)));

  pub_boxes_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/boxes", rclcpp::QoS{1});
}

void TRTBEVDetNode::checkInitialization()
{
  if (camera_info_received_flag_) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Camera Info and TF Transform Initialization completed!");
    initModel();
    startImageSubscription();
    timer_->cancel();
    timer_.reset();
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for Camera Info and TF Transform Initialization...");
  }
}

void TRTBEVDetNode::startImageSubscription()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  using std::placeholders::_5;
  using std::placeholders::_6;

  sub_f_img_.subscribe(
    this, "~/input/topic_img_f",
    rclcpp::QoS{1}.get_rmw_qos_profile());  // rmw_qos_profile_sensor_data
  sub_b_img_.subscribe(this, "~/input/topic_img_b", rclcpp::QoS{1}.get_rmw_qos_profile());

  sub_fl_img_.subscribe(this, "~/input/topic_img_fl", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_fr_img_.subscribe(this, "~/input/topic_img_fr", rclcpp::QoS{1}.get_rmw_qos_profile());

  sub_bl_img_.subscribe(this, "~/input/topic_img_bl", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_br_img_.subscribe(this, "~/input/topic_img_br", rclcpp::QoS{1}.get_rmw_qos_profile());

  sync_ = std::make_shared<Sync>(
    MySyncPolicy(10), sub_fl_img_, sub_f_img_, sub_fr_img_, sub_bl_img_, sub_b_img_, sub_br_img_);

  sync_->registerCallback(std::bind(&TRTBEVDetNode::callback, this, _1, _2, _3, _4, _5, _6));
}

void TRTBEVDetNode::startCameraInfoSubscription()
{
  // cams: ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK",
  // "CAM_BACK_RIGHT"]
  sub_fl_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_fl/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { camera_info_callback(0, msg); });

  sub_f_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_f/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { camera_info_callback(1, msg); });

  sub_fr_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_fr/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { camera_info_callback(2, msg); });

  sub_bl_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_bl/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { camera_info_callback(3, msg); });

  sub_b_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_b/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { camera_info_callback(4, msg); });

  sub_br_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_br/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { camera_info_callback(5, msg); });
}

void image_transport(std::vector<cv::Mat> imgs, uchar * out_imgs, size_t width, size_t height)
{
  uchar * temp = new uchar[width * height * 3];
  uchar * temp_gpu = nullptr;
  CHECK_CUDA(cudaMalloc(&temp_gpu, width * height * 3));

  for (size_t i = 0; i < imgs.size(); i++) {
    cv::cvtColor(imgs[i], imgs[i], cv::COLOR_BGR2RGB);
    CHECK_CUDA(cudaMemcpy(temp_gpu, imgs[i].data, width * height * 3, cudaMemcpyHostToDevice));
    convert_RGBHWC_to_BGRCHW(temp_gpu, out_imgs + i * width * height * 3, 3, height, width);
  }
  delete[] temp;
  CHECK_CUDA(cudaFree(temp_gpu));
}

void TRTBEVDetNode::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img)
{
  cv::Mat img_fl, img_f, img_fr, img_bl, img_b, img_br;
  std::vector<cv::Mat> imgs;
  img_fl = cv_bridge::toCvShare(msg_fl_img, "bgr8")->image;
  img_f = cv_bridge::toCvShare(msg_f_img, "bgr8")->image;
  img_fr = cv_bridge::toCvShare(msg_fr_img, "bgr8")->image;
  img_bl = cv_bridge::toCvShare(msg_bl_img, "bgr8")->image;
  img_b = cv_bridge::toCvShare(msg_b_img, "bgr8")->image;
  img_br = cv_bridge::toCvShare(msg_br_img, "bgr8")->image;

  imgs.emplace_back(img_fl);
  imgs.emplace_back(img_f);
  imgs.emplace_back(img_fr);
  imgs.emplace_back(img_bl);
  imgs.emplace_back(img_b);
  imgs.emplace_back(img_br);

  image_transport(imgs, imgs_dev_, img_w_, img_h_);

  // uchar *imgs_dev
  sampleData_.imgs_dev = imgs_dev_;

  std::vector<Box> ego_boxes;
  ego_boxes.clear();
  float time = 0.f;

  bevdet_->DoInfer(sampleData_, ego_boxes, time);

  autoware_perception_msgs::msg::DetectedObjects bevdet_objects;
  bevdet_objects.header.frame_id = "base_link";
  bevdet_objects.header.stamp = msg_f_img->header.stamp;

  box3DToDetectedObjects(ego_boxes, bevdet_objects, class_names_, score_thre_);

  pub_boxes_->publish(bevdet_objects);
}

void TRTBEVDetNode::camera_info_callback(int idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (caminfo_received_[idx])
    return;  // already received;  not expected to modify because of we init the model only once

  Eigen::Matrix3f intrinsics;
  getCameraIntrinsics(msg, intrinsics);
  cams_intrin[idx] = intrinsics;

  Eigen::Quaternion<float> rot;
  Eigen::Translation3f translation;
  getTransform(
    tf_buffer_->lookupTransform("base_link", msg->header.frame_id, rclcpp::Time(0)), rot,
    translation);
  cams2ego_rot[idx] = rot;
  cams2ego_trans[idx] = translation;

  caminfo_received_[idx] = true;
  camera_info_received_flag_ =
    std::all_of(caminfo_received_.begin(), caminfo_received_.end(), [](bool i) { return i; });
}

TRTBEVDetNode::~TRTBEVDetNode()
{
  delete imgs_dev_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto bevdet_node = std::make_shared<TRTBEVDetNode>("tensorrt_bevdet_node", node_options);
  rclcpp::spin(bevdet_node);
  return 0;
}
