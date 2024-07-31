// Copyright 2024 TIER IV, Inc.
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
#include "bevdet_node.hpp"

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

void Getinfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
    printf("  Shared memory in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf(
      "  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1],
      prop.maxThreadsDim[2]);
    printf(
      "  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
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

TRTBEVDetNode::TRTBEVDetNode(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  using std::placeholders::_5;
  using std::placeholders::_6;
  using std::placeholders::_7;

  score_thre_ = this->declare_parameter<float>("post_process_params.score_thre", 0.2);

  img_N_ = this->declare_parameter<int>("data_params.N", 6);     // camera num 6
  img_w_ = this->declare_parameter<int>("data_params.W", 1600);  // W: 1600
  img_h_ = this->declare_parameter<int>("data_params.H", 900);   // H: 900

  model_config_ = this->declare_parameter("model_config", "bevdet_r50_4dlongterm_depth.yaml");

  onnx_file_ = this->declare_parameter("onnx_file", "bevdet_one_lt_d.onnx");
  engine_file_ = this->declare_parameter("engine_file", "bevdet_one_lt_d.engine");

  camconfig_ = YAML::LoadFile(this->declare_parameter("cam_config", "cams_param.yaml"));

  imgs_name_ = this->declare_parameter<std::vector<std::string>>("data_params.cams");
  class_names_ =
    this->declare_parameter<std::vector<std::string>>("post_process_params.class_names");

  RCLCPP_INFO_STREAM(this->get_logger(), "Successful load config!");

  sampleData_.param = camParams(camconfig_, img_N_, imgs_name_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Successful load image params!");

  bevdet_ = std::make_shared<BEVDet>(
    model_config_, img_N_, sampleData_.param.cams_intrin, sampleData_.param.cams2ego_rot,
    sampleData_.param.cams2ego_trans, onnx_file_, engine_file_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Successful create bevdet!");

  CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&imgs_dev_), img_N_ * 3 * img_w_ * img_h_ * sizeof(uchar)));

  pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/painting_cloud", rclcpp::SensorDataQoS());
  pub_boxes_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/boxes", rclcpp::QoS{1});

  sub_cloud_.subscribe(
    this, "~/input/topic_cloud",
    rclcpp::QoS{1}.get_rmw_qos_profile());  // rclcpp::QoS{1}.get_rmw_qos_profile()
  sub_f_img_.subscribe(
    this, "~/input/topic_img_f",
    rclcpp::QoS{1}.get_rmw_qos_profile());  // rmw_qos_profile_sensor_data
  sub_b_img_.subscribe(this, "~/input/topic_img_b", rclcpp::QoS{1}.get_rmw_qos_profile());

  sub_fl_img_.subscribe(this, "~/input/topic_img_fl", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_fr_img_.subscribe(this, "~/input/topic_img_fr", rclcpp::QoS{1}.get_rmw_qos_profile());

  sub_bl_img_.subscribe(this, "~/input/topic_img_bl", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_br_img_.subscribe(this, "~/input/topic_img_br", rclcpp::QoS{1}.get_rmw_qos_profile());

  sync_ = std::make_shared<Sync>(
    MySyncPolicy(10), sub_cloud_, sub_fl_img_, sub_f_img_, sub_fr_img_, sub_bl_img_, sub_b_img_,
    sub_br_img_);

  sync_->registerCallback(
    std::bind(&TRTBEVDetNode::callback, this, _1, _2, _3, _4, _5, _6, _7));  // 绑定回调函数
}

void TRTBEVDetNode::callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_cloud,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  pcl::fromROSMsg(*msg_cloud, *cloud);

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

  std::vector<std::vector<char>> imgs_data;
  // Mat -> Vetor<char>
  cvImgToArr(imgs, imgs_data);

  // cpu imgs_data -> gpu imgs_dev
  decode_cpu(imgs_data, imgs_dev_, img_w_, img_h_);

  // uchar *imgs_dev
  sampleData_.imgs_dev = imgs_dev_;

  std::vector<Box> ego_boxes;
  ego_boxes.clear();
  float time = 0.f;

  bevdet_->DoInfer(sampleData_, ego_boxes, time);

  autoware_perception_msgs::msg::DetectedObjects bevdet_objects;
  bevdet_objects.header.frame_id = "base_link";
  bevdet_objects.header.stamp = msg_cloud->header.stamp;

  box3DToDetectedObjects(ego_boxes, bevdet_objects, class_names_, score_thre_);

  pub_boxes_->publish(bevdet_objects);

  sensor_msgs::msg::PointCloud2 msg_cloud_painting;
  pcl::toROSMsg(*cloud, msg_cloud_painting);

  msg_cloud_painting.header.frame_id = msg_cloud->header.frame_id;
  msg_cloud_painting.header.stamp = msg_cloud->header.stamp;
  pub_cloud_->publish(msg_cloud_painting);
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
