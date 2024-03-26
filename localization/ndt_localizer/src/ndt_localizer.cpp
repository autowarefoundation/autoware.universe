#include "ndt_localizer.h"

NdtLocalizer::NdtLocalizer()
{

  key_value_stdmap_["state"] = "Initializing";
  init_params();

  diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
  diagnostic_thread_.detach();
}

NdtLocalizer::~NdtLocalizer() {}

void NdtLocalizer::timer_diagnostic()
{
  const std::chrono::milliseconds rate(100);
  while (true)
  {
    DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto &key_value : key_value_stdmap_)
    {
      KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing")
    {
      diag_status_msg.level = DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
        key_value_stdmap_.count("skipping_publish_num") &&
        std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1)
    {
      diag_status_msg.level = DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
        key_value_stdmap_.count("skipping_publish_num") &&
        std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5)
    {
      diag_status_msg.level = DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    DiagnosticArray diag_msg;
    diag_msg.header.stamp = std::chrono::system_clock::now();
    diag_msg.status.push_back(diag_status_msg);

    std::this_thread::sleep_for(rate);
  }
}

void NdtLocalizer::callback_init_pose(
    std::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> &initial_pose_msg_ptr)
{
  if (initial_pose_msg_ptr->header.frame_id == map_frame_)
  {
    initial_pose_cov_msg_ = *initial_pose_msg_ptr;
  }
  else
  {
    // get TF from pose_frame to map_frame
    std::shared_ptr<geometry_msgs::TransformStamped> TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> mapTF_initial_pose_msg_ptr(new geometry_msgs::PoseWithCovarianceStamped);

    Eigen::Isometry3d eigen_transform;
    eigen_transform.translation() << (*TF_pose_to_map_ptr).transform.translation.x,
        (*TF_pose_to_map_ptr).transform.translation.y,
        (*TF_pose_to_map_ptr).transform.translation.z;
    Eigen::Quaterniond eigen_q((*TF_pose_to_map_ptr).transform.rotation.w,
                               (*TF_pose_to_map_ptr).transform.rotation.x,
                               (*TF_pose_to_map_ptr).transform.rotation.y,
                               (*TF_pose_to_map_ptr).transform.rotation.z);
    Eigen::Matrix3d rotation_matrix = eigen_q.toRotationMatrix();
    eigen_transform.rotate(rotation_matrix);

    Eigen::Matrix<double, 6, 6> covariance;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        covariance(i, j) = (*initial_pose_msg_ptr).pose.covariance[i * 6 + j];
      }
    }
    Eigen::Isometry3d eigen_pose;
    eigen_pose.translation() << (*initial_pose_msg_ptr).pose.pose.position.x,
        (*initial_pose_msg_ptr).pose.pose.position.y,
        (*initial_pose_msg_ptr).pose.pose.position.z;
    Eigen::Quaterniond eigen_p((*initial_pose_msg_ptr).pose.pose.orientation.w,
                               (*initial_pose_msg_ptr).pose.pose.orientation.x,
                               (*initial_pose_msg_ptr).pose.pose.orientation.y,
                               (*initial_pose_msg_ptr).pose.pose.orientation.z);
    Eigen::Matrix3d pose_rotation_matrix = eigen_p.toRotationMatrix();
    eigen_pose.rotate(pose_rotation_matrix);

    Eigen::Isometry3d eigen_map_init_pose;
    eigen_map_init_pose = eigen_transform * eigen_pose;
    Eigen::Vector3d v_map_pose(eigen_map_init_pose.translation());
    Eigen::Quaterniond q_map_pose(eigen_map_init_pose.rotation());
    (*mapTF_initial_pose_msg_ptr).pose.pose.position.x = v_map_pose.x();
    (*mapTF_initial_pose_msg_ptr).pose.pose.position.y = v_map_pose.y();
    (*mapTF_initial_pose_msg_ptr).pose.pose.position.z = v_map_pose.z();
    (*mapTF_initial_pose_msg_ptr).pose.pose.orientation.x = q_map_pose.x();
    (*mapTF_initial_pose_msg_ptr).pose.pose.orientation.y = q_map_pose.y();
    (*mapTF_initial_pose_msg_ptr).pose.pose.orientation.z = q_map_pose.z();
    (*mapTF_initial_pose_msg_ptr).pose.pose.orientation.w = q_map_pose.w();

    Eigen::Matrix3d cov_position(covariance.topLeftCorner<3, 3>());
    Eigen::Matrix3d cov_orientation(covariance.bottomRightCorner<3, 3>());
    Eigen::Matrix3d rotation_transform = eigen_transform.rotation();
    Eigen::Vector3d translation_transform = eigen_transform.translation();
    cov_orientation = rotation_transform.transpose() * cov_orientation * rotation_transform;
    cov_position = rotation_transform * cov_position * rotation_transform.transpose();
    Eigen::Matrix<double, 6, 6> cov_transformed;
    cov_transformed.topLeftCorner<3, 3>() = cov_position;
    cov_transformed.bottomRightCorner<3, 3>() = cov_orientation;
    cov_transformed.topRightCorner<3, 3>() = rotation_transform * covariance.topRightCorner<3, 3>();
    cov_transformed.bottomLeftCorner<3, 3>() = covariance.bottomLeftCorner<3, 3>() * rotation_transform.transpose();

    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        (*mapTF_initial_pose_msg_ptr).pose.covariance[i * 6 + j] = cov_transformed(i, j);
      }
    }
    (*mapTF_initial_pose_msg_ptr).header.frame_id = map_frame_;
    (*mapTF_initial_pose_msg_ptr).header.stamp = (*initial_pose_msg_ptr).header.stamp;
    initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
  }
  // if click the initpose again, re init！
  init_pose = false;
}

void NdtLocalizer::callback_pointsmap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_.getTransformationEpsilon();
  const auto step_size = ndt_.getStepSize();
  const auto resolution = ndt_.getResolution();
  const auto max_iterations = ndt_.getMaximumIterations();

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

  ndt_new.setTransformationEpsilon(trans_epsilon);
  ndt_new.setStepSize(step_size);
  ndt_new.setResolution(resolution);
  ndt_new.setMaximumIterations(max_iterations);

  ndt_new.setInputTarget(map_points_msg_ptr);
  // create Thread
  // detach
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ = ndt_new;
  ndt_map_mtx_.unlock();
}

void NdtLocalizer::callback_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  auto system_time_point = std::chrono::system_clock::from_time_t(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
  std::chrono::time_point<std::chrono::system_clock> sensor_ros_time =
      std::chrono::system_clock::from_time_t(std::chrono::system_clock::to_time_t(system_time_point));
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::nanoseconds(sensor_points_sensorTF_msg_ptr->header.stamp) - std::chrono::duration_cast<std::chrono::nanoseconds>(sensor_ros_time.time_since_epoch()));
  sensor_ros_time += duration;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(*sensor_points_sensorTF_msg_ptr));

  //  get TF base to sensor
  std::shared_ptr<geometry_msgs::TransformStamped> TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

  Eigen::Quaterniond quaterniond(
      (*TF_base_to_sensor_ptr).transform.rotation.w,
      (*TF_base_to_sensor_ptr).transform.rotation.x,
      (*TF_base_to_sensor_ptr).transform.rotation.y,
      (*TF_base_to_sensor_ptr).transform.rotation.z);
  Eigen::Vector3d translation(
      (*TF_base_to_sensor_ptr).transform.translation.x,
      (*TF_base_to_sensor_ptr).transform.translation.y,
      (*TF_base_to_sensor_ptr).transform.translation.z);
  Eigen::Matrix3d r = quaterniond.toRotationMatrix();
  Eigen::Matrix4d t;
  t.block<3, 3>(0, 0) = quaterniond.toRotationMatrix();
  t.block<3, 1>(0, 3) = translation;
  t(3, 3) = 1;
  const Eigen::Affine3d base_to_sensor_affine(t);

  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_baselinkTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud(
      *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);

  // set input point cloud
  ndt_.setInputSource(sensor_points_baselinkTF_ptr);

  if (ndt_.getInputTarget() == nullptr)
  {
    std::cerr << "No MAP!" << std::endl;
    return;
  }
  // align
  Eigen::Matrix4f initial_pose_matrix;
  if (!init_pose)
  {
    Eigen::Affine3d initial_pose_affine;

    Eigen::Quaterniond quat;
    quat.x() = initial_pose_cov_msg_.pose.pose.orientation.x;
    quat.y() = initial_pose_cov_msg_.pose.pose.orientation.y;
    quat.z() = initial_pose_cov_msg_.pose.pose.orientation.z;
    quat.w() = initial_pose_cov_msg_.pose.pose.orientation.w;
    Eigen::Vector3d vec;
    vec << initial_pose_cov_msg_.pose.pose.position.x,
        initial_pose_cov_msg_.pose.pose.position.y,
        initial_pose_cov_msg_.pose.pose.position.z;
    Eigen::Matrix3d R_ = quat.toRotationMatrix();
    Eigen::Matrix4d T_;
    T_.block<3, 3>(0, 0) = R_;
    T_.block<3, 1>(0, 3) = vec;
    T_(3, 3) = 1;
    Eigen::Affine3d temp(T_);
    initial_pose_affine = temp;
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
    // for the first time, we don't know the pre_trans, so just use the init_trans,
    // which means, the delta trans for the second time is 0
    pre_trans = initial_pose_matrix;
    init_pose = true;
  }
  else
  {
    // use predicted pose as init guess (currently we only impl linear model)
    initial_pose_matrix = pre_trans * delta_trans;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_.align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();

  Eigen::Matrix3d temp_rot = result_pose_affine.matrix().block<3, 3>(0, 0);
  const Eigen::Quaterniond rotation_(temp_rot);

  const Eigen::Vector3d translation_ = result_pose_affine.translation();
  geometry_msgs::Pose result_pose_msg;
  result_pose_msg.position.x = translation_.x();
  result_pose_msg.position.y = translation_.y();
  result_pose_msg.position.z = translation_.z();
  result_pose_msg.orientation.x = rotation_.x();
  result_pose_msg.orientation.y = rotation_.y();
  result_pose_msg.orientation.z = rotation_.z();
  result_pose_msg.orientation.w = rotation_.w();

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt_.getTransformationProbability();
  const int iteration_num = ndt_.getFinalNumIteration();
  const auto sorce = ndt_.getFitnessScore();

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (

      iteration_num >= ndt_.getMaximumIterations() + 2 ||
      transform_probability < converged_param_transform_probability_)
  {
    ++skipping_publish_num;
    is_converged = false;
    std::cout << "Not Converged" << std::endl;
  }
  else
  {
    skipping_publish_num = 0;
  }
  // calculate the delta tf from pre_trans to current_trans
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  std::cout << "delta x: " << delta_translation(0) << " y: " << delta_translation(1) << " z: " << delta_translation(2) << std::endl;

  Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2, 1, 0);
  std::cout << "delta yaw: " << delta_euler(0) << " pitch: " << delta_euler(1) << " roll: " << delta_euler(2) << std::endl;

  pre_trans = result_pose_matrix;

  // publish
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
      *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);

  std::chrono::nanoseconds nanoseconds_since_epoch =
      std::chrono::duration_cast<std::chrono::nanoseconds>(sensor_ros_time.time_since_epoch());
  (*sensor_points_mapTF_ptr).header.stamp = static_cast<uint64_t>(nanoseconds_since_epoch.count());
  (*sensor_points_mapTF_ptr).header.frame_id = map_frame_;

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
  std::cout << "sorce: " << sorce << std::endl;
};

void NdtLocalizer::init_params()
{

  std::cout << "base_frame_id: %s", base_frame_.c_str();

  double trans_epsilon = ndt_.getTransformationEpsilon();
  double step_size = ndt_.getStepSize();
  double resolution = ndt_.getResolution();
  int max_iterations = ndt_.getMaximumIterations();

  //  std::ifstream file("../ndt_localizer_config.yaml");
  //  YAML::Node config = YAML::Load(file);
  //  base_frame_ = config["base_frame"].as<std::string>();
  //  trans_epsilon = config["trans_epsilon"].as<double>();
  //  step_size = config["step_size"].as<double>();
  //  resolution = config["resolution"].as<double>();
  //  max_iterations = config["max_iterations"].as<int>();
  base_frame_ = "base_link";
  trans_epsilon = 0.01;
  step_size = 0.4;
  resolution = 2.0;
  max_iterations = 30.0;

  map_frame_ = "map";

  ndt_.setTransformationEpsilon(trans_epsilon);
  ndt_.setStepSize(step_size);
  ndt_.setResolution(resolution);
  ndt_.setMaximumIterations(max_iterations);

  std::cout << "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon,
      step_size, resolution, max_iterations;

  // converged_param_transform_probability_ = config["converged_param_transform_probability"].as<double>();
  converged_param_transform_probability_ = 3;
}

bool NdtLocalizer::get_transform(
    const std::string &target_frame, const std::string &source_frame,
    const std::shared_ptr<geometry_msgs::TransformStamped> &transform_stamped_ptr)
{
  if (target_frame == source_frame)
  {
    transform_stamped_ptr->header.stamp = std::chrono::system_clock::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }
  else
  {
    if (target_frame == base_frame_)
    {
      transform_stamped_ptr->header.stamp = std::chrono::system_clock::now();
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 1.9;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return true;
    }
    else
    {
      transform_stamped_ptr->header.stamp = std::chrono::system_clock::now();
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 0.0;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return true;
    }
  }
}

void NdtLocalizer::publish_tf(
    const std::string &frame_id, const std::string &child_frame_id,
    const geometry_msgs::PoseStamped &pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  Eigen::Quaterniond tf_quaternion;

  tf_quaternion.x() = pose_msg.pose.orientation.x;
  tf_quaternion.y() = pose_msg.pose.orientation.y;
  tf_quaternion.z() = pose_msg.pose.orientation.z;
  tf_quaternion.w() = pose_msg.pose.orientation.w;
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();
}
