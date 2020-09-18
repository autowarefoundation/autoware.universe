/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "traffic_light_ssd_fine_detector/node.hpp"
#include <ros/package.h>
#include "cuda_utils.h"

namespace traffic_light
{
TrafficLightSSDFineDetector::TrafficLightSSDFineDetector()
: nh_(),
  pnh_("~"),
  image_transport_(nh_),
  image_sub_(image_transport_, "input/image", 1),
  roi_sub_(pnh_, "input/rois", 1)
{
  std::string package_path = ros::package::getPath("traffic_light_ssd_fine_detector");
  std::string data_path = package_path + "/data/";
  std::string engine_path = package_path + "/data/mb2-ssd-lite.engine";
  std::ifstream fs(engine_path);
  int max_batch_size;
  std::string onnx_file;
  std::string label_file;
  std::string mode;
  std::vector<std::string> labels;
  pnh_.param<int>("max_batch_size", max_batch_size, 8);
  pnh_.param<std::string>("onnx_file", onnx_file, "");
  pnh_.param<std::string>("label_file", label_file, "");
  pnh_.param<std::string>("mode", mode, "FP32");

  if (readLabelFile(label_file, labels)) {
    if (!getTlrIdFromLabel(labels, tlr_id_)) {
      ROS_ERROR("Could not find tlr id");
    }
  }

  if (fs.is_open()) {
    ROS_INFO("Found %s", engine_path.c_str());
    net_ptr_.reset(new ssd::Net(engine_path, false));
    if (max_batch_size != net_ptr_->getMaxBatchSize()) {
      ROS_INFO(
        "Required max batch size %d does not correspond to Profile max batch size %d. Rebuild "
        "engine "
        "from onnx",
        max_batch_size, net_ptr_->getMaxBatchSize());
      net_ptr_.reset(new ssd::Net(onnx_file, mode, max_batch_size));
      net_ptr_->save(engine_path);
    }
  } else {
    ROS_INFO("Could not find %s, try making TensorRT engine from onnx", engine_path.c_str());
    net_ptr_.reset(new ssd::Net(onnx_file, mode, max_batch_size));
    net_ptr_->save(engine_path);
  }
  pnh_.param<bool>("approximate_sync", is_approximate_sync_, false);
  pnh_.param<double>("score_thresh", score_thresh_, 0.7);
  if (!pnh_.getParam("mean", mean_)) {
    mean_ = {0.5, 0.5, 0.5};
  }
  if (!pnh_.getParam("std", std_)) {
    std_ = {0.5, 0.5, 0.5};
  }
  output_roi_pub_ =
    pnh_.advertise<autoware_perception_msgs::TrafficLightRoiArray>("output/rois", 1);
  exe_time_pub_ = pnh_.advertise<std_msgs::Float32>("debug/exe_time_ms", 1);
  if (is_approximate_sync_) {
    approximate_sync_ =
      std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(1000);
    approximate_sync_->connectInput(image_sub_, roi_sub_);
    approximate_sync_->registerCallback(
      boost::bind(&TrafficLightSSDFineDetector::callback, this, _1, _2));
  } else {
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(1000);
    sync_->connectInput(image_sub_, roi_sub_);
    sync_->registerCallback(boost::bind(&TrafficLightSSDFineDetector::callback, this, _1, _2));
  }

  channel_ = net_ptr_->getInputSize()[0];
  width_ = net_ptr_->getInputSize()[1];
  height_ = net_ptr_->getInputSize()[2];
  detection_per_class_ = net_ptr_->getOutputScoreSize()[0];
  class_num_ = net_ptr_->getOutputScoreSize()[1];
}

TrafficLightSSDFineDetector::~TrafficLightSSDFineDetector() {}

void TrafficLightSSDFineDetector::callback(
  const sensor_msgs::Image::ConstPtr & in_image_msg,
  const autoware_perception_msgs::TrafficLightRoiArray::ConstPtr & in_roi_msg)
{
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;
  const auto exe_start_time = high_resolution_clock::now();
  cv::Mat original_image;
  autoware_perception_msgs::TrafficLightRoiArray out_rois;

  rosMsg2CvMat(in_image_msg, original_image);
  int num_rois = in_roi_msg->rois.size();
  int batch_count = 0;
  const int batch_size = net_ptr_->getMaxBatchSize();
  while (num_rois != 0) {
    const int num_infer = (num_rois / batch_size > 0) ? batch_size : num_rois % batch_size;
    auto data_d = cuda::make_unique<float[]>(num_infer * channel_ * width_ * height_);
    auto scores_d = cuda::make_unique<float[]>(num_infer * detection_per_class_ * class_num_);
    auto boxes_d = cuda::make_unique<float[]>(num_infer * detection_per_class_ * 4);
    std::vector<void *> buffers = {data_d.get(), scores_d.get(), boxes_d.get()};
    std::vector<cv::Point> lts, rbs;
    std::vector<cv::Mat> cropped_imgs;

    for (int i = 0; i < num_infer; ++i) {
      int roi_index = i + batch_count * batch_size;
      lts.push_back(cv::Point(
        in_roi_msg->rois.at(roi_index).roi.x_offset, in_roi_msg->rois.at(roi_index).roi.y_offset));
      rbs.push_back(cv::Point(
        in_roi_msg->rois.at(roi_index).roi.x_offset + in_roi_msg->rois.at(roi_index).roi.width,
        in_roi_msg->rois.at(roi_index).roi.y_offset + in_roi_msg->rois.at(roi_index).roi.height));
      fitInFrame(lts.at(i), rbs.at(i), cv::Size(original_image.size()));
      cropped_imgs.push_back(cv::Mat(original_image, cv::Rect(lts.at(i), rbs.at(i))));
    }

    std::vector<float> data(num_infer * channel_ * width_ * height_);
    if (!cvMat2CnnInput(cropped_imgs, num_infer, data)) {
      ROS_ERROR("Fail to preprocess image");
      return;
    }

    cudaMemcpy(data_d.get(), data.data(), data.size() * sizeof(float), cudaMemcpyHostToDevice);

    try {
      net_ptr_->infer(buffers, num_infer);
    } catch (std::exception & e) {
      ROS_ERROR("%s", e.what());
      return;
    }

    auto scores = std::make_unique<float[]>(num_infer * detection_per_class_ * class_num_);
    auto boxes = std::make_unique<float[]>(num_infer * detection_per_class_ * 4);
    cudaMemcpy(
      scores.get(), scores_d.get(), sizeof(float) * num_infer * detection_per_class_ * class_num_,
      cudaMemcpyDeviceToHost);
    cudaMemcpy(
      boxes.get(), boxes_d.get(), sizeof(float) * num_infer * detection_per_class_ * 4,
      cudaMemcpyDeviceToHost);
    // Get Output
    std::vector<Detection> detections;
    if (!cnnOutput2BoxDetection(
          scores.get(), boxes.get(), tlr_id_, cropped_imgs, num_infer, detections)) {
      ROS_ERROR("Fail to postprocess image");
      return;
    }

    for (int i = 0; i < num_infer; ++i) {
      if (detections.at(i).prob > score_thresh_) {
        cv::Point lt_roi =
          cv::Point(lts.at(i).x + detections.at(i).x, lts.at(i).y + detections.at(i).y);
        cv::Point rb_roi = cv::Point(
          lts.at(i).x + detections.at(i).x + detections.at(i).w,
          lts.at(i).y + detections.at(i).y + detections.at(i).h);
        fitInFrame(lt_roi, rb_roi, cv::Size(original_image.size()));
        autoware_perception_msgs::TrafficLightRoi tl_roi;
        cvRect2TlRoiMsg(
          cv::Rect(lt_roi, rb_roi), in_roi_msg->rois.at(i + batch_count * batch_size).id, tl_roi);
        out_rois.rois.push_back(tl_roi);
      }
    }
    num_rois -= num_infer;
    ++batch_count;
  }
  out_rois.header = in_roi_msg->header;
  output_roi_pub_.publish(out_rois);
  const auto exe_end_time = high_resolution_clock::now();
  const double exe_time =
    std::chrono::duration_cast<milliseconds>(exe_end_time - exe_start_time).count();
  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);
}

bool TrafficLightSSDFineDetector::cvMat2CnnInput(
  const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<float> & data)
{
  for (int i = 0; i < num_rois; ++i) {
    cv::Mat rgb;
    cv::cvtColor(in_imgs.at(i), rgb, CV_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(width_, height_));

    cv::Mat pixels;
    resized.convertTo(pixels, CV_32FC3, 1.0 / 255, 0);
    std::vector<float> img;
    if (pixels.isContinuous()) {
      img.assign((float *)pixels.datastart, (float *)pixels.dataend);
    } else {
      return false;
    }

    for (int c = 0; c < channel_; ++c) {
      for (int j = 0, hw = width_ * height_; j < hw; ++j) {
        data[i * channel_ * width_ * height_ + c * hw + j] =
          (img[channel_ * j + 2 - c] - mean_[c]) / std_[c];
      }
    }
  }
  return true;
}

bool TrafficLightSSDFineDetector::cnnOutput2BoxDetection(
  const float * scores, const float * boxes, const int tlr_id, const std::vector<cv::Mat> & in_imgs,
  const int num_rois, std::vector<Detection> & detections)
{
  if (tlr_id > class_num_ - 1) {
    return false;
  }
  for (int i = 0; i < num_rois; ++i) {
    std::vector<float> tlr_scores;
    Detection det;
    for (int j = 0; j < detection_per_class_; ++j) {
      tlr_scores.push_back(scores[i * detection_per_class_ * class_num_ + tlr_id + j * class_num_]);
    }
    std::vector<float>::iterator iter = std::max_element(tlr_scores.begin(), tlr_scores.end());
    size_t index = std::distance(tlr_scores.begin(), iter);
    size_t box_index = i * detection_per_class_ * 4 + index * 4;
    det.x = boxes[box_index] * in_imgs.at(i).cols;
    det.y = boxes[box_index + 1] * in_imgs.at(i).rows;
    det.w = (boxes[box_index + 2] - boxes[box_index]) * in_imgs.at(i).cols;
    det.h = (boxes[box_index + 3] - boxes[box_index + 1]) * in_imgs.at(i).rows;
    det.prob = tlr_scores[index];
    detections.push_back(det);
  }
  return true;
}

bool TrafficLightSSDFineDetector::rosMsg2CvMat(
  const sensor_msgs::Image::ConstPtr & image_msg, cv::Mat & image)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
    image = cv_image->image;
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR("Failed to convert sensor_msgs::Image to cv::Mat \n%s", e.what());
    return false;
  }

  return true;
}

bool TrafficLightSSDFineDetector::fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size)
{
  try {
    if (rb.x > size.width) rb.x = size.width;
    if (rb.y > size.height) rb.y = size.height;
    if (lt.x < 0) lt.x = 0;
    if (lt.y < 0) lt.y = 0;
  } catch (cv::Exception & e) {
    ROS_ERROR(
      "Failed to fit bounding rect in size [%d, %d] \n%s", size.width, size.height, e.what());
    return false;
  }

  return true;
}

void TrafficLightSSDFineDetector::cvRect2TlRoiMsg(
  const cv::Rect & rect, const int32_t id, autoware_perception_msgs::TrafficLightRoi & tl_roi)
{
  tl_roi.id = id;
  tl_roi.roi.x_offset = rect.x;
  tl_roi.roi.y_offset = rect.y;
  tl_roi.roi.width = rect.width;
  tl_roi.roi.height = rect.height;
}

bool TrafficLightSSDFineDetector::readLabelFile(
  std::string filepath, std::vector<std::string> & labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    ROS_ERROR("Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels.push_back(label);
  }
  return true;
}

bool TrafficLightSSDFineDetector::getTlrIdFromLabel(
  const std::vector<std::string> & labels, int & tlr_id)
{
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels.at(i) == "traffic_light") {
      tlr_id = i;
      return true;
    }
  }
  return false;
}

}  // namespace traffic_light
