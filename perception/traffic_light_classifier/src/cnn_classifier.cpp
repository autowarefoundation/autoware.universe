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
#include "traffic_light_classifier/cnn_classifier.hpp"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace traffic_light
{
CNNClassifier::CNNClassifier() : nh_(""), pnh_("~"), image_transport_(pnh_)
{
  image_pub_ = image_transport_.advertise("output/debug/image", 1);

  std::string precision;
  std::string label_file_path;
  std::string model_file_path;
  pnh_.param<std::string>("precision", precision, "fp16");
  pnh_.param<std::string>("label_file_path", label_file_path, "labels.txt");
  pnh_.param<std::string>("model_file_path", model_file_path, "model.onnx");
  pnh_.param<int>("input_c", input_c_, 3);
  pnh_.param<int>("input_h", input_h_, 224);
  pnh_.param<int>("input_w", input_w_, 224);

  readLabelfile(label_file_path, labels_);

  std::string cache_dir = ros::package::getPath("traffic_light_classifier") + "/data";
  trt_ = std::make_shared<Tn::TrtCommon>(model_file_path, cache_dir, precision);
  trt_->setup();
}

bool CNNClassifier::getLampState(
  const cv::Mat & input_image, std::vector<autoware_perception_msgs::LampState> & states)
{
  if (!trt_->isInitialized()) {
    ROS_WARN("failed to init tensorrt");
    return false;
  }

  int num_input = trt_->getNumInput();
  int num_output = trt_->getNumOutput();

  float * input_data_host = (float *)malloc(num_input * sizeof(float));

  cv::Mat image = input_image.clone();
  preProcess(image, input_data_host, true);

  float * input_data_device;
  cudaMalloc((void **)&input_data_device, num_input * sizeof(float));
  cudaMemcpy(input_data_device, input_data_host, num_input * sizeof(float), cudaMemcpyHostToDevice);

  float * output_data_device;
  cudaMalloc((void **)&output_data_device, num_output * sizeof(float));

  // do inference
  void * bindings[2];
  bindings[trt_->getInputBindingIndex()] = (void *)input_data_device;
  bindings[trt_->getOutputBindingIndex()] = (void *)output_data_device;

  std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
  trt_->context_->executeV2(bindings);
  std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
  double elapsed_time =
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) /
    1000;
  // ROS_INFO("inference elapsed time: %f [ms]", elapsed_time);

  float * output_data_host = (float *)malloc(num_output * sizeof(float));
  cudaMemcpy(
    output_data_host, output_data_device, num_output * sizeof(float), cudaMemcpyDeviceToHost);

  postProcess(output_data_host, states);

  /* debug */
  if (0 < image_pub_.getNumSubscribers()) {
    cv::Mat debug_image = input_image.clone();
    outputDebugImage(debug_image, states);
  }

  cudaFree(input_data_device);
  cudaFree(output_data_device);

  return true;
}

void CNNClassifier::outputDebugImage(
  cv::Mat & debug_image, const std::vector<autoware_perception_msgs::LampState> & states)
{
  float probability;
  std::string label;
  for (int i=0; i<states.size(); i++) {
    auto state = states.at(i);
    // all lamp confidence are the same
    probability = state.confidence;
    label += state2label_[state.type];
    if (i < states.size() - 1) label += ",";
  }

  int expand_w = 200;
  int expand_h = static_cast<int>((expand_w * debug_image.rows) / debug_image.cols);

  cv::resize(debug_image, debug_image, cv::Size(expand_w, expand_h));
  cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0, 0, 0));
  std::string text = label + " " + std::to_string(probability);
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);

  sensor_msgs::ImagePtr debug_image_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
  image_pub_.publish(debug_image_msg);
}

void CNNClassifier::preProcess(cv::Mat & image, float * input_tensor, bool normalize)
{
  /* normalize */
  /* ((channel[0] / 255) - mean[0]) / std[0] */

  cv::cvtColor(image, image, cv::COLOR_BGR2RGB, 3);
  cv::resize(image, image, cv::Size(input_w_, input_h_));

  const size_t strides_cv[3] = {static_cast<size_t>(input_w_ * input_c_),
                                static_cast<size_t>(input_c_), 1};
  const size_t strides[3] = {static_cast<size_t>(input_h_ * input_w_), 
                             static_cast<size_t>(input_w_), 1};

  for (int i = 0; i < input_h_; i++) {
    for (int j = 0; j < input_w_; j++) {
      for (int k = 0; k < input_c_; k++) {
        const size_t offset_cv = i * strides_cv[0] + j * strides_cv[1] + k * strides_cv[2];
        const size_t offset = k * strides[0] + i * strides[1] + j * strides[2];
        if (normalize) {
          input_tensor[offset] = (((float)image.data[offset_cv] / 255) - mean_[k]) / std_[k];
        } else {
          input_tensor[offset] = (float)image.data[offset_cv];
        }
      }
    }
  }
}

bool CNNClassifier::postProcess(
  float * output_tensor, std::vector<autoware_perception_msgs::LampState> & states)
{
  std::vector<float> probs;
  int num_output = trt_->getNumOutput();
  calcSoftmax(output_tensor, probs, num_output);
  std::vector<size_t> sorted_indices = argsort(output_tensor, num_output);

  // ROS_INFO("label: %s, score: %.2f\%",
  //          labels_[sorted_indices[0]].c_str(),
  //          probs[sorted_indices[0]] * 100);

  std::string match_label = labels_[sorted_indices[0]];
  float probability = probs[sorted_indices[0]];

  // label names are assumed to be comma-separated to represent each lamp
  // e.g.
  // match_label: "left,red,right,straight"
  // splited_label: ["left","red","right","straight"]
  std::vector<std::string> splited_label;
  boost::algorithm::split(splited_label, match_label, boost::is_any_of(","));
  for (auto label : splited_label) {
    if (label2state_.find(label) == label2state_.end()) {
      ROS_DEBUG("cnn_classifier does not have a key [%s]", label);
      continue;
    }
    autoware_perception_msgs::LampState state;
    state.type = label2state_[label];
    state.confidence = probability;
    states.push_back(state);
  }

  return true;
}

bool CNNClassifier::readLabelfile(std::string filepath, std::vector<std::string> & labels)
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

void CNNClassifier::calcSoftmax(float * data, std::vector<float> & probs, int num_output)
{
  float exp_sum = 0.0;
  for (int i = 0; i < num_output; ++i) {
    exp_sum += exp(data[i]);
  }

  for (int i = 0; i < num_output; ++i) {
    probs.push_back(exp(data[i]) / exp_sum);
  }
}

std::vector<size_t> CNNClassifier::argsort(float * tensor, int num_output)
{
  std::vector<size_t> indices(num_output);
  for (int i = 0; i < num_output; i++) indices[i] = i;
  std::sort(indices.begin(), indices.begin() + num_output, [tensor](size_t idx1, size_t idx2) {
    return tensor[idx1] > tensor[idx2];
  });

  return indices;
}

}  // namespace traffic_light
