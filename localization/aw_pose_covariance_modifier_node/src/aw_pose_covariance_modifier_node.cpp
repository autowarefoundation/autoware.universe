// Copyright 2024 The Autoware Foundation
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

#include "include/aw_pose_covariance_modifier_node.hpp"
#include <rclcpp/rclcpp.hpp>


AWPoseCovarianceModifierNode::AWPoseCovarianceModifierNode()
            : Node("AWPoseCovarianceModifierNode")
{
    trusted_pose_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "input_trusted_pose_with_cov_topic",10000,std::bind(&AWPoseCovarianceModifierNode::trusted_pose_with_cov_callback,this,std::placeholders::_1));


    new_pose_estimator_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("output_pose_with_covariance_topic",10);

    client_ = this->create_client<std_srvs::srv::SetBool>("/localization/pose_estimator/covariance_modifier");

    startNDTCovModifier = AWPoseCovarianceModifierNode::callNDTCovarianceModifier();
    if(startNDTCovModifier == 1){
        RCLCPP_INFO(get_logger(), "NDT pose covariance modifier activated ...");

    }

}

bool AWPoseCovarianceModifierNode::callNDTCovarianceModifier(){

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future_result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_result.get();
        RCLCPP_INFO(get_logger(), "Response: %d", response->success);
        return true;
    }
    else {
        RCLCPP_ERROR(get_logger(), "Failed to receive response.");
        return false;
    }
}
void AWPoseCovarianceModifierNode::trusted_pose_with_cov_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped  pose_estimator_pose = *msg;

    trusted_pose_rmse_ = (std::sqrt(pose_estimator_pose.pose.covariance[0]) + std::sqrt(pose_estimator_pose.pose.covariance[7]) ) / 2;
    trusted_pose_yaw_rmse_in_degrees_ = std::sqrt(pose_estimator_pose.pose.covariance[35]) * 180 / M_PI;

    if (trusted_pose_rmse_ > 0.25){
        RCLCPP_INFO(this->get_logger(),"Trusted Pose RMSE is under the threshold. It will not be used as a pose source.");
    }
    else{

        if (trusted_pose_yaw_rmse_in_degrees_ >= 0.3){
            pose_estimator_pose.pose.covariance[35] = 1000000;
        }

        new_pose_estimator_pub_->publish(pose_estimator_pose);

    }

}
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AWPoseCovarianceModifierNode>());
    rclcpp::shutdown();
    return 0;
}