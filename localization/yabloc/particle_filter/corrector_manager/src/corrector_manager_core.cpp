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

#include "corrector_manager/corrector_manager.hpp"

namespace pcdless
{
CorrectorManager::CorrectorManager() : Node("predictor")
{
  using std::placeholders::_1;
  auto on_gnss_pose = std::bind(&CorrectorManager::on_gnss_pose, this, _1);
  auto on_init_area = std::bind(&CorrectorManager::on_init_area, this, _1);
  sub_init_area_ =
    create_subscription<PointCloud2>("/localization/map/ll2_transition_area", 10, on_init_area);
  sub_gnss_pose_ = create_subscription<PoseStamped>("/sensing/gnss/ublox/pose", 10, on_gnss_pose);

  // Timer
  using namespace std::literals::chrono_literals;
  auto on_timer = std::bind(&CorrectorManager::on_timer, this);
  timer_ = rclcpp::create_timer(this, get_clock(), 1000ms, std::move(on_timer));

  // Service definition
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_ =
    create_client<SetBool>("/switch", rmw_qos_profile_services_default, service_callback_group_);
  while (!client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }
}

void CorrectorManager::on_timer()
{  // TODO:
}

void CorrectorManager::on_init_area(const PointCloud2 & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "initialize pcdless init/deinit areas");
  init_area_ = InitArea(msg);
}

void CorrectorManager::on_gnss_pose(const PoseStamped & msg)
{
  if (!init_area_) return;

  const auto p = msg.pose.position;
  bool init_area;
  if (init_area_->is_inside({p.x, p.y, p.z}, &init_area)) {
    RCLCPP_WARN_STREAM(get_logger(), "Init/Deinit corrector because gnss enters init/deinit area");
    call_service(init_area);
  }
}

void CorrectorManager::call_service(bool data)
{
  using namespace std::chrono_literals;
  auto request = std::make_shared<SetBool::Request>();
  request->data = data;

  while (rclcpp::ok()) {
    auto result_future = client_->async_send_request(request);
    std::future_status status = result_future.wait_for(1000ms);
    if (status == std::future_status::ready) {
      RCLCPP_INFO_STREAM(get_logger(), "service responce is received successfully");
      return;
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "service responce is time out");
    }
  }
}

}  // namespace pcdless