#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_srvs/srv/empty.hpp>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

class PlannerDataServer : public rclcpp::Node
{
public:
  PlannerDataServer() : Node("planner_data_server")
  {
    server_ = this->create_service<std_srvs::srv::Empty>(
      "/planning/planner_data_server",
      std::bind(
        &PlannerDataServer::on_service, this, std::placeholders::_1, std::placeholders::_2));
    planner_data_.route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
  }

private:
  void take_data()
  {
    {
      const auto msg = route_subscriber_.takeData();
      if (msg) {
        if (msg->segments.empty()) {
          RCLCPP_ERROR(this->get_logger(), "input route is empty, ignore");
        }
        route_ptr_ = msg;
      }
    }
    autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr map_ptr = nullptr;
    {
      const auto msg = vector_map_subscriber_.takeData();
      if (msg) {
        map_ptr = msg;
      }
    }
    // handle route_handler
    if (map_ptr) {
      planner_data_.route_handler->setMap(*map_ptr);
    }
    if (route_ptr_) {
      planner_data_.route_handler->setRoute(*route_ptr_);
    }

    // velocity
    {
      const auto msg = velocity_subscriber_.takeData();
      if (msg) {
        planner_data_.self_odometry = msg;
      }
    }
    // acceleration
    {
      const auto msg = acceleration_subscriber_.takeData();
      if (msg) {
        planner_data_.self_acceleration = msg;
      }
    }
    // perception
    {
      const auto msg = perception_subscriber_.takeData();
      if (msg) {
        planner_data_.dynamic_object = msg;
      }
    }
    // occupancy_grid
    {
      const auto msg = occupancy_grid_subscriber_.takeData();
      if (msg) {
        planner_data_.occupancy_grid = msg;
      }
    }
    // costmap
    {
      const auto msg = costmap_subscriber_.takeData();
      if (msg) {
        planner_data_.costmap = msg;
      }
    }
    // traffic_signal
    {
      const auto msg = traffic_signals_subscriber_.takeData();
      if (msg) {
        planner_data_.traffic_light_id_map.clear();
        for (const auto & signal : msg->traffic_light_groups) {
          autoware::behavior_path_planner::TrafficSignalStamped traffic_signal;
          traffic_signal.stamp = msg->stamp;
          traffic_signal.signal = signal;
          planner_data_.traffic_light_id_map[signal.traffic_light_group_id] = traffic_signal;
        }
      }
    }
    // lateral_offset
    {
      const auto msg = lateral_offset_subscriber_.takeData();
      if (msg) {
        if (!planner_data_.lateral_offset) {
          planner_data_.lateral_offset = msg;
          return;
        }

        const auto & new_offset = msg->lateral_offset;
        const auto & old_offset = planner_data_.lateral_offset->lateral_offset;

        // offset is not changed.
        if (std::abs(old_offset - new_offset) < 1e-4) {
          return;
        }

        planner_data_.lateral_offset = msg;
      }
    }
    // operation_mode
    {
      const auto msg = operation_mode_subscriber_.takeData();
      if (msg) {
        planner_data_.operation_mode = msg;
      }
    }
    /*
    // external_velocity_limiter
    {
      const auto msg = external_limit_max_velocity_subscriber_.takeData();
      if (msg) {
        planner_data_.external_limit_max_velocity = msg;
      }
    }
    */
  }

  autoware::behavior_path_planner::PlannerData planner_data_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;

  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_ptr_;

  // subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware::universe_utils::polling_policy::Newest>
    route_subscriber_{this, "/planning/mission_planning/route", rclcpp::QoS{1}.transient_local()};

  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_map_msgs::msg::LaneletMapBin, autoware::universe_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "/map/vector_map", rclcpp::QoS{1}.transient_local()};

  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>
    velocity_subscriber_{this, "/localization/kinematic_state"};

  autoware::universe_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "/localization/acceleration"};

  /*
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::Scenario>
    scenario_subscriber_{this, "/planning/scenario_planning/scenario"};
  */

  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::PredictedObjects>
    perception_subscriber_{this, "/perception/object_recognition/objects"};

  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::OccupancyGrid>
    occupancy_grid_subscriber_{this, "/perception/occupancy_grid_map/map"};

  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::OccupancyGrid>
    costmap_subscriber_{
      this, "/planning/scenario_planning/parking/costmap_generator/occupancy_grid"};

  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    traffic_signals_subscriber_{this, "/perception/traffic_light_recognition/traffic_signals"};

  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::LateralOffset>
    lateral_offset_subscriber_{
      this,
      "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/"
      "lateral_offset"};

  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>
    operation_mode_subscriber_{
      this, "/system/operation_mode/state", rclcpp::QoS{1}.transient_local()};

  /*
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::VelocityLimit>
    external_limit_max_velocity_subscriber_{this, "/planning/scenario_planning/max_velocity"};
  */

  void on_service(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Received planner_data save request");
    take_data();

    YAML::Node planner_data;
    if (route_ptr_) {
      planner_data["route"] = YAML::Load(autoware_planning_msgs::msg::to_yaml(*route_ptr_));
    }
    if (planner_data_.self_odometry) {
      planner_data["self_odometry"] =
        YAML::Load(nav_msgs::msg::to_yaml(*planner_data_.self_odometry));
    }
    if (planner_data_.self_acceleration) {
      planner_data["self_acceleration"] =
        YAML::Load(geometry_msgs::msg::to_yaml(*planner_data_.self_acceleration));
    }
    if (planner_data_.dynamic_object) {
      planner_data["dynamic_object"] =
        YAML::Load(autoware_perception_msgs::msg::to_yaml(*planner_data_.dynamic_object));
    }
    if (planner_data_.lateral_offset) {
      planner_data["lateral_offset"] =
        YAML::Load(tier4_planning_msgs::msg::to_yaml(*planner_data_.lateral_offset));
    }
    if (planner_data_.operation_mode) {
      planner_data["operation_mode"] =
        YAML::Load(autoware_adapi_v1_msgs::msg::to_yaml(*planner_data_.operation_mode));
    }
    for (const auto & [id, traffic_signal] : planner_data_.traffic_light_id_map) {
      planner_data["traffic_light_id_map"][std::to_string(id)]["stamp"] =
        YAML::Load(builtin_interfaces::msg::to_yaml(traffic_signal.stamp));
      planner_data["traffic_light_id_map"][std::to_string(id)]["signal"] =
        YAML::Load(autoware_perception_msgs::msg::to_yaml(traffic_signal.signal));
    }

    std::ofstream ofs("planner_data.yaml");
    ofs << planner_data;

    RCLCPP_INFO(this->get_logger(), "saved planner_data");
  };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto node = std::make_shared<PlannerDataServer>();

  exec.add_node(node);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
