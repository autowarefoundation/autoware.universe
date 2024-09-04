#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_goal_planner_module/manager.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_planner/shift_pull_over.hpp>
#include <autoware/behavior_path_goal_planner_module/util.hpp>
#include <autoware/behavior_path_planner/behavior_path_planner_node.hpp>
#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_io/Io.h>
#include <matplotlibcpp17/pyplot.h>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

void plot_path_with_lane_id(
  matplotlibcpp17::axes::Axes & axes, const tier4_planning_msgs::msg::PathWithLaneId & path,
  const std::optional<std::string> label)
{
  std::vector<double> xs, ys;
  for (const auto & point : path.points) {
    xs.push_back(point.point.pose.position.x);
    ys.push_back(point.point.pose.position.y);
  }
  if (label) {
    axes.plot(
      Args(xs, ys), Kwargs("color"_a = "red", "linewidth"_a = 1.0, "label"_a = label.value()));
  } else {
    axes.plot(Args(xs, ys), Kwargs("color"_a = "red", "linewidth"_a = 1.0));
  }
}

void plot_lanelet_linestring(
  matplotlibcpp17::axes::Axes & axes, lanelet::ConstLineString3d linestring,
  const std::optional<std::string> label, const std::string color = "blue",
  const double linewidth = 0.5, const std::string linestyle = "solid")
{
  std::vector<double> xs, ys;
  for (const auto & point : linestring) {
    xs.push_back(point.x());
    ys.push_back(point.y());
  }

  if (label) {
    axes.plot(
      Args(xs, ys), Kwargs(
                      "color"_a = color, "linewidth"_a = linewidth, "linestyle"_a = linestyle,
                      "label"_a = label.value()));
  } else {
    axes.plot(
      Args(xs, ys),
      Kwargs("color"_a = color, "linewidth"_a = linewidth, "linestyle"_a = linestyle));
  }
}

void plot_lanelet(
  matplotlibcpp17::axes::Axes & axes, lanelet::ConstLanelet lanelet,
  const std::string color = "black", const double linewidth = 0.5)
{
  plot_lanelet_linestring(axes, lanelet.leftBound(), std::nullopt, color, linewidth);

  plot_lanelet_linestring(axes, lanelet.rightBound(), std::nullopt, color, linewidth);

  plot_lanelet_linestring(axes, lanelet.centerline(), std::nullopt, color, linewidth, "dashed");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto road_shoulder_test_map_path =
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner_common") +
    "/test_map/road_shoulder/lanelet2_map.osm";

  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  const lanelet::LaneletMapPtr lanelet_map_ptr =
    lanelet::load(road_shoulder_test_map_path, projector, &errors);
  if (!errors.empty()) {
    for (const auto & error : errors) {
      std::cout << error << std::endl;
    }
    return 1;
  }

  autoware_planning_msgs::msg::LaneletRoute route_msg;
  route_msg.start_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                           .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                                       .x(530.707103865218)
                                       .y(436.8758309382301)
                                       .z(100.0))
                           .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                          .x(0.0)
                                          .y(0.0)
                                          .z(0.2187392230193602)
                                          .w(0.9757833531644647));
  route_msg.goal_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                          .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                                      .x(571.8018955394537)
                                      .y(435.6819504507543)
                                      .z(100.0))
                          .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                                         .x(0.0)
                                         .y(0.0)
                                         .z(-0.3361155457734493)
                                         .w(0.9418207578352774));

  route_msg.segments = std::vector<autoware_planning_msgs::msg::LaneletSegment>{
    autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletSegment>()
      .preferred_primitive(
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(17757)
          .primitive_type(""))
      .primitives(std::vector<autoware_planning_msgs::msg::LaneletPrimitive>{
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(17756)
          .primitive_type("lane"),
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(17757)
          .primitive_type("lane"),
      }),
    autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletSegment>()
      .preferred_primitive(
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(18494)
          .primitive_type(""))
      .primitives(std::vector<autoware_planning_msgs::msg::LaneletPrimitive>{
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(18494)
          .primitive_type("lane"),
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(18493)
          .primitive_type("lane"),
      }),
    autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletSegment>()
      .preferred_primitive(
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(18496)
          .primitive_type(""))
      .primitives(std::vector<autoware_planning_msgs::msg::LaneletPrimitive>{
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(18496)
          .primitive_type("lane"),
        autoware_planning_msgs::build<autoware_planning_msgs::msg::LaneletPrimitive>()
          .id(18497)
          .primitive_type("lane"),
      }),
  };
  route_msg.allow_modification = false;
  autoware_map_msgs::msg::LaneletMapBin map_bin;
  lanelet::utils::conversion::toBinMsg(
    lanelet_map_ptr, &map_bin);  // TODO(soblin): pass lanelet_map_ptr to RouteHandler

  auto node_options = rclcpp::NodeOptions{};
  node_options.parameter_overrides(
    std::vector<rclcpp::Parameter>{{"launch_modules", std::vector<std::string>{}}});
  node_options.arguments(std::vector<std::string>{
    "--ros-args", "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner") +
      "/config/behavior_path_planner.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner") +
      "/config/drivable_area_expansion.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner") +
      "/config/scene_module_manager.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/config/test_common.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/config/test_nearest_search.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/config/test_vehicle_info.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_goal_planner_module") +
      "/config/goal_planner.param.yaml"});
  auto node = rclcpp::Node::make_shared("plot_map", node_options);

  auto planner_data = std::make_shared<autoware::behavior_path_planner::PlannerData>();
  planner_data->init_parameters(*node);
  planner_data->route_handler->setMap(map_bin);
  planner_data->route_handler->setRoute(route_msg);
  nav_msgs::msg::Odometry odom;
  odom.pose.pose = route_msg.start_pose;
  auto odometry = std::make_shared<const nav_msgs::msg::Odometry>(odom);
  planner_data->self_odometry = odometry;
  lanelet::ConstLanelet current_route_lanelet;
  planner_data->route_handler->getClosestLaneletWithinRoute(
    route_msg.start_pose, &current_route_lanelet);
  std::cout << "current_route_lanelet is " << current_route_lanelet.id() << std::endl;
  auto reference_output =
    autoware::behavior_path_planner::utils::getReferencePath(current_route_lanelet, planner_data);
  auto goal_planner_parameter =
    autoware::behavior_path_planner::GoalPlannerModuleManager::initGoalPlannerParameters(
      node.get(), "goal_planner.");
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();
  autoware::lane_departure_checker::LaneDepartureChecker lane_departure_checker{};
  lane_departure_checker.setVehicleInfo(vehicle_info);
  autoware::lane_departure_checker::Param lane_depature_checker_params;
  lane_depature_checker_params.footprint_extra_margin =
    goal_planner_parameter.lane_departure_check_expansion_margin;
  lane_departure_checker.setParam(lane_depature_checker_params);
  auto shift_pull_over_planner = autoware::behavior_path_planner::ShiftPullOver(
    *node, goal_planner_parameter, lane_departure_checker);
  const auto pull_over_path_opt =
    shift_pull_over_planner.plan(planner_data, reference_output, route_msg.goal_pose);

  const auto current_lanes = autoware::behavior_path_planner::utils::getExtendedCurrentLanes(
    planner_data, goal_planner_parameter.backward_goal_search_length,
    goal_planner_parameter.forward_goal_search_length, false);
  const auto pull_over_lanes =
    autoware::behavior_path_planner::goal_planner_utils::getPullOverLanes(
      *(planner_data->route_handler), true /*left_side_parking */,
      goal_planner_parameter.backward_goal_search_length,
      goal_planner_parameter.forward_goal_search_length);
  const auto drivable_lanes =
    autoware::behavior_path_planner::utils::generateDrivableLanesWithShoulderLanes(
      current_lanes, pull_over_lanes);
  const auto target_drivable_lanes =
    autoware::behavior_path_planner::utils::getNonOverlappingExpandedLanes(
      reference_output.path, drivable_lanes, planner_data->drivable_area_expansion_parameters);

  autoware::behavior_path_planner::DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = target_drivable_lanes;
  current_drivable_area_info = autoware::behavior_path_planner::utils::combineDrivableAreaInfo(
    current_drivable_area_info, reference_output.drivable_area_info);

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, ax] = plt.subplots();

  const std::vector<lanelet::Id> ids{17756, 17757, 18493, 18494, 18496, 18497,
                                     18492, 18495, 18498, 18499, 18500};
  for (const auto & id : ids) {
    const auto lanelet = lanelet_map_ptr->laneletLayer.get(id);
    plot_lanelet(ax, lanelet);
  }

  plot_path_with_lane_id(
    ax, reference_output.path, std::make_optional<std::string>("reference path"));
  std::cout << pull_over_path_opt.has_value() << std::endl;
  if (pull_over_path_opt) {
    const auto & pull_over_path = pull_over_path_opt.value();
    const auto full_path = pull_over_path.getFullPath();
    plot_path_with_lane_id(ax, full_path, std::make_optional<std::string>("shift path"));
  }

  for (const auto & drivable_lane : current_drivable_area_info.drivable_lanes) {
    plot_lanelet_linestring(ax, drivable_lane.left_lane.leftBound(), std::nullopt, "blue", 2.0);
    plot_lanelet_linestring(ax, drivable_lane.right_lane.rightBound(), std::nullopt, "blue", 2.0);
  }

  ax.set_aspect(Args("equal"));
  plt.legend();
  plt.show();

  rclcpp::shutdown();
  return 0;
}
