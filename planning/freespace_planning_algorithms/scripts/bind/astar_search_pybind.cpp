#include "freespace_planning_algorithms/astar_search.hpp"
#include "freespace_planning_algorithms/abstract_algorithm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


#include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>
// #include <pybind11/eigen.h>
// #include <pybind11/chrono.h>
// #include <pybind11/complex.h>
// #include <pybind11/functional.h>

using namespace freespace_planning_algorithms;

class AstarSearchPython : public AstarSearch
{
    using AstarSearch::AstarSearch;
    public:
        void setMapByte(const std::string & costmap_byte)
        {
            rclcpp::SerializedMessage serialized_msg;
            static constexpr size_t message_header_length = 8u;
            serialized_msg.reserve(message_header_length + costmap_byte.size());
            serialized_msg.get_rcl_serialized_message().buffer_length = costmap_byte.size();
            for (size_t i = 0; i < costmap_byte.size(); ++i) {
                serialized_msg.get_rcl_serialized_message().buffer[i] = costmap_byte[i];
            }
            nav_msgs::msg::OccupancyGrid costmap;
            static rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;
            serializer.deserialize_message(&serialized_msg, &costmap);

            AstarSearch::setMap(costmap);
        }

        bool makePlanByte(const std::string & start_pose_byte, const std::string & goal_pose_byte)
        {
            rclcpp::SerializedMessage serialized_start_msg;
            static constexpr size_t message_header_length = 8u;
            serialized_start_msg.reserve(message_header_length + start_pose_byte.size());
            serialized_start_msg.get_rcl_serialized_message().buffer_length = start_pose_byte.size();
            for (size_t i = 0; i < start_pose_byte.size(); ++i) {
                serialized_start_msg.get_rcl_serialized_message().buffer[i] = start_pose_byte[i];
            }
            geometry_msgs::msg::Pose start_pose;
            static rclcpp::Serialization<geometry_msgs::msg::Pose> start_serializer;
            start_serializer.deserialize_message(&serialized_start_msg, &start_pose);

            rclcpp::SerializedMessage serialized_goal_msg;
            serialized_goal_msg.reserve(message_header_length + goal_pose_byte.size());
            serialized_goal_msg.get_rcl_serialized_message().buffer_length = goal_pose_byte.size();
            for (size_t i = 0; i < goal_pose_byte.size(); ++i) {
                serialized_goal_msg.get_rcl_serialized_message().buffer[i] = goal_pose_byte[i];
            }
            geometry_msgs::msg::Pose goal_pose;
            static rclcpp::Serialization<geometry_msgs::msg::Pose> goal_serializer;
            goal_serializer.deserialize_message(&serialized_goal_msg, &goal_pose);

            return AstarSearch::makePlan(start_pose, goal_pose);
        }
};


namespace py = pybind11;

PYBIND11_MODULE(freespace_planning_algorithms_python, p)
{
    auto pyAstarParam = py::class_<AstarParam>(p, "AstarParam", py::dynamic_attr())
        .def(py::init<>())
        .def_readwrite("only_behind_solutions", &AstarParam::only_behind_solutions)
        .def_readwrite("use_back", &AstarParam::use_back)
        .def_readwrite("distance_heuristic_weight", &AstarParam::distance_heuristic_weight);
    auto pyPlannerCommonParam = py::class_<PlannerCommonParam>(p, "PlannerCommonParam", py::dynamic_attr())
        .def(py::init<>())
        .def_readwrite("time_limit", &PlannerCommonParam::time_limit)
        .def_readwrite("minimum_turning_radius", &PlannerCommonParam::minimum_turning_radius)
        .def_readwrite("maximum_turning_radius", &PlannerCommonParam::maximum_turning_radius)
        .def_readwrite("turning_radius_size", &PlannerCommonParam::turning_radius_size)
        .def_readwrite("theta_size", &PlannerCommonParam::theta_size)
        .def_readwrite("curve_weight", &PlannerCommonParam::curve_weight)
        .def_readwrite("reverse_weight", &PlannerCommonParam::reverse_weight)
        .def_readwrite("lateral_goal_range", &PlannerCommonParam::lateral_goal_range)
        .def_readwrite("longitudinal_goal_range", &PlannerCommonParam::longitudinal_goal_range)
        .def_readwrite("angle_goal_range", &PlannerCommonParam::angle_goal_range)
        .def_readwrite("obstacle_threshold", &PlannerCommonParam::obstacle_threshold);
    auto pyVSehicleShape = py::class_<VehicleShape>(p, "VehicleShape", py::dynamic_attr())
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def_readwrite("length", &VehicleShape::length)
        .def_readwrite("width", &VehicleShape::width)
        .def_readwrite("base2back", &VehicleShape::base2back);

    py::class_<AbstractPlanningAlgorithm>(p, "AbstractPlanningAlgorithm");
    py::class_<AstarSearch, AbstractPlanningAlgorithm>(p, "AstarSearchCpp");
    py::class_<AstarSearchPython, AstarSearch>(p, "AstarSearch")
        .def(py::init<PlannerCommonParam &, VehicleShape &, AstarParam &>())
        .def("setMap", &AstarSearchPython::setMapByte)
        .def("makePlan", &AstarSearchPython::makePlanByte);
}
