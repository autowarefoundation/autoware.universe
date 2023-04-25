# Planning Interface Test Manager

## Background

planningモジュールの各ノードにおいて、特殊な経路や道路から大きく外れた自己位置など(今後特殊な入力と呼ぶ)が、入力として与えられたとき、そのノードが特殊な入力を想定しておらず、してしまうことが多く、ノードクラッシュによって生じるデバッグに時間を要することがある。例えば、空のtrajectoryなどをinputとして入ってくることは実装時に想定しておらず、対策が取られないまま変更がmergeされ、シナリオテストの実行時や実車でシステムを動作している最中にノードがクラッシュしてしまう場合がある。

## Purpose

各ノードが例外的な入力を受け取った際にノードが正常に動作することを確認するためのテストを実装するためのユーティリティを提供する。このユーティリティを活用し、特殊な入力に対するテストを実装することで、PRのmerge前に特殊な入力に対する対策が必要になり、実際に動かした際に初めて発覚するバグを減らすことを目的とする。

## Features

### 正常動作の確認

テスト対象のノードに対して、そのノードが正常に動作し、後段のノードで必要なメッセージをpublishすることを確認します。そのために、test_nodeから必要なメッセージをpublishし、ノードのoutputがpublishされていることを確認します。

### 特殊な入力に対する堅牢性の確認

正常動作が確認できた後、テスト対象のノードに対して、特殊な入力を与えた際にノードがクラッシュしないことを確認します。そのために、test_nodeから特殊な入力を与え、ノードがクラッシュしないことを確認します。

## Flowchart

(WIP)

## Usage

```cpp

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  // instantiate test_manager with PlanningInterfaceTestManager type
  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  // get package directories for necessary configuration files
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto target_node_dir =
    ament_index_cpp::get_package_share_directory("target_node");

  // set arguments to get the config file
  node_options.arguments(
    {"--ros-args", "--params-file",
     planning_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     planning_validator_dir + "/config/planning_validator.param.yaml"});

  // instantiate the TargetNode with node_options
  auto test_target_node = std::make_shared<TargetNode>(node_options);

  // publish the necessary topics from test_manager second argument is topic name
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "motion_velocity_smoother/input/external_velocity_limit_mps");

  // set scenario_selector's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("input/parking/trajectory");

  // test with normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  // make sure target_node is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with trajectory input with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(test_target_node));

  // shutdown ROS context
  rclcpp::shutdown();
}


## Implemented tests

| Node name                                  | Test name                                      | exception input     | output                                                                                                     | Exceptional trajectory/route/path_with_lane_id pattern | Current test result       |
| ------------------------------------------ | ---------------------------------------------- | ------------------- | ---------------------------------------------------------------------------------------------------------- | ------------------------------------------------------ | ------------------------- |
| planning_validator                         | NodeTestWithExceptionTrajectory                | trajectory          | trajectory                                                                                                 | Empty, single point, path with duplicate points        |                           |
| motion_velocity_smoother                   | NodeTestWithExceptionTrajectory                | trajectory          | trajectory                                                                                                 | Empty, single point, path with duplicate points        | Commented out due to fail |
| obstacle_cruise_planner                    | NodeTestWithExceptionTrajectory                | trajectory          | trajectory                                                                                                 | Empty, single point, path with duplicate points        |                           |
| obstacle_stop_planner                      | NodeTestWithExceptionTrajectory                | trajectory          | trajectory                                                                                                 | Empty, single point, path with duplicate points        |                           |
| obstacle_velocity_limiter                  | NodeTestWithExceptionTrajectory                | trajectory          | trajectory                                                                                                 | Empty, single point, path with duplicate points        |                           |
| obstacle_avoidance_planner                 | NodeTestWithExceptionTrajectory                | trajectory          | trajectory                                                                                                 | Empty, single point, path with duplicate points        |                           |
| scenario_selector                          | NodeTestWithExceptionTrajectoryLaneDrivingMode |
| NodeTestWithExceptionTrajectoryParkingMode | trajectory                                     | scenario            | Empty, single point, path with duplicate points. There are 2 patterns of scenarios:LANEDRIVING and PARKING |                                                        |
| freespace_planner                          | NodeTestWithExceptionRoute                     | route               | trajectory                                                                                                 | Empty route                                            |                           |
| behavior_path_planner                      | NodeTestWithExceptionRoute                     | route, ego position | path_with_lane_id                                                                                          | Empty route, TBD                                       |                           |
| behavior_velocity_planner                  | NodeTestWithExceptionPathWithLaneID            | path_with_lane_id   | path                                                                                                       | Empty path                                             |                           |

## Assumptions / Known limits

When launch a node, the parameters are loaded from the package's parameter file, which is located in the config directory.Please be aware that if there are missing parameters, the node can't be launched during testing.

## Future extensions / Unimplemented parts
```
