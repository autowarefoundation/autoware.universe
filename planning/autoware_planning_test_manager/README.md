# Autoware Planningテストマネージャ

## 背景

異常な経路や大幅に乖離した自車位置などの例外的な入力がPlanningモジュール内の各ノードに与えられると、ノードはそのような入力を処理できない場合があり、クラッシュすることがあります。その結果、ノードのクラッシュをデバッグするには、時間がかかる場合があります。例えば、空の軌道が入力として与えられ、実装時に予想されていなかった場合、変更の統合時、シナリオテスト時、またはシステムが実際の車両で動作している間に、ノードは対処されていない例外的な入力によってクラッシュする可能性があります。

## 目的

例外的な入力を受信したときにノードが正しく動作することを保証するためのテストを実装するためのユーティリティを提供することが目的です。このユーティリティを利用して例外的な入力のテストを実装することにより、PRをマージする前に例外的な入力の対策を要求することで、システムを実際に実行したときにのみ発見されるバグを減らすことが目的です。

## 機能

### 通常動作の確認

テスト対象ノードについて、ノードが正しく動作し、後続のノードに必要なメッセージをパブリッシュすることを確認します。これを行うには、test\_nodeに必要なメッセージをパブリッシュし、ノードの出力が出力されていることを確認します。

### 特殊な入力のロバスト性確認

通常動作を確認した後、例外的な入力が与えられたときにテスト対象ノードがクラッシュしないことを確認します。これを行うには、test\_nodeから例外的な入力を提供し、ノードがクラッシュしないことを確認します。

（WIP）

## 使用方法


```cpp

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  // instantiate test_manager with PlanningInterfaceTestManager type
  auto test_manager = std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  // get package directories for necessary configuration files
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto target_node_dir =
    ament_index_cpp::get_package_share_directory("target_node");

  // set arguments to get the config file
  node_options.arguments(
    {"--ros-args", "--params-file",
     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     autoware_planning_validator_dir + "/config/planning_validator.param.yaml"});

  // instantiate the TargetNode with node_options
  auto test_target_node = std::make_shared<TargetNode>(node_options);

  // publish the necessary topics from test_manager second argument is topic name
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "velocity_smoother/input/external_velocity_limit_mps");

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
```

## 実装されたテスト

| ノード                        | テスト名                                                                                 | 例外的入力 | 出力         | 例外的な入力パターン                                                             |
| --------------------------- | ----------------------------------------------------------------------------------------- | ----------------- | -------------- | ------------------------------------------------------------------------------------- |
| autoware_planning_validator | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | 空、単一ポイント、重複ポイントを含むパス                                       |
| velocity_smoother           | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | 空、単一ポイント、重複ポイントを含むパス                                       |
| obstacle_cruise_planner     | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | 空、単一ポイント、重複ポイントを含むパス                                       |
| obstacle_stop_planner       | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | 空、単一ポイント、重複ポイントを含むパス                                       |
| obstacle_velocity_limiter   | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | 空、単一ポイント、重複ポイントを含むパス                                       |
| path_optimizer              | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | 空、単一ポイント、重複ポイントを含むパス                                       |
| scenario_selector           | NodeTestWithExceptionTrajectoryLaneDrivingMode NodeTestWithExceptionTrajectoryParkingMode | trajectory        | scenario       | LANEDRIVING および PARKING シナリオの空、単一ポイント、重複ポイントを含むパス |
| freespace_planner           | NodeTestWithExceptionRoute                                                                | route             | trajectory     | 空のルート                                                                           |
| behavior_path_planner       | NodeTestWithExceptionRoute NodeTestWithOffTrackEgoPose                                    | route             | route odometry | 空のルート オフレーン自己位置                                                     |
| behavior_velocity_planner   | NodeTestWithExceptionPathWithLaneID                                                       | path_with_lane_id | path           | 空のパス                                                                            |

## 重要な注意事項

テストの実行中、ノードを起動すると、パラメータはパッケージ内のパラメータファイルからロードされます。そのため、パラメータを追加する場合は、ノード起動時にパラメータファイルからパラメータを取得する場合にパラメータが不足してノードが起動できなくなるのを防ぐため、対象のノードパッケージ内のパラメータファイルに必要なパラメータを追加する必要があります。

## 今後の拡張 / 未実装部分

(WIP)

