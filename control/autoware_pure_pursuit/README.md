# ピュアパーシュート制御

ピュアパーシュート制御モジュールは、ピュアパーシュートアルゴリズムを使用して、 desired trajectory の追跡に使用するステアリングアングルを計算します。これは、`autoware_trajectory_follower_node` のラテラルコントローラープラグインとして使用されます。

## 入力

[controller_node](../autoware_trajectory_follower_node/README.md) から以下を設定します。

- `autoware_planning_msgs/Trajectory`: 追従する基準軌道
- `nav_msgs/Odometry`: 自車位置と速度に関する情報

## 出力

次の情報を含むラテラル出力をコントローラーノードに返します。

- `autoware_control_msgs/Lateral`: 目標ステアリングアングル
- LateralSyncData
  - ステアリングアングルの収束
- `autoware_planning_msgs/Trajectory`: 自車に対する予測パス

