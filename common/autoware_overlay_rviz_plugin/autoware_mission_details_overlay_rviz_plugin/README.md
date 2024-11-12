# autoware_mission_details_overlay_rviz_plugin

このRVizプラグインは、現在のミッションの残り距離と時間を表示します。

## 入出力

### 入力

| 名称                                       | 種類                                                        | 説明                                               |
| ------------------------------------------- | ----------------------------------------------------------- | ------------------------------------------------------ |
| `/planning/mission_remaining_distance_time` | `autoware_planning_msgs::msg::MissionRemainingDistanceTime` | ミッションの残り距離と時間を表すトピック            |

## オーバーレイパラメータ

| 名前     | タイプ | デフォルト値 | 説明                       |
| -------- | ---- | ------------- | --------------------------------- |
| `Width`  | int  | 170           | オーバーレイの幅 [`px`]         |
| `Height` | int  | 100           | オーバーレイの高さ [`px`]        |
| `Right`  | int  | 10            | 右端からの余白 [`px`] |
| `Top`    | int  | 10            | 上端からの余白 [`px`]   |

ミッションの詳細表示は、画面の右上に配置されています。

## 使用方法

[autoware_overlay_rviz_plugin](../autoware_overlay_rviz_plugin/README.md)に似ています。

## クレジット

[jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)パッケージをベースにしています。

### アイコン

- <https://fonts.google.com/icons?selected=Material+Symbols+Outlined:conversion_path:FILL@1;wght@400;GRAD@200;opsz@20&icon.size=20&icon.color=%23e8eaed&icon.query=path>
- <https://fonts.google.com/icons?selected=Material+Symbols+Outlined:av_timer:FILL@1;wght@400;GRAD@200;opsz@20&icon.size=20&icon.color=%23e8eaed&icon.query=av+timer>

