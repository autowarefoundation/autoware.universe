## tier4_traffic_light_rviz_plugin

## 目的

このプラグインパネルは、ダミー信号機信号をパブリッシュします。

## 入出力

### 出力

| 名称                                                   | 型                                                   | 説明                   |
| ------------------------------------------------------ | ------------------------------------------------------ | ----------------------------- |
| `/perception/traffic_light_recognition/traffic_signals` | `autoware_perception_msgs::msg::TrafficLightGroupArray` | 交通信号の公開 |

## 操作方法

<div align="center">
  <img src="images/select_panels.png" width=50%>
</div>
<div align="center">
  <img src="images/select_traffic_light_publish_panel.png" width=50%>
</div>
<div align="center">
  <img src="images/select_traffic_light_id.png" width=50%>
</div>

1. rvizを起動して[パネル]->[新しいパネルを追加]を選択します。
2. TrafficLightPublishPanelを選択してOKを押します。
3. [`Traffic Light ID`]と[`Traffic Light Status`]を設定して[`SET`]ボタンを押します。
4. [`PUBLISH`]ボタンが押されている間、信号機信号がパブリッシュされます。

<div align="center">
  <img src="images/traffic_light_publish_panel.gif">
</div>

