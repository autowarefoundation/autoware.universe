## autoware_joy_controller

## 役割

`autoware_joy_controller`は、車両のAutowareコマンド（ステアリングホイール、シフト、ターンシグナル、エンゲージなど）にジョイメッセージを変換するパッケージです。

## 使用方法

### ROS 2起動


```bash
# With default config (ds4)
ros2 launch autoware_joy_controller joy_controller.launch.xml

# Default config but select from the existing parameter files
ros2 launch autoware_joy_controller joy_controller_param_selection.launch.xml joy_type:=ds4 # or g29, p65, xbox

# Override the param file
ros2 launch autoware_joy_controller joy_controller.launch.xml config_file:=/path/to/your/param.yaml
```

## 入出力

### 入力トピック

| 名称               | 型                    | 説明                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | ジョイコントローラの指令            |
| `~/input/odometry` | nav_msgs::msg::Odometry | 自車位置を取得するための自己位置推定 |

### 出力トピック

| 名前                                | タイプ                                                | 説明                              |
| ----------------------------------- | --------------------------------------------------- | ---------------------------------------- |
| `~/output/control_command`          | `autoware_control_msgs::msg::Control`                 | 横方向および縦方向制御コマンド |
| `~/output/external_control_command` | `tier4_external_api_msgs::msg::ControlCommandStamped` | 横方向および縦方向制御コマンド |
| `~/output/shift`                    | `tier4_external_api_msgs::msg::GearShiftStamped`      | ギアコマンド                             |
| `~/output/turn_signal`              | `tier4_external_api_msgs::msg::TurnSignalStamped`     | ウインカーコマンド                      |
| `~/output/gate_mode`                | `tier4_control_msgs::msg::GateMode`                   | ゲートモード（AutoまたはExternal）             |
| `~/output/heartbeat`                | `tier4_external_api_msgs::msg::Heartbeat`             | ハートビート                                |
| `~/output/vehicle_engage`           | `autoware_vehicle_msgs::msg::Engage`                  | 車両エンゲージ                           |

## パラメータ

| パラメーター           | タイプ | 説明                                                                                                                                                          |
| ---------------------- | ------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `joy_type`              | string | ジョイコントローラータイプ (初期値: DS4)                                                                                                                                  |
| `update_rate`           | double | 制御コマンドを公開するための更新レート                                                                                                                                |
| `accel_ratio`           | double | 加速度を計算するための比率 (指示加速度は比率 \* 操作量)                                                                                                                       |
| `brake_ratio`           | double | 減速度を計算するための比率 (指示加速度は -比率 \* 操作量)                                                                                                                    |
| `steer_ratio`           | double | 減加速度を計算するための比率 (指示舵角は比率 \* 操作量)                                                                                                                  |
| `steering_angle_velocity` | double | 操作のときの舵角速度                                                                                                                                            |
| `accel_sensitivity`     | double | 外部APIの加速度を計算するための感度 (指示加速度は pow(操作量, 1 / 感度))                                                                                                |
| `brake_sensitivity`     | double | 外部APIの減加速度を計算するための感度 (指示加速度は pow(操作量, 1 / 感度))                                                                                               |
| `raw_control`           | bool   | 真の場合、入力車体運動をスキップする                                                                                                                                 |
| `velocity_gain`         | double | 加速度によって計算される速度の比率                                                                                                                                |
| `max_forward_velocity`  | double | 前方へ進む絶対最大速度                                                                                                                                         |
| `max_backward_velocity` | double | 後方へ進む絶対最大速度                                                                                                                                         |
| `backward_accel_ratio`  | double | 減速度を計算するための比率 (Commanded acceleration is -ratio \* 操作量)                                                                                                            |

## P65 ジョイスティック キーマップ

| 操作 | ボタン |
|---|---|
| 加速度 | R2 |
| ブレーキ | L2 |
| ステアリング | レフトスティックの左右 |
| アップシフト | カーソル上 |
| ダウンシフト | カーソル下 |
| Driveシフト | カーソル左 |
| リバースシフト | カーソル右 |
| 左折ウィンカー | L1 |
| 右折ウィンカー | R1 |
| ウィンカー解除 | A |
| ゲートモード | B |
| 緊急停止 | セレクト |
| 緊急停止解除 | スタート |
| Autoware起動 | X |
| Autoware停止 | Y |
| Vehicle起動 | PS |
| Vehicle停止 | 右トリガー |

## DS4 ジョイスティック キーマップ

| アクション               | ボタン                     |
| -------------------- | -------------------------- |
| 加速                  | R2、×、または右スティック上 |
| 減速                  | L2、□、または右スティック下 |
| ステアリング          | 左スティック左右          |
| シフトアップ          | カーソル上                  |
| シフトダウン          | カーソル下                  |
| シフトドライブ        | カーソル左                  |
| シフトリバース        | カーソル右                  |
| 左ウィンカー           | L1                         |
| 右ウィンカー           | R1                         |
| ウィンカー解除         | SHARE                      |
| ゲートモード            | OPTIONS                    |
| 緊急停止             | PS                         |
| 緊急停止解除           | PS                         |
| Autoware起動          | ○                          |
| Autoware停止           | ○                          |
| 車両起動              | △                          |
| 車両停止              | △                          |

## XBOX ジョイスティック キーマッピング

---

## 自動運転ソフトウェア ドキュメント（日本語訳）

### 操作方法

| 操作 | ボタン |
|---|---|
| 加速 | RT |
| ブレーキ | LT |
| ステアリング | 左スティック<br>左/右 |
| アップシフト | カーソル上 |
| ダウンシフト | カーソル下 |
| ドライブシフト | カーソル左 |
| リバースシフト | カーソル右 |
| 左ウインカー | LB |
| 右ウインカー | RB |
| ウインカー解除 | A |
| ゲートモード | B |
| 緊急停止 | ビュー |
| 緊急停止解除 | メニュー |
| Autoware 起動 | X |
| Autoware 停止 | Y |
| Vehicle 起動 | 左スティックボタン |
| Vehicle 停止 | 右スティックボタン |

### Planningコンポーネント

#### `post resampling`動作

#### 自車位置喪失時のリカバリ動作

#### velocity逸脱量の計算

#### acceleration逸脱量の計算

