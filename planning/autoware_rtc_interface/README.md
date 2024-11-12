# RTCインタフェース

## 目的

RTCインタフェースは、ビヘイビアプランニングモジュールの決定ステータスを公開し、自動運転システム外部からの実行コマンドを受け取るインターフェースです。

## 内部動作/アルゴリズム

### 使用例


```c++
// Generate instance (in this example, "intersection" is selected)
autoware::rtc_interface::RTCInterface rtc_interface(node, "intersection");

// Generate UUID
const unique_identifier_msgs::msg::UUID uuid = generateUUID(getModuleId());

// Repeat while module is running
while (...) {
  // Get safety status of the module corresponding to the module id
  const bool safe = ...

  // Get distance to the object corresponding to the module id
  const double start_distance = ...
  const double finish_distance = ...

  // Get time stamp
  const rclcpp::Time stamp = ...

  // Update status
  rtc_interface.updateCooperateStatus(uuid, safe, start_distance, finish_distance, stamp);

  if (rtc_interface.isActivated(uuid)) {
    // Execute planning
  } else {
    // Stop planning
  }
  // Get time stamp
  const rclcpp::Time stamp = ...

  // Publish status topic
  rtc_interface.publishCooperateStatus(stamp);
}

// Remove the status from array
rtc_interface.removeCooperateStatus(uuid);
```

## 入出力

### RTCInterface (コンストラクター)


```c++
autoware::rtc_interface::RTCInterface(rclcpp::Node & node, const std::string & name);
```

#### 説明

`autoware::rtc_interface::RTCInterface` のコンストラクタです。

#### 入力

- `node` : このインターフェースを呼び出すノード
- `name` : 協調ステータスの配列のトピック名と協調コマンドのサービス名
  - 協調ステータスの配列のトピック名 : `~/{name}/cooperate_status`
  - 協調コマンドのサービス名 : `~/{name}/cooperate_commands`

#### 出力

`RTCInterface` のインスタンス

### publishCooperateStatus



```c++
autoware::rtc_interface::publishCooperateStatus(const rclcpp::Time & stamp)
```

#### 説明

登録された協調状態を公開します。

#### 入力

- `stamp` : タイムスタンプ

#### 出力

なし

### updateCooperateStatus


```c++
autoware::rtc_interface::updateCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid, const bool safe, const double start_distance, const double finish_distance, const rclcpp::Time & stamp)
```

#### 概要

`uuid` に対応する連携状態を更新します。
`uuid` に対応する連携状態が未登録の場合は、新しい連携状態を追加します。

#### 入力

- `uuid` : リクエストするモジュールの UUID
- `safe` : リクエストするモジュールの安全状態
- `start_distance` : 自車位置から開始オブジェクトまでの距離
- `finish_distance` : 自車位置から終了オブジェクトまでの距離
- `stamp` : タイムスタンプ

#### 出力

なし

### removeCooperateStatus


```c++
autoware::rtc_interface::removeCooperateStatus(const unique_identifier_msgs::msg::UUID & uuid)
```

#### 説明

登録済みステータスから `uuid` に相当する協調ステータスを削除する。

#### 入力

- `uuid` : 有効期限切れモジュールの UUID

#### 出力

なし

### clearCooperateStatus


```c++
autoware::rtc_interface::clearCooperateStatus()
```

#### 概要

すべての協調ステータスを削除します。

#### 入力

なし

#### 出力

なし

### isActivated


```c++
autoware::rtc_interface::isActivated(const unique_identifier_msgs::msg::UUID & uuid)
```

#### 説明

`uuid` に対応する受信コマンドステータスを返します。

#### 入力

- `uuid` : モジュールを確認するための UUID

#### 出力

自動モードが有効な場合は、安全ステータスに基づいて返します。
それ以外の場合は、受信コマンドが `ACTIVATED` の場合、`true` を返します。
それ以外の場合は、`false` を返します。

### isRegistered


```c++
autoware::rtc_interface::isRegistered(const unique_identifier_msgs::msg::UUID & uuid)
```

#### 概要

`uuid`が登録されている場合に`true`を返します。

#### 入力

- `uuid` : モジュールチェック用UUID

#### 出力

`uuid`が登録されている場合に`true`を返します。登録されていない場合は`false`を返します。

## デバッグツール

RTCインターフェース用の[RTC Replayer](https://autowarefoundation.github.io/autoware_tools/main/planning/autoware_rtc_replayer/)というデバッグツールがあります。

## 想定/既知の制限

## 将来の拡張/未実装の部分

