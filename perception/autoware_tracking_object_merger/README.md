# トラッキングオブジェクトマージャー

## 目的

このパッケージは、異なるセンサーの2つのトラッキングオブジェクトをマージしようとします。

## 内部処理/アルゴリズム

異なるセンサーのトラッキングオブジェクトをマージすることは、データ関連付けと状態フュージョンのアルゴリズムの組み合わせです。

詳細なプロセスは、マージャーポリシーによって異なります。

### decorative_tracker_merger

`decorative_tracker_merger`では、ドミナントなトラッキングオブジェクトとサブトラッキングオブジェクトがあると仮定します。
`decorative`という名前は、サブトラッキングオブジェクトがメインオブジェクトを補完するために使用されることを意味します。

通常、ドミナントなトラッキングオブジェクトはLiDARから、サブトラッキングオブジェクトはレーダーまたはカメラから取得されます。

以下に処理パイプラインを示します。

![decorative_tracker_merger](./image/decorative_tracker_merger.drawio.svg)

#### タイムシンク

サブオブジェクト（レーダーまたはカメラ）は、ドミナントオブジェクト（LiDAR）よりも高い頻度で取得されることがよくあります。したがって、サブオブジェクトの時間をドミナントオブジェクトに同期させる必要があります。

![タイムシンク](image/time_sync.drawio.svg)

#### データ関連付け

データ関連付けでは、以下のルールを使用して2つのトラッキングオブジェクトが同じオブジェクトかどうかを判別します。

- ゲーティング
  - `distance gate`: 2つのトラッキングオブジェクト間の距離
  - `angle gate`: 2つのトラッキングオブジェクト間の角度
  - `mahalanobis_distance_gate`: 2つのトラッキングオブジェクト間のマハラノビス距離
  - `min_iou_gate`: 2つのトラッキングオブジェクト間の最小IoU
  - `max_velocity_gate`: 2つのトラッキングオブジェクト間の最大速度差
- スコア
  - マッチングで使用されるスコアは、2つのトラッキングオブジェクト間の距離と同等です

#### トラックレット更新

サブトラッキングオブジェクトはドミナントトラッキングオブジェクトにマージされます。

トラックレットの入力センサー状態に応じて、異なるルールでトラックレット状態を更新します。

| ステート/優先度             | 1番目 | 2番目 | 3番目 |
| -------------------------- | ------ | ----- | ------ |
| キネマティクス（速度以外） | LiDAR  | レーダー | カメラ |
| 前方速度                   | レーダー  | LiDAR | カメラ |
| オブジェクト分類           | カメラ | LiDAR | レーダー |

#### トラックレットマネジメント

トラックレットの管理には`existence_probability`を使用します。

- 新しいトラックレットを作成するときは、`existence_probability`を$p_{sensor}$値に設定します。
- 特定のセンサーでの各更新で、`existence_probability`を$p_{sensor}$値に設定します。
- トラックレットが特定のセンサーでの更新がない場合は、`existence_probability`を`decay_rate`だけ減らします。
- `existence_probability`が`publish_probability_threshold`より大きく、前回の更新からの時間が`max_dt`より小さい場合は、オブジェクトをパブリッシュできます。
- `existence_probability`が`remove_probability_threshold`より小さく、前回の更新からの時間が`max_dt`より大きい場合は、オブジェクトは削除されます。

![tracklet_management](./image/tracklet_management.drawio.svg)

これらのパラメーターは`config/decorative_tracker_merger.param.yaml`で設定できます。


```yaml
tracker_state_parameter:
  remove_probability_threshold: 0.3
  publish_probability_threshold: 0.6
  default_lidar_existence_probability: 0.7
  default_radar_existence_probability: 0.6
  default_camera_existence_probability: 0.6
  decay_rate: 0.1
  max_dt: 1.0
```

#### 入力/パラメータ

| トピック名                      | メッセージタイプ                               | 説明                                                                           |
| ------------------------------- | ------------------------------------------ | ------------------------------------------------------------------------------------- |
| `~input/main_object`            | `autoware_perception_msgs::TrackedObjects` | 主要な追跡対象。この主要な対象のスタンプで出力がパブリッシュされます。 |
| `~input/sub_object`             | `autoware_perception_msgs::TrackedObjects` | サブ追跡対象。                                                                 |
| `output/object`                 | `autoware_perception_msgs::TrackedObjects` | マージされた追跡対象。                                                              |
| `debug/interpolated_sub_object` | `autoware_perception_msgs::TrackedObjects` | 補間されたサブ追跡対象。                                                    |

デフォルトパラメータは [config/decorative_tracker_merger.param.yaml](./config/decorative_tracker_merger.param.yaml) に設定されています。

| パラメーター名                | 説明                                                                                                                                                              | デフォルト値 |
| ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| `base_link_frame_id`          | ベースリンクフレームID。追跡対象の変換に使用されます。                                                                                                                | "base_link"   |
| `time_sync_threshold`         | 時間同期しきい値。2つの追跡対象の差が小さければ、それらは同一対象とみなされます。                                                                                               | 0.05          |
| `sub_object_timeout_sec`      | サブオブジェクトタイムアウト。サブオブジェクトがこの時間更新されなければ、存在しないとみなされます。                                                                                | 0.5           |
| `main_sensor_type`            | メインセンサータイプ。主な追跡対象を判別するために使用します。                                                                                                              | "lidar"       |
| `sub_sensor_type`             | サブセンサータイプ。サブ追跡対象を判別するために使用します。                                                                                                              | "radar"       |
| `tracker_state_parameter`     | トラッカー状態パラメーター。トラッキングに使用されます。                                                                                                                          |               |

- `tracker_state_parameter` の詳細については、[Tracklet Management](#tracklet-management) で説明しています。

#### 調整

[Tracklet Management](#tracklet-management) で説明したように、この Tracker Merger は通常、両方の入力トラッキングオブジェクトを維持します。

誤検出のトラッキングオブジェクトが多い場合、

- そのセンサの `default_<sensor>_existence_probability` を下げる
- `decay_rate` を上げる
- 信頼できるトラッキングオブジェクトのみを公開するために `publish_probability_threshold` を上げる

### equivalent_tracker_merger

これは今後の予定です。

