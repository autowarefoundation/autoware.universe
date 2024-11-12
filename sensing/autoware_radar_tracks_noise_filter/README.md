## autoware_radar_tracks_noise_filter

このパッケージには、`radar_msgs/msg/RadarTrack`用のレーダーオブジェクトフィルタモジュールが含まれています。
このパッケージは、レーダートラック内のノイズオブジェクトを除去することができます。

## アルゴリズム

このパッケージのコアアルゴリズムは、`RadarTrackCrossingNoiseFilterNode::isNoise()` 関数です。
詳細は、関数とそのパラメータを参照してください。

- Y軸しきい値

レーダーはx軸速度をドップラー速度として検出できますが、y軸速度は検出できません。
一部のレーダーはデバイス内部でy軸速度を推定できますが、精度が低い場合があります。
y軸しきい値フィルタでは、レーダートラックのy軸速度が`velocity_y_threshold`より大きい場合、ノイズオブジェクトとして処理されます。

## 入力

| Name | Type | Description |
|---|---|---|
| `~/input/tracks` | `radar_msgs/msg/RadarTracks.msg` | 3D 検出されたトラック(レーダートラック) |

## 出力

このドキュメントは、AutowareオペレーティングシステムのPlanningコンポーネントの設計について記載しています。

### Planningの概要

Planningコンポーネントは、自車位置に基づいて、目的地までの安全な経路を生成します。この経路は、速度、加速度、およびステアリング角の制限に従って生成されます。

### 機能

Planningコンポーネントには、以下のような機能があります。

- 目的地までの経路を生成する。
- velocity逸脱量、acceleration逸脱量、ステアリング角逸脱量を考慮した安全な経路を生成する。
- `post resampling`を実行して、経路上の潜在的な障害物を避ける。
- 経路をNavigationコンポーネントに提供する。

### アーキテクチャ

Planningコンポーネントは、次のモジュールで構成されています。

- **Path Planner:** 自車位置に基づいて安全な経路を生成する。
- **Trajectory Planner:** velocity逸脱量、acceleration逸脱量、ステアリング角逸脱量を考慮した安全な経路を生成する。
- **Obstacle Avoidance:** `post resampling`を実行して、経路上の潜在的な障害物を避ける。
- **Route Planner:** Navigationコンポーネントに経路を提供する。

### 使用方法

Planningコンポーネントを使用するには、次の手順に従ってください。

1. 目的地を指定します。
2. Planningコンポーネントを起動します。
3. Planningコンポーネントが経路を生成するのを待ちます。
4. 経路が生成されたら、Navigationコンポーネントに渡します。

### 制限事項

Planningコンポーネントには、以下のような制限があります。

- 障害物を検出できません。
- 動的な障害物に対応できません。
- 高速道路での運転に対応していません。

### 貢献

Autoware Planningコンポーネントに貢献するには、以下のリポジトリを参照してください。

[https://github.com/AutowareFoundation/autoware/tree/master/ros/planning](https://github.com/AutowareFoundation/autoware/tree/master/ros/planning)

| 名称                       | 種類                              | 説明      |
| -------------------------- | --------------------------------- | ---------------- |
| `~/output/noise_tracks`    | radar_msgs/msg/RadarTracks.msg   | ノイズオブジェクト    |
| `~/output/filtered_tracks` | radar_msgs/msg/RadarTracks.msg   | フィルタ処理されたオブジェクト |

## パラメータ

| 名前                   | タイプ   | 説明                                                                                                                         | デフォルト値 |
| :--------------------- | :----- | :------------------------------------------------------------------------------------------------------------------------------ | :------------ |
| `velocity_y_threshold` | double | Y軸速度の閾値 [m/s]。レーダーのトラックのY軸速度が`velocity_y_threshold`を超える場合、ノイズオブジェクトとして扱われます。 | 7.0           |

