# gnss_poser

## 目的

`gnss_poser`は、GNSSセンシングメッセージを購読し、共分散と共に vehicle pose を計算するノードです。

このノードは、**base_link** の pose をパブリッシュするために NavSatFix を購読します。NavSatFix のデータはアンテナの位置を表します。したがって、`base_link` からアンテナの位置への tf を使用して、座標変換を実行します。アンテナの位置の frame_id は NavSatFix の `header.frame_id` を参照します。
(**NavSatFix の `header.frame_id` は地球や基準楕円体ではなく、アンテナの frame_id を示していることに注意してください。[NavSatFix の定義も参照してください。](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))**)

`base_link` からアンテナへの変換が取得できない場合、座標変換を実行せずにアンテナ位置の pose を出力します。

## 内部動作 / アルゴリズム

## 入力 / 出力

### 入力

| 名称 | タイプ | 説明 |
|---|---|---|
| `/map/map_projector_info` | `tier4_map_msgs::msg::MapProjectorInfo` | 地図投影情報 |
| `~/input/fix` | `sensor_msgs::msg::NavSatFix` | GNSS状態メッセージ |
| `~/input/autoware_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | 姿勢 [詳細はこちら](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs) |

### アウトプット

**背景**

Autoware 自動運転ソフトウェアでは、Planning コンポーネントが周囲環境を感知し、車輌の軌道計画を算出します。**Path Planning** モジュールはこの計画を実行します。

**Path Planning**

Path Planning モジュールは、以下を含む一連の操作を通じて、経路計画を実行します。

1. **衝突回避**
   - 障害物検出と衝突逸脱量計算
   - 障害物を避けるための経路修正
2. **ルート追従**
   - 指定された経路からの逸脱量計算
   - 経路に沿って車輌を誘導するための経路修正
3. **加減速制御**
   - 車輌の目標速度の計算
   - 加減速逸脱量計算
   - 車輌の速度を制御するための加減速修正
4. **自車位置推定**
   - センサーから受信した情報の統合
   - 'post resampling' ステップにおける自車位置と姿勢の推定

**制御フロー**

Path Planning モジュールの制御フローは次のとおりです。

1. センサーデータを受信します。
2. 自車位置を推定します。
3. 障害物とルート逸脱量を計算します。
4. 経路を修正します。
5. 加減速軌道を計算します。
6. 車輌を制御します。

| 名前                     | タイプ                                            | 説明                                                       |
| ------------------------ | ----------------------------------------------- | ------------------------------------------------------------ |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`               | GNSS センシングデータから計算された自車位置                        |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped` | GNSS センシングデータから計算された、共分散行列を含む自車位置 |
| `~/output/gnss_fixed`    | `tier4_debug_msgs::msg::BoolStamped`            | GNSS 固定ステータス                                          |

## パラメータ

### コアパラメータ

{{ json_to_markdown("sensing/autoware_gnss_poser/schema/gnss_poser.schema.json") }}

## 想定/既知の限界

## (オプション) エラー検出と処理

## (オプション) パフォーマンス特性

## (オプション) 参照/外部リンク

## (オプション) 将来の拡張/未実装部分

