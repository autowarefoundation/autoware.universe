# autoware_geo_pose_projector

## 概要

このノードは、地理参照された姿勢トピックを購読し、マップフレーム内の姿勢を公開するシンプルなノードです。

## 購読トピック

| 名前                      | タイプ                                                 | 説明                 |
| ------------------------- | ---------------------------------------------------- | -------------------- |
| `input_geo_pose`          | `geographic_msgs::msg::GeoPoseWithCovarianceStamped` | 地理参照された位置 |
| `/map/map_projector_info` | `tier4_map_msgs::msg::MapProjectedObjectInfo`        | マッププロジェクター情報 |

## 公開トピック

| 名称          | 型                                                | 説明                                   |
| ------------- | ------------------------------------------------- | ---------------------------------------- |
| `output_pose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | マップフレーム内のポーズ                |
| `/tf`         | `tf2_msgs::msg::TFMessage`                        | 親リンクと子リンク間のtf              |

## パラメータ

{{ json_to_markdown("localization/autoware_geo_pose_projector/schema/geo_pose_projector.schema.json") }}

## 制限事項

使用する投影タイプによっては、共分散を変換できない可能性があります。入力トピックの共分散は、対角行列として(緯度、経度、高度)で表されます。
現在、x軸を東向き、y軸を北向きと想定しています。そのため、この仮定が破られると、特に緯度と経度の共分散が異なる場合は、変換が正しく処理されない可能性があります。

