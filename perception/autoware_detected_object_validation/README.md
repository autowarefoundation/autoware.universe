# detected_object_validation

## 目的

このパッケージの目的は、DetectedObjects の明らかな誤検知を除去することです。

## 参照/外部リンク

- [障害物点群ベースのバリデータ](obstacle-pointcloud-based-validator-ja.md)
- [占有グリッドベースのバリデータ](occupancy-grid-based-validator-ja.md)
- [オブジェクトレーンレットフィルタ](object-lanelet-filter-ja.md)
- [オブジェクト位置フィルタ](object-position-filter-ja.md)

### ノードパラメータ

#### object_lanelet_filter

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/object_lanelet_filter.schema.json", "ja") }}

#### object_position_filter

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/object_position_filter.schema.json", "ja") }}

#### obstacle_pointcloud_based_validator

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/obstacle_pointcloud_based_validator.schema.json", "ja") }}

#### occupancy_grid_based_validator

{{ json_to_markdown("perception/autoware_detected_object_validation/schema/occupancy_grid_based_validator.schema.json", "ja") }}

