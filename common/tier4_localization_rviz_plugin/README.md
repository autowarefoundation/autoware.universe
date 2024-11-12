## tier4_localization_rviz_plugin

## 目的

このプラグインは、ekf_localizer、ndt_scan_matching、GNSSによって取得されたローカリゼーションの履歴を表示できます。推定ポーズの不確かさが与えられた場合、それも表示できます。

## 入出力

### 入力

### Pose履歴

| 名称 | タイプ | 説明 |
| ---- | ----- | ------------------------------------------------------------------------------------------------- |
| `input/pose` | `geometry_msgs::msg::PoseStamped` | `input/pose` には、ekf_localizer、ndt_scan_matching、または GNSS で計算された局所化の結果を入れます |

### 共分散を持つ自車位置履歴

| 名称                         | 型                                            | 説明                                                                                                           |
| ---------------------------- | ----------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `input/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | `input/pose_with_covariance` には、ekf_localizer、ndt_scan_matching、GNSS によって計算された、ローカリゼーションの結果を配置します。 |

## パラメータ

### コアパラメータ

### 自車位置履歴

| 名称 | タイプ | デフォルト値 | 説明 |
| ----------------------- | ------ | ------------- | ------------------------- |
| `property_buffer_size_` | int    | 100           | トピックのバッファサイズ |
| `property_line_view_`   | bool   | true          | Lineプロパティを使用 |
| `property_line_width_`  | float  | 0.1           | Lineプロパティの幅 [m] |
| `property_line_alpha_`  | float  | 1.0           | Lineプロパティのアルファ |
| `property_line_color_`  | QColor | Qt::white     | Lineプロパティの色 |

### 自車位置の共分散履歴

| 名称                          | タイプ   | デフォルト値  | 説明                                    |
| ----------------------------- | ------ | ------------ | --------------------------------------- |
| `property_buffer_size_`        | int    | 100          | トピックのバッファサイズ                    |
| `property_path_view_`          | bool   | true         | パスプロパティを使用するか否か               |
| `property_shape_type_`         | string | Line         | LineまたはArrow                           |
| `property_line_width_`         | float  | 0.1          | Lineプロパティの幅 [m]                    |
| `property_line_alpha_`         | float  | 1.0          | Lineプロパティのアルファ                   |
| `property_line_color_`         | QColor | Qt::white    | Lineプロパティの色                        |
| `property_arrow_shaft_length`  | float  | 0.3          | Arrowプロパティのシャフトの長さ               |
| `property_arrow_shaft_diameter` | float  | 0.15         | Arrowプロパティのシャフトの直径               |
| `property_arrow_head_length`   | float  | 0.2          | Arrowプロパティのヘッドの長さ                  |
| `property_arrow_head_diameter` | float  | 0.3          | Arrowプロパティのヘッドの直径                  |
| `property_arrow_alpha_`        | float  | 1.0          | Arrowプロパティのアルファ                    |
| `property_arrow_color_`        | QColor | Qt::white    | Arrowプロパティの色                        |
| `property_sphere_scale_`       | float  | 1.0          | Sphereプロパティのスケール                   |
| `property_sphere_alpha_`       | float  | 0.5          | Sphereプロパティのアルファ                    |
| `property_sphere_color_`       | QColor | (204, 51, 204) | Sphereプロパティの色                        |

## 想定/既知の制限

TBD.

## 使用方法

1. rvizを起動し、[Displays]パネルで[Add]を選択します。
   ![select_add](./images/select_add.png)
2. [tier4_localization_rviz_plugin/PoseHistory]または[PoseWithCovarianceHistory]を選択します。次に、[OK]を押します。
   ![select_localization_plugin](./images/select_localization_plugin.png)
3. 軌道と共分散を表示するトピックの名前を入力します。
   ![select_topic_name](./images/select_topic_name.png)
4. 軌道と共分散を表示できます。
   ![ex_pose_with_covariance_history](./images/ex_pose_with_covariance_history.png)

