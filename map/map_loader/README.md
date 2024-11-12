## map_loader パッケージ

このパッケージは、さまざまなマップをロードする機能を提供します。

## pointcloud_map_loader

### 機能

`pointcloud_map_loader` は、さまざまな構成でその他の Autoware ノードにポイントクラウド マップを提供します。
現在、次の 2 つのタイプをサポートしています。

- Raw ポイントクラウド マップを公開します
- ダウンサンプリングされたポイントクラウド マップを公開します
- ROS 2 サービス経由で一部ポイントクラウド マップのローディングを送信します
- ROS 2 サービス経由で差分ポイントクラウド マップのローディングを送信します

注意: **大規模ポイントクラウド マップを使用する場合、後者の 2 つの機能 (部分および差分ロード) を有効にするには分割されたマップを使用することを強くお勧めします。詳細は前提条件セクションを参照し、マップを分割してメタデータの準備に関する指示に従ってください。**

### 前提条件

#### ポイントクラウド マップ ファイルの前提条件

単一の .pcd ファイルまたは複数の .pcd ファイルを提供できます。複数の PCD データを使用する場合は、次のルールに従う必要があります。

1. **ポイントクラウド マップは、`map_projection_loader` で定義された同じ座標に投影する必要があります**。これは、車線マップと、局所座標と測地座標を変換する他のパッケージと整合性を保つためです。詳細については、[``map_projection_loader`` の README](https://github.com/autowarefoundation/autoware.universe/tree/main/map/autoware_map_projection_loader/README.md) を参照してください。
2. **x 軸と y 軸に平行な直線で分割する必要があります**。このシステムは、対角線または曲線による分割をサポートしていません。
3. **各軸に沿った分割サイズは等しくする必要があります**。特に、分割サイズが大きすぎる (たとえば、100 m 以上) 場合は、[ndt_scan_matcher](https://github.com/autowarefoundation/autoware.universe/tree/main/localization/autoware_ndt_scan_matcher) と [autoware_compare_map_segmentation](https://github.com/autowarefoundation/autoware.universe/tree/main/perception/autoware_compare_map_segmentation) の動的マップのローディング機能に悪影響を与える可能性があるため注意してください。
4. **すべての分割マップは互いに重複しないようにする必要があります**。
5. **メタデータ ファイルも提供する必要があります。**メタデータ構造の説明を以下に示します。

#### メタデータ構造

メタデータは次のようになります。


```yaml
x_resolution: 20.0
y_resolution: 20.0
A.pcd: [1200, 2500] # -> 1200 < x < 1220, 2500 < y < 2520
B.pcd: [1220, 2500] # -> 1220 < x < 1240, 2500 < y < 2520
C.pcd: [1200, 2520] # -> 1200 < x < 1220, 2520 < y < 2540
D.pcd: [1240, 2520] # -> 1240 < x < 1260, 2520 < y < 2540
```

**点群マップのフォーマット**

`open_planner.voxel_grid.VoxelGridConfig`を使用して、点群マップ`voxel grid`を作成します。詳細については、`open_planner.voxel_grid.VoxelGridConfig`のドキュメントを参照してください。

点群マップのフォーマットは次のとおりです。

````
<PCDファイル名>.pcd
````

- `x_resolution`と`y_resolution`
- `A.pcd`, `B.pcd`などはPCDファイルの名前です。
- `[1200, 2500]`などのリストは、このPCDファイルのx座標が1200～1220（`x_resolution` + `x_coordinate`）の間、y座標が2500～2520（`y_resolution` + `y_coordinate`）の間にあることを示す値です。

点群マップを分割したり、互換性のある`metadata.yaml`を生成したりするには、[pointcloud_divider](https://github.com/autowarefoundation/autoware_tools/tree/main/map/autoware_pointcloud_divider)を使用できます。

**これらのファイルのディレクトリ構造**

点群マップが1つしかない場合、Autowareはデフォルトで次のディレクトリ構造を使用します。


```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
```

複数ロズバッグがある場合は、次のようになります。マルチポイントクラウドマップファイルがある場合はメタデータを用意する必要があります。


```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
│ ├── A.pcd
│ ├── B.pcd
│ ├── C.pcd
│ └── ...
├── map_projector_info.yaml
└── pointcloud_map_metadata.yaml
```

### 固有の機能

#### 生の点群マップを公開（ROS 2トピック）

ノードは、`.pcd` ファイルから読み込まれた生の点群マップを公開します。`leaf_size` パラメーターを変更することで、ダウンサンプルの解像度を指定できます。

#### ダウンサンプルされた点群マップを公開（ROS 2トピック）

ノードは、`.pcd` ファイルから読み込まれたダウンサンプルされた点群マップを公開します。`leaf_size` パラメーターを変更することで、ダウンサンプルの解像度を指定できます。

#### 点群マップのメタデータを公開（ROS 2トピック）

ノードは、IDが添付された点群メタデータを公開します。メタデータは `.yaml` ファイルから読み込まれます。詳細については、[PointCloudMapMetaData.msg の説明](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#pointcloudmapmetadatamsg) を参照してください。

#### 部分的な点群マップを送信（ROS 2サービス）

ここでは、点群マップがグリッドに分割されていると仮定します。

クライアントノードからのクエリを受け取ると、ノードはクエリされた領域と重複する点群マップのセットを送信します。詳細については、[GetPartialPointCloudMap.srv の説明](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getpartialpointcloudmapsrv) を参照してください。

#### 差分点群マップを送信（ROS 2サービス）

ここでは、点群マップがグリッドに分割されていると仮定します。

クエリとマップIDのセットを受け取ると、ノードはクエリされた領域と重複し、マップIDのセットに含まれていない点群マップのセットを送信します。詳細については、[GetDifferentialPointCloudMap.srv の説明](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getdifferentialpointcloudmapsrv) を参照してください。

#### 選択された点群マップを送信（ROS 2サービス）

ここでは、点群マップがグリッドに分割されていると仮定します。

クライアントノードからのクエリを受け取ると、ノードはクエリによって指定された一意のIDが添付された点群マップのセットを送信します。詳細については、[GetSelectedPointCloudMap.srv の説明](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_map_msgs#getselectedpointcloudmapsrv) を参照してください。

### パラメーター

{{ json_to_markdown("map/map_loader/schema/pointcloud_map_loader.schema.json") }}

### インターフェース

- `output/pointcloud_map`（sensor_msgs/msg/PointCloud2）：生の点群マップ
- `output/pointcloud_map_metadata`（autoware_map_msgs/msg/PointCloudMapMetaData）：点群マップのメタデータ
- `output/debug/downsampled_pointcloud_map`（sensor_msgs/msg/PointCloud2）：ダウンサンプルされた点群マップ
- `service/get_partial_pcd_map`（autoware_map_msgs/srv/GetPartialPointCloudMap）：部分的な点群マップ
- `service/get_differential_pcd_map`（autoware_map_msgs/srv/GetDifferentialPointCloudMap）：差分点群マップ
- `service/get_selected_pcd_map`（autoware_map_msgs/srv/GetSelectedPointCloudMap）：選択された点群マップ
- 点群マップファイル（.pcd）
- 点群マップのメタデータ（.yaml）

---

## lanelet2_map_loader

### 特徴

lanelet2_map_loaderはLanelet2ファイルを読み込み、マップデータをautoware_map_msgs/LaneletMapBinメッセージとして公開します。
このノードは、`map_projection_loader`からの`/map/map_projector_info`で定義された任意の座標系にlan/lon座標を投影します。

### 実行方法

`ros2 run map_loader lanelet2_map_loader --ros-args -p lanelet2_map_path:=path/to/map.osm`

### サブスクライブするトピック

- ~input/map_projector_info (tier4_map_msgs/MapProjectorInfo) : Autowareの射影タイプ

### パブリッシュするトピック

- ~output/lanelet2_map (autoware_map_msgs/LaneletMapBin) : ロードされたLanelet2マップのバイナリデータ

### パラメーター

{{ json_to_markdown("map/map_loader/schema/lanelet2_map_loader.schema.json") }}

`use_waypoints`はセンターラインの処理方法を決定します。
このフラグは、`overwriteLaneletsCenterline`ではなく`overwriteLaneletsCenterlineWithWaypoints`関数を使用できます。詳細は [autoware_lanelet2_extensionパッケージのドキュメント](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md#centerline) を参照してください。

---

## lanelet2_map_visualization

### 機能

lanelet2_map_visualizationは、autoware_map_msgs/LaneletMapBinメッセージをvisualization_msgs/MarkerArrayに可視化します。

### 実行方法

`ros2 run map_loader lanelet2_map_visualization`

### サブスクライブするトピック

- ~input/lanelet2_map (autoware_map_msgs/LaneletMapBin) : Lanelet2マップのバイナリデータ

### パブリッシュするトピック

- ~output/lanelet2_map_marker (visualization_msgs/MarkerArray) : RViz用の可視化メッセージ

