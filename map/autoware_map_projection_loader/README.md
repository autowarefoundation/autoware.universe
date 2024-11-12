# autoware_map_projection_loader

## 特徴

`autoware_map_projection_loader` は、Autoware が動作する座標系を定義する `map_projector_info` をパブリッシュする役割を持ちます。
これは、特に座標を地図（ジオイド）から局所座標に変換する場合、またはその逆を行う場合に必要です。

- `map_projector_info_path` が存在する場合、このノードはそれをロードしてそれに応じて地図投影情報をパブリッシュします。
- `map_projector_info_path` が存在しない場合、ノードはあなたが `MGRS` 投影タイプを使用していることを想定し、代わりにレーンレット 2 マップをロードして MGRS グリッドを抽出します。
  - **非推奨の警告: レーンレット 2 マップを使用するこのインターフェイスは推奨されません。代わりに YAML ファイルを準備してください。**

## 地図投影情報ファイルの仕様

`map_path` ディレクトリに `map_projector_info.yaml` という名前の YAML ファイルを用意する必要があります。 `pointcloud_map_metadata.yaml` については、`map_loader` の Readme を参照してください。


```bash
sample-map-rosbag
├── lanelet2_map.osm
├── pointcloud_map.pcd
├── map_projector_info.yaml
└── pointcloud_map_metadata.yaml
```

緯度経度からXYZ座標系への変換は、次の図に示すように3種類あります。各プロジェクタータイプに必要なパラメータの詳細については、以下を参照してください。

![node_diagram](docs/map_projector_type.svg)

### ローカル座標を使用



```yaml
# map_projector_info.yaml
projector_type: local
```

#### 制限事項

経度と緯度を必要とする機能は利用できなくなります。

現在、利用できないことが判明している機能は次のとおりです。

- GNSS局所化
- ADAPIを使用した経度と緯度での自車位置の送信

### MGRSの使用

MGRSを使用する場合は、MGRSグリッドも指定してください。


```yaml
# map_projector_info.yaml
projector_type: MGRS
vertical_datum: WGS84
mgrs_grid: 54SUE
```

#### 制限

2つ以上のMGRSグリッドにまたがるマップでは使用できません。単一のMGRSグリッドの範囲内でのみ使用してください。

### LocalCartesianUTMを使用する場合

ローカルカートシャンUTMを使用する場合は、マップの原点も指定してください。


```yaml
# map_projector_info.yaml
projector_type: LocalCartesianUTM
vertical_datum: WGS84
map_origin:
  latitude: 35.6762 # [deg]
  longitude: 139.6503 # [deg]
  altitude: 0.0 # [m]
```

#### TransverseMercatorの使用

TransverseMercator投影を使用する場合は、マップ原点も指定してください。


```yaml
# map_projector_info.yaml
projector_type: TransverseMercator
vertical_datum: WGS84
map_origin:
  latitude: 35.6762 # [deg]
  longitude: 139.6503 # [deg]
  altitude: 0.0 # [m]
```

## 送信トピック

- `~/map_projector_info` (tier4\_map\_msgs/MapProjectorInfo): このトピックは、マッププロジェクターの定義情報を示します。

## パラメーター

これらのパラメーターは起動引数から渡されると想定されており、`map_projection_loader.param.yaml`に直接書き込むことを推奨しません。

{{ json_to_markdown("map/autoware_map_projection_loader/schema/map_projection_loader.schema.json") }}

