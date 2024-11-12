# 静的センターライン生成器

## 目的

このパッケージは、走行可能な領域内にパスフットプリントを収めたセンターラインを静的に計算します。

狭い道路での走行では、車線の左右の境界線の中間線であるデフォルトのセンターラインは、しばしばパスフットプリントを走行可能領域の外に出してしまいます。パスフットプリントを走行可能領域内に収めるために、[autoware_path_optimizerパッケージ](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/autoware_path_optimizer/)によるオンラインパス形状最適化を使用します。

オンラインパス形状最適化の代わりに、静的センターライン最適化を導入します。この静的センターライン最適化により、次の利点があります。

- 最適化されたセンターライン形状を事前に確認できます。
  - デフォルトのAutowareでは、車両がそこを走行するまでパス形状は決定されません。
  - これにより、オフラインパス形状評価が可能になります。
- パスフットプリントが走行可能領域内にあるため、重く不安定なパス最適化を計算する必要がありません。

## ユースケース

センターライン最適化と通信するためのインターフェイスが2つあります。

### ベクトルマップビルダーインターフェイス

注: Vector Map Builderのこの機能はリリースされていません。しばらくお待ちください。
現在、この機能に関するベクターマップビルダーの操作に関するドキュメントはありません。

最適化されたセンターラインは、ベクターマップビルダーの操作から生成できます。

次のコマンドで`<vehicle_model>`を指定して実行できます。
```bash
# path planning server
rosrun autoware_path_optimizer path_optimizer_node <vehicle_model> --api_path /autoware_path_optimizer/path_optimizer
# http server to connect path planning server and Vector Map Builder
rosrun autoware_path_optimizer center_line_generator_node
```


```sh
ros2 launch autoware_static_centerline_generator run_planning_server.launch.xml vehicle_model:=<vehicle-model>
```

FYI、HTTPサーバのポートIDはデフォルトで4010です。

### コマンドラインインターフェイス

最適化されたcenterlineは、コマンドラインインターフェイスから指定することで生成できます。

- `<input-osm-path>`
- `<output-osm-path>`（必須ではありません）
- `<start-lanelet-id>`
- `<end-lanelet-id>`
- `<vehicle-model>`


```sh
ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml run_backgrond:=false lanelet2_input_file_path:=<input-osm-path> lanelet2_output_file_path:=<output-osm-path> start_lanelet_id:=<start-lane-id> end_lanelet_id:=<end-lane-id> vehicle_model:=<vehicle-model>
```

既定の最適化されたセンターラインを含む出力マップパスの場所は `/tmp/lanelet2_map.osm` です。出力マップパスを変更したい場合は、`<output-osm-path>` を指定することでパスを再マップできます。

## 可視化

パスプランニングサーバを起動すると、rviz も次のように起動します。
![rviz](./media/rviz.png)

- 黄色のフットプリントは osm マップファイルからのオリジナルのフットプリントです。
  - FYI: フットプリントはセンターラインと車輌のサイズに基づいて生成されます。
- 赤色のフットプリントは最適化されたフットプリントです。
- 灰色領域は走行可能な領域です。
- 黄色のフットプリントは走行可能領域の外側にあるのに対し、赤いフットプリントは走行可能領域の内側にあることがわかります。

### セーフティに欠けるフットプリント

場合によっては、最適化されたセンターラインのフットプリントが車線の境界線に近くなります。`unsafe footprints` マーカーを使って、それらがどの程度近いかを確認することができます。

フットプリントの色は境界線からの距離によって決まり、テキストはこの距離を表します。

![rviz](./media/unsafe_footprints.png)

既定では、フットプリントの色は次のようになります。

- 距離が 0.1 [m] 未満の場合: 赤
- 距離が 0.2 [m] 未満の場合: 緑
- 距離が 0.3 [m] 未満の場合: 青

