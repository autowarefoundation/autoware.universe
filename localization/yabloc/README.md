# YabLoc

**YabLoc** はベクターマップを利用したビジョンベースのローカリゼーションです。[https://youtu.be/Eaf6r_BNFfk](https://youtu.be/Eaf6r_BNFfk)

[![サムネイル](docs/yabloc_thumbnail.jpg)](https://youtu.be/Eaf6r_BNFfk)

画像から抽出した路面マーキングをベクターマップと照合することで、自車位置を推定します。
点群マップやLiDAR は必要ありません。YabLoc は、LiDAR を搭載していない車両や、点群マップが利用できない環境での車両のローカリゼーションを可能にします。

## パッケージ

- [yabloc_common](yabloc_common/README.md)
- [yabloc_image_processing](yabloc_image_processing/README.md)
- [yabloc_particle_filter](yabloc_particle_filter/README.md)
- [yabloc_pose_initializer](yabloc_pose_initializer/README.md)

## YabLoc を NDT の代わりに起動する方法

Autoware を起動するときに、`pose_source:=yabloc` を引数として設定すると、YabLoc が NDT の代わりに起動されます。
デフォルトでは、`pose_source` は `ndt` です。

YabLoc を実行するためのコマンドの例を以下に示します。


```shell
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag\
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  pose_source:=yabloc
```

## アーキテクチャ

![node_diagram](docs/yabloc_architecture.drawio.svg)

## 原理

下図はYabLocの基本原理を示しています。
グラフベースのセグメンテーションから得られた道路領域を使用して線分を抽出することで、路面標示を抽出します。
図の中央上部の赤線は、路面標示として識別された線分を表しています。
YabLocはこれらのセグメントを各パーティクルに変換し、それらをLanelet2から生成されたコストマップと比較することでパーティクルの重みを決めます。

![principle](docs/yabloc_principle.png)

## 可視化

### コア可視化トピック

これらのトピックはデフォルトでは可視化されません。

<img src="docs/yabloc_rviz_description.png" width=800>

| index | トピック名                                                     | 説明                                                                                                                                                               |
| ----- | -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1     | `/localization/yabloc/pf/predicted_particle_marker`            | パーティクルフィルタの粒子分布。赤い粒子は有力候補を示す。                                                                                                             |
| 2     | `/localization/yabloc/pf/scored_cloud`                         | 3D投影線分群。色はマップとの一致度を示す。                                                                                                                         |
| 3     | `/localization/yabloc/image_processing/lanelet2_overlay_image` | 推定された自車位置に基づいて画像にレーンレット2（黄色の線）を重ねた画像。実際の道路標識とよく一致していれば、ローカリゼーションが正確に行われていることを示す。 |

### デバッグ用の画像トピック

これらのトピックはデフォルトでは視覚化されません。

<img src="docs/yabloc_image_description.png" width=800>

| index | トピック名                                                              | 説明                                                                                     |
| ----- | ----------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| 1     | `/localization/yabloc/pf/cost_map_image`                                | レーンレット2から作成されたコストマップ                                                  |
| 2     | `/localization/yabloc/pf/match_image`                                   | 射影された線分                                                                            |
| 3     | `/localization/yabloc/image_processing/image_with_colored_line_segment` | 分類された線分。緑色の線分は、パーティクル補正に使用されます                        |
| 4     | `/localization/yabloc/image_processing/lanelet2_overlay_image`          | レーンレット2のオーバーレイ                                                              |
| 5     | `/localization/yabloc/image_processing/segmented_image`                 | グラフベースのセグメンテーション結果                                                   |

## 制限事項

- YabLocとNDTを同時に動作させることはサポートされていません。
   - 二つを同時に動作させることは計算コストが高くなる可能性があるためです。
   - また、ほとんどの場合NDTはYabLocよりも優れているため、同時に動作させることのメリットは少ないです。
- ロールとピッチの推定を行わないため、一部の認識ノードが正しく動作しない可能性があります。
- 現在、複数のカメラはサポートされていません。将来的にはサポートされる予定です。
- 交差点など、路面標示が少ない場所では、推定はGNSS、IMU、車両のホイールオドメトリに大きく依存します。
- Lanelet2に路面境界または路面標示が含まれていない場合、推定は失敗する可能性があります。
- Autowareチュートリアルで提供されているサンプルrosbagには画像が含まれていないため、YabLocで実行することはできません。
   - YabLocの機能をテストしたい場合は、この[PR](https://github.com/autowarefoundation/autoware.universe/pull/3946)で提供されているサンプルテストデータが役立ちます。

