# autoware_lidar_centerpoint

## 目的

autoware_lidar_centerpointは、動的3Dオブジェクトを検出するためのパッケージです。

## 内部処理やアルゴリズム

この実装では、TensorRTによる推論にPointPillarsベースのネットワークを使用しているCenterPoint [1]が使用されています。

このモデルは、<https://github.com/open-mmlab/mmdetection3d>を使用してトレーニングされました。

## 入出力

### 入力
- LiDAR点群

### 出力
- クラスID
- Bounding Box
- 速度ベクトル
- 加速度ベクトル

| 名称                   | タイプ                              | 説明                                  |
| ---------------------- | ----------------------------------- | ---------------------------------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | 入力ポイントクラウド                           |

### 出力

**計画（Planning）コンポーネントの評価シナリオ**

**目的:**
本シナリオ群は、自動運転ソフトウェアの計画コンポーネントの機能と性能を評価することを目的としています。

**シナリオ:**

1. **回避（回避距離）:** 車両が障害物を回避するときの、各Planningアルゴリズムの性能を評価します。
2. **停止線（停止位置）:** 車両が停止線で停止するときの、Planningアルゴリズムの精度を評価します。
3. **駐車（最終位置）:** 車両が駐車スペースに駐車するときの、Planningアルゴリズムの堅牢性を評価します。
4. **車線逸脱（逸脱量）:** 車両が車線から逸脱するときの、Planningアルゴリズムの応答性を評価します。
5. **速度逸脱（逸脱量）:** 車両が設定された速度を逸脱するときの、Planningアルゴリズムの安定性を評価します。
6. **加速度逸脱（逸脱量）:** 車両が設定された加速度を逸脱するときの、Planningアルゴリズムの滑らかさを評価します。
7. **急カーブ（最小曲率半径）:** 車両が急カーブを走行するときの、Planningアルゴリズムの安全性と快適性を評価します。
8. **勾配（勾配角度）:** 車両が勾配を走行するときの、Planningアルゴリズムの適切さと堅牢性を評価します。
9. **周囲車両予測（TTC）:** 車両が周囲車両と遭遇するときの、Planningアルゴリズムの予測性能と応答時間を評価します。
10. **ディスタンスセンシング（距離）:** 車両が障害物や他の車両との距離を測定するときの、Planningアルゴリズムの精度と信頼性を評価します。

**評価指標:**

* 各シナリオにおける目標ポイント（回避距離、停止位置、駐車位置など）からの偏差
* 'post resampling'前のPlanning出力と'post resampling'後のPlanning出力との間の軌跡の滑らかさ
* 車両の自車位置とPlanningが生成する軌跡との間の誤差
* 計画された速度と加速度と、実際の速度と加速度との間の偏差
* 計画された軌跡の曲率半径と、実際の軌跡の曲率半径との間の偏差
* 車両周囲の障害物や他の車両を回避するために必要な回避時間
* 車両が障害物や他の車両と衝突する可能性のある衝突時間（TTC）

Autoware Planningコンポーネントの性能評価には、これらのシナリオが利用できます。

| 名前                     | タイプ                                                 | 説明 |
| ------------------------ | ------------------------------------------------------ | ------ |
| `~/output/objects`         | `autoware_perception_msgs::msg::DetectedObjects`       | 検出オブジェクト     |
| `debug/cyclic_time_ms`     | `tier4_debug_msgs::msg::Float64Stamped`                   | サイクルタイム (msg) |
| `debug/processing_time_ms` | `tier4_debug_msgs::msg::Float64Stamped`                   | 処理時間 (ms)       |

## パラメータ

### MLモデルパラメータ

これらのパラメータはONNXファイルに関連付けられており、トレーニングフェーズ中に定義済みです。このパラメータを変更する際は、ONNXファイルも変更してください。また、ONNXファイルを更新する際は、必ずこれらの値を確認してください。

| 名称                                    | タイプ         | デフォルト値                                               | 説明                                                                  |
| --------------------------------------- | ------------ | ---------------------------------------------------------- | --------------------------------------------------------------------- |
| `model_params.class_names`              | list[string] | ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"]           | モデル出力のクラス名のリスト                                         |
| `model_params.point_feature_size`       | int          | `4`                                                       | 点群内の各ポイントのフィーチャ数                                    |
| `model_params.max_voxel_size`           | int          | `40000`                                                   | ボクセルの最大数                                                      |
| `model_params.point_cloud_range`        | list[double] | [-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]                       | 検出範囲 [min_x, min_y, min_z, max_x, max_y, max_z] [m]               |
| `model_params.voxel_size`               | list[double] | [0.32, 0.32, 10.0]                                          | 各ボクセルのサイズ [x, y, z] [m]                                      |
| `model_params.downsample_factor`        | int          | `1`                                                       | 座標のダウンサンプル係数                                           |
| `model_params.encoder_in_feature_size`  | int          | `9`                                                       | エンコーダへの入力フィーチャ数                                    |
| `model_params.has_variance`             | bool         | `false`                                                   | モデルが各バウンディングボックスの姿勢分散と姿勢を出力する場合 `true` |
| `model_params.has_twist`                | bool         | `false`                                                   | モデルが各バウンディングボックスの速度と姿勢を出力する場合 `true`     |

### コア・パラメータ

| 名前                                          | タイプ    | デフォルト値   | 説明                                                            |
| ------------------------------------------- | -------- | -------------- | ------------------------------------------------------------------ |
| `encoder_onnx_path`                             | 文字列  | `""`           | VoxelFeatureEncoder ONNX ファイルへのパス                           |
| `encoder_engine_path`                           | 文字列  | `""`           | VoxelFeatureEncoder TensorRT Engine ファイルへのパス                 |
| `head_onnx_path`                               | 文字列  | `""`           | DetectionHead ONNX ファイルへのパス                                 |
| `head_engine_path`                             | 文字列  | `""`           | DetectionHead TensorRT Engine ファイルへのパス                       |
| `build_only`                                   | ブール  | `false`        | TensorRT エンジンファイルが作成されたらノードをシャットダウンする  |
| `trt_precision`                               | 文字列  | `fp16`         | TensorRT 推論の精度: `fp32` または `fp16`                            |
| `post_process_params.score_threshold`           | double  | `0.4`          | スコアが閾値未満の検出オブジェクトは無視される                    |
| `post_process_params.yaw_norm_thresholds`       | doubleのリスト | [0.3, 0.3, 0.3, 0.3, 0.0] | Yaw ノルムの距離閾値の配列 [rad]                                   |
| `post_process_params.iou_nms_target_class_names` | 文字列のリスト | -              | IoU ベースの非最大抑制のターゲットクラス                           |
| `post_process_params.iou_nms_search_distance_2d` | double  | -              | 2 つ以上のオブジェクトが値よりも遠い場合、NMS は適用されない      |
| `post_process_params.iou_nms_threshold`         | double  | -              | IoU ベースの非最大抑制の IoU 閾値                                  |
| `post_process_params.has_twist`                 | ブール  | false          | モデルが出力値を捻じっているかどうかを示す                        |
| `densification_params.world_frame_id`           | 文字列  | `map`          | マルチフレーム点群を統合するワールドフレーム ID                      |
| `densification_params.num_past_frames`          | 整数    | `1`            | 現在フレームと統合する過去フレームの数                              |

### `build_only` オプション

`autoware_lidar_centerpoint` ノードには、ONNX ファイルから TensorRT エンジンファイルを構築するための `build_only` オプションがあります。
Autoware Universe の `.param.yaml` ファイル内のすべての ROS パラメータを移動することが好ましいですが、`build_only` オプションは現時点では `.param.yaml` ファイルに移動されていません。これは、構築をプリタスクとして実行するためのフラグとして使用されることがあるためです。次のコマンドで実行できます。


```bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=/home/autoware/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix autoware_lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
```

## 仮定/既知の制限

- `object.existence_probability` は DNN 分類信頼度の値であり、確率ではありません。

## トレーニング済みモデル

以下のリンクをクリックして、トレーニング済みモデルの onnx 形式をダウンロードできます。

- Centerpoint: [pts_voxel_encoder_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx), [pts_backbone_neck_head_centerpoint.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx)
- Centerpoint tiny: [pts_voxel_encoder_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx), [pts_backbone_neck_head_centerpoint_tiny.onnx](https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx)

`Centerpoint`は、`nuScenes`（約28,000個のライダーフレーム）[8]とTIER IVの内部データベース（約11,000個のライダーフレーム）で60エポックトレーニングしました。
`Centerpoint tiny`は、`Argoverse 2`（約110,000個のライダーフレーム）[9]とTIER IVの内部データベース（約11,000個のライダーフレーム）で20エポックトレーニングしました。

## CenterPointモデルのトレーニングとAutowareへの展開

### 概要

このガイドでは、**mmdetection3d**リポジトリを使用してCenterPointモデルをトレーニングし、Autoware内にシームレスに展開する方法について説明します。

### インストール

#### 前提条件のインストール

**ステップ1.** [公式ウェブサイト](https://mmpretrain.readthedocs.io/en/latest/get_started.html)からMinicondaをダウンロードしてインストールします。

**ステップ2.** conda仮想環境を作成してアクティブにします


```bash
conda create --name train-centerpoint python=3.8 -y
conda activate train-centerpoint
```

**ステップ 3.** PyTorch をインストール

PyTorch がインストール済みで、Autoware の要件である CUDA 11.6 に対応していることを確認してください。


```bash
conda install pytorch==1.13.1 torchvision==0.14.1 pytorch-cuda=11.6 -c pytorch -c nvidia
```

#### mmdetection3dのインストール

**ステップ 1.** MIMを使用してMMEngine、MMCV、MMDetectionをインストールします。


```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**手順 2.** mmdetection3d フォークリポジトリのインストール

mmdetection3d リポジトリをフォークし、いくつかの貴重な機能強化を加えました。
特に、PointPillar z ボクセルの特徴量の入力を省略できるようにし、元の論文との互換性を保ちました。
さらに、追加機能として PyTorch から ONNX へのコンバーターと T4 形式のリーダーを統合しました。


```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### Dockerを使ってトレーニングレポジトリを使用する

また、Dockerを使用してmmdetection3dレポジトリを実行できます。mmdetection3dレポジトリとその依存関係を含むDockerイメージを作成するためのDockerfileを提供しています。

mmdetection3dレポジトリのフォークをクローンする


```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
```

以下のコマンドを実行して Docker イメージを構築します。


```bash
cd mmdetection3d
docker build -t mmdetection3d -f docker/Dockerfile .
```

Dockerコンテナを実行する:


```bash
docker run --gpus all --shm-size=8g -it -v {DATA_DIR}:/mmdetection3d/data mmdetection3d
```

### NuScenes データセットをトレーニング用に準備する

**手順 1.** [公式サイト](https://www.nuscenes.org/download) から NuScenes データセットをダウンロードし、任意のフォルダに解凍します。

**注:** NuScenes データセットは大きく、大量のディスク容量が必要です。処理に取りかかる前に、十分な容量が確保されていることを確認してください。

**手順 2.** データセットフォルダにシンボリックリンクを作成する


```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**ステップ 3.** NuScenesデータを実行して準備:


```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes
```

### NuScenes データセットによる CenterPoint のトレーニング

#### 設定ファイルの用意

NuScenes データセットを使用して CenterPoint モデルをトレーニングする方法を説明する設定ファイルは、`mmdetection3d/projects/AutowareCenterPoint/configs` にあります。この設定ファイルは、[mmdetection3D のこの centerpoint 設定ファイル](https://github.com/autowarefoundation/mmdetection3d/blob/5c0613be29bd2e51771ec5e046d89ba3089887c7/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py) から派生したバージョンです。
このカスタム設定では、**use_voxel_center_z パラメーター**が **False** に設定されており、ボクセルの中心点の z 座標が無効になっています。これにより、元の論文の仕様に沿っており、モデルを Autoware と互換性を持たせます。さらに、フィルターのサイズは **[32, 32]** に設定されています。

CenterPoint モデルは、設定ファイル内のさまざまなパラメーターを変更することで、特定の要件に合わせて調整できます。これには、前処理操作、トレーニング、テスト、モデルアーキテクチャ、データセット、オプティマイザー、学習率スケジューラーなどに関連する調整が含まれます。

#### トレーニングの開始


```bash
python tools/train.py projects/AutowareCenterPoint/configs/centerpoint_custom.py --work-dir ./work_dirs/centerpoint_custom
```

#### 学習モデルの評価

評価目的で、車両からキャプチャしたサンプル・データセットが含まれています。このデータセットは、以下のLiDARセンサーで構成されています。
1 x Velodyne VLS128、4 x Velodyne VLP16、1 x Robosense RS Bpearl。このデータセットは600のLiDARフレームを含んでおり、5つの異なるクラス、6905台の車、3951人の歩行者、75人の自転車乗り、162台のバス、326台のトラックの3Dアノテーションが含まれています。サンプル・データセットでは、フレームは2秒ごとに2フレームずつアノテーションされます。このデータセットは、モデルの学習、評価、微調整など、幅広い目的に使用できます。T4フォーマットで構成されています。

##### サンプル・データセットのダウンロード


```bash
wget https://autoware-files.s3.us-west-2.amazonaws.com/dataset/lidar_detection_sample_dataset.tar.gz
#Extract the dataset to a folder of your choice
tar -xvf lidar_detection_sample_dataset.tar.gz
#Create a symbolic link to the dataset folder
ln -s /PATH/TO/DATASET/ /PATH/TO/mmdetection3d/data/tier4_dataset/
```

##### データセットの準備と、トレーニングされたモデルの評価

トレーニング、評価、テスト用の `.pkl` ファイルを作成します。

データセットは T4Dataset 仕様に従ってフォーマットされており、「sample_dataset」をそのバージョンの 1 つとして指定します。


```bash
python tools/create_data.py T4Dataset --root-path data/sample_dataset/ --out-dir data/sample_dataset/ --extra-tag T4Dataset --version sample_dataset --annotation-hz 2
```

評価を実行する


```bash
python tools/test.py projects/AutowareCenterPoint/configs/centerpoint_custom_test.py /PATH/OF/THE/CHECKPOINT  --task lidar_det
```

サンプルデータセットとトレーニングデータセットの間のセンサーモダリティのばらつきにより、評価の結果が比較的低くなる可能性があります。モデルのトレーニングパラメータはもともと、車両上部に単一のLiDARセンサーを搭載したNuScenesデータセットに合わせて調整されています。一方、提供されたサンプルデータセットは、車両のベースリンクの位置に配置された連結ポイントクラウドで構成されています。

### AutowareへのCenterPointモデルのデプロイ

#### CenterPoint PyTorchモデルをONNX形式に変換する

autoware_lidar_centerpointの実装では、ONNXモデルを2つ入力として必要とします。ボクセルエンコーダーとCenterPointモデルのバックボーン・ネック・ヘッドなど、ネットワークの他の側面は外部的に実装されています。mmdetection3dリポジトリのフォークの下で、CenterPointモデルをAutoware互換のONNX形式に変換するスクリプトを用意しました。これは `mmdetection3d/projects/AutowareCenterPoint` ファイルにあります。


```bash
python projects/AutowareCenterPoint/centerpoint_onnx_converter.py --cfg projects/AutowareCenterPoint/configs/centerpoint_custom.py --ckpt work_dirs/centerpoint_custom/YOUR_BEST_MODEL.pth --work-dir ./work_dirs/onnx_models
```

#### カスタムモデルの設定ファイルを生成

autoware_lidar_centerpointノードの設定ファイルディレクトリ内に**centerpoint_custom.param.yaml**という新しい設定ファイルを作成する。トレーニング設定ファイルに従ってpoint_cloud_range、point_feature_size、voxel_sizeなどの設定ファイルのパラメータを設定する。


```yaml
/**:
  ros__parameters:
    class_names: ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"]
    point_feature_size: 4
    max_voxel_size: 40000
    point_cloud_range: [-51.2, -51.2, -3.0, 51.2, 51.2, 5.0]
    voxel_size: [0.2, 0.2, 8.0]
    downsample_factor: 1
    encoder_in_feature_size: 9
    # post-process params
    circle_nms_dist_threshold: 0.5
    iou_nms_target_class_names: ["CAR"]
    iou_nms_search_distance_2d: 10.0
    iou_nms_threshold: 0.1
    yaw_norm_thresholds: [0.3, 0.3, 0.3, 0.3, 0.0]
```

#### lidar_centerpoint ノードの起動


```bash
cd /YOUR/AUTOWARE/PATH/Autoware
source install/setup.bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml  model_name:=centerpoint_custom  model_path:=/PATH/TO/ONNX/FILE/
```

### 変更履歴

#### v1 (2022/07/06)

| 名称                | URL                                                                                                       | 説明                                                                                                                                 |
| ------------------- | --------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `centerpoint`        | [pts_voxel_encoder][v1-encoder-centerpoint] <br> [pts_backbone_neck_head][v1-head-centerpoint]            | このパッケージの実装の制限のため、1 つの変更があります。`PillarFeatureNet` の `num_filters=[32, 32]` |
| `centerpoint_tiny`    | [pts_voxel_encoder][v1-encoder-centerpoint-tiny] <br> [pts_backbone_neck_head][v1-head-centerpoint-tiny] | `v0`の`default`と同じモデル                                                                                                           |

以下の変更はこの[構成](https://github.com/tianweiy/CenterPoint/blob/v0.2/configs/waymo/pp/waymo_centerpoint_pp_two_pfn_stride1_3x.py)と比較されています。

#### v0 (2021/12/03)

| 名前      | URL                                                                                   | 説明                                                                                                                                            |
| --------- | -------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `default` | [pts_voxel_encoder][v0-encoder-default] <br> [pts_backbone_neck_head][v0-head-default] | オリジナルのCenterPointアーキテクチャから変更が2つある(`PillarFeatureNet`の`num_filters=[32]`と`RPN`の`ds_layer_strides=[2, 2, 2]`) |

## (省略可能) エラー検出と処理

<!-- エラーの検出方法と回復方法について記述する。

例:
  このパッケージは最大20個の障害物に対応できます。それ以上の障害物が見つかると、このノードは処理を放棄し、診断エラーを発生させます。
-->

## (省略可能) パフォーマンス特性評価

<!-- 複雑さなどのパフォーマンス情報を記述する。ボトルネックにならない場合は不要。

例:
  ### 複雑さ

  このアルゴリズムは O(N) です。

  ### 処理時間

  ...
-->

## 参照/外部リンク

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).

[2] Lang, Alex H., et al. "PointPillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://github.com/tianweiy/CenterPoint>

[4] <https://github.com/open-mmlab/mmdetection3d>

[5] <https://github.com/open-mmlab/OpenPCDet>

[6] <https://github.com/yukkysaito/autoware_perception>

[7] <https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars>

[8] <https://www.nuscenes.org/nuscenes>

[9] <https://www.argoverse.org/av2.html>

## (省略可能) 今後の拡張/未実装部分

<!-- このパッケージの今後の拡張について記述する。

例:
  現在、このパッケージはチャタリング障害物を適切に処理できません。知覚レイヤーにいくつかの確率的フィルタを追加して改善する予定です。
  また、グローバルにする必要があるパラメータがいくつかあります（例: 車両サイズ、最大ステアリングなど）。これらはリファクタリングされ、グローバルパラメータとして定義されるため、さまざまなノード間で同じパラメータを共有できます。
-->

[v0-encoder-default]: https://awf.ml.dev.web.auto/perception/models/pts_voxel_encoder_default.onnx
[v0-head-default]: https://awf.ml.dev.web.auto/perception/models/pts_backbone_neck_head_default.onnx
[v1-encoder-centerpoint]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_voxel_encoder_centerpoint.onnx
[v1-head-centerpoint]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_backbone_neck_head_centerpoint.onnx
[v1-encoder-centerpoint-tiny]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_voxel_encoder_centerpoint_tiny.onnx
[v1-head-centerpoint-tiny]: https://awf.ml.dev.web.auto/perception/models/centerpoint/v1/pts_backbone_neck_head_centerpoint_tiny.onnx

## 謝辞: deepen.ai の 3D アノテーションツールに関する貢献

## 法的情報

_nuScenesデータセットは、クリエイティブコモンズ帰属-非営利-継承同様条件4.0国際公共ライセンス下で非商用利用のために公開されています。
追加利用規約は<https://www.nuscenes.org/terms-of-use>で確認できます。
商用ライセンスについてのお問い合わせは<nuscenes@motional.com>までご連絡ください。_

## 自動運転ソフトウェアのアーキテクチャ

Autowareのアーキテクチャは、主に以下のようなコンポーネントから構成されます。

**ハードウェア層**

- センサー（カメラ、レーダー、LiDARなど）
- タスクを実行するためのコンピューティングリソース

**ソフトウェア層**

- Perception
  - 物体検出およびセグメンテーション
  - 車線検出
  - 道路区画検出
- Fusion
  - さまざまなセンサーからのデータを統合して、周囲のより包括的なモデルを生成
- Localization
  - 車両の自車位置を求める
- Planning
  - 安全で効率的な経路を生成
  - 道路上の障害物を回避
- Control
  - 車両の運動を制御
  - 加速、減速、ステアリングを担当

**Human-Machine Interface（HMI）**

- ドライバーとの対話のためのインターフェイス
- 車両の現在のステータスと、ソフトウェアが作成した計画に関する информацию表示

### コンポーネント間のインタラクション

コンポーネントは、次の順序でインタラクトします。

1. Perceptionモジュールは、センサーからのデータを処理して、周囲に関する情報を出力します。
2. Fusionモジュールは、Perceptionモジュールからのデータと、内部マップ、および自己位置推定情報などの他の情報源を組み合わせて、周囲の統一されたモデルを作成します。
3. Planningモジュールは、Fusionモジュールからの周囲モデルを使用して、安全で効率的な経路を生成します。
4. Controlモジュールは、Planningモジュールからの経路を「post resampling」して、車両の運動を制御するための低レベルの制御コマンドを生成します。
5. HMIは、車両の現在のステータスと、ソフトウェアが作成した計画に関する情報を表示します。

このインタラクションにより、Autowareは周囲を感知し、安全で効率的な経路を生成し、車両を制御することができ、自動運転の実現に不可欠です。

このドキュメントでは、Autowareのアーキテクチャの詳細について説明します。また、コンポーネントのインタラクションと、それらが自動運転システム全体にどのように寄与するかについても説明します。

