# autoware_shape_estimation

## 目的

このノードは、ポイントクラスタがラベルに従ってフィットする、洗練されたオブジェクトシェイプ（バウンディングボックス、円柱、凸包）を計算します。

## 内部動作/アルゴリズム

### フィッティングアルゴリズム

- バウンディングボックス

  - L字シェイプフィッティング: 詳細については以下の文献を参照してください
  - MLベースのシェイプフィッティング: 詳細については、以下のMLベースのシェイプフィッティングの実装のセクションを参照してください

- 円柱

  `cv::minEnclosingCircle`

- 凸包

  `cv::convexHull`

## 入出力

### 入力

| 名称      | タイプ                                                     | 説明                           |
| --------- | -------------------------------------------------------- | ------------------------------------- |
| `input` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | ラベル付きクラスタによる検出オブジェクト |

### 出力

**自動運転ソフトウェアのドキュメント**

**要約**

このドキュメントでは、Autowareの自己位置推定コンポーネントの計画モジュールについての詳細な技術情報を提供します。このモジュールは、自己位置推定Planningのロジックを担当し、Planningモジュールによって生成された軌道計画に基づいて、車両の運動を制御します。

**自己位置推定 Planningモジュール**

自己位置推定Planningモジュールは、次のタスクを実行します。

- 自車位置の追従
- 軌道計画のpost resampling
- 計画された軌道の修正
- 計画された軌道の速度逸脱量と加速度逸脱量の計算

**自車位置追従**

自車位置追従サブモジュールは、センサーデータを使用して自己位置を推定し、追従します。このサブモジュールは、Extended Kalman Filter（EKF）を使用して、GPS、IMU、車輪速度などのセンサーデータのフュージョンを行います。

**軌道計画のpost resampling**

軌道計画のpost resamplingサブモジュールは、Planningモジュールから生成された軌道計画に再サンプリングを行います。これにより、軌道計画が車両の現在の運動状態に適合します。

**計画された軌道の修正**

計画された軌道の修正サブモジュールは、障害物やその他の環境要因に応じて、計画された軌道を修正します。このサブモジュールは、制約条件ベースの最適化を使用して、安全で効率的な軌道を作成します。

**計画された軌道の速度逸脱量と加速度逸脱量の計算**

計画された軌道の速度逸脱量と加速度逸脱量の計算サブモジュールは、計画された軌道に対して、現在の車両の速度と加速度を比較して、逸脱量を計算します。これらの逸脱量は、車両の制御システムにフィードバックされ、車両の動作を計画された軌道に近づけるために使用されます。

**結論**

Autowareの自己位置推定Planningモジュールは、自己位置の追従、軌道計画のpost resampling、計画された軌道の修正、計画された軌道の速度逸脱量と加速度逸脱量の計算を含む、自己位置推定Planningの重要なコンポーネントです。このモジュールは、車両の安全で効率的な動作を確実にします。

| 名前             | タイプ                                             | 説明                         |
| ---------------- | ------------------------------------------------ | ----------------------------------- |
| `output/objects` | `autoware_perception_msgs::msg::DetectedObjects` | 洗練された形状を持つ検出されたオブジェクト |

## パラメータ

{{ json_to_markdown("perception/autoware_shape_estimation/schema/shape_estimation.schema.json") }}

## MLベースの形状実装

このモデルは、ポイントクラウドと物体ラベル（カメラの検出/Apolloインスタンスセグメンテーションによって提供）を入力として、物体の3Dバウンディングボックスを出力します。

MLベースの形状推定アルゴリズムは、バックボーンとしてPointNetモデルを使用して、物体の3Dバウンディングボックスを推定します。このモデルは、車両ラベル（車、トラック、バス、トレーラー）を持つNuScenesデータセットでトレーニングされています。

実装されたモデルは、入力ポイントクラウドを正規空間に変換するためにSTN（空間変換ネットワーク）と結合され、PointNetを使用して物体の3Dバウンディングボックスを予測します。_RGB-Dデータからの3D物体検出用のFrustum PointNets_論文のバウンディングボックス推定部分が参考として使用されました。

このモデルは、各オブジェクトに対して以下の出力を予測します。

- オブジェクト中心のx、y、z座標
- オブジェクトの進行方向角の分類結果（角度分類には12のビンを使用 - 各ビンは30度）
- オブジェクトの進行方向角の残差
- オブジェクトサイズの分類結果
- オブジェクトサイズの残差

### MLベースの形状推定モデルのトレーニング

モデルをトレーニングするには、正解の3Dバウンディングボックスアノテーションが必要です。3D物体検出アルゴリズムのトレーニングにmmdetection3dリポジトリを使用する場合、これらの正解アノテーションは保存され、データ拡張に使用されます。これらのアノテーションは、形状推定モデルを効果的にトレーニングするための不可欠なデータセットとして使用されます。

### データセットの準備

#### MMDetection3Dの必須コンポーネントをインストールする

**ステップ 1.** [公式ウェブサイト](https://mmpretrain.readthedocs.io/en/latest/get_started.html)からMinicondaをダウンロードしてインストールします。

**ステップ 2.** conda仮想環境を作成してアクティブにする


```bash
conda create --name train-shape-estimation python=3.8 -y
conda activate train-shape-estimation
```

**手順 3.** PyTorch をインストールする


```bash
conda install pytorch torchvision -c pytorch
```

#### mmdetection3d のインストール

**手順 1.** MIM を使用して MMEngine、MMCV、および MMDetection をインストールする


```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**ステップ2.** AutowareのMMDetection3Dフォークをインストールする


```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### NuScenesデータセットの準備

**ステップ 1**. [公式ウェブサイト](https://www.nuscenes.org/download)からNuScenesデータセットをダウンロードして、選択したフォルダーに展開します。

**注意:** NuScenesデータセットは大きく、かなりのディスク容量が必要です。続ける前に十分なストレージがあることを確認してください。

**ステップ 2**. データセットフォルダーへのシンボリックリンクを作成します


```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**ステップ3.** 次のコマンドを実行してNuScenesデータを準備:


```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes --only-gt-database True
```

#### クローニング バウンディング ボックス エスティメータ モデル


```bash
git clone https://github.com/autowarefoundation/bbox_estimator.git
```

#### データセットをトレーニングセットと検証セットに分割する


```bash

cd bbox_estimator
python3 utils/split_dbinfos.py --dataset_path /path/to/mmdetection3d/data/nuscenes --classes 'car' 'truck' 'bus' 'trailer'  --train_ratio 0.8
```

### モデルの学習と展開

#### モデルの学習


```bash
# Detailed training options can be found in the training script
# For more details, run `python3 train.py --help`
python3 train.py --dataset_path /path/to/mmdetection3d/data/nuscenes
```

#### モデルの展開

Autoware.Autoのパイプラインの `planning` モジュールでは、予測モデルを展開して使用します。このモデルは、他のモーションプランニングモジュールへの入力として、走行中の障害物の軌跡を予測します。

この予測モデルには、さまざまな手法があります。代表的な手法としては、カルマンフィルタ、自己回帰和分移動平均（ARIMA）、隠れマルコフモデル（HMM）などがあります。

使用する手法の選択は、予測タスクの特性によって異なります。たとえば、交通流予測にはARIMAが適していますが、歩行者の振る舞いを予測するにはHMMがより適しています。

この予測モデルは、トレーニングデータセットを使用してトレーニングされます。このデータセットには、障害物の位置、速度、加速度に関する情報が含まれます。トレーニング後、モデルは新しい入力に対して障害物の軌跡を予測するために使用されます。

予測された軌跡は、他のモーションプランニングモジュールによって、衝突を防ぐための安全な経路を計画するために使用されます。

### モデルの評価

予測モデルの性能は、次のような指標を使用して評価できます。

* **平均絶対誤差（MAE）**： 障害物軌跡の予測値と実際の値との差の平均
* **平均二乗誤差（MSE）**： 予測値と実際の値との二乗差の平均
* **相関関係**： 予測値と実際の値との間の相関関係

### モデルの展開

トレーニングされた予測モデルは、Autoware.Autoパイプラインの `planning` モジュールに展開できます。これは、次の手順で行えます。

1. モデルをYAML形式で保存します。
2. YAMLファイルをパイプラインに読み込みます。
3. パイプラインを起動します。

展開後、予測モデルはリアルタイムで障害物の軌跡を予測するために使用されます。この情報は、他のモーションプランニングモジュールによって安全な経路を計画するために使用されます。


```bash
# Convert the trained model to ONNX format
python3 onnx_converter.py --weight_path /path/to/best_checkpoint.pth --output_path /path/to/output.onnx
```

`model_path`パラメータにONNXモデルの出力を`shape_estimation`ノードの起動ファイルで指定します。

## 想定/既知の制限

未定

## 参考資料/外部リンク

論文内のL字型フィッティングの実装:


```bibtex
@conference{Zhang-2017-26536,
author = {Xiao Zhang and Wenda Xu and Chiyu Dong and John M. Dolan},
title = {Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners},
booktitle = {2017 IEEE Intelligent Vehicles Symposium},
year = {2017},
month = {June},
keywords = {autonomous driving, laser scanner, perception, segmentation},
}
```

**RGB-Dデータからの3D物体検出のためのFrustum PointNets**

## はじめに

このドキュメントでは、RGB-Dデータから3D物体検出を行うためのFrustum PointNets手法について説明します。具体的には、次のようなトピックを扱います。

- Frustum PointNetsの手法の概要
- AutowareにおけるFrustum PointNetsの統合
- パフォーマンスの評価

## 手法

Frustum PointNetsは、RGB-Dデータから3D物体を検出するための2段階の手法です。

### 第1段階：フラストム生成

最初の段階では、RGB入力から3Dフラストムを生成します。フラストムとは、自車位置から見通せる空間の錐台形の近似です。フラストムは、点群データの3D空間内の空間的制約を提供します。

### 第2段階：3D物体検出

第2段階では、生成されたフラストム内の点群データを使用して、3D物体検出を行います。Frustum PointNetsアーキテクチャは、3D空間内の点群データを処理するように設計されており、物体のサイズ、形状、向きを推定できます。

## Autowareへの統合

Frustum PointNets手法は、Autowareフレームワークに統合されています。統合されたモジュールは、次のような機能を提供します。

- RGB-Dデータからのフラストム生成
- Frustum PointNetsネットワークによる3D物体検出
- 検出された物体の3Dバウンディングボックスの出力

## パフォーマンス

Autowareに統合されたFrustum PointNetsモジュールの性能は、大規模なデータセットで評価されています。評価結果は、この手法がRGB-Dデータからの3D物体検出において、高い精度と効率性を示していることを示しています。

## Planningへの影響

Frustum PointNetsモジュールは、Planningコンポーネントに次の影響を与えます。

- **障害物検出の向上：**より正確な3D物体検出により、Planningコンポーネントは周辺環境をより正確に認識できます。
- **軌道の改善：**より正確な障害物検出により、Planningコンポーネントは、衝突を回避しながら、より効率的な軌道を計算できます。

## 結論

Frustum PointNetsは、RGB-Dデータからの3D物体検出に高い精度を実現する強力な手法です。Autowareフレームワークに統合することにより、Planningコンポーネントの障害物検出と軌道計算の能力を向上させることができます。


````bibtex
@inproceedings{qi2018frustum,
title={Frustum pointnets for 3d object detection from rgb-d data},
author={Qi, Charles R and Liu, Wei and Wu, Chenxia and Su, Hao and Guibas, Leonidas J},
booktitle={Proceedings of the IEEE conference on computer vision and pattern recognition},
pages={918--927},
year={2018}
}```


````

