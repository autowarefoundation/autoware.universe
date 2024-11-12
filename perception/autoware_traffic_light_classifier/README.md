# traffic_light_classifier

## 目的

traffic_light_classifierは、交通信号機の周囲をトリミングした画像を使用して、交通信号機ラベルを分類するパッケージです。このパッケージには、`cnn_classifier`と`hsv_classifier`という2つの分類モデルがあります。

## 内部動作/アルゴリズム

### cnn_classifier

交通信号機のラベルは、EfficientNet-b1またはMobileNet-v2によって分類されます。
完全に、日本の交通信号の83400（トレーニング用58600、評価用14800、テスト用10000）TIER IV社内イメージを使用して微調整しました。
モデルの情報は次のとおりです。

## 自動運転ソフトウェアのドキュメント

**Planning コンポーネント**

**モジュール**

* **Local Planning**
  * 自車位置の追跡
  * 障害物検出
  * 経路生成
  * 'post resampling' 軌跡生成
* **Behavior Planning**
  * 経路追従制御
  * 速度計画
  * 車線維持
* **Path Planning**
  * ダイナミック経路生成
  * 障害物回避

**センシングコンポーネント**

* **LiDAR**
  * 3D نقطه群データの取得
  * 障害物検出
* **カメラ**
  * 視覚データの取得
  * レーンマーカー検出
  * 交通標識認識
* **IMU (慣性計測装置)**
  * 加速度や角速度の測定
  * 自車位置の推定

**制御コンポーネント**

* **Longitudinal Controller**
  * 車両速度の制御
  * 加速度逸脱量を最小化
* **Lateral Controller**
  * 車両ヨーの制御
  * 車線逸脱量を最小化

**Autoware**

Autoware は自動運転ソフトウェアのオープンソースプラットフォームです。主要なモジュールには以下が含まれます。

* **Perception**
  * センサーデータの処理と融合
  * 障害物認識と分類
* **Fusion**
  * ローカライゼーション、マッピング、トラッキング
  * 環境の動的モデルの構築
* **Control**
  * 車両の制御とガイダンス
  * PATH と BEHAVIOR Planner の実装

**評価方法**

* **シミュレーションパフォーマンス**
  * レーン逸脱量、クラッシュ回避数
* **実世界パフォーマンス**
  * テスト走行における速度逸脱量、加減速逸脱量
* **主要性能指標 (KPI)**
  * 時間あたりの走行距離
  * 介入率

### hsv_classifier

信号機の色（緑、黄、赤）はHSVモデルで分類します。

### ラベルについて

メッセージタイプは、[ウィーン条約](https://ja.wikipedia.org/wiki/%E3%83%AF%E3%82%A4%E3%83%B3%E5%90%88%E7%B4%84%E7%B4%A0%E9%81%93%E3%81%BE%E3%81%97%E3%82%87%E3%81%86%E8%B7%AF%E4%BA%A4%E8%A8%80%E3%81%B8%E3%81%AE%E9%95%B7%E7%B5%90%E3%81%A8%E7%94%BB%E5%93%81%E3%81%AE%E5%90%88%E7%B4%84%E7%B4%A0%E9%81%93)で提案されている統一された道路標識に準拠するように設計されています。このアイデアは[Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/merge_requests/16)でも提案されています。

ノードが受信するラベルの命名規則があります。信号機1つは、カンマで区切られた次の文字列で表されます。`color1-shape1, color2-shape2` .

例えば、単純な赤と赤十字の信号機ラベルは "red-circle, red-cross" と表現する必要があります。

これらの色と形状は、以下の通りメッセージに割り当てられます:
![TrafficLightDataStructure.jpg](./image/TrafficLightDataStructure.jpg)

## 入出力

### 入力

| Name            | Type                                                  | Description            |
| --------------- | ------------------------------------------------------- | ---------------------- |
| `~/input/image` | `sensor_msgs::msg::Image`                            | 入力画像            |
| `~/input/rois`  | `tier4_perception_msgs::msg::TrafficLightRoiArray` | 信号機のroi |

### 出力

このドキュメントは、[Autoware](https://github.com/autowarefoundation/autoware.auto/blob/master/docs/en/planning_with_mpc/planning_with_mpc.md)のPlanningコンポーネントにおけるModel Predictive Control (MPC)のアルゴリズムの概要を説明します。

#### Planningのアルゴリズム

Planningは、複数のPlanningモジュールで構成されています。
* **TrajectoryGenerator** プランニング可能な軌道を生成します。
* **Planner** 軌道の評価および最適化を実施します。

#### モデル予測制御 (MPC)

MPCは、Planningで使用される最適化アルゴリズムです。MPCでは、現在の状態からの一連の制御入力を計算し、将来のシステム挙動を最適化します。

MPCのアルゴリズムは次の手順で行われます:

1. **問題の設定:** プランニングモジュールから、自車位置、障害物、目標状態などの情報を取得します。
2. **コスト関数の設定:** 目標状態への追従、衝突回避、経路逸脱などの目的を反映するコスト関数を定義します。
3. **最適化:** コスト関数の最小化を目的とする最適化問題を解きます。
4. **制御入力の生成:** 最適化の結果から、最初の制御入力を生成します。
5. **計画の再計算:** 制御入力が適用されると、自車位置が変化します。MPCは、変化した自車位置に基づいて計画を再計算します。

#### PlanningにおけるMPC

Planningでは、MPCを使用して、以下のタスクを実行します:

* **経路追従:** 目標経路に沿った軌道を生成します。
* **障害物回避:** 障害物を回避するための軌道を生成します。
* **速度制御:** 目標速度を維持するための制御入力を計算します。

MPCのPlanningにおける主要な利点は、他の車両や障害物などの障害物を考慮して、将来のシステム挙動を予測できることです。これにより、Planningはリアルタイムで安全かつ効率的な軌道を作成できます。

#### 実装の詳細

MPCのAutowareにおける実装では、以下を使用しています:

* **コスト関数:** 位置逸脱量、速度逸脱量、加速度逸脱量の最小化
* **最適化器:** 'post resampling`を備えたquadprog
* **制御入力:** ステアリング角、加速度

#### 参考資料

* [Autoware Foundation Planning](https://github.com/autowarefoundation/autoware.auto/tree/master/ros/autoware/core/planning_with_mpc)
* [Model Predictive Control Theory and Design](https://link.springer.com/book/10.1007/978-0-89838-330-2)

| 名前 | 種類 | 説明 |
|---|---|---|
| `~/output/traffic_signals` | `tier4_perception_msgs::msg::TrafficLightArray` | 分類済み信号 |
| `~/output/debug/image` | `sensor_msgs::msg::Image` | デバッグ用画像 |

## パラメータ

### ノードパラメータ

| 名前 | タイプ | 説明 |
|---|---|---|
| `classifier_type` | int | 値が `1` の場合、cnn_classifier が使用されます。 |
| `data_path` | str | packages データおよびアーティファクトのディレクトリパス |
| `backlight_threshold` | float | 強度がこの値を超えた場合、対応する RoI で UNKNOWN で上書きします。この値がはるかに大きい場合、ノードは過酷な逆光状況でのみ上書きするということに注意してください。したがって、この機能を使用しない場合は、この値を `1.0` に設定してください。この値は `[0.0, 1.0]` にできます。上書きされた信号の信頼性は `0.0` に設定されます。 |

### コアパラメータ

#### cnn_classifier

| 名称                    | タイプ            | 説明                          |
| ----------------------- | --------------- | ------------------------------------ |
| `classifier_label_path` | str             | モデルファイルのパス               |
| `classifier_model_path` | str             | ラベルファイルのパス               |
| `classifier_precision`  | str             | TensorRT精度、`fp16`または`int8` |
| `classifier_mean`       | vector\<double> | 3チャネル入力画像の平均           |
| `classifier_std`        | vector\<double> | 3チャネル入力画像の標準偏差            |
| `apply_softmax`         | bool            | ソフトマックスを適用するかどうか         |

#### hsv_classifier

## HSV カラーベース分類器

このコンポーネントは、Hue, Saturation, Value（HSV）カラー空間を使用して障害物を分類します。これにより、セグメンテーションのフィルタリングや、Planningコンポーネントに対する追加的な入力として使用できます。

### 入出力

**入力:**

* `PointCloud`: 障害物のPointCloudデータ
* `CameraInfo`: カメラの内部および外部パラメータ

**出力:**

* `ClassifiedPointCloud`: HSV色相に基づいて分類された障害物のPointCloudデータ

### パラメータ

| パラメータ | 説明 | デフォルト値 |
|---|---|---|
| `hue_threshold`: HSV色相の閾値 (度) | 30 |
| `saturation_threshold`: HSV彩度の閾値 | 0.1 |
| `value_threshold`: HSV明度の閾値 | 0.1 |
| `post resampling`: ポストリサンプリングフィルターを使用するかどうか | True |

### アルゴリズム

このコンポーネントは、各点のHSV色相、彩度、明度を計算し、指定されたしきい値と比較します。指定されたしきい値を超える点は、障害物として分類されます。

### 注意事項

* この分類器は単一のカメラからのデータだけを使用することに注意してください。より堅牢な障害物検出を行うには、複数のカメラからのデータを使用する必要があります。
* パラメータは特定のシーンやライティング条件に応じて調整する必要があります。
* この分類器は、距離情報を使用しません。そのため、障害物が自車位置に近いか遠いかを区別できません。

| 名 | 型 | 説明 |
|---|---|---|
| `green_min_h` | int | 緑色の最小色相 |
| `green_min_s` | int | 緑色の最小彩度 |
| `green_min_v` | int | 緑色の最小明度 |
| `green_max_h` | int | 緑色の最大色相 |
| `green_max_s` | int | 緑色の最大彩度 |
| `green_max_v` | int | 緑色の最大明度 |
| `yellow_min_h` | int | 黄色の最小色相 |
| `yellow_min_s` | int | 黄色の最小彩度 |
| `yellow_min_v` | int | 黄色の最小明度 |
| `yellow_max_h` | int | 黄色の最大色相 |
| `yellow_max_s` | int | 黄色の最大彩度 |
| `yellow_max_v` | int | 黄色の最大明度 |
| `red_min_h` | int | 赤色の最小色相 |
| `red_min_s` | int | 赤色の最小彩度 |
| `red_min_v` | int | 赤色の最小明度 |
| `red_max_h` | int | 赤色の最大色相 |
| `red_max_s` | int | 赤色の最大彩度 |
| `red_max_v` | int | 赤色の最大明度 |

## 交通信号機分類器モデルのトレーニング

### 概要

このガイドでは、**[mmlab/mmpretrain](https://github.com/open-mmlab/mmpretrain)** リポジトリを使用して交通信号機分類器モデルをトレーニングし、**[mmlab/mmdeploy](https://github.com/open-mmlab/mmdeploy)** を使用して展開する方法に関する詳細な手順を説明します。独自のデータセットを使用して独自の交通信号機分類器モデルを作成する場合は、以下に示す手順に従ってください。

### データの準備

#### サンプルデータセットの使用

Autoware は交通信号機の分類のためのトレーニング手順を示すサンプルデータセットを提供しています。このデータセットは、赤、緑、黄色のラベルに分類された 1045 枚の画像で構成されています。このサンプルデータセットを使用するには、**[リンク](https://autoware-files.s3.us-west-2.amazonaws.com/dataset/traffic_light_sample_dataset.tar.gz)** からダウンロードして、選択した指定フォルダーに解凍してください。

#### カスタムデータセットの使用

交通信号機分類器をトレーニングするには、各サブフォルダーが異なるクラスを表す、構造化されたサブフォルダー形式を採用します。以下に示すのは、データセット構造の例です。


```python
DATASET_ROOT
    ├── TRAIN
    │    ├── RED
    │    │   ├── 001.png
    │    │   ├── 002.png
    │    │   └── ...
    │    │
    │    ├── GREEN
    │    │    ├── 001.png
    │    │    ├── 002.png
    │    │    └──...
    │    │
    │    ├── YELLOW
    │    │    ├── 001.png
    │    │    ├── 002.png
    │    │    └──...
    │    └── ...
    │
    ├── VAL
    │       └──...
    │
    │
    └── TEST
           └── ...


```

### インストール

#### 前提条件

<!-- cspell:ignore Miniconda -->

**ステップ 1.** [公式ウェブサイト](https://mmpretrain.readthedocs.io/en/latest/get_started.html)からMinicondaをダウンロードしてインストールします。

**ステップ 2.** conda仮想環境を作成してアクティベートします


```bash
conda create --name tl-classifier python=3.8 -y
conda activate tl-classifier
```

**手順 3.** PyTorchのインストール

CUDA 11.6と互換性のあるPyTorchをインストールしてください。これは、現在のAutowareの要件です。


```bash
conda install pytorch==1.13.1 torchvision==0.14.1 pytorch-cuda=11.6 -c pytorch -c nvidia
```

#### mmlab/mmpretrain のインストール

**手順 1.** mmpretrain をソースからインストール


```bash
cd ~/
git clone https://github.com/open-mmlab/mmpretrain.git
cd mmpretrain
pip install -U openmim && mim install -e .
```

### トレーニング

MMPretrainは構成ファイルによって制御されるトレーニングスクリプトを提供します。
継承デザインパターンを利用することで、構成ファイルとしてPythonファイルを使用してトレーニングスクリプトを簡単に調整できます。

この例では、MobileNetV2モデルを使ったトレーニングステップを示しますが、EfficientNetV2、EfficientNetV3、ResNetなど、別の分類モデルを使用する柔軟性があります。

#### 構成ファイルの作成

`configs`フォルダ内で使用するモデルの構成ファイルを作成します。


```bash
touch ~/mmpretrain/configs/mobilenet_v2/mobilenet-v2_8xb32_custom.py
```

好みのテキストエディターでコンフィグレーションファイルを開き、提供されたコンテンツのコピーを作成してください。データセットのパスに合わせて **data_root** 変数を調整します。モデル、データセット、スケジューラーのコンフィグレーションパラメーターは、好みに合わせてカスタマイズできます。


```python
# Inherit model, schedule and default_runtime from base model
_base_ = [
    '../_base_/models/mobilenet_v2_1x.py',
    '../_base_/schedules/imagenet_bs256_epochstep.py',
    '../_base_/default_runtime.py'
]

# Set the number of classes to the model
# You can also change other model parameters here
# For detailed descriptions of model parameters, please refer to link below
# (Customize model)[https://mmpretrain.readthedocs.io/en/latest/advanced_guides/modules.html]
model = dict(head=dict(num_classes=3, topk=(1, 3)))

# Set max epochs and validation interval
train_cfg = dict(by_epoch=True, max_epochs=50, val_interval=5)

# Set optimizer and lr scheduler
optim_wrapper = dict(
    optimizer=dict(type='SGD', lr=0.001, momentum=0.9))
param_scheduler = dict(type='StepLR', by_epoch=True, step_size=1, gamma=0.98)

dataset_type = 'CustomDataset'
data_root = "/PATH/OF/YOUR/DATASET"

# Customize data preprocessing and dataloader pipeline for training set
# These parameters calculated for the sample dataset
data_preprocessor = dict(
    mean=[0.2888 * 256, 0.2570 * 256, 0.2329 * 256],
    std=[0.2106 * 256, 0.2037 * 256, 0.1864 * 256],
    num_classes=3,
    to_rgb=True,
)

# Customize data preprocessing and dataloader pipeline for train set
# For detailed descriptions of data pipeline, please refer to link below
# (Customize data pipeline)[https://mmpretrain.readthedocs.io/en/latest/advanced_guides/pipeline.html]
train_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='Resize', scale=224),
    dict(type='RandomFlip', prob=0.5, direction='horizontal'),
    dict(type='PackInputs'),
]
train_dataloader = dict(
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        ann_file='',
        data_prefix='train',
        with_label=True,
        pipeline=train_pipeline,
    ),
    num_workers=8,
    batch_size=32,
    sampler=dict(type='DefaultSampler', shuffle=True)
)

# Customize data preprocessing and dataloader pipeline for test set
test_pipeline = [
    dict(type='LoadImageFromFile'),
    dict(type='Resize', scale=224),
    dict(type='PackInputs'),
]

# Customize data preprocessing and dataloader pipeline for validation set
val_cfg = dict()
val_dataloader = dict(
    dataset=dict(
        type=dataset_type,
        data_root=data_root,
        ann_file='',
        data_prefix='val',
        with_label=True,
        pipeline=test_pipeline,
    ),
    num_workers=8,
    batch_size=32,
    sampler=dict(type='DefaultSampler', shuffle=True)
)

val_evaluator = dict(topk=(1, 3,), type='Accuracy')

test_dataloader = val_dataloader
test_evaluator = val_evaluator

```

#### トレーニングの開始


```bash
cd ~/mmpretrain
python tools/train.py configs/mobilenet_v2/mobilenet-v2_8xb32_custom.py
```

トレーニングログと重みは、`work_dirs/mobilenet-v2_8xb32_custom` フォルダに保存されます。

### PyTorchモデルからONNXモデルへの変換

#### mmdeployのインストール

「mmdeploy」ツールセットは、トレーニングされたモデルをさまざまなターゲットデバイスにデプロイするように設計されています。
その機能により、PyTorchモデルをONNX形式にシームレスに変換できます。


```bash
# Activate your conda environment
conda activate tl-classifier

# Install mmenigne and mmcv
mim install mmengine
mim install "mmcv>=2.0.0rc2"

# Install mmdeploy
pip install mmdeploy==1.2.0

# Support onnxruntime
pip install mmdeploy-runtime==1.2.0
pip install mmdeploy-runtime-gpu==1.2.0
pip install onnxruntime-gpu==1.8.1

#Clone mmdeploy repository
cd ~/
git clone -b main https://github.com/open-mmlab/mmdeploy.git
```

#### PyTorchモデルをONNXモデルに変換する


```bash
cd ~/mmdeploy

# Run deploy.py script
# deploy.py script takes 5 main arguments with these order; config file path, train config file path,
# checkpoint file path, demo image path, and work directory path
python tools/deploy.py \
~/mmdeploy/configs/mmpretrain/classification_onnxruntime_static.py\
~/mmpretrain/configs/mobilenet_v2/train_mobilenet_v2.py \
~/mmpretrain/work_dirs/train_mobilenet_v2/epoch_300.pth \
/SAMPLE/IAMGE/DIRECTORY \
--work-dir mmdeploy_model/mobilenet_v2
```

## ONNXモデル変換

変換されたONNXモデルは、`mmdeploy/mmdeploy_model/mobilenet_v2`フォルダに保存されます。

ONNXモデルを入手したら、起動ファイルで定義されているパラメータ（例：`model_file_path`、`label_file_path`、`input_h`、`input_w`など）を更新します。[tier4_perception_msgs::msg::TrafficLightElement](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_perception_msgs/msg/traffic_light/TrafficLightElement.msg)で定義されているラベルのみサポートすることに注意してください。

## 仮定/既知の制限

<!-- 実装の仮定と制約事項を記述してください。

例:
  このアルゴリズムは障害物が動かないことを前提としています。そのため、障害物の回避を開始した後に障害物が急激に動いた場合、衝突する可能性があります。
  また、このアルゴリズムはブラインドスポットを考慮しません。一般に、センシング性能の限界により近すぎる障害物は見えないため、障害物に対する十分なマージンをとってください。
-->

## (オプション) エラー検出および処理

<!-- エラーを検出する方法と、それらから回復する方法を記述してください。

例:
  このパッケージは、最大20個の障害物を処理できます。それ以上の障害物が検出された場合、このノードは失敗し、診断エラーを引き起こします。
-->

## (オプション) パフォーマンス特性

<!-- 複雑さなどのパフォーマンス情報を記述します。ボトルネックにならない場合は、必要ありません。

例:

### 複雑さ

このアルゴリズムはO（N）です。

### 処理時間

...
-->

<!-- cspell:ignore Mingxing, Quoc, PMLR -->

## 参照/外部リンク

[1] M. Sandler、A. Howard、M. Zhu、A. Zhmoginov、L. Chen、「MobileNetV2：逆残差と線形ボトルネック」、2018 IEEE/CVFコンピュータビジョンアンドパターン認識会議、ソルトレイクシティ、ユタ州、2018年、pp。 4510-4520、doi：10.1109/CVPR.2018.00474。

[2] タン、ミングシン、クオック・レ。「EfficientNet：畳み込みニューラルネットワークのモデルスケーリングを再考する」。機械学習に関する国際会議。PMLR、2019年。

## (オプション) 将来の拡張/未実装部分

<!-- このパッケージの将来の拡張を記述します。

例:
  現在、このパッケージはチャタリング障害物を適切に処理できません。知覚レイヤーに確率フィルタを追加して改善する予定です。
  また、グローバルにすべきパラメータがいくつかあります（例：車両サイズ、最大ステアリングなど）。これらはリファクタリングされてグローバルパラメータとして定義されるため、異なるノード間で同じパラメータを共有できます。
-->

