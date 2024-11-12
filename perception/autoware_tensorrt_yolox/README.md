# autoware_tensorrt_yolox

## 目的

このパッケージは、[YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) モデルに基づく、マルチヘッダー構造で、車、トラック、自転車、歩行者などの対象のオブジェクトを検出し、車、トラック、バス、歩行者、建屋、植生、道路、歩道などの対象のオブジェクトをセグメントします。

## 内部動作 / アルゴリズム

### 引用

<!-- cspell: ignore Zheng, Songtao, Feng, Zeming, Jian, semseg -->

鄭 Ge、劉 鬆涛、王 鋒、李 則明、孫 健、「YOLOX: 2021 年に YOLO シリーズを超える」、arXiv プレプリント arXiv:2107.08430、2021 [[ref](https://arxiv.org/abs/2107.08430)]

## 入力 / 出力

### 入力

| 名称       | 型                | 説明     |
| ---------- | ------------------- | --------------- |
| `in/image` | `sensor_msgs/Image` | 入力画像 |

### 出力

**自動運転ソフトウェアドキュメント**

Autowareは、オープンソース自動運転ソフトウェアスタックです。このドキュメントでは、Autowareのさまざまなコンポーネントとモジュールについて説明します。

**Planningコンポーネント**

* **Path Planning:** 目的地までの経路を生成します。
* **Speed Planning:** 経路上の最適速度プロファイルを作成します。
* **Trajectory Planning:** 経路と速度プロファイルを組み合わせた、車両の軌跡を生成します。

**Perceptionコンポーネント**

* **Localization:** 自車位置と周囲環境の地図を特定します。
* **Object Detection:** 周囲の車両、歩行者、障害物を検出します。
* **Obstacle Tracking:** 時間とともに動いている物体を追跡します。

**Controlコンポーネント**

* **Longitudinal Control:** 車両の速度を制御します。
* **Lateral Control:** 車両の向きを制御します。
* **Model Predictive Control (MPC):** 車両の挙動を予測し、最適な制御入力を決定します。

**Drivingポリシー**

* **Lane Keeping Assist:** 車両が車線内を維持するのに役立ちます。
* **Adaptive Cruise Control:** 前方車両との安全な間隔を維持します。
* **Collision Avoidance:** 衝突の可能性を検出し、回避策を実行します。

**安全機能**

* **Velocity Violation Check:** `post resampling`による速度逸脱量を確認します。
* **Acceleration Violation Check:** 加速度逸脱量を確認します。
* **Obstacle Proximity Check:** 障害物との近接性を監視します。

| 名称             | タイプ                                                | 説明                                                           |
| ---------------- | ----------------------------------------------------- | -------------------------------------------------------------------- |
| `out/objects`    | `tier4_perception_msgs/DetectedObjectsWithFeature` | 2Dバウンディングボックス付きの検出オブジェクト                         |
| `out/image`      | `sensor_msgs/Image`                                | 視覚化のための2Dバウンディングボックス付きのイメージ                    |
| `out/mask`       | `sensor_msgs/Image`                                | セマンティックセグメンテーションマスク                                |
| `out/color_mask` | `sensor_msgs/Image`                                | 視覚化のためのセマンティックセグメンテーションマスクの色付けイメージ |

## パラメータ

{{ json_to_markdown("perception/autoware_tensorrt_yolox/schema/yolox_s_plus_opt.schema.json") }}
{{ json_to_markdown("perception/autoware_tensorrt_yolox/schema/yolox_tiny.schema.json") }}

## 前提条件／既知の制限

検出された 2D バウンディングボックス（例: `out/objects`）に含まれるラベルは、次のいずれかになります。

- CAR
- PEDESTRIAN（"PERSON" も "PEDESTRIAN" として分類されます。）
- BUS
- TRUCK
- BICYCLE
- MOTORCYCLE

他のラベル（大文字小文字の区別はしない）が `label_file` パラメータで指定されたファイルに含まれる場合、それらは `UNKNOWN` としてラベル付けされ、検出された四角形は視覚化結果（`out/image`）に描画されます。

セマンティックセグメンテーションマスクは、各ピクセルが次のクラスのいずれかのインデックスであるグレースケール画像です。

| インデックス | シマンティック名 |
|---|---|
| 0     | 道路 |
| 1     | 建物 |
| 2     | 壁 |
| 3     | 障害物 |
| 4     | 交通信号 |
| 5     | 交通標識 |
| 6     | 歩行者 |
| 7     | 車両 |
| 8     | 自転車 |
| 9     | 道路 |
| 10    | 歩道 |
| 11    | 道路ペイント |
| 12    | 側石 |
| 13    | 交差点（その他） |
| 14    | 草木 |
| 15    | 空 |

## Onnxモデル

Ansibleスクリプトによって準備段階の環境でサンプルモデル(`yolox-tiny.onnx`)がダウンロードされます。ダウンロードされない場合は、[成果物の手動ダウンロード](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts)に従ってください。

オブジェクト検出推論後の一般的な後処理の1つである非最大値抑制(NMS)を高速化するために、通常のYOLOX(tiny)ネットワークの後には`EfficientNMS_TRT`モジュールが組み込まれています。

`EfficientNMS_TRT`モジュールには`score_threshold`と`nms_threshold`の固定値が含まれているため、ユーザーがこのモジュールを含むONNXモデルを指定した場合、これらのパラメーターは無視されます。

このパッケージは、`EfficientNMS_TRT`を組み込んだONNXと[公式のYOLOXリポジトリから公開されているモデル](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#download-onnx-models)（「プレーン」モデルと呼ぶ）の両方を許可します。

`yolox-tiny.onnx`に加えて、`yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls`という名前のカスタムモデルを利用することもできます。

このモデルはYOLOX-sをベースとしたマルチヘッダー構造モデルであり、`yolox-tiny`とほぼ同等の実行速度でより正確な検出を実行するように調整されています。

このモデルでより良い結果を得るには、`precision:=int8`、`calibration_algorithm:=Entropy`、`clip_value:=6.0`などの特定の実行引数を使用することをお勧めします。

ユーザーは`launch/yolox_sPlus_opt.launch.xml`を参照して、このモデルを使用する方法を確認できます。

検出結果に加えて、このモデルはポイントクラウドのフィルタリングに使用できる画像セマンティックセグメンテーション結果も出力します。

すべてのモデルは、TensorRT形式に自動的に変換されます。

変換後のファイルは、指定されたONNXファイルと同じディレクトリに`engine`ファイル名拡張子で保存され、次回の実行から再利用されます。

変換プロセスには時間がかかる場合があります（**通常10〜20分**）。また、変換が完了するまでは推論プロセスがブロックされるため、検出結果が公開されるまでに時間がかかります（**トピックリストに表示されるまで**）。

### パッケージで許容されるモデルの生成

PyTorchの`pth`フォーマットで保存されたユーザー独自のモデルをONNXに変換するには、公式リポジトリで提供されているコンバーターを利用できます。

便宜上、手順のみ以下に示します。

詳細については、[公式ドキュメント](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime#convert-your-model-to-onnx)を参照してください。

#### プレーンモデルの場合

1. 依存関係をインストール



   ```shell
   git clone git@github.com:Megvii-BaseDetection/YOLOX.git
   cd YOLOX
   python3 setup.py develop --user
   ```

2. pth を ONNX に変換する


   ```shell
   python3 tools/export_onnx.py \
     --output-name YOUR_YOLOX.onnx \
     -f YOUR_YOLOX.py \
     -c YOUR_YOLOX.pth
   ```

#### EfficientNMS_TRT埋め込みモデル向け

1. 依存関係のインストール


   ```shell
   git clone git@github.com:Megvii-BaseDetection/YOLOX.git
   cd YOLOX
   python3 setup.py develop --user
   pip3 install git+ssh://git@github.com/wep21/yolox_onnx_modifier.git --user
   ```

2. pth を ONNX に変換する


   ```shell
   python3 tools/export_onnx.py \
     --output-name YOUR_YOLOX.onnx \
     -f YOUR_YOLOX.py \
     -c YOUR_YOLOX.pth
     --decode_in_inference
   ```

3. YOLOX の最後に `EfficientNMS_TRT` を埋め込む


   ```shell
   yolox_onnx_modifier YOUR_YOLOX.onnx -o YOUR_YOLOX_WITH_NMS.onnx
   ```

## ラベルファイル

env準備プロセスの際に、サンプルラベルファイル（`label.txt`という名前）とセマンティックセグメンテーションカラーマップファイル（`semseg_color_map.csv`という名前）も自動的にダウンロードされます
(**注:** このファイルはCOCOデータセット（例：公式YOLOXリポジトリのモデル）のラベルを出力するモデルとは互換性がありません)。

このファイルは、クラスインデックス（YOLOXネットワークから出力される整数）とクラスラベル（理解を容易にする文字列）との対応を表します。このパッケージは、このファイルの順序に従って、クラスID（0から増加）をラベルにマッピングします。

## 参照リポジトリ

- <https://github.com/Megvii-BaseDetection/YOLOX>
- <https://github.com/wep21/yolox_onnx_modifier>
- <https://github.com/tier4/trt-yoloXP>

