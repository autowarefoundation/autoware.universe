## autoware_detected_object_feature_remover

## 目的

`autoware_detected_object_feature_remover` パッケージは、トピックの型を `DetectedObjectWithFeatureArray` から `DetectedObjects` に変換するためのものです。

## 内部動作 / アルゴリズム

## 入出力

### 入力
- `/detected_objects_with_features`

### 出力
- `/detected_objects`

## パラメータ

| 名前 | 説明 | 型 | デフォルト値 |
|---|---|---|---|
| `remove_probability` | 確率が低すぎる対象物を削除する | ブール値 | `false` |
| `remove_points` | 点数の低い対象物を削除する | ブール値 | `false` |
| `threshold_probability` | 確率のしきい値 | 倍精度浮動小数点 | `0.5` |
| `threshold_points` | 点数のしきい値 | 倍精度浮動小数点 | `10` |

## インターフェース

### サブスクライバー

- **トピック:** `/detected_objects_with_features`
- **型:** `autoware_msgs::DetectedObjectWithFeatureArray`
- **説明:** 特徴を持つ検出されたオブジェクトの配列

### パブリッシャー

- **トピック:** `/detected_objects`
- **型:** `autoware_msgs::DetectedObjects`
- **説明:** 検出されたオブジェクトの配列

## 使い方

このパッケージを使用するには、次の手順を実行します。

1. ROS ワークスペースで `catkin_make` を実行して、パッケージをビルドします。
2. ノードを実行します。

```bash
rosrun autoware_detected_object_feature_remover autoware_detected_object_feature_remover
```

### パラメータの調整

パラメータは、`launch` ファイルまたは動的に ROS パラメータサーバーを使用して調整できます。

**launch ファイルを使用する場合:**

```xml
<launch>
    <node pkg="autoware_detected_object_feature_remover" type="autoware_detected_object_feature_remover" name="autoware_detected_object_feature_remover">
        <param name="remove_probability" value="true" />
        <param name="threshold_probability" value="0.6" />
    </node>
</launch>
```

**動的に ROS パラメータサーバーを使用する場合:**

```bash
rosparam set /autoware_detected_object_feature_remover/remove_probability true
rosparam set /autoware_detected_object_feature_remover/threshold_probability 0.6
```

## 制限事項

- このパッケージは、`autoware_msgs::DetectedObjectWithFeature` メッセージのみを処理します。他のタイプのメッセージはサポートされていません。
- 確率または点数がしきい値以下のオブジェクトはすべて削除されます。部分的にしきい値を満たすオブジェクトは保持されません。

## トラブルシューティング

パッケージに問題がある場合は、次の手順を実行します。

1. ROS コンソールログをチェックして、エラーメッセージがないか確認します。
2. パラメータが正しく設定されていることを確認します。
3. 入力トピックが正しく発行されていることを確認します。
4. 出力トピックが正しく受信されていることを確認します。

| 名前   | タイプ | 説明 |
| --------- | --------- | --------- |
| `~/input` | `tier4_perception_msgs::msg::DetectedObjectWithFeatureArray` | 特徴を持つ検出オブジェクト |

### 出力

### 自動運転ソフトウェアのドキュメント

このドキュメントでは、自動運転ソフトウェアの概要と、その主要コンポーネントであるPerception、Planning、Controlの役割について説明します。

**Perceptionコンポーネント**

Perceptionコンポーネントは、センサーデータから周囲環境に関する情報を抽出します。これには以下が含まれます。

- 物体検出（車両、歩行者、自転車など）
- フリースペースの検出（障害物のない領域）
- レーン検出

この情報を使用して、Perceptionコンポーネントは、自車位置に対する周囲環境の正確な認識を提供します。

**Planningコンポーネント**

Planningコンポーネントは、認識された周囲環境情報に基づいて、車両の安全で効率的な経路を計画します。これには以下が含まれます。

- グローバルパスプランニング（目的地までの高レベルの経路）
- ローカルパスプランニング（障害物回避を考慮した経路の微調整）

Planningコンポーネントは、制約の中で最適な経路を決定し、自車を目的地まで安全に誘導します。

**Controlコンポーネント**

Controlコンポーネントは、Planningコンポーネントが出力した経路に基づいて、車両を制御します。これには以下が含まれます。

- ステアリング制御
- ブレーキ制御
- スロットル制御

Controlコンポーネントは、車両の運動を制御し、計画された経路に従って自車を移動させます。

**Autowareソフトウェア**

Autowareは、オープンソースの自動運転ソフトウェアプラットフォームです。Perception、Planning、Controlコンポーネントを含む、自動運転に必要な主要機能を提供します。

Autowareを使用すると、開発者はカスタマイズして特定の車両や環境に適応させることができ、自動運転機能を独自のアプリケーションに統合できます。

**パフォーマンス指標**

自動運転ソフトウェアのパフォーマンスを評価するための重要な指標には以下が含まれます。

- **経路逸脱量：**計画された経路からの自車位置の逸脱量
- **速度逸脱量：**計画された速度からの自車速度の逸脱量
- **加速度逸脱量：**計画された加速度からの自車加速度の逸脱量
- **post resampling**：計画された経路からの自車位置の最大逸脱量

これらの指標は、自動運転システムの安全、効率、正確さを評価するために使用されます。

| 名称       | タイプ                                              | 説明      |
| ---------- | --------------------------------------------------- | --------- |
| `~/output` | `autoware_perception_msgs::msg::DetectedObjects` | 検出オブジェクト |

## パラメータ

なし

## 想定/既知の制限事項

