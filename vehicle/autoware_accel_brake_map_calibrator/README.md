# accel_brake_map_calibrator

本ノードの役割は、`autoware_raw_vehicle_cmd_converter` ノードで使用される `accel_map.csv` / `brake_map.csv` を自動的に較正することです。

基本マップ（デフォルトはlexusのもの）が、ロードされた運転データで反復的に更新されます。

## 較正方法

### キャリブレータの起動

Autoware の起動後、次のコマンドで `autoware_accel_brake_map_calibrator` を実行し、自律走行を行います。注：自律走行中に同じ車両インターフェイスを使用できる場合は、手動走行でもデータを収集できます（例：ジョイスティックの使用）。


```sh
ros2 launch autoware_accel_brake_map_calibrator accel_brake_map_calibrator.launch.xml rviz:=true
```

もしrosbagファイルを使用したい場合は、以下のコマンドを実行してください。


```sh
ros2 launch autoware_accel_brake_map_calibrator accel_brake_map_calibrator.launch.xml rviz:=true use_sim_time:=true
ros2 bag play <rosbag_file> --clock
```

**キャリブレーション**

パラメータ `progress_file_output` を true に設定してキャリブレーションすると、ログファイルが [autoware_accel_brake_map_calibrator ディレクトリ]/config/ に出力されます。キャリブレーション後は、[autoware_accel_brake_map_calibrator ディレクトリ]/config/accel_map.csv および [autoware_accel_brake_map_calibrator ディレクトリ]/config/brake_map.csv でアクセルマップとブレーキマップを確認できます。

### キャリブレーションプラグイン

`rviz:=true` オプションを使用すると、RViz が以下のキャリブレーションプラグインと一緒に表示されます。

<p align="center">
<img src="./media/calib_rviz_image_sample.png" width="600">
</p>

現在の状態（速度とペダル）がプラグインに表示されます。現在のセルの色は、現在のデータが有効か無効かに応じて緑色または赤色で変わります。次の条件を満たさないデータは無効と見なされ、推定には使用されません。これは、過激なデータ（ペダルが急速に移動しているときなど）がキャリブレーションの精度を低下させるためです。

- 速度とペダルの状態がインデックス値から一定の範囲内にある
- ステア値、ペダル速度、ピッチ値などが対応するしきい値よりも小さい
- 速度がしきい値よりも大きい

詳細なパラメータは、パラメータセクションで説明されています。

**注:**キャリブレーション中は、現在の状態が赤色か緑色かを気にしないでください。すべてのセルが赤色になるまでデータを取得し続ければ十分です。

マップ内の各セルの値は最初は灰色ですが、そのセル内の有効なデータの数が蓄積されるにつれて青色から赤色に変化します。マップの各セルが赤色に近くなるまでキャリブレーションを続けることをお勧めします。特に、停止付近の性能は 0 ～ 6 m/s の速度と +0.2 ～ -0.4 のペダル値に大きく依存するため、これらの領域に焦点を当てることが望ましいです。

### 診断

`accel brake map_calibrator` はキャリブレーションのステータスに応じて診断メッセージを発行します。診断タイプ `WARN` は、現在のアクセル/ブレーキマップが不正確であると推定されることを示します。この状況では、アクセル/ブレーキマップの再キャリブレーションを実行することを強くお勧めします。

| 状態                   | 診断タイプ | 診断メッセージ                                 | 説明                                        |
| ----------------------- | ---------------- | ------------------------------------------------- | -------------------------------------------- |
| キャリブレーション不要 | `OK`             | "OK"                                             |                                            |
| キャリブレーション必要 | `WARN`           | "アクセル/ブレーキマップのキャリブレーションが必要です。" | 現在のアクセル/ブレーキマップの精度は低いかもしれません。 |

この診断ステータスは、以下のROSトピックでも確認できます。


```sh
ros2 topic echo /accel_brake_map_calibrator/output/update_suggest
```

診断タイプが `WARN` の場合、このトピックに `True` がパブリッシュされ、accel/brake マップの更新が提案されます。

### accel / brake マップ精度の評価

マップの精度は、観測加速度と予測加速度の **2乗平均平方根誤差 (RMSE)** で評価されます。

**用語:**

- `観測加速度`: ホイール速度の微分から計算される現在の車両の加速度。

- `予測加速度`: Autoware が期待する、accel/brake マップの元の出力。この値は、現在のペダルと速度を使用して計算されます。

以下のトピックで追加のエラー情報を確認できます。

- `/accel_brake_map_calibrator/output/current_map_error` : `csv_path_accel/brake_map` パスで設定された元のマップのエラー。この値が大きい場合、元のマップは正確ではありません。
- `/accel_brake_map_calibrator/output/updated_map_error` : このノードで較正されたマップのエラー。この値が大きい場合、較正品質は低いです。
- `/accel_brake_map_calibrator/output/map_error_ratio` : 元のマップと更新されたマップのエラー率 (比率 = 更新 / 現在のマップ)。この値が 1 未満の場合、マップを更新することが望ましいです。

### 較正データを視覚化する

較正のプロセスは以下のように視覚化できます。これらのスクリプトには較正のログ出力が必要なので、視覚化のために較正を実行している間は `pedal_accel_graph_output` のパラメータを true に設定する必要があります。

#### アクセルと加速度の関係のグラフを視覚化する

以下のコマンドで、較正で使用されたデータのプロットを表示します。各速度範囲のプロットでは、ペダルと加速度の関係が分布表示されており、ピッチ角に応じて色付けされた生のデータポイントが含まれています。


```sh
ros2 run autoware_accel_brake_map_calibrator view_plot.py
```

#### 加速度/速度/ペダルのデータに関する統計を視覚化

以下のコマンドは、キャリブレーションの統計を示します。

- 平均値
- 標準偏差
- データ数

各地図セル内のすべてのデータについて。


```sh
ros2 run autoware_accel_brake_map_calibrator view_statistics.py
```

### キャリブレートされた加速 / ブレーキマップをいつでも保存する方法

次のコマンドでいつでも加速およびブレーキマップを保存できます。


```sh
ros2 service call /accel_brake_map_calibrator/update_map_dir tier4_vehicle_msgs/srv/UpdateAccelBrakeMap "path: '<accel/brake map directory>'"
```

AutowareでAccel_map.csv/brake_map.csvを読み込むデフォルトのディレクトリにaccelマップとブレーキマップを保存するには、RVizプラグイン（AccelBrakeMapCalibratorButtonPanel）を使用します。

1.「パネル」タブをクリックして、AccelBrakeMapCalibratorButtonPanelを選択します。

   ![add_panel](./media/add_panel.png)

2.パネルを選択すると、ボタンがRVizの下部に表示されます。

   ![calibrator_button_panel](./media/calibrator_button_panel.png)

3.ボタンを押すと、accel /ブレーキマップが保存されます。（キャリブレーターノードが実行されていない場合など、特定の状況ではボタンを押せません。）

   ![push_calibration_button](./media/push_calibration_button.png)

## パラメータ

## システムパラメータ

| 名称                                 | タイプ   | 説明                                                                                                                                                                                                  | デフォルト値                                                    |
| :-----------------------------------| :----- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------- |
| update_method                         | string | マップのキャリブレーション手法を選択できます。"update_offset_each_cell" は、マップ上の各グリッドセルのオフセットを計算します。 "update_offset_total" は、マップの合計オフセットを計算します。 | "update_offset_each_cell"                                        |
| get_pitch_method                      | string | "tf": tf からピッチを取得する、"none": ピッチ検証とピッチ補正を実行できません。                                                                                                                                     | "tf"                                                             |
| pedal_accel_graph_output              | bool   | true の場合、ペダルアクセルのログを出力します。                                                                                                                                                            | true                                                             |
| progress_file_output                  | bool   | true の場合、更新プロセスのログと CSV ファイルを出力します。                                                                                                                                                     | false                                                            |
| default_map_dir                       | str    | デフォルトのマップディレクトリ                                                                                                                                                                            | [autoware_raw_vehicle_cmd_converter のディレクトリ]/data/default/ |
| calibrated_map_dir                    | str    | キャリブレーションされたマップディレクトリ                                                                                                                                                                   | [autoware_accel_brake_map_calibrator のディレクトリ]/config/      |
| update_hz                             | double | 更新の Hz                                                                                                                                                                                                  | 10.0                                                             |

## アルゴリズムパラメータ

| 名称                     | 種別   | 説明                                                                                                                                      | デフォルト値 |
| :----------------------- | :----- | :---------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| initial_covariance       | double | 初期加速度マップの共分散 (共分散が大きいほど更新速度が向上)                                                                                 | 0.05          |
| velocity_min_threshold   | double | これより小さい速度は更新に使用されない。                                                                                                      | 0.1           |
| velocity_diff_threshold  | double | 速度データがこのしきい値よりグリッド参照速度 (中央値) から離れている場合、関連データは更新に使用されない。                                    | 0.556         |
| max_steer_threshold      | double | ステアリング角度がこの値より大きい場合、関連データは更新に使用されない。                                                                        | 0.2           |
| max_pitch_threshold      | double | ピッチ角度がこの値より大きい場合、関連データは更新に使用されない。                                                                        | 0.02          |
| max_jerk_threshold       | double | 自車加速度から計算された自車ジャークがこの値より大きい場合、関連データは更新に使用されない。                                              | 0.7           |
| pedal_velocity_thresh    | double | ペダル移動速度がこの値より大きい場合、関連データは更新に使用されない。                                                                    | 0.15          |
| pedal_diff_threshold     | double | 現在ペダル値がこのしきい値より前の値から離れている場合、関連データは更新に使用されない。                                                  | 0.03          |
| max_accel                | double | 速度ソースから計算される加速度の最大値。                                                                                                 | 5.0           |
| min_accel                | double | 速度ソースから計算される加速度の最小値。                                                                                                 | -5.0          |
| pedal_to_accel_delay     | double | 加速度への actuation_cmd との間の遅延時間 (更新ロジックで考慮)。                                                                           | 0.3           |
| update_suggest_thresh    | double | RMSE 比率がこの値になる更新の提案フラグ (RMSE 比率: [新しいマップの RMSE] / [元のマップの RMSE])。                                           | 0.7           |
| max_data_count           | int    | 視覚化用。各グリッドのデータ数がこの値になると、グリッドの色が赤になる。                                                                     | 100           |
| accel_brake_value_source | string | 加速度/ブレーキソースとして actuation_status または actuation_command を使用するかどうか。                                                          | status        |

## テストユーティリティースクリプト

### 定常加速/制動コマンドテスト

これらのスクリプトは、加速/制動マップ較正のテストに役立ちます。これらは、ユーザーがCLIを介して対話的に与える定数の加速/制動値を持つ`ActuationCmd`を生成します。

- accel_tester.py
- brake_tester.py
- actuation_cmd_publisher.py

`accel/brake_tester.py`は、CLIからターゲットの加速/制動コマンドを受け取ります。ターゲット値を`actuation_cmd_publisher.py`に送信し、これが`ActuationCmd`を生成します。次のコマンドを別のターミナルで実行して、これらのスクリプトを実行でき、以下のスクリーンショットのようになります。


```bash
ros2 run autoware_accel_brake_map_calibrator accel_tester.py
ros2 run autoware_accel_brake_map_calibrator brake_tester.py
ros2 run autoware_accel_brake_map_calibrator actuation_cmd_publisher.py
```

![actuation_cmd_publisher_util](./media/actuation_cmd_publisher_util.png)

## 校正手法

2つのアルゴリズム、[update_offset_four_cell_around](#update_offset_four_cell_around-1)と[update_offset_each_cell](#update_offset_each_cell)が加速度マップ更新用に選択できます。詳細についてはリンクを参照してください。

### データの前処理

校正の前に、欠損または使用不能なデータ（例：ハンドル角が大きすぎる）をまず取り除く必要があります。以下のパラメーターは、除去するデータを決定するために使用されます。

#### パラメーター

| 名                   | 説明                  | デフォルト値 |
| ---------------------- | ---------------------------- | ------------- |
| velocity_min_threshold | 最小速度を除外     | 0.1           |
| max_steer_threshold    | 大きなステアリング角を除外 | 0.2           |
| max_pitch_threshold    | 大きなピッチ角を除外    | 0.02          |
| max_jerk_threshold     | 大きなジャークを除外           | 0.7           |
| pedal_velocity_thresh  | 大きなペダリング速度を除外 | 0.15          |

### update_offset_each_cell

各格子に十分近いデータを使用して、再帰的最小二乗法（RLS）手法で更新します。

**利点** : 各グリッドに十分近いデータのみがキャリブレーションに使用されるため、各ポイントで正確な更新が可能です。

**欠点** : 除外するデータ量が多いため、キャリブレーションに時間がかかります。

#### パラメータ

データの選択は、次のしきい値によって決定されます。

| 名前 | 既定値 |
| ------------------- | -------- |
| velocity_diff_threshold | 0.556 |
| pedal_diff_threshold | 0.03 |

#### フォーミュラの更新

$$
\begin{align}
    \theta[n]=&
    \theta[n-1]+\frac{p[n-1]x^{(n)}}{\lambda+p[n-1]{(x^{(n)})}^2}(y^{(n)}-\theta[n-1]x^{(n)})\\
    p[n]=&\frac{p[n-1]}{\lambda+p[n-1]{(x^{(n)})}^2}
\end{align}
$$

#### 変数

| 変数名     | 記号     |
|---|---|
| 共分散     | $p[n-1]$   |
| マップオフセット | $\theta[n]$ |
| forgetting factor | $\lambda$   |
| フィー | $x(=1)$     |
| 測定加速度 | $y$         |

### update_offset_four_cell_around [1]

RLS（再帰的最小二乗法）により、新規取得データ周囲の 4 つのグリッドにおけるオフセットを更新します。この更新では、線形補間を考慮するため、適切な重み付けが適用されます。そのため、しきい値によるデータの除去を行う必要はありません。

**利点:** 適切な重み付けを使用してデータ周囲の 4 つのグリッドにおいて更新を行うため、データが無駄になりません。
**欠点:** データの極端なバイアスにより、精度の低下が発生する可能性があります。たとえば、図 2 の $Z_{RR}$ 付近でデータ $z(k)$ がバイアスされた場合、4 つの周囲点（$Z_{RR}$、$Z_{RL}$、$Z_{LR}$、および $Z_{LL}$）で更新が実行されますが、$Z_{LL}$ の精度は期待できません。

<!-- cspell: ignore fourcell -->
<p align="center">
  <img src="./media/fourcell_RLS.png" width="600">
</p>

#### 実装

更新式については、[1] の式 (7)～(10) を参照してください。さらに、Anti-Windup については [1] の式 (17) および (18) が使用されます。

### 参考資料

<!-- cspell: ignore Lochrie, Doljevic, Yongsoon, Yoon, IFAC -->

[1] [Gabrielle Lochrie, Michael Doljevic, Mario Nona, Yongsoon Yoon, Anti-Windup Recursive Least Squares Method for Adaptive Lookup Tables with Application to Automotive Powertrain Control Systems, IFAC-PapersOnLine, Volume 54, Issue 20, 2021, Pages 840-845](https://www.sciencedirect.com/science/article/pii/S240589632102320X)

