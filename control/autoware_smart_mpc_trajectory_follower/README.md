# マークダウン形式で書かれた自動運転ソフトウェアに関するドキュメントの日本語訳

<p align="center">
  <a href="https://proxima-ai-tech.com/">
    <img width="500px" src="./images/proxima_logo.png">
  </a>
</p>

## スマート MPC トレジャクサリー追従

スマート MPC (Model Predictive Control) は、モデル予測制御と機械学習を組み合わせた制御アルゴリズムです。モデル予測制御の利点を継承すると同時に、機械学習を利用したデータドリブン手法でモデリングの難しさを解決します。

この技術により、環境データの収集が可能な限り、実装コストの高いモデル予測制御を比較的容易に運用できます。

<p align="center">
  <a href="https://youtu.be/j7bgK8m4-zg?si=p3ipJQy_p-5AJHOP)">
    <image width="700px" src="./images/autoware_smart_mpc.png">
  </a>
</p>

## 提供されている機能

このパッケージは、パス追従制御向けのスマート MPC ロジックと、学習および評価の仕組みを提供します。これらの機能を以下に示します。

### iLQR/MPPI ベースのトレジャクサリー追従制御

制御モードは "ilqr"、"mppi"、"mppi_ilqr" から選択でき、[mpc_param.yaml](./autoware_smart_mpc_trajectory_follower/param/mpc_param.yaml) の `mpc_parameter:system:mode` として設定できます。
"mppi_ilqr" モードでは、iLQR の初期値が MPPI ソリューションによって与えられます。

> [!注意]
> デフォルト設定では、"mppi" モードのパフォーマンスがサンプル数の不足により制限されます。この問題は、GPU サポートを導入する継続的な作業によって解決されています。

シミュレーションを実行するには、次のコマンドを実行します。


```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit trajectory_follower_mode:=smart_mpc_trajectory_follower
```

[!NOTE]
> 名目モデルの[nominal_param.yaml](./autoware_smart_mpc_trajectory_follower/param/nominal_param.yaml)が設定されている場合は、[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml)の`trained_model_parameter:control_application:use_trained_model`を`false`に設定してください。学習済みモデルを使用して実行するには、`trained_model_parameter:control_application:use_trained_model`を`true`に設定しますが、学習済みモデルは次の手順に従って生成する必要があります。

### モデルの学習と制御への反映

学習データを収集するには、autowareを起動して、走行を行い、次のコマンドでrosbagデータを記録します。


```bash
ros2 bag record /localization/kinematic_state /localization/acceleration /vehicle/status/steering_status /control/command/control_cmd /control/trajectory_follower/control_cmd /control/trajectory_follower/lane_departure_checker_node/debug/deviation/lateral /control/trajectory_follower/lane_departure_checker_node/debug/deviation/yaw /system/operation_mode/state /vehicle/status/control_mode /sensing/imu/imu_data /debug_mpc_x_des /debug_mpc_y_des /debug_mpc_v_des /debug_mpc_yaw_des /debug_mpc_acc_des /debug_mpc_steer_des /debug_mpc_X_des_converted /debug_mpc_x_current /debug_mpc_error_prediction /debug_mpc_max_trajectory_err /debug_mpc_emergency_stop_mode /debug_mpc_goal_stop_mode /debug_mpc_total_ctrl_time /debug_mpc_calc_u_opt_time
```

rosbagのディレクトリに[rosbag2.bash](./autoware_smart_mpc_trajectory_follower/training_and_data_check/rosbag2.bash)を移動させて、ディレクトリで下記コマンドを実行します


```bash
bash rosbag2.bash
```

rosbagデータをCSV形式に変換してモデルをトレーニングします。

> [!NOTE]
> 実行時に大量の端末が自動的に開きますが、rosbagデータの変換が完了すると自動的に閉じられます。
> このプロセスを開始してからすべての端末が閉じられるまで、Autowareは実行しないでください。

代わりに、Python環境で次のコマンドを実行することで同様の結果を得ることができます。


```python
from autoware_smart_mpc_trajectory_follower.training_and_data_check import train_drive_NN_model
model_trainer = train_drive_NN_model.train_drive_NN_model()
model_trainer.transform_rosbag_to_csv(rosbag_dir)
```

`rosbag_dir` は rosbag ディレクトリを表します。
この時、`rosbag_dir` 内のすべての CSV ファイルは最初に自動的に削除されます。

モデルのトレーニングの方法について説明します。
[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml) の `trained_model_parameter:memory_for_training:use_memory_for_training` が `true` に設定されている場合、LSTM を含むモデルに対するトレーニングが行われ、`false` に設定されている場合、LSTM を含まないモデルに対するトレーニングが行われます。
LSTM を使用すると、セル状態および隠れ状態は履歴時系列データに基づいて更新され、予測に反映されます。

トレーニングと検証に使用される rosbag ディレクトリのパス (`dir_0`、`dir_1`、`dir_2`、`dir_val_0`、`dir_val_1`、`dir_val_2`...) と、モデルを保存するディレクトリ (`save_dir`) から、Python 環境で次のようにモデルを保存できます。


```python
from autoware_smart_mpc_trajectory_follower.training_and_data_check import train_drive_NN_model
model_trainer = train_drive_NN_model.train_drive_NN_model()
model_trainer.add_data_from_csv(dir_0, add_mode="as_train")
model_trainer.add_data_from_csv(dir_1, add_mode="as_train")
model_trainer.add_data_from_csv(dir_2, add_mode="as_train")
...
model_trainer.add_data_from_csv(dir_val_0, add_mode="as_val")
model_trainer.add_data_from_csv(dir_val_1, add_mode="as_val")
model_trainer.add_data_from_csv(dir_val_2, add_mode="as_val")
...
model_trainer.get_trained_model()
model_trainer.save_models(save_dir)
```

`add_mode`が指定されなかった場合、または検証データが追加されなかった場合、トレーニングデータはトレーニングおよび検証に使用するために分割されます。

多項式回帰の実行後は、次のとおり、NNを残差でトレーニングできます。


```python
model_trainer.get_trained_model(use_polynomial_reg=True)
```

> [!NOTE]
> デフォルト設定では、回帰はいくつかの事前に選択された多項式によって実行されます。
> `get_trained_model` の引数として `use_selected_polynomial=False` が設定されている場合、`deg` 引数によって使用される多項式の最大次数を設定できます。

NN モデルが使用されず、多項式回帰のみが実行される場合は、次のコマンドを実行します:


```python
model_trainer.get_trained_model(use_polynomial_reg=True,force_NN_model_to_zero=True)
```

`model_for_test_drive.pth`と`polynomial_reg_info.npz`を`save_dir`からホームディレクトリに移動し、Trained Modelの反映のため、[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml)内の`trained_model_parameter:control_application:use_trained_model`を `true` に設定します。

### 性能評価

ここではサンプル車両のホイールベースが2.79 mであるところ、コントローラ側に2.0 mという誤った値を入力した場合の適応性能の検証を例として示します。
コントローラに2.0 mのホイールベースを与えるため、[nominal_param.yaml](./autoware_smart_mpc_trajectory_follower/param/nominal_param.yaml)の`nominal_parameter:vehicle_info:wheel_base`の値を2.0に設定し、次のコマンドを実行します。


```bash
python3 -m smart_mpc_trajectory_follower.clear_pycache
```

#### Autoware でのテスト

トレーニング前に公称モデルで Autoware に対する制御テストを実行するには、[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml) 内の `trained_model_parameter:control_application:use_trained_model` が `false` であることを確認し、「iLQR/MPPI に基づく Trajectory 以下の制御」で説明した方法で Autoware を起動します。今回は、次のルートをテストに使用します。

<p><img src="images/test_route.png" width=712px></p>

ROS バッグを記録し、ROS バッグを記録し、「モデルのトレーニングと制御への反映」で説明した方法でモデルをトレーニングし、生成されたファイル `model_for_test_drive.pth` と `polynomial_reg_info.npz` をホームディレクトリに移動します。サンプルモデルは [trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml) 内の `trained_model_parameter:memory_for_training:use_memory_for_training` が `true` に設定された条件下で動作します。[sample_models/wheel_base_changed](./sample_models/wheel_base_changed/) から取得できます。

> [!NOTE]
> トレーニングに使用されるデータは少量ですが、簡略化するために、このデータ量でどの程度のパフォーマンスが向上するかを確認します。

ここで取得したトレーニング済みモデルを使用して制御するには、`trained_model_parameter:control_application:use_trained_model` を `true` に設定し、同様に Autoware を起動し、同じルートで ROS バッグを記録しながら走行します。
走行が完了したら、「モデルのトレーニングと制御への反映」で説明した方法を使用して ROS バッグファイルを CSV 形式に変換します。`control/autoware_smart_mpc_trajectory_follower/autoware_smart_mpc_trajectory_follower/training_and_data_check/data_checker.ipynb` で `lateral_error_visualize` 関数を公称の ROS バッグファイル `rosbag_nominal` とトレーニング済み ROS バッグファイル `rosbag_trained` に対して実行すると、次のように横方向偏差のグラフを取得できます。


```python
lateral_error_visualize(dir_name=rosbag_nominal,ylim=[-1.2,1.2])
lateral_error_visualize(dir_name=rosbag_trained,ylim=[-1.2,1.2])
```

以下の結果が得られました。

<div style="display: flex; justify-content: center; align-items: center;">
    <img src="images/lateral_error_nominal_model.png">
    <img src="images/lateral_error_trained_model.png">
</div>

#### Pythonシミュレータでのテスト

まず、Pythonシミュレータでホイールベースを2.79 mにするには、次のファイルを作成し、名前を`sim_setting.json`にして`control/autoware_smart_mpc_trajectory_follower/autoware_smart_mpc_trajectory_follower/python_simulator`に保存します。

```
{
  "wheelbase": 2.79
}
```


```json
{ "wheel_base": 2.79 }
```

次に、`control/autoware_smart_mpc_trajectory_follower/autoware_smart_mpc_trajectory_follower/python_simulator` に移動した後、以下のコマンドを実行してパイソンシミュレータ上でスラローム走行をノミナル制御を使用してテストします。


```bash
python3 run_python_simulator.py nominal_test
```

運転の結果は `test_python_nominal_sim` に格納されます。

以下の結果が得られました。

<p style="text-align: center;">
    <img src="images/python_sim_lateral_error_nominal_model_wheel_base.png" width="712px">
</p>

最上段の中央は横方向逸脱量を表します。

純粋追従の制御下で、8の字走行データを使用してトレーニングを実行するには、以下のコマンドを実行します。

得られたモデルに基づく8の字走行と運転を使用してトレーニングを実行するには、以下のコマンドを実行します。


```bash
python3 run_python_simulator.py
```

運転の結果は `test_python_trined_sim` に格納されています。

[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml) の `trained_model_parameter:memory_for_training:use_memory_for_training` が `true` に設定された場合、以下の結果が得られました。

<p style="text-align: center;">
    <img src="images/python_sim_lateral_error_trained_model_lstm_wheel_base.png" width="712px">
</p>

[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml) の `trained_model_parameter:memory_for_training:use_memory_for_training` が `false` に設定された場合、以下の結果が得られました。

<p style="text-align: center;">
    <img src="images/python_sim_lateral_error_trained_model_wheel_base.png" width="712px">
</p>

横方向偏差が大幅に改善されていることがわかります。
ただし、LSTM の有無による運転の違いはあまり明らかではありません。

違いを明確にするために、例として `steer_time_delay` などのパラメータを試行できます。

まず、公称モデル設定の値をデフォルト値に戻すために、[nominal_param.yaml](./autoware_smart_mpc_trajectory_follower/param/nominal_param.yaml) の `nominal_parameter:vehicle_info:wheel_base` の値を 2.79 に設定して、次のコマンドを実行します。


```bash
python3 -m smart_mpc_trajectory_follower.clear_pycache
```

次に、`sim_setting.json` を次のように修正します:


```json
{ "steer_time_delay": 1.01 }
```

 このように、`steer_time_delay` を 1.01 秒に設定して実験を実施します。

公称モデルを使用した走行の結果は次のとおりです。

<p style="text-align: center;">
    <img src="images/python_sim_lateral_error_nominal_model_steer_time_delay.png" width="712px">
</p>

LSTM を使用した学習済みモデルを使用した走行の結果は次のとおりです。

<p style="text-align: center;">
    <img src="images/python_sim_lateral_error_trained_model_lstm_steer_time_delay.png" width="712px">
</p>

LSTM を使用しない学習済みモデルを使用した走行の結果は次のとおりです。

<p style="text-align: center;">
    <img src="images/python_sim_lateral_error_trained_model_steer_time_delay.png" width="712px">
</p>

LSTM を含むモデルを使用したパフォーマンスは、含まないモデルを使用したパフォーマンスよりも大幅に良好であることがわかります。

Python シミュレータに渡すことができるパラメータは次のとおりです。

| パラメータ                | 型        | 説明                                                                                                                                                                                                                                                                              |
| ------------------------ | ----------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| steer_bias               | 浮動      | ステアリングバイアス [rad]                                                                                                                                                                                                                                                        |
| steer_rate_lim           | 浮動      | ステアリングレートの制限 [rad/s]                                                                                                                                                                                                                                                        |
| vel_rate_lim             | 浮動      | 加速度制限 [m/s^2]                                                                                                                                                                                                                                                        |
| wheel_base               | 浮動      | ホイールベース [m]                                                                                                                                                                                                                                                        |:
| steer_dead_band          | 浮動      | ステアリングデッドバンド [rad]                                                                                                                                                                                                                                                        |
| adaptive_gear_ratio_coef | リスト[浮動] | タイヤ角からステアリングホイール角への速度依存ギア比に関する情報を指定する 6 個の長さを持つフローティングポイントのリスト                                                                                                                                             |
| acc_time_delay           | 浮動      | 加速度遅延時間 [s]                                                                                                                                                                                                                                                          |
| steer_time_delay         | 浮動      | ステアリング遅延時間 [s]                                                                                                                                                                                                                                                          |
| acc_time_constant        | 浮動      | 加速度時定数 [s]                                                                                                                                                                                                                                                            |
| steer_time_constant      | 浮動      | ステアリング時定数 [s]                                                                                                                                                                                                                                                           |
| accel_map_scale          | 浮動      | 加速度入力値から実際の加速度の実現への対応する歪みを拡大するパラメータ。 <br>対応情報は `control/autoware_smart_mpc_trajectory_follower/autoware_smart_mpc_trajectory_follower/python_simulator/accel_map.csv` に格納されています。 |
| acc_scaling              | 浮動      | 加速度スケーリング                                                                                                                                                                                                                                                          |
| steer_scaling            | 浮動      | ステアリングスケーリング                                                                                                                                                                                                                                                         |
| vehicle_type             | 整数      | 0 から 4 までの値を取ります。 <br>各車両タイプについては以下で説明します。                                                                                                                                                                                                              |

例えば、シミュレーション側に0.01 [rad]のステア係数バイアスと0.001 [rad]のステアデッドバンドを与える場合は、`sim_setting.json`を次のように編集します。


```json
{ "steer_bias": 0.01, "steer_dead_band": 0.001 }
```

##### vehicle_type_0

この車両タイプは、コントローラーで使用される既定の車両タイプと一致します。

## 自動運転ソフトウェアのパラメータ設定

| パラメータ | 値 |
|---|---|
| ホイールベース | 2.79 |
| 加速度応答遅れ時間 | 0.1 |
| ステアリング応答遅れ時間 | 0.27 |
| 加速度応答時間定数 | 0.1 |
| ステアリング応答時間定数 | 0.24 |
| 加速度スケーリング | 1.0 |

##### vehicle_type_1

このvehicle typeは大型バスを想定しています。

## 自動運転ソフトウェアに関するドキュメントの翻訳

### パラメータ

| パラメータ | 値 |
|---|---|
| ホイールベース | 4.76 |
| 加速度タイム遅延 | 1.0 |
| ステアリングタイム遅延 | 1.0 |
| 加速度タイム定数 | 1.0 |
| ステアリングタイム定数 | 1.0 |
| 加速度スケーリング | 0.2 |

##### vehicle_type_2

この車両タイプは、小型バスを想定しています。

| パラメータ           | 値 |
| ------------------- | ----- |
| ホイールベース          | 4.76 |
| 加速度遅延時間      | 0.5 |
| 操舵遅延時間       | 0.5 |
| 加速度タイムコンスタント   | 0.5 |
| 操舵タイムコンスタント   | 0.5 |
| 加速度スケーリング         | 0.5 |

##### vehicle_type_3

この車両種は小型車両を想定しています。

## 自動運転ソフトウェア

### パラメータ

| パラメータ | 値 |
|---|---|
| ホイーベース | 1.335 |
| 加速度時間遅延 | 0.3 |
| 操舵時間遅延 | 0.3 |
| 加速度時間定数 | 0.3 |
| 操舵時間定数 | 0.3 |
| 加速度スケーリング | 1.5 |

##### vehicle_type_4

この車両タイプは小型ロボット向けです。

| パラメータ           | 値 |
| ------------------- | ---- |
| ホイールベース          | 0.395 |
| 加速遅延時間      | 0.2   |
| ステアリング遅延時間    | 0.2   |
| 加速時間定数   | 0.2   |
| ステアリング時間定数 | 0.2   |
| 加速度スケーリング         | 1.0   |

#### Pythonシミュレーターでの自動テスト

ここでは、シミュレーション側にモデルパラメータの事前定義された範囲を提供し、制御側に定数モデルパラメータを提供することで、適応性能をテストする方法について説明します。

[run_sim.py](./autoware_smart_mpc_trajectory_follower/python_simulator/run_sim.py)で設定されたパラメータ変更範囲内で走行実験を実行するには、`control/autoware_smart_mpc_trajectory_follower/autoware_smart_mpc_trajectory_follower/python_simulator`に移動し、次のコマンドを実行します。


```bash
python3 run_sim.py --param_name steer_bias
```

実験手順はステアバイアスについて説明しましたが、他のパラメーターでも同様の方法を使用できます。

一度に制限値以外のすべてのパラメーターのテストを実行するには、次のコマンドを実行します:


```bash
python3 run_auto_test.py
```

Auto_testの実行結果を`auto_test`ディレクトリに保存しています。
実行が完了したら、[plot_auto_test_result.ipynb](./autoware_smart_mpc_trajectory_follower/python_simulator/plot_auto_test_result.ipynb)を実行して次の結果を取得してください。

<p style="text-align: center;">
    <img src="images/proxima_test_result_with_lstm.png" width="712px">
</p>

オレンジの線は純粋追従のフィギュラエイト走行を使用してトレーニングされた中間モデルを示しており、青い線は中間モデルとフィギュラエイト走行の両方からのデータを使用してトレーニングされた最終モデルを示しています。
ほとんどの場合、十分な性能が得られますが、大型バスを想定した`vehicle_type_1`では、横方向逸脱量はおよそ2 mで、納得のいくものではありません。

`run_sim.py`で次のパラメータを設定できます。

| パラメータ                  | 型               | 説明                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ------------------------- | ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| USE_TRAINED_MODEL_DIFF    | bool               | トレーニングされたモデルの導関数が制御に反映されるか?                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| DATA_COLLECTION_MODE      | DataCollectionMode | どの方式でトレーニングデータを収集するか <br> "DataCollectionMode.ff": フィードフォワード入力で直線走行 <br> "DataCollectionMode.pp": ピュアパーシュート制御で8の字走行 <br> "DataCollectionMode.mpc": mpcでスラローム走行                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| USE_POLYNOMIAL_REGRESSION | bool               | NNの前に多項式回帰を実行するか?                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| USE_SELECTED_POLYNOMIAL   | bool               | USE_POLYNOMIAL_REGRESSIONがTrueの場合、あらかじめ選択された多項式のみを使用して多項式回帰を実行する。 <br> 多項式の選択は、車両の公称モデルに基づくいくつかパラメータのシフトを吸収できるように意図されている。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   

> [!注意]
> `run_sim.py` を実行すると、`run_sim.py` で設定された `use_trained_model_diff` が [trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml) で設定された `trained_model_parameter:control_application:use_trained_model_diff` より優先されます。

#### Pure Pursuit の走行データのカーネル密度推定

Pure Pursuit 走行から取得したデータの分布は、カーネル密度推定を使用して表示できます。これを行うには、[density_estimation.ipynb](./autoware_smart_mpc_trajectory_follower/python_simulator/density_estimation.ipynb) を実行します。

密度推定の最小値と走行結果の横方向逸脱の相関関係は低くなっています。横方向逸脱値をより適切に予測するスカラー指標を開発中です。

## 公称パラメータの変更とその再ロード

車両モデルの公称パラメータは、[nominal_param.yaml](./autoware_smart_mpc_trajectory_follower/param/nominal_param.yaml) ファイルを編集することで変更できます。
公称パラメータを変更した後、次のコマンドを実行してキャッシュを削除する必要があります。
```bash
rm ~/.cache/autoware/autoware_smart_mpc_trajectory_follower/params/
```


```bash
python3 -m smart_mpc_trajectory_follower.clear_pycache
```

**通常パラメータは次のとおりです。**

| パラメータ                                        | 型  | 説明                                     |
| ------------------------------------------------ | ----- | -------------------------------------- |
| nominal_parameter:vehicle_info:wheel_base        | float | ホイールベース [m]                           |
| nominal_parameter:acceleration:acc_time_delay    | float | 加速度タイム遅延 [s]                       |
| nominal_parameter:acceleration:acc_time_constant | float | 加速度タイム定数 [s]                      |
| nominal_parameter:steering:steer_time_delay      | float | ステアリングタイム遅延 [s]      |
| nominal_parameter:steering:steer_time_constant   | float | ステアリングタイム定数 [s]         |

## コントロールパラメータの変更と再読み込み

制御パラメータは、ファイル[mpc_param.yaml](./autoware_smart_mpc_trajectory_follower/param/mpc_param.yaml)と[trained_model_param.yaml](./autoware_smart_mpc_trajectory_follower/param/trained_model_param.yaml)を変更することで変更できます。
パラメータの変更はAutowareを再起動することで反映できますが、次のコマンドを使用することでAutowareを実行中のまま反映できます。


```bash
ros2 topic pub /pympc_reload_mpc_param_trigger std_msgs/msg/String "data: ''" --once
```

主な制御パラメータは次のとおりです。

### `mpc_param.yaml`

| パラメータ                               | 型         | 説明 |
| ----------------------------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| mpc_parameter:system:mode                | str         | 制御モード <br>"ilqr": iLQRモード <br>"mppi": MPPIモード <br>"mppi_ilqr": iLQRの初期値はMPPIソリューションによって与えられる |
| mpc_parameter:cost_parameters:Q            | list[float] | 状態のステージコスト <br>長さ8のリスト、順に: 直線偏差、横方向偏差、速度偏差、偏航角偏差、加速度偏差、ステア偏差、加速度入力偏差、ステア入力偏差のコスト重み |
| mpc_parameter:cost_parameters:Q_c          | list[float] | 状態の次のtiming_Q_cに相当するhorizon内のコスト <br>リストの構成要素の対応はQの場合と同じ |
| mpc_parameter:cost_parameters:Q_f          | list[float] | 状態の終端コスト <br>リストの構成要素の対応はQの場合と同じ |
| mpc_parameter:cost_parameters:R            | list[float] | 長さ2のリスト、R[0]は加速度入力値の変化率のコストの重み、R[1]はステア入力値の変化率のコストの重み |
| mpc_parameter:mpc_setting:timing_Q_c       | list[int]   | 状態のステージコストがQ_cに設定されるhorizon番号 |
| mpc_parameter:compensation:acc_fb_decay    | float       | MPC外部のコンペンセータの観測された加速度値と予測加速度値の間の誤差を積分する際の減哀係数 |
| mpc_parameter:compensation:acc_fb_gain     | float       | 加速度補償のゲイン |
| mpc_parameter:compensation:max_error_acc   | float       | 最大加速度補償 (m/s^2) |
| mpc_parameter:compensation:steer_fb_decay  | float       | MPC外部のコンペンセータにおける観測ステアリング値と予測ステアリング値の間の誤差を積分する際の減衰係数 |
| mpc_parameter:compensation:steer_fb_gain   | float       | ステアリング補償のゲイン |
| mpc_parameter:compensation:max_error_steer | float       | 最大ステアリング補償 (rad) |

### `trained_model_param.yaml`

| パラメータ                                                           | 型   | 説明                                                                                                                                                                                                                                                                                             |
| ------------------------------------------------------------------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| trained_model_parameter:control_application:use_trained_model       | bool | 学習済みモデルが制御に反映されるかどうかを示します。                                                                                                                                                                                                                                                    |
| trained_model_parameter:control_application:use_trained_model_diff  | bool | 学習済みモデルの微分値が制御に反映されるかどうかを示します。 <br> use_trained_modelがTrueの場合にのみ意味があり、Falseの場合は、運動の方程式の微分には公称モデルが使用され、学習済みモデルは予測にのみ使用されます。 |
| trained_model_parameter:memory_for_training:use_memory_for_training | bool | 学習のためにLSTMを含むモデルを使用するかどうかを示します。                                                                                                                                                                                                                                                        |
| trained_model_parameter:memory_for_training:use_memory_diff         | bool | LSTMの前時点でのセル状態および隠れ状態に対する微分が制御に反映されるかどうかを示します。                                                                                                                                                                                                            |

## 減速停止モードの解除要求

予測軌跡がターゲット軌跡から大きく逸脱した場合、システムは減速停止モードに入り、車両は停止します。
減速停止モードをキャンセルして車両を走行可能にするには、次のコマンドを実行します。


```bash
ros2 topic pub /pympc_stop_mode_reset_request std_msgs/msg/String "data: ''" --once
```

## 制限事項

- 初期位置/姿勢が目標から大きく離れている場合は開始できない可能性があります。

- 最初の制御の開始時に numba 関数をコンパイルするまで、Plannin の終了まで少し時間がかかる場合があります。

- ゴール付近の停止動作では、制御が別の簡単な制御則に切り替わります。結果として、停止動作はゴール付近を除いて機能しない場合があります。加速度マップが大幅にシフトしている場合も停止は困難です。

- `vehicle_type_1` のように大型バス向けに想定されているように、ダイナミクスが公称モデルから大きく逸脱している場合、うまく制御できない可能性があります。

