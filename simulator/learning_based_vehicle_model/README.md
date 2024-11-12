# 学習済みモデル

これは、`simple_planning_simulator`パッケージで使用される、Python学習済みモデルの設計ドキュメントです。

## 目的/ユースケース

<!-- 必須 -->
<!-- 考慮事項:
    - この機能を実装した理由は? -->

このライブラリは、PythonのモデルとPSIM（C++）のインタフェースを作成します。複雑なC++の実装をせずにPython学習済みモデルをPSIMに迅速に展開するために使用されます。

## 設計

<!-- 必須 -->
<!-- 考慮事項:
    - どのように機能するのか -->

このパッケージのアイデアは、シミュレーションに使用したいモデルが複数のサブモデル（例：ステアリングモデル、駆動モデル、車両運動学など）で構成されていることです。これらのサブモデルはPythonで実装されており、トレーニング可能です。各サブモデルには、すべての入出力用文字列名が用意されており、それらを使用してモデルを自動的に接続します（下の画像を参照）。これにより、サブモデルを簡単に切り替えて、シミュレータをよりカスタマイズできます。

![py_model_interface](./image/python_model_interface.png "PyModelインタフェース")

## 仮定/既知の制限

<!-- 必須 -->

このパッケージを使用するには、`python3`と`pybind11`をインストールする必要があります。Pythonサブモデルの唯一の仮定は、それらのインタフェースです。


```python
class PythonSubmodelInterface:

    def forward(self, action, state):  # Required
        """
        Calculate forward pass through the model and returns next_state.
        """
        return list()

    def get_state_names(self):  # Required
        """
        Return list of string names of the model states (outputs).
        """
        return list()

    def get_action_names(self):  # Required
        """
        Return list of string names of the model actions (inputs).
        """
        return list()

    def reset(self):  # Required
        """
        Reset model. This function is called after load_params().
        """
        pass

    def load_params(self, path):  # Required
        """
        Load parameters of the model.
        Inputs:
            - path: Path to a parameter file to load by the model.
        """
        pass

    def dtSet(self, dt):  # Required
        """
        Set dt of the model.
        Inputs:
            - dt: time step
        """
        pass
```

## API

<!-- Required -->
<!-- Things to consider:
    - パッケージ / API の使用方法 -->

車両モデルを正常に作成するには、`InterconnectedModel` クラスを適切に設定する必要があります。

### `InterconnectedModel` クラス

#### `コンストラクタ`

コンストラクタは引数を受け取りません。

#### `void addSubmodel(std::tuple<std::string, std::string, std::string> model_descriptor)`

モデルに新しいサブモデルを追加します。

入力:

- `model_descriptor`: 使用するモデルを示します。モデル記述子は 3 つの文字列を含みます。
  - 最初の文字列は、モデルが実装されている Python モジュールのパスです。
  - 2 番目の文字列は、モデルのパラメータが格納されているファイルのパスです。
  - 3 番目の文字列は、モデルを実装するクラスの名前です。

出力:

- なし

#### `void generateConnections(std::vector<char *> in_names, std::vector<char*> out_names)`

サブモデルとモデルの入力/出力の接続を生成します。

入力:

- `in_names`: モデルのすべての入力の文字列名（順序あり）。
- `out_names`: モデルのすべての出力の文字列名（順序あり）。

出力:

- なし

#### `void initState(std::vector<double> new_state)`

モデルの初期状態を設定します。

入力:

- `new_state`: モデルの新しい状態。

出力:

- なし

#### `std::vector<double> updatePyModel(std::vector<double> psim_input)`

すべてのサブモデルの次状態を計算して、モデルの次状態を計算します。

- psim_input: モデルへの入力。

出力:

- next_state: モデルの次の状態。

#### `dtSet(double dt)`

モデルの時間ステップを設定します。

入力:

- dt: 時間ステップ

出力:

- なし

### 例

最初にモデルを設定する必要があります。


```C++
InterconnectedModel vehicle;

// Example of model descriptors
std::tuple<char*, char*, char*> model_descriptor_1 = {
    (char*)"path_to_python_module_with_model_class_1",
    (char*)nullptr,  // If no param file is needed you can pass 'nullptr'
    (char*)"ModelClass1"
    };

std::tuple<char*, char*, char*> model_descriptor_2 =   {
    (char*)"path_to_python_module_with_model_class_2",
    (char*)"/path_to/param_file",
    (char*)"ModelClass2"  // Name of the python class. Needs to use the interface from 'Assumptions'
    };

// Create sub-models based on descriptors
vehicle.addSubmodel(model_descriptor_1);
vehicle.addSubmodel(model_descriptor_2);

// Define STATE and INPUT names of the system
std::vector<char*> state_names = {(char*)"STATE_NAME_1", (char*)"STATE_NAME_2"};
std::vector<char*> input_names = {(char*)"INPUT_NAME_1", (char*)"INPUT_NAME_2"};

// Automatically connect sub-systems with model input
vehicle.generateConnections(input_names, state_names);

// Set the time step of the model
vehicle.dtSet(dt);
```

モデルが正しく設定された後、以下のように使用できます。


```C++
// Example of an model input
std::vector<double> vehicle_input = {0.0, 1.0}; // INPUT_NAME_1, INPUT_NAME_2

// Example of an model state
std::vector<double> current_state = {0.2, 0.5}; // STATE_NAME_1, STATE_NAME_2

// Set model state
vehicle.initState(current_state);

// Calculate the next state of the model
std::vector<double> next_state = vehicle.updatePyModel(vehicle_input);
```

## 参考文献 / 外部リンク

<!-- オプション -->

## 関連する問題

<!-- 必須 -->

