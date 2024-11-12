## テンプレート

`autoware_behavior_velocity_speed_bump_module` に基づく動作速度モジュールのテンプレート。

# Autoware 動作速度モジュールテンプレート

## `Scene`

### `TemplateModule` クラス

`TemplateModule` クラスは Autoware 動作速度プランナー内のシーンモジュールを作成するための基盤として機能します。モジュールの動作に必要なコアメソッドと機能を定義します。プレースホルダーコードは、特定の動作速度モジュールに合わせた実際のインプリメンテーションに置き換える必要があります。

#### コンストラクタ

- `TemplateModule` のコンストラクタはモジュールを作成するための重要なパラメータを受け取ります: `const int64_t module_id`, `const rclcpp::Logger & logger`, and `const rclcpp::Clock::SharedPtr clock`。これらのパラメータは、新しいモジュールを登録するときに `TemplateModuleManager` によって供給されます。特定のモジュールインプリメンテーションで必要に応じて、他のパラメータをコンストラクタに追加できます。

#### `modifyPathVelocity` メソッド

- `TemplateModule` クラスで定義されたこのメソッドは、特定の条件に基づいて入力パスの速度を変更することが期待されています。提供されるコードでは、テンプレートモジュールが実行されるときに一度だけ情報メッセージをログに記録します。
- 速度変更の特定のロジックは、モジュールの要件に基づいてこのメソッドにインプリメントする必要があります。

#### `createDebugMarkerArray` メソッド

- `TemplateModule` クラスで定義されているこのメソッドは、デバッグマーカーのビジュアライゼーションを作成し、`visualization_msgs::msg::MarkerArray` として返します。提供されるコードでは、空の `MarkerArray` を返します。
- モジュールの機能に固有のデバッグマーカーを生成するロジックを実装する必要があります。

#### `createVirtualWalls` メソッド

- `createVirtualWalls` メソッドはシーンの仮想壁を作成し、`autoware::motion_utils::VirtualWalls` として返します。提供されるコードでは、空の `VirtualWalls` オブジェクトが返されます。
- モジュールの要件に基づいて仮想壁を作成するロジックを実装する必要があります。

## `Manager`

モジュールの管理は manager.hpp と manager.cpp で定義されます。管理は 2 つのクラスによって処理されます:

- `TemplateModuleManager` クラスは、動作速度テンプレートシーンの管理と起動 (behavior_velocity_template_module/src/scene.cpp/hpp で定義) のコアロジックを定義します。親クラス `SceneModuleManagerInterface` から重要なマネージャー属性を継承します。
- `TemplateModulePlugin` クラスは、`TemplateModuleManager` を動作速度プランナーのロジックに統合する方法を提供します。

### `TemplateModuleManager` クラス

#### コンストラクタ `TemplateModuleManager`

- これは `TemplateModuleManager` クラスのコンストラクタであり、パラメータとして `rclcpp::Node` 参照を受け取ります。
- メンバー変数 `dummy_parameter_` を 0.0 に初期化します。

#### `getModuleName()` メソッド

- このメソッドは `SceneModuleManagerInterface` クラスの仮想メソッドのオーバーライドです。
- モジュールの名前である定数文字列へのポインタを返します。この場合、モジュール名を「テンプレート」として返します。

#### `launchNewModules()` メソッド

- これは `tier4_planning_msgs::msg::PathWithLaneId` 型の引数を取るプライベートメソッドです。
- 与えられたパス情報 (PathWithLaneId) に基づいて新しいモジュールを起動する責任があります。このメソッドの実装には、`TemplateModule` クラスを使用して動作速度プランナーに固有のモジュールの初期化と設定が含まれます。
- 提供されるソースコードでは、`module_id` を 0 に初期化し、同じ ID を持つモジュールがすでに登録されているかどうかを確認します。登録されていない場合、`TemplateModule` をモジュール ID で新しく登録します。`TemplateModuleManager` によって管理される各モジュールは一意の ID を持つ必要があることに注意してください。テンプレートコードは単一のモジュールを登録するため、`module_id` は単純さのために 0 に設定されます。

#### `getModuleExpiredFunction()` メソッド

- これは `tier4_planning_msgs::msg::PathWithLaneId` 型の引数を取るプライベートメソッドです。

- このメソッドの実装は、モジュールの有効期限ステータスを確認するために使用できる関数を返すことが期待されています。

メソッド `launchNewModules()` と `getModuleExpiredFunction()` の具体的な機能は、ビヘイビア速度モジュールの詳細とその Autoware システム内での管理方法によって異なります。モジュールの要件に従ってこれらのメソッドを実装する必要があります。

### `TemplateModulePlugin` クラス

#### `TemplateModulePlugin` クラス

- このクラスは `PluginWrapper<TemplateModuleManager>` から継承します。これは本質的に、動的にロードおよび管理できるプラグイン内への `TemplateModuleManager` クラスをラップします。

## `Example Usage`

次の例では、パスの各点を取得して 2 倍にします。つまり、速度を複製します。すべてのビヘイビア速度モジュールが実行された後、Velocity Smoother がパス速度をさらに変更することに注意してください。


```cpp
bool TemplateModule::modifyPathVelocity(
  [[maybe_unused]] PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  for (auto & p : path->points) {
    p.point.longitudinal_velocity_mps *= 2.0;
  }

  return false;
}
```

