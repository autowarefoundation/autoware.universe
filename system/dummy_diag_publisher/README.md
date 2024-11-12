# dummy_diag_publisher

## 目的

このパッケージは、デバッグや開発用のダミー診断データをアウトプットします。

## 入出力

### 出力

| 名前           | タイプ                                     | 説明         |
| -------------- | ---------------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs::msgs::DiagnosticArray` | 診断出力 |

## パラメータ

### ノードパラメータ

パラメータ`DIAGNOSTIC_NAME`は、パラメータYAMLファイルに存在する名前でなければなりません。`status`パラメータがコマンドラインから指定された場合、`is_active`パラメータは自動的に`true`に設定されます。

| 名称                         | タイプ | 既定値 | 説明                                        | 再構成可能 |
| ----------------------------- | ------ | ---------- | -------------------------------------------- | ----------- |
| `update_rate`                | int    | `10`       | タイマーコールバックの周期 [Hz]             | false       |
| `DIAGNOSTIC_NAME.is_active` | bool   | `true`     | 強制的に更新するかどうか                      | true        |
| `DIAGNOSTIC_NAME.status`    | string | `"OK"`     | ダミー診断パブリッシャーによる診断ステータス | true        |

### dummy_diag_publisher の YAML フォーマット

値が `default` の場合、既定値が設定されます。

| キー                                     | タイプ   | デフォルト値 | 説明                                                               |
| --------------------------------------- | ------ | ----------- | ---------------------------------------------------------------------|
| `required_diags.DIAGNOSTIC_NAME.is_active` | bool   | `true`        | 強制アップデートするか否か                                           |
| `required_diags.DIAGNOSTIC_NAME.status`    | string | `"OK"`        | ダミー診断パブリッシャーによって設定された診断ステータス              |

## 想定事項/既知の制約事項

まだありません。

## 使用方法

### 起動


```sh
ros2 launch dummy_diag_publisher dummy_diag_publisher.launch.xml
```

### 再構成


```sh
ros2 param set /dummy_diag_publisher velodyne_connection.status "Warn"
ros2 param set /dummy_diag_publisher velodyne_connection.is_active true
```

