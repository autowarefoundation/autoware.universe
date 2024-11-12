# bluetooth_monitor

## 説明

このノードは、L2pingを使用したワイヤレスデバイスへのBluetooth接続を監視します。<br>
L2pingはBluetooth L2CAPレイヤーでPINGエコーコマンドを生成し、ワイヤレスデバイスからのエコーレスポンスを受信して確認できます。

## ブロックダイアグラム

L2pingはデフォルトではrootユーザーのみ許可されているため、このパッケージは次のアプローチを提供して、可能な限りセキュリティリスクを最小限に抑えます。

- L2pingを実行し、ソケットプログラミングを使用してワイヤレスデバイス情報を`bluetooth_monitor`に提供する`l2ping_service`という名前の小さなプログラムを提供します。
- `bluetooth_monitor`は、ソケット通信によってこれらの情報が送信されるため、非特権ユーザーとしてワイヤレスデバイス情報とL2pingステータスを知ることができます。

![block_diagram](docs/block_diagram.drawio.svg)

## 出力

### <u>bluetooth_monitor: bluetooth_connection</u>

<b>[概要]</b>

| レベル | メッセージ |
| ----- | -------- |
| OK    | OK |
| WARN  | RTT警告 |
| ERROR | ロスト |
|       | 関数エラー |

**[値]**

| キー                          | 値 (例)                                                               |
| ---------------------------- | ------------------------------------------------------------------------- |
| デバイス [0-9]: ステータス | OK / RTT 警告 / 検証エラー / ロスト / Ping 拒否 / 機能エラー |
| デバイス [0-9]: 名称         | ワイヤレスコントローラー                                                   |
| デバイス [0-9]: 製造元       | MediaTek, Inc.                                                          |
| デバイス [0-9]: アドレス      | AA:BB:CC:DD:EE:FF                                                       |
| デバイス [0-9]: RTT          | 0.00ms                                                                  |

- 「bluetooth_monitor」が「機能エラー」をレポートすると、次のキーが追加されます。<br>
  例) 「connect」システムコールが失敗する。

| キー（例）          | 値（例）             |
| -------------------- | ---------------------- |
| デバイス [0-9]: 接続 | そのようなファイルまたはディレクトリはありません |

## パラメータ

{{ json_to_markdown("system/bluetooth_monitor/schema/bluetooth_monitor.schema.json") }}

- `rtt_warn`

  - **0.00(ゼロ)**: RTTのチェックを無効化
  - **それ以外**: 指定した秒数でRTTをチェック

- `addresses`
  - **\***: すべての接続デバイス
  - **AA:BB:CC:DD:EE:FF**: Bluetoothアドレスを設定することで監視するデバイスを指定できます

## 開始前の手順

- ルートユーザーとして`l2ping_service`を実行する場合は、以下の手順をスキップできます。

1. L2pingは`cap_net_raw+eip`機能を必要とするため、`cap_net_raw+eip`機能を`l2ping_service`に割り当てます。


   ```sh
   sudo setcap 'cap_net_raw+eip' ./build/bluetooth_monitor/l2ping_service
   ```

2. `l2ping_service` と `bluetooth_monitor` を実行します。


   ```sh
   ./build/bluetooth_monitor/l2ping_service
   ros2 launch bluetooth_monitor bluetooth_monitor.launch.xml
   ```

## 周知の制限と問題

なし。

