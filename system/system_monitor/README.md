# Autoware向けシステムモニタ

**Autowareのシステムモニタ機能のさらなる向上。**

## 概要

このパッケージは、システムのモニタリング用として次のノードを提供します。

- CPUモニタ
- HDDモニタ
- メモリモニタ
- ネットワークモニタ
- NTPモニタ
- プロセスモニタ
- GPUモニタ
- 電圧モニタ

### サポート対象アーキテクチャ

- x86_64
- arm64v8/aarch64

### 動作確認済みプラットフォーム

- PCシステムインテルCore i7
- NVIDIA Jetson AGX Xavier
- Raspberry Pi4モデルB

## 使用方法

他のパッケージと同様に、colcon buildとlaunchを使用します。


```sh
colcon build
source install/setup.bash
ros2 launch system_monitor system_monitor.launch.xml
```

プラットフォームによってCPUおよびGPU監視方法は異なります。<br>
CMakeは自動的にビルド環境に応じたビルド対象のソースを選択します。<br>
このパッケージをインテルプラットフォームでビルドする場合、インテルプラットフォームで動作するCPUモニターとGPUモニターがビルドされます。

## システムモニターによって公開されるROSトピック

各トピックは1分間隔で公開されます。

- [CPUモニター](docs/topics_cpu_monitor.md)
- [HDDモニター](docs/topics_hdd_monitor.md)
- [Memモニター](docs/topics_mem_monitor.md)
- [Netモニター](docs/topics_net_monitor.md)
- [NTPモニター](docs/topics_ntp_monitor.md)
- [Processモニター](docs/topics_process_monitor.md)
- [GPUモニター](docs/topics_gpu_monitor.md)
- [電圧モニター](docs/topics_voltage_monitor.md)

[使用状況] ✓：サポート、-：サポートなし

| ノード          | メッセージ                | Intel | arm64(tegra) | arm64(raspi) | メモ                                                                                                                                                                                                                                                                                     |
| --------------- | ---------------------------- | :---: | :----------: | :----------: | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CPUモニター     | CPU温度                   |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | CPU使用率                 |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | CPU負荷平均               |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | CPUサーマルスロットリング   |   ✓   |      -       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | CPU周波数                 |   ✓   |      ✓       |      ✓       | 周波数の通知のみで、通常エラーは生成されません。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| HDDモニター     | HDD温度                   |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDD通電時間               |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDD総書き込みデータ量       |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDD復旧エラー             |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDD使用率                 |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDDリードデータレート       |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDDライトデータレート      |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDDリードIOPS              |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDDライトIOPS             |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | HDD接続                  |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| メモリモニター  | メモリ使用率               |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| ネットワークモニタ | ネットワーク接続           |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | ネットワーク使用率         |   ✓   |      ✓       |      ✓       | 使用率の通知のみで、通常エラーは生成されません。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                 | ネットワークCRCエラー       |   ✓   |      ✓       |      ✓       | 周期内のCRCエラー数が閾値に達すると警告が発生します。発生するCRCエラー数は、`ip`コマンドで確認できる値と同じです。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|                 | IPパケット再構成失敗     |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| NTPモニター     | NTPオフセット              |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| プロセスモニター | タスクの概要               |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | 高負荷プロセス[0-9]        |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | 高メモリプロセス[0-9]       |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| GPUモニター     | GPU温度                   |   ✓   |      ✓       |      -       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | GPU使用率                 |   ✓   |      ✓       |      -       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | GPUメモリ使用率           |   ✓   |      -       |      -       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | GPUサーマルスロットリング   |   ✓   |      -       |      -       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                 | GPU周波数                 |   ✓   |      ✓       |      -       | インテルプラットフォームでは、現在のGPUクロックがGPUでサポートされているかどうかを監視します。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 電圧モニター     | CMOSバッテリーステータス  |   ✓   |      -       |      -       | RTCとBIOSのバッテリーの健全性                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

## ROSパラメータ

[ROSパラメータ](docs/ros_parameters.md)を参照してください。

## 注意

### <u>Intelプラットフォーム用CPUモニター</u>

サーマルスロットリングイベントはMSR(モデル固有レジスタ)の内容を読み取ることで監視でき、デフォルトではrootのみがMSRにアクセスできます。そのため、このパッケージでは、セキュリティリスクを可能な限り最小限に抑えるために、次のアプローチを提供しています。

- MSRにアクセスしてサーマルスロットリングステータスをCPUモニターにソケットプログラミングを使用して送信する「msr_reader」という小さなプログラムを提供する。
- rootではなく、特定のユーザーとして「msr_reader」を実行する。
- CPUモニターは、サーマルスロットリングステータスがソケット通信で送信されるため、非特権ユーザーとしてステータスを知ることができます。

### 開始前の手順

1. 「msr_reader」を実行するユーザーを作成します。


   ```sh
   sudo adduser <username>
   ```

2. 対象システムに `msr` カーネルモジュールをロードする。<br>
   `/dev/cpu/CPUNUM/msr` パスが出現する。


   ```sh
   sudo modprobe msr
   ```

3. アクセス制御リスト（ACL）を使用して、読み取り専用アクセス権でMSRにユーザーアクセスを許可します。


   ```sh
   sudo setfacl -m u:<username>:r /dev/cpu/*/msr
   ```

4. 'msr_reader'にcapabilityを割り当てます。msrカーネルモジュールにはrawioのcapabilityが必要です。


   ```sh
   sudo setcap cap_sys_rawio=ep install/system_monitor/lib/system_monitor/msr_reader
   ```

5. 作成したユーザとして「msr_reader」を実行し、汎用ユーザとしてsystem_monitorを実行します。


   ```sh
   su <username>
   install/system_monitor/lib/system_monitor/msr_reader
   ```

### 参照

[msr_reader](docs/msr_reader.md)

## <u>HDDモニター</u>

HDDの温度と寿命の監視には一般にS.M.A.R.T.情報が使用され、通常、diskデバイスノードへのアクセスはrootユーザーまたはdiskグループに制限されています。<br>
CPUモニターと同様に、このパッケージはセキュリティリスクをできるだけ最小限に抑えるアプローチを提供します。<br>

- 'hdd_reader'という小さなプログラムを提供し、S.M.A.R.T.情報にアクセスしてソケットプログラミングを使用してHDDモニターに一部のアイテムを送信します。
- 'hdd_reader'を特定のユーザーとして実行します。
- HDDモニターは、ソケット通信によって送信されるため、非特権ユーザーとして一部のS.M.A.R.T.情報を知ることができます。

### 開始前の手順

1. 'hdd_reader'を実行するユーザーを作成します。


   ```sh
   sudo adduser <username>
   ```

2. ディスクグループにユーザーを追加します。


   ```sh
   sudo usermod -a -G disk <username>
   ```

3. SCSIカーネルモジュールがATA PASS-THROUGH (12)コマンドを送信するには `rawio` 機能が必要で、NVMeカーネルモジュールは管理コマンドを送信するには `admin` 機能が必要なので、`hdd_reader`に機能を割り当てます。


   ```sh
   sudo setcap 'cap_sys_rawio=ep cap_sys_admin=ep' install/system_monitor/lib/system_monitor/hdd_reader
   ```

4. 作成したユーザーとして `hdd_reader` を実行し、一般的ユーザーとして `system_monitor` を実行します。


   ```sh
   su <username>
   install/system_monitor/lib/system_monitor/hdd_reader
   ```

### 関連情報

[hdd_reader](docs/hdd_reader.md)

## <u>インテルプラットフォームのGPUモニター</u>

現在、インテルプラットフォームのGPUモニターは、NVML APIから情報にアクセスできるNVIDIA GPUのみをサポートしています。

また、CUDAライブラリのインストールが必要です。
CUDA 10.0のインストール手順については、[NVIDIA CUDAインストールガイドfor Linux](https://docs.nvidia.com/cuda/archive/10.0/cuda-installation-guide-linux/index.html)を参照してください。

## <u>CMOSバッテリの電圧モニター</u>

一部のプラットフォームには、RTCとCMOS用の内蔵バッテリがあります。このノードは、cat /proc/driver/rtcの実行結果からバッテリの状態を判断します。
また、lm-sensorsがインストールされていれば、結果を利用することが可能です。
ただし、sensorsの戻り値はチップセットによって異なるため、対応する電圧を抽出するための文字列を設定する必要があります。
また、警告とエラーの電圧を設定する必要があります。
たとえば、電圧が2.9V未満になると警告し、2.7V未満になるとエラーを出す場合。
チップセットnct6106でのsensorsの実行結果は次のとおりで、「in7:」はCMOSバッテリの電圧です。


```txt
$ sensors
pch_cannonlake-virtual-0
Adapter: Virtual device
temp1:        +42.0°C

nct6106-isa-0a10
Adapter: ISA adapter
in0:           728.00 mV (min =  +0.00 V, max =  +1.74 V)
in1:             1.01 V  (min =  +0.00 V, max =  +2.04 V)
in2:             3.34 V  (min =  +0.00 V, max =  +4.08 V)
in3:             3.34 V  (min =  +0.00 V, max =  +4.08 V)
in4:             1.07 V  (min =  +0.00 V, max =  +2.04 V)
in5:             1.05 V  (min =  +0.00 V, max =  +2.04 V)
in6:             1.67 V  (min =  +0.00 V, max =  +2.04 V)
in7:             3.06 V  (min =  +0.00 V, max =  +4.08 V)
in8:             2.10 V  (min =  +0.00 V, max =  +4.08 V)
fan1:          2789 RPM  (min =    0 RPM)
fan2:             0 RPM  (min =    0 RPM)
```

次の通り、voltage_monitor.param.yaml の設定値です。


```yaml
/**:
  ros__parameters:
    cmos_battery_warn: 2.90
    cmos_battery_error: 2.70
    cmos_battery_label: "in7:"
```

以下の2.7Vと2.90Vの値は仮説です。マザーボードとチップセットによって値は異なる場合があります。ただし、リチウム電池の電圧が2.7Vを下回った場合は、交換することをお勧めします。
上記の例では、トピック/診断に出力されるメッセージは次のとおりです。
電圧<2.9Vの場合：


```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: Warning
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: Low Battery
```

電圧 < 2.7V の場合:


```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: Warning
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: Battery Died
```

そうでない場合は、


```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: OK
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: OK
```

CMOS バッテリー電圧が `voltage_error` または `voltage_warn` より低下した場合、警告となります。
バッテリーが切れた場合、電源がオフになると RTC の動作が停止します。ただし、車両が走行できるため、これはエラーではありません。エラーが発生すると車両は停止しますが、すぐに停止する必要はありません。
「Low Battery」または「Battery Died」の値で判断できます。

## UML 図

[クラス図](docs/class_diagrams.md) を参照してください。
[シーケンス図](docs/seq_diagrams.md) を参照してください。

