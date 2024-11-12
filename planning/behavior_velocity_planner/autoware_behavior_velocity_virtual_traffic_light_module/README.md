### バーチャル交通信号機

#### 役割

自動運転車は、以下のようなインフラと連携する必要があります。

- 倉庫のシャッター
- V2X対応信号機
- 交差点の通信機器
- フリート管理システム (FMS)

例を以下に示します。

1. V2X対応信号機による交通制御
   ![traffic_light](docs/V2X_support_traffic_light.png)

2. FMSによる複数の車両の交差点調整
   ![FMS](docs/intersection-coordination.png)

これらの機能は個別に実現することもできますが、ユースケースは次の3つの要素で一般化できます。

1. `start`: 車両が特定のゾーンに入った後、連携手順を開始します。
2. `stop`: インフラから受信したステータスに従って、定義された停止線で停止します。
3. `end`: 車両が終了ゾーンに到達した後、連携手順を完了します。これは、安定した通信範囲内で行う必要があります。

このモジュールは、インフラからステータスを送受信し、連携結果の速度を計画します。

### システム構成図


```plantuml
@startuml
!theme cerulean-outline

' Component
node "Autoware ECU" as autoware_ecu {
  component "Behavior Planner" as behavior_planner
  component "Autoware API" as autoware_api
  database "Vector Map" as vector_map
  note bottom of vector_map
    Communication metadata is stored.
  end note
}

package "Infrastructures" as infrastructures {
  node "FMS" as fms
  node "Automatic Shutter" as automatic_shutter
  node "Manual Shutter" as manual_shutter
  node "Remove Controllable Traffic Light" as remote_controllable_traffic_light
  node "Warning Light" as warning_light
}

' Relationship
'' Behavior Planner <-> Autoware API
behavior_planner -up-> autoware_api : infrastructure\n command
autoware_api -down-> behavior_planner : infrastructure\n state

'' Vector Map
vector_map -left-> behavior_planner : vector map

'' Autoware API <-> Infrastructure
autoware_api -up-> fms : <color:blue>lock\n <color:blue>request
fms -down-> autoware_api : <color:blue>right-of-way\n <color:blue>state

autoware_api -up-> automatic_shutter : <color:green>approach\n <color:green>notification
automatic_shutter -down-> autoware_api : <color:green>shutter\n <color:green>state

autoware_api -up-> manual_shutter : <color:blue>open/close\n <color:blue>command
manual_shutter -down-> autoware_api : <color:blue>shutter\n <color:blue>state

autoware_api -up-> remote_controllable_traffic_light : <color:green>light change\n <color:green>command
remote_controllable_traffic_light -down-> autoware_api : <color:green>light\n <color:green>state

autoware_api -up-> warning_light : <color:blue>activation\n <color:blue>command
warning_light -down-> autoware_api : <color:blue>warning light\n <color:blue>state

' Layout
'' Infrastructure
fms -[hidden]right-> automatic_shutter
automatic_shutter -[hidden]right-> manual_shutter
manual_shutter -[hidden]right-> remote_controllable_traffic_light
remote_controllable_traffic_light -[hidden]right-> warning_light

@enduml
```

プランナーと各インフラストラクチャは共通抽象メッセージを使用して相互に通信します。

- 各インフラストラクチャに対する特別な処理は拡張性がありません。インターフェイスは Autoware API として定義されています。
- 各インフラストラクチャの要件はわずかに異なりますが、柔軟に対応します。

FMS: 複数の車両が運用中で関連レーンの占有がある場合の交差点調整

- 自動シャッター: 接近時にシャッターを開き、離開時に閉じる
- 手動シャッター: ドライバーにシャッターを開閉してもらう。
- リモートコントロール信号: ドライバーに進行方向に合わせて信号状態を変更してもらう。
- 警告灯: 警告灯を作動させる。

インフラストラクチャごとに異なる通信方法をサポートする

- HTTP
- Bluetooth
- ZigBee

地理的な場所ごとに異なるメタ情報を保有する

- 関連するレーン ID
- ハードウェア ID
- 通信方式

FMS: フリートマネジメントシステム


```plantuml
@startuml
!theme cerulean-outline

' Component
node "Autoware ECU" as autoware_ecu {
component "Behavior Planner" as behavior_planner
component "Autoware API" as autoware_api
component "Web.Auto Agent" as web_auto_agent
note right of web_auto_agent : (fms_bridge)
database "Vector Map" as vector_map

package "Infrastructure Bridges" as infrastructure_bridges {
  component "Automatic Shutter Bridge" as automatic_shutter_bridge
  component "Manual Shutter Bridge" as manual_shutter_bridge
  component "Remove Controllable Traffic Light Bridge" as remote_controllable_traffic_light_bridge
  component "Warning Light Bridge" as warning_light_bridge
}
}

cloud "FMS" as fms {
  component "FMS Gateway" as fms_gateway

  component "Intersection Arbitrator" as intersection_arbitrator
  database "Intersection Lock Table" as intersection_lock_table

  component "Vector Map Builder" as vector_map_builder
  database "Vector Map Database" as vector_map_database
}

package "Infrastructures" as infrastructures {
  node "Automatic Shutter" as automatic_shutter
  node "Manual Shutter" as manual_shutter
  node "Remote Controllable Traffic Light" as remote_controllable_traffic_light
  node "Warning Light" as warning_light
}

' Relationship
'' Behavior Planner <-> Autoware API
behavior_planner -up-> autoware_api : infrastructure\n command
autoware_api -down-> behavior_planner : infrastructure state\n as virtual traffic light

'' Autoware API <-> Web.Auto
autoware_api -up-> web_auto_agent : infrastructure\n command
web_auto_agent -down-> autoware_api : infrastructure state\n as virtual traffic light

'' Autoware API <-> Infrastructure Bridge
autoware_api -right-> infrastructure_bridges : infrastructure\n command
infrastructure_bridges -left-> autoware_api : infrastructure state\n as virtual traffic light

'' Infrastructure Bridge <-> Infrastructure
automatic_shutter_bridge -right-> automatic_shutter : approach notification
automatic_shutter -left-> automatic_shutter_bridge : shutter state

manual_shutter_bridge -right-> manual_shutter : open/close command
manual_shutter -left-> manual_shutter_bridge : shutter state

remote_controllable_traffic_light_bridge -right-> remote_controllable_traffic_light : light change command
remote_controllable_traffic_light -left-> remote_controllable_traffic_light_bridge : light state

warning_light_bridge -right-> warning_light : activation command
warning_light -left-> warning_light_bridge : warning light state

'' Web.Auto
web_auto_agent -up-> fms_gateway : infrastructure\n command
fms_gateway -down-> web_auto_agent : infrastructure state\n as virtual traffic light

fms_gateway -up-> intersection_arbitrator : lock request
intersection_arbitrator -down-> fms_gateway : right-of-way state

intersection_arbitrator -up-> intersection_lock_table : lock request
intersection_lock_table -down-> intersection_arbitrator : lock result

vector_map_builder -down-> vector_map_database : create vector map
vector_map_database -left-> intersection_arbitrator : vector map

'' Vector Map
vector_map_database .down.> web_auto_agent : vector map
web_auto_agent -left-> vector_map : vector map
vector_map -down-> behavior_planner : vector map

' Layout
'' Infrastructure Bridge
automatic_shutter_bridge -[hidden]down-> manual_shutter_bridge
manual_shutter_bridge -[hidden]down-> remote_controllable_traffic_light_bridge
remote_controllable_traffic_light_bridge -[hidden]down-> warning_light_bridge

'' Infrastructure
automatic_shutter -[hidden]down-> manual_shutter
manual_shutter -[hidden]down-> remote_controllable_traffic_light
remote_controllable_traffic_light -[hidden]down-> warning_light

@enduml
```

#### モジュールパラメータ

## 自動運転ソフトウェア パラメータ

| パラメータ | 型 | 説明 |
|---|---|---|
| `max_delay_sec` | double | [秒] コマンドの最大許容遅延 |
| `near_line_distance` | double | [m] 停車線を停止するために停止線までの距離のしきい値 |
| `dead_line_margin` | double | [m] このモジュールが停止線を挿入し続けるしきい値 |
| `hold_stop_margin_distance` | double | [m] 再起動防止のパラメータ（以降のセクションを参照） |
| `check_timeout_after_stop_line` | bool | [-] リンクが切断されたときに停止するタイムアウトの確認 |

#### 再始動防止

車両の制御性能が低下するために、車両の動き始め時に停止するのに X メートル（例: 0.5 メートル）を要する場合、車両は停止点に近づこうとして動き始めるときは厳守すべき停止位置を超過します（例: 0.3 メートル距離）。

このモジュールには、これらの余分な再始動を防止するためのパラメータ `hold_stop_margin_distance` があります。車両がモジュールの停止位置（_front_to_stop_line < hold_stop_margin_distance）から `hold_stop_margin_distance` メーター以内に停止した場合、モジュールは車両がモジュールの停止位置で既に停止していると判断し、他の要素により車両が停止した場合にも、現在の位置で停止し続けることを計画します。

<figure markdown>
  ![例](docs/restart_prevention.svg){width=1000}
  <figcaption>パラメータ</figcaption>
</figure>

<figure markdown>
  ![例](docs/restart.svg){width=1000}
  <figcaption>hold_stop_margin_distanceの外側</figcaption>
</figure>

<figure markdown>
  ![例](docs/keep_stopping.svg){width=1000}
  <figcaption>hold_stop_margin_distanceの内側</figcaption>
</figure>

#### フローチャート


```plantuml
@startuml
!theme cerulean-outline
start

if (before start line?) then (yes)
  stop
else (no)
endif

if (after end line?) then (yes)
  stop
else (no)
endif

:send command to infrastructure;

if (no stop line?) then (yes)
  stop
else (no)
endif

:check infrastructure state;

if (timeout or not received?) then (yes)
  :stop at stop line;
  stop
else (no)
endif

if (no right of way?) then (yes)
  :stop at stop line;
else (no)
endif

if (finalization is requested?) then (yes)
  if (not finalized?) then (yes)
    :stop at end line;
  else (no)
  endif
else (no)
endif

stop
@enduml
```

#### マップフォーマット

- 急ブレーキを回避するため、仮想信号機の始点と停止線の距離は、始点通過時の速度が$v_0$で、Autowareで定義された最小加速度が$a_{\mathrm{min}}$と仮定して計算される$l_{\mathrm{min}}$よりも長くなければなりません。

$$
\begin{align}
l_{\mathrm{min}} = -\frac{v_0^2}{2 a_{\mathrm{min}}}
\end{align}
$$

#### 制限事項

- 未定

