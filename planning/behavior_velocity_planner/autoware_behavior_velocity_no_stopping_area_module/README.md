### 停车禁止区域

#### 角色

该模块计划避免在“停车禁止区域”停车。

![brief](./docs/no-stopping-area.svg)

- 直通过去的情况
  - 如果本车已通过通过判断点，则本车无法以最大加加减减速停车，因此该模块也不会插入停车速度。在这种情况下就需要 Override 或外部操作。
- 停车情况
  - 如果“停车禁止区域”周围有处在停滞状态的车辆或停车速度，则车辆会在“停车禁止区域”内停车，因此该模块会在“停车禁止区域”前设定停车速度。
- 行驶情况
  - 其他情况

### 限制

该模块允许开发人员使用特定规则设计“停车禁止区域”模块中的车辆速度。一旦本车通过过境点后，本车就不会插入停车速度，也不会改变 GO 的决策。此外，该模块仅考虑动态物体，以避免不必要地停车。

#### ModelParameter

| パラメーター                     | タイプ   | 説明                                                        |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| `state_clear_time`           | 数値   | [s] 停止状態を解除する時間                                |
| `stuck_vehicle_vel_thr`      | 数値   | [m/s] この速度以下の車両は停止車両とみなす。                |
| `stop_margin`                | 数値   | [m] 停止禁止区域での停止線へのマージン                     |
| `dead_line_margin`           | 数値   | [m] 自車がこの位置を通過すると進路を許可                    |
| `stop_line_margin`           | 数値   | [m] 停止禁止区域での自動生成停止線へのマージン              |
| `detection_area_length`      | 数値   | [m] 検索ポリゴンの長                                      |
| `stuck_vehicle_front_margin` | 数値   | [m] 障害物停止最大距離                                     |

#### フローチャート


```plantuml
@startuml
title modifyPathVelocity
start

if (ego path has "no stopping area" ?) then (yes)
else (no)
  stop
endif

partition pass_through_condition {
if (ego vehicle is not after dead line?) then (yes)
else (no)
  stop
endif
if (ego vehicle is stoppable before stop line consider jerk limit?) then (yes)
else (no)
  stop
endif
}
note right
  - ego vehicle is already over dead line(1.0[m] forward stop line) Do Not Stop.
  - "pass through or not" considering jerk limit is judged only once to avoid chattering.
end note

:generate ego "stuck_vehicle_detect_area" polygon;
note right
"stuck_vehicle_detect_area" polygon includes space of
 vehicle_length + obstacle_stop_max_distance
 after "no stopping area"
end note

:generate ego "stop_line_detect_area" polygon;
note right
"stop_line_detect_area" polygon includes space of
 vehicle_length + margin
 after "no stopping area"
end note

:set current judgement as GO;
if (Is stuck vehicle inside "stuck_vehicle_detect_area" polygon?) then (yes)
note right
only consider stuck vehicle following condition.
- below velocity 3.0 [m/s]
- semantic type of car bus truck or motorbike
only consider stop line as following condition.
- low velocity that is in path with lane id is considered.
end note
if (Is stop line inside "stop_line_detect_area" polygon?) then (yes)
  :set current judgement as STOP;
endif
endif

partition set_state_with_margin_time {

if (current judgement is same as previous state) then (yes)
  :reset timer;
else if (state is GO->STOP) then (yes)
  :set state as STOP;
  :reset timer;
else if (state is STOP -> GO) then (yes)
  if (start time is not set) then (yes)
    :set start time;
  else(no)
   :calculate duration;
   if(duration is more than margin time)then (yes)
    :set state GO;
    :reset timer;
  else(no)
   endif
  endif
else(no)
endif

}

note right
  - it takes 2 seconds to change state from STOP -> GO
  - it takes 0 seconds to change state from GO -> STOP
  - reset timer if no state change
end note

if (state is STOP) then (yes)
  :set stop velocity;
  :set stop reason and factor;
  else(no)
endif
stop


@enduml
```

