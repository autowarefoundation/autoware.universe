# Planning Debug Tools

![how this works](https://user-images.githubusercontent.com/21360593/179361367-a9fa136c-cd65-4f3c-ad7c-f542346a8d37.mp4)

add below to reactive script and save it!

![image](./image/lua.png)

in Global code

```lua
behavior_path = '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id/debug_info'
behavior_velocity = '/planning/scenario_planning/lane_driving/behavior_planning/path/debug_info'
motion_avoid = '/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory/debug_info'
motion_smoother_latacc = '/planning/scenario_planning/motion_velocity_smoother/debug/trajectory_lateral_acc_filtered/debug_info'
motion_smoother = '/planning/scenario_planning/trajectory/debug_info'
```

in function(tracker_time)

```lua
PlotCurvatureOverArclength('k_behavior_path', behavior_path, tracker_time)
PlotCurvatureOverArclength('k_behavior_velocity', behavior_velocity, tracker_time)
PlotCurvatureOverArclength('k_motion_avoid', motion_avoid, tracker_time)
PlotCurvatureOverArclength('k_motion_smoother', motion_smoother, tracker_time)

PlotVelocityOverArclength('v_behavior_path', behavior_path, tracker_time)
PlotVelocityOverArclength('v_behavior_velocity', behavior_velocity, tracker_time)
PlotVelocityOverArclength('v_motion_avoid', motion_avoid, tracker_time)
PlotVelocityOverArclength('v_motion_smoother_latacc', motion_smoother_latacc, tracker_time)
PlotVelocityOverArclength('v_motion_smoother', motion_smoother, tracker_time)

PlotYawOverArclength('yaw_behavior_path', behavior_path, tracker_time)
PlotYawOverArclength('yaw_behavior_velocity', behavior_velocity, tracker_time)
PlotYawOverArclength('yaw_motion_avoid', motion_avoid, tracker_time)
PlotYawOverArclength('yaw_motion_smoother_latacc', motion_smoother_latacc, tracker_time)
PlotYawOverArclength('yaw_motion_smoother', motion_smoother, tracker_time)

PlotCurrentVelocity('localization_kinematic_state', '/localization/kinematic_state', tracker_time)
```

in Function Library
![image](./image/script.png)

```lua

function PlotValue(name, path, timestamp, value)
  new_series = ScatterXY.new(name)
  index = 0
  while(true) do
    series_k = TimeseriesView.find( string.format( "%s/"..value..".%d", path, index) )
    series_s = TimeseriesView.find( string.format( "%s/arclength.%d", path, index) )
    series_size = TimeseriesView.find( string.format( "%s/size", path) )

    if series_k == nil or series_s == nil then break end

    k = series_k:atTime(timestamp)
    s = series_s:atTime(timestamp)
    size = series_size:atTime(timestamp)

    if index >= size then break end

    new_series:push_back(s,k)
    index = index+1
  end

function PlotCurvatureOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"curvature")
end

function PlotVelocityOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"velocity")
end

function PlotYawOverArclength(name, path, timestamp)
  PlotValue(name, path, timestamp,"yaw")
end

function PlotCurrentVelocity(name, kinematics_name, timestamp)
  new_series = ScatterXY.new(name)
  series_v = TimeseriesView.find( string.format( "%s/twist/twist/linear/x", kinematics_name))
  if series_v == nil then
    print("error")
    return
  end
  v = series_v:atTime(timestamp)
  new_series:push_back(0.0, v)
end
```

## environment setting

```.sh
ros-galactic-plotjuggler-dbgsym/focal 3.5.1-1focal.20220730.024308 amd64
ros-galactic-plotjuggler-msgs-dbgsym/focal 0.2.3-1focal.20220729.134451 amd64
ros-galactic-plotjuggler-msgs/focal 0.2.3-1focal.20220729.134451 amd64
ros-galactic-plotjuggler-ros-dbgsym/focal 1.7.3-1focal.20220730.075513 amd64
ros-galactic-plotjuggler-ros/focal,now 1.7.3-1focal.20220730.075513 amd64
ros-galactic-plotjuggler/focal 3.5.1-1focal.20220730.024308 amd64
ros-galactic-rqt-plot/focal 1.1.1-1focal.20220730.023507 amd64
```

for more description tbd...
