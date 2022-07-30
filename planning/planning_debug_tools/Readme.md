# Planning Debug Tools

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

PlotHeightOverArclength('z_behavior_path', behavior_path, tracker_time)
PlotHeightOverArclength('z_behavior_velocity', behavior_velocity, tracker_time)
PlotHeightOverArclength('z_motion_avoid', motion_avoid, tracker_time)
PlotHeightOverArclength('z_motion_smoother_latacc', motion_smoother_latacc, tracker_time)
PlotHeightOverArclength('z_motion_smoother', motion_smoother, tracker_time)

PlotCurrentVelocity('localization_kinematic_state', '/localization/kinematic_state', tracker_time)
```

in Function Library

```lua
function PlotCurvatureOverArclength(name, path, timestamp)
  new_series = ScatterXY.new(name)
  index = 0
  while(true) do
    series_k = TimeseriesView.find( string.format( "%s/curvature.%d", path, index) )
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
end

function PlotVelocityOverArclength(name, path, timestamp)
  new_series = ScatterXY.new(name)
  index = 0
  while(true) do
    series_v = TimeseriesView.find( string.format( "%s/velocity.%d", path, index) )
    series_s = TimeseriesView.find( string.format( "%s/arclength.%d", path, index) )
    series_size = TimeseriesView.find( string.format( "%s/size", path) )

    if series_v == nil or series_s == nil then break end

    v = series_v:atTime(timestamp)
    s = series_s:atTime(timestamp)
    size = series_size:atTime(timestamp)

    if index >= size then break end

    new_series:push_back(s,v)
    index = index+1
  end
end

function PlotHeightOverArclength(name, path, timestamp)
  new_series = ScatterXY.new(name)
  index = 0
  while(true) do
    series_z = TimeseriesView.find( string.format( "%s/height.%d", path, index) )
    series_s = TimeseriesView.find( string.format( "%s/arclength.%d", path, index) )
    series_size = TimeseriesView.find( string.format( "%s/size", path) )

    if series_z == nil or series_s == nil then break end

    v = series_z:atTime(timestamp)
    s = series_s:atTime(timestamp)
    size = series_size:atTime(timestamp)

    if index >= size then break end

    new_series:push_back(s,z)
    index = index+1
  end
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

for more description tbd...
