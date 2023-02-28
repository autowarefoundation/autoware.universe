# Debug

## Debug visualization

The rviz config file for `obstacle_avoidance_planner` is [here](../rviz/obstacle_avoidance_planner.rviz).
All the following visualization markers are contained.

### Input
- **Path**
  - The path generated in the `behavior` planner.
  - The semitransparent and thick, green and red band, that is visualized by default.

![path](../media/path_visualization.png)

- **Path Footprint**
  - The path generated in the `behavior` planner is converted to footprints.

![path_footprint](../media/path_footprint_visualization.png)

- **Drivalbe Area**
  - The Drivable area generated in the `behavior` planner.
  - The skyblue left and right line strings, that is visualized by default.

![drivable_area](../media/drivable_area_visualization.png)

### Elastic Band (EB)
- **EB Fixed Trajectory**
  - The fixed trajectory points for elastic band.

![eb_fixed_traj](../media/eb_fixed_traj_visualization.png)

- **EB Trajectory**
  - The optimized trajectory points by elastic band.

![eb_traj](../media/eb_traj_visualization.png)

### Model Predictive Trajectory (MPT)
- **MPT Reference Trajectory**
  - The fixed trajectory points for model predictive trajectory.

![mpt_ref_traj](../media/mpt_ref_traj_visualization.png)

- **MPT Fixed Trajectory**
  - The fixed trajectory points for model predictive trajectory.

![mpt_fixed_traj](../media/mpt_fixed_traj_visualization.png)

- **MPT Trajectory**
  - The optimized trajectory points by model predictive trajectory.

![mpt_traj](../media/mpt_traj_visualization.png)

### Output
- **Trajectory**
  - The output trajectory.
  - The dark and thin, green and red band, that is visualized by default.

![traj](../media/traj_visualization.png)

- **Trajectory Footprint**
  - The output trajectory is converted to footprints.

![traj_footprint](../media/traj_footprint_visualization.png)

## Q&A for Debug
### When a part of the trajectory has high curvature

### When the trajectory's shape is zigzag


## Calculation cost

Obstacle avoidance planner consists of many functions such as clearance map generation, boundary search, reference path smoothing, trajectory optimization, ...
We can see the calculation time for each function as follows.

### Raw data

```sh
$ ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/calculation_time --field data

      getDrivableAreaInCV:= 0.21 [ms]
      getClearanceMap:= 1.327 [ms]
      drawObstaclesOnImage:= 0.436 [ms]
      getAreaWithObjects:= 0.029 [ms]
      getClearanceMap:= 2.186 [ms]
    getMaps:= 4.213 [ms]
          calculateTrajectory:= 2.417 [ms]
        getOptimizedTrajectory:= 5.203 [ms]
      getEBTrajectory:= 5.231 [ms]
          calcBounds:= 0.821 [ms]
          calcVehicleBounds:= 0.27 [ms]
        getReferencePoints:= 2.866 [ms]
        generateMPTMatrix:= 0.195 [ms]
        generateValueMatrix:= 0.019 [ms]
          getObjectiveMatrix:= 0.559 [ms]
          getConstraintMatrix:= 1.776 [ms]
          initOsqp:= 9.074 [ms]
          solveOsqp:= 3.809 [ms]
        executeOptimization:= 15.46 [ms]
        getMPTPoints:= 0.049 [ms]
      getModelPredictiveTrajectory:= 18.928 [ms]
    optimizeTrajectory:= 24.234 [ms]
    insertZeroVelocityOutsideDrivableArea:= 0.023 [ms]
      getDebugVisualizationMarker:= 0.446 [ms]
      publishDebugVisualizationMarker:= 2.146 [ms]
      getDebugVisualizationWallMarker:= 0.001 [ms]
      publishDebugVisualizationWallMarker:= 0.013 [ms]
    publishDebugDataInOptimization:= 2.696 [ms]
    getExtendedTrajectory:= 0.016 [ms]
    generateFineTrajectoryPoints:= 0.118 [ms]
    alignVelocity:= 1.227 [ms]
  generatePostProcessedTrajectory:= 1.375 [ms]
    makePrevTrajectories:= 1.411 [ms]
  generateOptimizedTrajectory:= 33.284 [ms]
    getExtendedTrajectory:= 0.018 [ms]
    generateFineTrajectoryPoints:= 0.084 [ms]
    alignVelocity:= 1.109 [ms]
  generatePostProcessedTrajectory:= 1.217 [ms]
    getDebugCostMap * 3:= 0 [ms]
  publishDebugDataInMain:= 0.023 [ms]
pathCallback:= 34.614 [ms]
```

### Plot

With a script, we can plot some of above time as follows.

```sh
python3 scripts/calclation_time_analyzer.py -f "solveOsqp,initOsqp,pathCallback"
```

![calculation_cost_plot](../media/calculation_cost_plot.svg)
