# Autoware Trajectory Modifier

## Unified Node and Pipeline

The `autoware_trajectory_modifier` package runs safety modifiers and trajectory optimizers in one ordered plugin pipeline. It takes candidate trajectories as input, applies modifier plugins first and optimizer plugins second by default, and publishes both the processed candidates and the first candidate as the selected trajectory.

### Features

- **Plugin-based architecture** - Modular optimization pipeline where each step is a separate plugin
- **Multiple smoothing methods**:
  - Elastic Band (EB) smoother for path optimization
  - Akima spline interpolation for smooth path interpolation
  - QP-based smoother with quadratic programming for path smoothing with jerk constraints
- **Velocity optimization** - Jerk-filtered velocity smoothing from `autoware_velocity_smoother`
- **Trajectory validation** - Removes invalid points and fixes trajectory orientation
- **Backward trajectory extension** - Extends trajectory using past ego states
- **Dynamic parameter reconfiguration** - Runtime parameter updates supported

## Architecture

The package uses one `TrajectoryModifier` ROS 2 component. Plugins are dynamically loaded at startup, inherit from `TrajectoryModifierPluginBase`, and execute in the exact order configured by `plugin_names`.

External packages can export modifier plugins through `pluginlib`. The modifier data snapshot includes candidate identity, vehicle state, perception data, route data, and both converted and raw lanelet maps. A plugin validates only the optional inputs that it uses.

### Plugin Loading and Execution

Plugins are loaded based on the `plugin_names` parameter, which defines both which plugins to load and their execution order:

```yaml
plugin_names:
  - "autoware::trajectory_modifier::plugin::StopPointFixer"
  - "autoware::trajectory_modifier::plugin::ObstacleStop"
  - "autoware::trajectory_modifier::plugin::VelocityModifier"
  - "autoware::trajectory_modifier::plugin::TrajectoryPointFixer"
  - "autoware::trajectory_modifier::plugin::TrajectoryQPSmoother"
  - "autoware::trajectory_modifier::plugin::TrajectoryEBSmootherOptimizer"
  - "autoware::trajectory_modifier::plugin::TrajectorySplineSmoother"
  - "autoware::trajectory_modifier::plugin::TrajectoryVelocityOptimizer"
  - "autoware::trajectory_modifier::plugin::TrajectoryExtender"
  - "autoware::trajectory_modifier::plugin::TrajectoryPointFixer"
```

### Available Plugins

1. **TrajectoryPointFixer** - Removes invalid/repeated points and fixes trajectory direction
2. **TrajectoryQPSmoother** - QP-based path smoothing with jerk constraints
3. **TrajectoryEBSmootherOptimizer** - Elastic Band path smoothing
4. **TrajectorySplineSmoother** - Akima spline interpolation
5. **TrajectoryMPTOptimizer** - Model predictive trajectory optimization with adaptive corridor bounds. Uses bicycle kinematics model for trajectory refinement. Disabled by default (experimental). See [docs/mpt_optimizer.md](docs/mpt_optimizer.md) for details.
6. **TrajectoryVelocityOptimizer** - Velocity profile optimization with lateral acceleration limits
7. **TrajectoryExtender** - Extends trajectory backward using past ego states
8. **TrajectoryKinematicFeasibilityEnforcer** - Enforces Ackermann steering and yaw rate constraints

Each plugin can be enabled/disabled at runtime via activation flags (e.g., `use_qp_smoother`) and manages its own configuration independently.

### ⚠️ Important: Plugin Ordering Constraints

**The order of plugin execution is critical and must be carefully maintained:**

- **QP Smoother must run before EB/Akima smoothers**: The QP solver relies on constant time intervals (Δt) between trajectory points (default: 0.1s). Both Elastic Band and Akima spline smoothers resample trajectories without preserving the time domain structure, which breaks the QP solver's assumptions. Therefore, when using multiple smoothers together, the QP smoother must execute first.

- **Trajectory Extender positioning**: The trajectory extender has known discontinuity issues when placed early in the pipeline. It negatively affects the QP solver results and introduces artifacts. For this reason, it has been moved to near the end of the pipeline and is **disabled by default** (`use_trajectory_extender: false`). Fixing the extender's discontinuity issues is future work.

- **Temporal MPT vs spatial MPT**: `TrajectoryTemporalMPTOptimizer` is independent of `TrajectoryMPTOptimizer` (different solver, no corridor bounds). It can replace the entire post-`TrajectoryPointFixer` plugin chain (kinematic enforcer, QP/EB/spline smoothers, velocity optimizer, spatial MPT) with a single MPC step; see [docs/temporal_mpt_optimizer.md](docs/temporal_mpt_optimizer.md). It overwrites at most the first 81 points (8.0 s at 0.1 s spacing).

### Design Specification

For a detailed description of the optimizer's core assumptions, the constant-dt contract,
how velocity and acceleration are derived from positions, plugin pipeline specification, and
known limitations, see [docs/trajectory_optimizer_specification.md](docs/trajectory_optimizer_specification.md).

### QP Smoother

The QP smoother uses quadratic programming (OSQP solver) to optimize trajectory paths with advanced features:

- **Objective**: Minimizes path curvature while maintaining fidelity to the original trajectory
- **Decision variables**: Path positions (x, y) for each trajectory point
- **Constraints**: Fixed initial position (optionally fixed last position)
- **Velocity-based fidelity**: Automatically reduces fidelity weight at low speeds for aggressive smoothing of noise
- **Post-processing**: Recalculates velocities, accelerations, and orientations from smoothed positions

**For detailed documentation**, see [docs/qp_smoother.md](docs/qp_smoother.md) which covers:

- Mathematical formulation
- Velocity-based fidelity weighting (sigmoid function)
- Parameter tuning guidelines
- Usage examples
- Performance characteristics

### Dependencies

- `autoware_motion_utils` - Trajectory manipulation utilities
- `autoware_osqp_interface` - QP solver interface for QP smoother
- `autoware_path_smoother` - Elastic Band smoother
- `autoware_velocity_smoother` - Velocity smoothing algorithms
- `autoware_utils` - Common utilities (geometry, ROS helpers)
- `autoware_vehicle_info_utils` - Vehicle information

### Parameters

{{ json_to_markdown("planning/autoware_trajectory_modifier/schema/trajectory_modifier.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

#### Parameter Types

1. **Plugin Loading** (`plugin_names`) - Array of plugin class names determining load order and execution sequence
2. **Activation Flags** - Boolean flags for runtime enable/disable (e.g., `use_qp_smoother`, `use_akima_spline_interpolation`)
3. **Plugin-Specific Parameters** - Namespaced parameters for each plugin (e.g., `trajectory_qp_smoother.weight_smoothness`)

#### Configuring Plugin Order

To change plugin execution order, modify the `plugin_names` array in `config/trajectory_modifier.param.yaml`:

```yaml
# Example: Run spline smoother before velocity optimizer
plugin_names:
  - "autoware::trajectory_modifier::plugin::TrajectoryPointFixer"
  - "autoware::trajectory_modifier::plugin::TrajectorySplineSmoother"
  - "autoware::trajectory_modifier::plugin::TrajectoryVelocityOptimizer"
  - "autoware::trajectory_modifier::plugin::TrajectoryPointFixer"
```

##### CRITICAL: QP Smoother Ordering Constraint

The `TrajectoryQPSmoother` plugin **MUST run before** any plugins that resample or modify trajectory structure:

- `TrajectorySplineSmoother` (Akima spline - resamples trajectory)
- `TrajectoryEBSmootherOptimizer` (Elastic Band - resamples trajectory)
- `TrajectoryVelocityOptimizer` (velocity smoothing with resampling)
- `TrajectoryExtender` (adds/modifies points at trajectory start)

The QP solver requires constant time intervals (Δt = 0.1s) between points. These plugins modify the time domain structure or add points, breaking the QP solver assumptions. If you need QP smoothing, it must appear first in the pipeline after `TrajectoryPointFixer`.

Note: Plugin order changes require node restart. Runtime enable/disable is controlled by activation flags.

## Modifier Plugins

Modifier plugins run in the same node and ordered pipeline as optimizer plugins. They post-process candidate trajectories to improve safety before the optimizer stage smooths and validates the result.

### Features

- Plugin-based architecture for extensible trajectory modifications
- Stop point fixing to prevent trajectory issues near stationary conditions
- Obstacle detection and stopping to prevent collision
- Configurable parameters to adjust modification behavior

### Architecture

Modifier and optimizer algorithms use the same plugin interface. Each plugin inherits from `TrajectoryModifierPluginBase`.

#### Plugin Interface

All modifier plugins must inherit from `TrajectoryModifierPluginBase` and implement:

- `process()` - Process trajectory points and return `ProcessingResult::Modified` or `ProcessingResult::Unchanged`
- `on_initialize()` - Initialize plugin members and parameters
- `update_params()` - Handle parameter updates

`is_trajectory_modification_required()` may be retained as a plugin-local helper, but is not part of the common interface.

#### Current Plugins

##### ExternalVelocityLimit

The `autoware::trajectory_modifier::plugin::ExternalVelocityLimit` plugin applies the latest
`autoware_internal_planning_msgs::msg::VelocityLimit` received on
`~/input/external_velocity_limit_mps` to every point of each trajectory. When `use_constraints` is
true, the absolute values of `constraints.min_acceleration` and `constraints.min_jerk` replace
`velocity_limits.max_deceleration` and `velocity_limits.max_jerk`; a zero constraint falls back to
the parameter. The velocity profile is generated as described in
[Velocity limit profile](#velocity-limit-profile).

| Parameter                          | Default | Description                                                          |
| ---------------------------------- | ------- | -------------------------------------------------------------------- |
| `use_external_velocity_limit`      | `true`  | Enable the plugin when included in `plugin_names`.                   |
| `velocity_limits.max_acceleration` | `1.0`   | Maximum acceleration of the generated profile [m/s²].                |
| `velocity_limits.max_deceleration` | `1.0`   | Maximum deceleration when the message provides no constraint [m/s²]. |
| `velocity_limits.max_jerk`         | `3.0`   | Maximum jerk when the message provides no constraint [m/s³].         |

##### MapVelocityLimits

The `autoware::trajectory_modifier::plugin::MapVelocityLimits` plugin limits trajectories with the
speed limits of the route lanelets at each point (lanelet `speed_limit` attributes are in km/h).
The velocity profile is generated as described in
[Velocity limit profile](#velocity-limit-profile).

| Parameter                                                          | Default | Description                                                |
| ------------------------------------------------------------------ | ------- | ---------------------------------------------------------- |
| `use_map_velocity_limits`                                          | `true`  | Enable the plugin when included in `plugin_names`.         |
| `velocity_limits.max_acceleration`                                 | `1.0`   | Maximum acceleration of the generated profile [m/s²].      |
| `velocity_limits.max_deceleration`                                 | `1.0`   | Maximum deceleration of the generated profile [m/s²].      |
| `velocity_limits.max_jerk`                                         | `3.0`   | Maximum jerk of the generated profile [m/s³].              |
| `map_velocity_limits.limit_velocity_from_map_debug_lanelet_ids`    | `[]`    | Lanelet IDs whose map limits are overridden for debugging. |
| `map_velocity_limits.limit_velocity_from_map_debug_max_velocities` | `[]`    | Corresponding override velocities in m/s.                  |

##### Velocity limit profile

Both velocity limit plugins leave a trajectory unchanged unless one of its points is faster than the
limit at its position. Otherwise, the velocity profile is regenerated by a forward simulation that
starts from the current ego velocity and acceleration and never violates the configured
acceleration, deceleration and jerk limits:

- At every arc length, the profile stays at or below the input velocity and the limit. A limit that
  cannot be reached in time without violating the deceleration or jerk limit is exceeded
  temporarily instead.
- Braking for a lower limit (or a lower input velocity) ahead starts near the latest point from
  which it is reached where it begins: when 95% of the deceleration limit is needed, which leaves a
  reserve to correct deviations. The braking then uses the deceleration actually required. Where
  the limit changes between two input points, the change is located by bisection along the
  segment.
- The profile does not accelerate above the lowest limit ahead in the trajectory.
- A velocity drop at the last input point is ignored, since planners commonly end their horizon
  with a zero velocity that is not a stop.
- A measured acceleration outside the acceleration and deceleration limits is clamped into them.

The point count and every `time_from_start` are kept. The first pose is kept and the other poses are
re-sampled along the input polyline so that the positions match the new velocities.

###### Fixed profile

With `velocity_limits.fixed_profile.enable` (default `true`), each plugin remembers the trajectory it
sent for each candidate and starts the next profile from that plan's velocity and acceleration at
the current odometry time, instead of the measured ego state. The profile then no longer follows
measurement noise or small tracking errors; the controller corrects those. Limits and the input are
still evaluated every cycle, so a changed limit, a new zone or a binding input deceleration are
followed without restarting. Since the forward simulation is deterministic, starting from the
previous plan reproduces it when nothing changes.

The profile restarts from the measured ego state when the previous plan cannot be used or the ego no
longer tracks it:

| Parameter                                                    | Default | Restart when                                                          |
| ------------------------------------------------------------ | ------- | --------------------------------------------------------------------- |
| `velocity_limits.fixed_profile.reset_velocity_deviation`     | `0.5`   | the measured velocity differs from the plan by more than this [m/s].  |
| `velocity_limits.fixed_profile.reset_acceleration_deviation` | `0.5`   | the measured acceleration differs by more than this [m/s²].           |
| `velocity_limits.fixed_profile.reset_distance_deviation`     | `1.0`   | the ego is farther than this from its planned position [m].           |
| `velocity_limits.fixed_profile.reset_time_gap`               | `0.5`   | the previous plan is older than this, or newer than the odometry [s]. |

Previous plans are kept per candidate index; a plan from another candidate is only used if it passes
the same checks against the measured state. They are forgotten when a parameter of the group changes
or the plugin is disabled. A restart moves the first point to the measured velocity, a step of at most
`reset_velocity_deviation`. Since an upstream trajectory usually starts at the measured velocity, a
fixed profile can start above it by up to the same amount.

##### ModelPlanningFactorID

Inspection-only plugin. It reads the current trajectory speed/acceleration, never mutates points, and publishes stop/slowdown factors on `/planning/planning_factors/diffusion_planner` with `module` `diffusion_planner` so existing RViz, FOA, and API consumers stay aligned.

Place it immediately after `TrajectoryTimeSequenceRawOptimizer` and before `ExternalVelocityLimit` / object-stop plugins so the labels describe the post-TSO model output rather than later velocity capping or ObstacleStop / TrafficLightStop insertions. The host always calls `publish_planning_factor()` even when this plugin returns `Unchanged`.

Disable the learning-based planner node's own `planning_factor.enable_stop` in launch to avoid two publishers on the same topic.

| Parameter                                               | Default | Description                                             |
| ------------------------------------------------------- | ------- | ------------------------------------------------------- |
| `use_model_planning_factor_id`                          | `true`  | Enable the plugin when included in `plugin_names`.      |
| `model_planning_factor_id.enable_stop`                  | `true`  | Publish STOP factors inferred from the trajectory.      |
| `model_planning_factor_id.enable_slowdown`              | `false` | Publish SLOW_DOWN factors inferred from the trajectory. |
| `model_planning_factor_id.stop_velocity_threshold`      | `0.1`   | Stop detection velocity [m/s].                          |
| `model_planning_factor_id.stop_keep_duration_threshold` | `1.0`   | Required stop duration [s].                             |
| `model_planning_factor_id.slowdown_accel_threshold`     | `-0.3`  | Slowdown detection acceleration [m/s²].                 |

##### Stop Point Fixer

The Stop Point Fixer plugin addresses trajectory issues when the ego vehicle is stationary or moving at very low speeds. It prevents problematic trajectory points that could cause planning issues by replacing the trajectory with a single stop point when either of two independently configurable conditions is met:

- **Close stop**: the trajectory's last point (stop point) is within a minimum distance threshold from ego
- **Long stop**: the trajectory commands ego to remain stopped for longer than a minimum duration threshold

Both conditions are individually enabled or disabled via parameters, allowing fine-grained control over when the override is applied.

##### Obstacle Stop

The Obstacle Stop plugin serves as a deterministic safety shield operating independently of the generative model to:

- **Enforce Longitudinal Safety**: Monitors the gap to dynamic and static obstacles to ensure a safe distance is maintained under all kinematic conditions.
- **Ensure Definitive Stopping For Obstacles**: Guarantees zero-velocity set-points for stationary objects (e.g., traffic lights, stopped vehicles) to prevent "creeping" or oscillating behavior near obstacles.

##### Velocity Modifier

The Velocity Modifier plugins is responsible for ensuring the velocity profile is smooth and feasible, and adjusts the velocity profile if an anomaly is detected while respecting deceleration and jerk constraints.

### Dependencies

This package depends on the following packages:

- `autoware_internal_planning_msgs`: For candidate trajectory message types
- `autoware_planning_msgs`: For output trajectory message types
- `autoware_motion_utils`: Motion-related utility functions
- `autoware_trajectory`: Trajectory data structures and utilities
- `autoware_utils`: Common utility functions

### Input/Output

- **Input**: `autoware_internal_planning_msgs::msg::CandidateTrajectories`
- **Output**: Modified `autoware_internal_planning_msgs::msg::CandidateTrajectories` and selected `autoware_planning_msgs::msg::Trajectory`
- **Velocity profile resets**: `~/debug/velocity_profile_reset`
  (`autoware_internal_debug_msgs::msg::StringStamped`). The map and external velocity limit
  plugins publish one event per candidate reset. The stamp is the publication time, and `data`
  contains `plugin=<instance> candidate=<index> reason=<reason>`. Reasons are `reset_time_gap`,
  `reset_velocity`, `reset_acceleration`, and `reset_distance`. The topic uses reliable, volatile
  QoS with a depth of 10.

The node also subscribes to odometry, acceleration, steering status, predicted and tracked objects, point cloud data, traffic signals, route, and vector map topics for plugin use.

### Parameters

See the unified parameter schema above and `config/trajectory_modifier.param.yaml`.

Parameters can be set via YAML configuration files in the `config/` directory.

### Adding New Modifier Plugins

To add a new modifier plugin:

1. Create header and source files in `trajectory_modifier_plugins/`
2. Inherit from `TrajectoryModifierPluginBase`
3. Implement the required virtual methods
4. Export the plugin with `TrajectoryModifierPluginBase` and register it in `plugins.xml`
5. Add plugin-specific parameters to the schema and config files
