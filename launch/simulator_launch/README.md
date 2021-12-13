# simulator_launch

## Structure

![simulator_launch](./simulator_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

```xml
  <include file="$(find-pkg-share simulator_launch)/launch/simulator.launch.xml">
    <arg name="vehicle_info_param_file" value="VEHICLE_INFO_PARAM_FILE" />
    <arg name="vehicle_model" value="VEHICLE_MODEL"/>
  </include>
```

The simultor model used in simple_plannings_simulator is loaded from "config/simulator_model.param.yaml" in the "`VEHICLE_MODEL`_description" package.