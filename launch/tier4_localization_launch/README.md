# tier4_localization_launch

## Structure

![tier4_localization_launch](./localization_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

Include `localization.launch.xml` in other launch files as follows.

```xml
  <include file="$(find-pkg-share tier4_localization_launch)/launch/localization.launch.xml">
    <arg name="parameter_configuration" value="YOUR_PARAMETER_CONFIGURATION"/>
  </include>
```

Note that you need to provide a parameter configuration file in the argument to specify which parameters to load. See [`autoware.launch.xml` in `autowarefoundation/autoware_launch`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml) for example.

The parameter configuration file should look like as follows.

```xml
<?xml version="1.0"?>
<launch>
  <arg name="package_A_param_path" default="$(find-pkg-share autoware_launch)/config/*/package_A.param.yaml"/>
  ...
</launch>

```
