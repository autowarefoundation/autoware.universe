# tier4_simulator_launch

## 構成

![tier4_simulator_launch](./simulator_launch.drawio.svg)

## パッケージ依存関係

詳細については、`package.xml` の `<exec_depend>` を参照してください。

## 使用方法


```xml
  <include file="$(find-pkg-share tier4_simulator_launch)/launch/simulator.launch.xml">
    <arg name="vehicle_info_param_file" value="VEHICLE_INFO_PARAM_FILE" />
    <arg name="vehicle_model" value="VEHICLE_MODEL"/>
  </include>
```

simple\_planning\_simulator で使用されるシミュレータ モデルは、パッケージ "`VEHICLE\_MODEL`\_description" の "config/simulator\_model.param.yaml" からロードされています。

