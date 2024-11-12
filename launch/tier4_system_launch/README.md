# tier4_system_launch

## 構造

![tier4_system_launch](./system_launch.drawio.svg)

## パッケージ依存

`package.xml` に含まれる `<exec_depend>` を参照してください。

## 使い方

パラメータパスを `PACKAGE_param_path` として指定する必要があります。必要となるパラメータパスのリストは `system.launch.xml` の先頭に記述されています。


```xml
  <include file="$(find-pkg-share tier4_system_launch)/launch/system.launch.xml">
    <arg name="run_mode" value="online"/>
    <arg name="sensor_model" value="SENSOR_MODEL"/>

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```

