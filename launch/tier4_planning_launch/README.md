# tier4_planning_launch

## 構成

![tier4_planning_launch](./planning_launch.drawio.svg)

## パッケージの依存関係

`package.xml`の`<exec_depend>`を参照してください。

## 用法

パラメータのパスを`PACKAGE_param_path`として指定する必要があることに注意してください。指定する必要のあるパラメータのパスは`planning.launch.xml`の先頭で記載されています。


```xml
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <!-- Parameter files -->
  <arg name="FOO_NODE_param_path" value="..."/>
  <arg name="BAR_NODE_param_path" value="..."/>
  ...
</include>
```

