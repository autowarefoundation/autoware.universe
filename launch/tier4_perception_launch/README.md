# tier4_perception_launch

## 構造

![tier4_perception_launch](./perception_launch.drawio.svg)

## パッケージの依存関係

`<exec_depend>` を `package.xml` で参照してください。

## 使用方法

`perception.launch.xml` を使用するには、次のように `*.launch.xml` に含めます。

パラメータパスを `PACKAGE_param_path` として指定する必要があることに注意してください。指定する必要のあるパラメータパスのリストは、`perception.launch.xml` の先頭部に記載されています。


```xml
  <include file="$(find-pkg-share tier4_perception_launch)/launch/perception.launch.xml">
    <!-- options for mode: camera_lidar_fusion, lidar, camera -->
    <arg name="mode" value="lidar" />

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```

