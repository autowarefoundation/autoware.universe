# tier4_map_launch

## 構造

![tier4_map_launch](./map_launch.drawio.svg)

## パッケージ依存関係

`<exec_depend>`を参照してください。

## 使用法

`*.launch.xml` で以下のように記載して `map.launch.py` を使用できます。

`PACKAGE_param_path` としてパラメータパスを提供する必要があることに注意してください。提供する必要があるパラメータパスのリストは、`map.launch.xml` の最上部に記載されています。


```xml
<arg name="map_path" description="point cloud and lanelet2 map directory path"/>
<arg name="lanelet2_map_file" default="lanelet2_map.osm" description="lanelet2 map file name"/>
<arg name="pointcloud_map_file" default="pointcloud_map.pcd" description="pointcloud map file name"/>

<include file="$(find-pkg-share tier4_map_launch)/launch/map.launch.py">
  <arg name="lanelet2_map_path" value="$(var map_path)/$(var lanelet2_map_file)" />
  <arg name="pointcloud_map_path" value="$(var map_path)/$(var pointcloud_map_file)"/>

  <!-- Parameter files -->
  <arg name="FOO_param_path" value="..."/>
  <arg name="BAR_param_path" value="..."/>
  ...
</include>
```

## 注釈

処理負荷を軽減するために、ROS 2 の [Component](https://docs.ros.org/en/galactic/Concepts/About-Composition.html) 機能を使用しています（ROS 1 の Nodelet と同様）。

