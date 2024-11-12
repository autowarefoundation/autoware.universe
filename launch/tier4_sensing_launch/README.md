# tier4_sensing_launch

## 構成

![tier4_sensing_launch](./sensing_launch.drawio.svg)

## パッケージの依存関係

`<package.xml>` の `<exec_depend>` を参照してください。

## 使用方法

`sensing.launch.xml` を使用するには、`*.launch.xml` に以下のように含めることができます。


```xml
  <include file="$(find-pkg-share tier4_sensing_launch)/launch/sensing.launch.xml">
    <arg name="launch_driver" value="true"/>
    <arg name="sensor_model" value="$(var sensor_model)"/>
    <arg name="vehicle_param_file" value="$(find-pkg-share $(var vehicle_model)_description)/config/vehicle_info.param.yaml"/>
    <arg name="vehicle_mirror_param_file" value="$(find-pkg-share $(var vehicle_model)_description)/config/mirror.param.yaml"/>
  </include>
```

## 起動画構成

このパッケージは、`launch`で指定されたセンサーモデルのセンサー設定を見つけます。


```bash
launch/
├── aip_x1 # Sensor model name
│   ├── camera.launch.xml # Camera
│   ├── gnss.launch.xml # GNSS
│   ├── imu.launch.xml # IMU
│   ├── lidar.launch.xml # LiDAR
│   └── pointcloud_preprocessor.launch.py # for preprocessing pointcloud
...
```

## 注釈

このパッケージは、変数を使用した設定を見つけます。

例）


```xml
<include file="$(find-pkg-share tier4_sensing_launch)/launch/$(var sensor_model)/lidar.launch.xml">
```

