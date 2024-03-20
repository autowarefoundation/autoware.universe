# Dora-rs pointcloud preprocessor


## 依赖
```bash
# eigen库
sudo apt install libeigen3-dev
```
## 编译
编译pointcloud_preprocessor预处理文件
```bash
cd pointcloud_preprocessor #这条指令可以不执行，但一定要在pointcloud_preprocessor路径下
mkdir build && cd build
cmake ..
cmake --build .
```

## 预处理
```bash
# 在pointcloud_preprocessor路径下
dora up
# 在启动前，需要把dataflow_pcap.yml中，节点的source
dora start pointcloud_preprocessor.yml --name test
```

## 在RVIZ2中显示

在pointcloud_preprocessor.yml文件中添加,如果不需要可以删除

```yaml
  - id: lidar_to_ros2
    operator:
        python: ../../dora-hardware/dora_to_ros2/lidar/lidar_to_ros2.py
        inputs:
          pointcloud: ring_outlier_fliter/pointcloud
```

效果如下：

![1](/home/crp/autoware.universe/localization/pointcloud_preprocessor/figure/1.gif)
