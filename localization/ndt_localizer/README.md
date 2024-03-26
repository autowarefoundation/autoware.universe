# NDT匹配

基于 Dora0.3.2环境开发的NDT匹配。

## 背景

通过数据流输入副本输入点云，与目标点云进行ndt匹配。

# 文件代码说明

1. input_points: 副本输入点云文件夹;
2. map_pcd: 放置目标点云地图；
3. src/map_loader: 用于上载pcd地图;
4. src/ndt_localier: 用于NDT匹配；
5. src/points_downsampler: 对输入点云进行过滤;
6. dataflow_pcap.yml： 数据流脚本。

## 用法

```
bash build.sh(须先修改为自己的路径)
mkdir build
cd build
cmake ..
make 
#把pcd地图放在map_pcd文件夹中
#把pcap文件放在input_points中
dora up
dora start dataflow_pcap.yml --name test
```

## 日志

```
dora logs test dora_ndt_localizer #查看ndt_localizer的数据
```
