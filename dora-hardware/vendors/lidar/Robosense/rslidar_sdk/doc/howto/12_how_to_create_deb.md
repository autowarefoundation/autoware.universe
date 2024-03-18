# 12 How to create deb



## 12.1 Introduction

Generating a ".deb" installable file is useful.



## 12.2 Create deb

Just run the shell script: 
```
./create_debian.sh
```
The deb file will be generated in the parent directory of `rslidar_sdk`.


## 12.3 Use the deb

Install the deb and set the right config_path. If leave the config_path empty, it will use the `config.yaml` in the ros package path.     

```
<launch>
  <node pkg="rslidar_sdk" name="rslidar_sdk_node" type="rslidar_sdk_node" output="screen">
    <param name="config_path" value=""/>
  </node>
  <!-- rviz -->
  <node pkg="rviz" name="rviz" type="rviz" args="-d $(find rslidar_sdk)/rviz/rviz.rviz" />
</launch>
```



