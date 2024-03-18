# 基于Dora平台的深度摄像头 D435I 驱动

## 1.1概述

本文件为基于dora平台创建节点，通过该节点调用D435i获得彩色数据流数据，深度数据流数据，左右红外数据流数据，imu数据和相机外参数据，并在dora中发布的代码。dora的安装方法可以参考：https://dora.carsmos.ai/docs/guides/Installation/installing/

其中我们在example中提供了几种接受到发布的数据后，将其可视化出来的例子，以便使用者更好的运用本文件

## 2.1快速开始
运行本代码前请确保已经安装了Python3.7以上的版本和D435i的SDK，SDK的安装方法可以参考：https://github.com/IntelRealSense/librealsense
### 2.1.1如果是Windows系统
下载本文件后，按下win+r，在窗口中输入cmd打开终端，然后输入
```
cd /Downloads/D435i-driver  ### /Downloads/D435i-driver为本文件夹所在位置
```
将本文件夹在终端打开，切换到本文件夹所在目录后输入以下命令完成相应依赖的安装
```
pip install -r requirements.txt ### 运行之前请确保已经安装了python3.7以上的版本
```
### 2.1.2如果是Ubuntu系统
按下ctrl+alt+t 打开终端，然后输入：
```
cd /Downloads/D435i-driver  ###/Downloads/D435i-driver为本文件夹所在位置
```
将本文件夹在终端打开，切换到本文件夹所在目录后输入以下命令完成相应依赖的安装
```angular2html
pip install -r requirements.txt ### 运行之前请确保已经安装了python3.7以上的版本
```
### 2.2 运行示例
输入以下代码运行dora：
```
dora up
```
输入以下代码，获得数据流的可视化窗口：
```
dora start examples/webcam_color.yaml     ### 获得彩色原图像的可视化窗口
dora start examples/webcam_depth.yaml     ### 获得深度图像的可视化窗口
dora start examples/webcam_infra1.yaml    ### 获得左红外图像的可视化窗口
dora start examples/webcam_infra2.yaml    ### 获得右红外图像的可视化窗口
```
如需退出，在终端中输入：
```angular2html
dora stop
```
选中数据流后按回车即可退出
#### 效果图如下所示：
![expected output](examples/images/color.png)

![expected output](examples/images/depth.png)

![expected output](examples/images/infra1.png)

![expected output](examples/images/infra2.png)