# 华测CGI610驱动

A Python module for accessing the CGI_610  by Dora.

## 文件说明

1. **CGI_610_driver_dora.py** 通过串口读取GNSS传感器

2. **CGI_610_driver_dora_with_file.py**   是读取目录下 cgi610_outdata.txt 文件中保存的GPS数据进行发布，以解决调试算法时没有GNSS 硬件设备。cgi610_outdata.txt中数据记录于试验场中，起点与终点重合。数据格式如下：

```
$GPCHC,2314,550816.75,167.54,2.15,0.42,0.65,-0.06,0.12,-0.0073,0.0373,0.9965,29.74681270,106.55385685,258.46,-0.006,0.005,-0.033,0.007,18,20,61,0,2*64
```



## 安装依赖项

```
pip install pyserial
pip install transforms3d
```
尝试使用pip的其他源，如使用清华大学源：pip install transforms3d -i https://pypi.tuna.tsinghua.edu.cn/simple


## 安装依赖项

```
PATH=$PATH:$(pwd)
cd abc_project(create folder without-> dora new abc_project --lang python) 
#cd /home/crp/dora_project/dora-rs/dora-hardware/vendors/gnss/GNSS_D300
dora up
sudo chmod 777 /dev/ttyUSB0
dora start gnss_D300_driver_dora.yaml --name first-dataflow
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
dora logs first-dataflow nmea_subscribeTopic
```
