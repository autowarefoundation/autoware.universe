# Overview

当前功能接收 gnss_poser 以及 CGI610提供的姿态信息，输出一个位姿（位置+姿态）

test_ekf_dataflow.yml 通过读取存放的GNSS传感器输出的 “$GPCHC” 语句，发布到 gnss_poser  进行坐标转换。

启动命令：

```
dora start  test_ekf_dataflow.yml --name test
```

说明：

gnss_ekf 节点，数据接收和数据发布分开执行，当接收事件有效时候，在事件处理函数中修改全局变量值。数据发布20ms发送一次全局变量。



下图中红色的箭头表示 gnss_ekf 输出的odom数据  ，实际gnss语句有4643条语句，gnss节点发送周期20ms

![gnss_loss_data](/home/crp/autoware.universe/localization/ekf_localizer/reference/gnss_loss_data.png)
