# 6 **错误码介绍**



## 6.1 概述

`rs_driver`在内部发生某些事件时，会通过回调函数通知调用者。

这些事件定义在`rs_driver`工程文件`common/error_code.hpp`中。

这些事件有三个级别：通知、警告和错误。每个事件有一个错误码，这里介绍错误码的含义。



## 6.2 错误码

+ ERRCODE_SUCCESS

​		正常状态。`rs_driver`不会通知这个事件。

+ ERRCODE_PCAPEXIT 

​		播放PCAP文件时，`rs_driver`在每次解析到文件结束时，通知调用者。

​		通过选项`RSInputParam.pcap_repeat`，可以指定是否循环播放文件。如果是循环播放，`rs_driver`报告事件ERRCODE_PCAPREPEAT；如果不是，则报告事件ERRCODE_PCAPEXIT。

+ ERRCODE_PCAPREPEAT

​		请参考ERRCODE_PCAPEXIT的说明。

+ ERRCODE_MSOPTIMEOUT 

​		`rs_driver`持续等待和接收MSOP Packet，如果在`1`秒内没有收到MSOP Packet，`rs_driver`报告事件ERRCODE_MSOPTIMEOUT。

+ ERRCODE_NODIFOPRECV

​		对于机械式雷达，DIFOP Packet中包含垂直角标定参数。如果缺少这些参数，则点云是扁平的。

​		收到MSOP Packet时，会检查这些参数是否可用。如果在`1`秒内不可用，`rs_driver`报告事件ERRCODE_NODIFOPRECV。

​		对于MEMS雷达，标定工作在雷达内部已经完成，DIFOP Packet不是必要的，所以`rs_driver`不会报告这个事件。

+ ERRCODE_WRONGMSOPLEN

​		每一种雷达的MSOP Packet的包长是确定的，且包头开始位置包括若干标志字节。

​		`rs_driver`接收到MSOP Packet后，先检查包长是否匹配，如果不匹配，则报告错误ERRCODE_WRONGMSOPLEN，然后再检查标志字节，如果不匹配，则报告错误ERRCODE_MSOPID。

+ ERRCODE_WRONGMSOPID

​		请参考ERRCODE_WRONGMSOPLEN的说明。

+ ERRCODE_WRONGMSOPBLKID

​		对于机械式雷达，MSOP Packet中，一组点组成一个`Block`，每个`Block`也有标志字节。`rs_driver`检查`Block`的标志字节是否匹配，如果不匹配，则报告错误ERRCODE_WRONGMSOPBLKID。

+ ERRCODE_WRONGDIFOPLEN

​		每一种雷达的DIFOP Packet的包长是确定的，且包头的开始位置包括若干标志字节。

​		`rs_driver`接收到DIFOP Packet后，会先检查包长是否匹配，如果不匹配，则报告错误ERRCODE_WRONGDIFOPLEN，然后再检查标志字节，如果不匹配，则报告错误ERRCODE_DIFOPID。

+ ERRCODE_WRONGDIFOPID

​		请参考ERRCODE_WRONGDIFOPLEN的说明。

+ ERRCODE_ZEROPOINTS

​		`rs_driver`构建好点云后，通过回调函数返还给调用者。返还前，它检查点云是否为空（即一个点都没有），如果为空，则报告错误ERRCODE_ZEROPOINTS。

+ ERRCODE_PKTBUFOVERFLOW

​		`rs_driver`有两个线程：接收线程和处理线程。为了让接收线程尽快接收，防止丢包，两个线程之间有一个MSOP/DIFOP Packet队列，接收线程将Packet放入队列，处理线程从队列中取出Packet。

​		如果处理线程太忙，来不及读出Packet，则这个队列的长度会超过指定的阈值，这时`rs_driver`会清空队列，并报告错误ERRCODE_PKTBUFOVERFLOW。

+ ERRCODE_CLOUDOVERFLOW

​		`rs_driver`从MSOP Packet解析点，并将它们分割成点云帧。

​		对于机械式雷达，按照Block的水平角分帧；对于MEMS雷达，根据Packet的序列号分帧。

​		如果Packet中的数据有问题，不能触发分帧，则`rs_driver`将在当前点云实例中持续累积点，并持续消耗内存。为了避免这个问题，`rs_driver`在收到解析MSOP Packet时，检查当前点云实例中点的数量，如果超过了指定的阈值，则报告错误ERRCODE_CLOUDOVERFLOW。

+ ERRCODE_STARTBEFOREINIT

​		使用`rs_driver`包括三个步骤：创建实例、初始化Init()、和启动Start()。使用者调用Start()之前必须先调用Init()，如果没有遵循这个次序，则`rs_driver`报告错误ERRCODE_STARTBEFOREINIT。

+ ERRCODE_PCAPWRONGPATH

​		解析PCAP文件时，`rs_driver`从读取RSInputParam.pcap_path指定的文件路径读取PCAP数据。如果这个文件打开失败，则rs_driver报告错误ERRCODE_PCAPWRONGPATH。

+ ERRCODE_POINTCLOUDNULL

​		`rs_driver`不负责分配点云实例，它通过回调函数从调用者获得空闲的点云实例，填充它，然后通过回调函数返还给调用者。

​		如果从调用者获得的点云实例无效，则`rs_driver`报告错误ERRCODE_POINTCLOUDNULL。

