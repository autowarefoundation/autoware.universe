# 6 Introduction to rs_driver's Error Code



## 6.1 Summary

When some events occur, rs_driver informs the caller via a callback function. 

Every event is defined as an error code. Please find their definitions in `common/error_code.hpp`.



## 6.2 Error Code

+ ERRCODE_SUCCESS 

​		Normal status. `rs_driver` doesn't report it.

+ ERRCODE_PCAPEXIT 

​		Every time `rs_driver` reaches the end of the PCAP file, it reports the event ERRCODE_PCAPEXIT.

​		The option `RSInputParam.pcap_repeat` determines whether to play the file repeatly. If yes, rs_driver reports the event ERRCODE_PCAPREPEAT, else reports ERRCODE_PCAPEXIT.

+ ERRCODE_PCAPREPEAT

​		Please see ERRCODE_PCAPEXIT.

+ ERRCODE_MSOPTIMEOUT 

​		`rs_driver` waits for MSOP packets. If it loses the packets more than 1 second, it reports the event ERRCODE_MSOPTIMEOUT.

+ ERRCODE_NODIFOPRECV

​		For mechanical LiDARs, DIFOP packets contains vertical/horizontal angle data. Without the data, point cloud is flat and inaccurate. 

​		On receiving MSOP packets, `rs_driver` checks if the angle data is available. If it is unavailable in 1 second, it reports ERRCODE_NODIFOPRECV.

​		For MEMS liDARs, the angle data is not needed, so `rs_driver` doesn't report this event.

+ ERRCODE_WRONGMSOPLEN

​		Every LiDAR has a fixed-length MSOP packet, which has a identy id at the beginning.

​		On receiving MSOP packets, rs_driver checks the packet length. If it is wrong, rs_driver reports the event ERRCODE_WRONGMSOPLEN. Then rs_driver checks the identity id. If it is wrong, rs_driver reports ERRCODE_MSOPID.

+ ERRCODE_WRONGMSOPID

​		Please see ERRCODE_WRONGMSOPLEN.

+ ERRCODE_WRONGMSOPBLKID

​		MSOP Packets contains blocks. For mechanical LiDARs, every block has its own identity id. rs_driver checks if this id is wrong. If so, rs_driver reports ERRCODE_WRONGMSOPBLKID.

+ ERRCODE_WRONGDIFOPLEN

​		Every LiDAR has a fixed-length DIFOP packet, which has a identy id at the beginning.

​		On receiving DIFOP packets, rs_driver checks the packet length. If it is wrong, rs_driver reports the event ERRCODE_WRONGDIFOPLEN. Then rs_driver checks the identity id. If it is wrong, rs_driver reports ERRCODE_DIFOPID.

+ ERRCODE_WRONGDIFOPID

​		Please see ERRCODE_WRONGDIFOPLEN.

+ ERRCODE_ZEROPOINTS

​		rs_driver gets free point cloud instance from the caller, stuffs it, and returns it to the caller.

​		Before returning, rs_driver checks if the point cloud is empty (no points). If so, rs_driver reports ERRCODE_ZEROPOINTS.

+ ERRCODE_PKTBUFOVERFLOW


​		To receive MSOP/DIFOP packets as soon as possible, there is a packet queue between the recieving thread and the handling thread.

​		If the handling thread is too busy to take packets out from the queue, the queue will overflow. rs_driver will clear it and reports 									   ERRCODE_PKTBUFOVERFLOW.

+ ERRCODE_CLOUDOVERFLOW

​		rs_driver parses MSOP packets to get points, and split them into point cloud frames.

​		For mechanical LiDARs, split by block's azimuth(horizontal angle); For MEMS liDARs, split by Packet's sequence number.

​		If something is wrong with the packet and splitting is not triggered, rs_driver will put more and more points into the point cloud. 

​		To avoid this, rs_driver checks the point cloud, and if it is too large, rs_driver clear it, and reports ERRCODE_CLOUDOVERFLOW.

+ ERRCODE_STARTBEFOREINIT

​		To use rs_driver, follow these steps: create instance, Init() and Start(). 

​		Call Init() before Start(). If not, rs_driver reports ERRCODE_STARTBEFOREINIT.

+ ERRCODE_PCAPWRONGPATH

​		`rs_driver` reads MSOP/DIFOP Packet from the PCAP file `RSInputParam.pcap_path`. If fails, it reports ERRCODE_PCAPWRONGPATH.

+ ERRCODE_POINTCLOUDNULL

​		`rs_driver` does't allocate the point cloud instance. Instead, it gets the instance from the caller via a callback function.

​		If the instance is null, rs_drive reports ERRCODE_POINTCLOUDNULL.

