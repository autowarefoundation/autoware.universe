# 19 **Splitting Rule**



## 19.1  Overview

This document illustrates how RoboSense LiDAR splits frame. Mechanical LiDARs and MEMS LiDARs have different splitting rules. 

This document imports some concepts from below documents.

[How_to avoid packet loss](./17_how_to_avoid_packet_loss.md)

[About point layout](./18_about_point_layout.md)



## 19.2  Mechanical LiDAR

Mechanical LiDAR rotates and sends out points. `rs_driver` splits them by an angle. Everytime`rs_driver` gets a frame. This angle's default value is `0`°. User may change it with the option `RSDecoderParam.split_angle`. 
![split angle](./img/19_01_split_angle.png)

For per scan, LiDAR gets a group of points, one point per channel. `rs_driver` packs them into a `Block`, and writes it into MSOP packet. 

`Block` holds the angle. So called "splitting frame", is to check if the angle across the splitting angle. If so, this group belongs to the next frame, and the older groups belongs to the previous one. 

### 19.2.1 Deviation of Splitting by angle

`Block`'s angle is from its first point, and for latter points, need to plus a offset. These points may across the splitting angle. Around this angle, points may lose or repeat.

Let's estimate the angle span of `Block`. 

With  `600` rpm, seconds per round is:

```c++
1 / (600 / 60) = 0.1 (second)
```

Generally, a scan takes 50~60 us. Take RS16 as an example, it takes `55.5` us. scans per round is:

```c++
 0.1 * 1000000 / 55.5 = 1801.8
```

Then the angle span of `Block` is:

```c++
360 / 1801.8 = 0.1998 (degree)
```

Then the deviation by angle is `0.2` degree.

### 19.2.2 Deviation of Splitting by time

Seconds per scans is `50`~`60` us. This is the deviation by time.

### 19.2.3 points per frame vary

Take RS16 as an example. 

With `600` rpm, it takes `1801.8` scans per round. Round up to `1802` times. Then points per frame is:

```
16 * 1802 = 28,832 (points）
```

LiDAR doesn't  rotate smoothly. takes more scans if faster, and takes less less if slower.

The difference is the laser number of LiDAR. For RS16, it is `16`. 

### 19.2.4 Packet Loss and Out of Order

Mechanical LiDAR sends MSOP packets smoothly. The time interval between two packets is enough, so packet loss and out of order is rare.



## 19.3 MEMS LiDAR

How MEMS LiDAR splits frame, is actually determined on the LiDAR end.

It scans `5` zones, gets 5 points per scan, packs into a `Block`, and writes into MSOP packet. 

In each zone, scan in the Z order. And then get a group of MSOP packets. 

Number these packets from 1.  Take M1 as an example,  the numbers are `1`~`630`.

What left for `rs_driver` to do, is to split packets by the numbers. 

### 19.3.1 Packet Loss and Out of Order

MEMS LiDAR may send multiple MSOP packets at the same time, so packet loss and out of order may happen.

Increase socket buffer is useful if packet loss happens on host, but make no help if it happens between LiDARs and host.

`rs_driver` use `Safe Range` to handle this.



First image how to handle this without `Safe Range`.

+ No packet loss or out of order.  Just check if the packet number is `1`. Split if so.
+ Packet loss, no out of order. For example, packet `1` is losed. Just add check if packet number is less than the previous. Split if so. 
+ Out of order. For example, packet `301` reaches before packet `300`, then this frame is splitted into two.  Dilemma.



Consider `Safe Range` as below.

+ Give a range `RANGE` around `prev_seq`, which is the previous packet number.

```c++
safe_seq_min = prev_seq_ - RANGE
safe_seq_max = prev_seq_ + RANGE
```

+ If MSOP packet number is in (`safe_seq_min_`, `safe_seq_max_`), accept it and don't split.
![safe range](./img/19_02_safe_range.png)

Slight packet loss and out of order is tolerated. 



