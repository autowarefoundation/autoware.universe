# patchwork-plusplus-ros

This is ROS package of Patchwork++ (@ IROS'22), which is a fast and robust ground segmentation method.

<p align="center"><img src=pictures/patchwork++.gif alt="animated" /></p>

> If you are not familiar with ROS, please visit the [original repository][patchwork++link].

> If you follow the [repository][patchwork++link], you can run Patchwork++ in Python and C++ easily.

[patchwork++link]: https://github.com/url-kaist/patchwork-plusplus

## :open_file_folder: What's in this repository

* ROS based Patchwork source code ([patchworkpp.hpp][codelink])
* Demo launch file ([demo.launch][launchlink]) with sample rosbag file. You can execute Patchwork++ simply!

[codelink]: https://github.com/url-kaist/patchwork-plusplus-ros/blob/master/include/patchworkpp/patchworkpp.hpp
[launchlink]: https://github.com/url-kaist/patchwork-plusplus-ros/blob/master/launch/demo.launch

## :package: Prerequisite packages
You may need to install ROS, PCL, Eigen, ...

## :gear: How to build Patchwork++
To build Patchwork++, you can follow below codes.

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin build # or catkin_make
```

## :runner: To run the demo codes
There is a demo which executes Patchwork++ with sample rosbag file. You can download a sample file with the following command.

> For the sample rosbag data, I utilizes [semantickitti2bag](https://github.com/amslabtech/semantickitti2bag) package.

```bash
$ wget https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag
```
> If you have any trouble to download the file by the above command, please click [here][kitti_sample_link] to download the file directly.

[kitti_sample_link]: https://urserver.kaist.ac.kr/publicdata/patchwork++/kitti_00_sample.bag

> The rosbag file is based on the [KITTI][kittilink] dataset. The bin files are merged into the rosbag file format.

> The sample file contains LiDAR sensor data only.

[kittilink]: http://www.cvlibs.net/datasets/kitti/raw_data.php

Then, you can run demo as follows.

```bash
# Start Patchwork++
$ roslaunch patchworkpp demo.launch
# Start the bag file
$ rosbag play kitti_00_sample.bag
```

## :pushpin: TODO List
- [ ] Update additional demo codes processing data with .bin file format
- [ ] Generalize point type in the source code
- [ ] Add visualization result of demo codes in readme

## Citation
If you use our codes, please cite our [paper][patchwork++arXivLink].

In addition, you can also check the paper of our baseline(Patchwork) [here][patchworkarXivlink].

[patchwork++arXivLink]: https://arxiv.org/abs/2207.11919
[patchworkarXivlink]: https://arxiv.org/abs/2108.05560

```
@inproceedings{lee2022patchworkpp,
    title={{Patchwork++: Fast and robust ground segmentation solving partial under-segmentation using 3D point cloud}},
    author={Lee, Seungjae and Lim, Hyungtae and Myung, Hyun},
    booktitle={Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.},
    year={2022},
    note={{Submitted}} 
}
```
```
@article{lim2021patchwork,
    title={Patchwork: Concentric Zone-based Region-wise Ground Segmentation with Ground Likelihood Estimation Using a 3D LiDAR Sensor},
    author={Lim, Hyungtae and Minho, Oh and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2021}
}
```

## :postbox: Contact
If you have any question, don't be hesitate let us know!

* [Seungjae Lee][sjlink] :envelope: (sj98lee at kaist.ac.kr)
* [Hyungtae Lim][htlink] :envelope: (shapelim at kaist.ac.kr)

[sjlink]: https://github.com/seungjae24
[htlink]: https://github.com/LimHyungTae

