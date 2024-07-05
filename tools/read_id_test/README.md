# Quick start
If you dora version is 0.3.5, you can: 
```bash
dora up
dora start dataflow_euclidean_cluster_detect.yml
```
If you have problems about node binaries,you can compile node with the following command.

##  Prepare Work

Before you start compiling, you should check line 256 in rslidar_driver/rslidar_driver_pcap.cc:

```cpp
  param.input_param.pcap_path ="/home/demo/bugreport/rslidar_driver/lidar.pcap"; ///< Set the pcap file directory
```
This path is an absolute path, so you should modify it according to your path.

For CMakeLists.txt, they will find libraries in `${HOME}/dora/......`, you should modify them according to your path.


## Compile node

In the `rslidar_driver` directory, run the following commands:
```bash
cd rs_driver
mkdir build && cd build
cmake ..
make
```

Then, in the `ground_filter` directory, run the following commands:
```bash
cd ../../ground_filter
mkdir build && cd build
cmake ..
make
```

## Start
Run the following command to start the dataflow:
```bash
dora start dataflow_euclidean_cluster_detect.yml
```
Please modify according to your actual path and needs.

