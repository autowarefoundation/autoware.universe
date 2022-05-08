# perception_benchmark_tool

This package contains various tools for testing the Waymo dataset and perception stack of Autoware.Universe. This tool
is written in a way that it can be extended with other daatsets.

## Installation

For testing 3D Object Tracking with Waymo Dataset, follow the given steps.

1- Download the Waymo dataset validation segment(.tfrecord) files from given link.

<https://waymo.com/open/download/>

Just one of the validation_000x.tar files is enough for evaluation. Unpack it to the desired directory.

```bash
cd ~/Downloads/
tar -xvf validation_validation_0000.tar
```

2- Install the Waymo Open Dataset Toolkit.

```bash
pip3 install waymo-open-dataset-tf-2-4-0
```

3- For running Autoware.Universe with the Waymo evaluation node,

Lidar point clouds and camera images are encoded in the .tfrecord file. It may take about ~60-90 seconds to decode
the data back.

Export waymo.launch file path to DATASET variable
export DATASET_PATH

```bash
ros2 run perception_benchmark_tool perception_benchmark_node \
$HOME/Downloads/validation_validation_0000 \
$YOUR_AUTOWARE_PATH/src/universe/autoware/benchmarking/benchmarking_launch/launch/waymo.launch.xml \
--ros-args \
-p prediction_path:=$YOUR_AUTOWARE_PATH/src/universe/autoware/benchmarking/perception_benchmark_tool/benchmarking_result/predictions.bin \
-p ground_truth_path:=$YOUR_AUTOWARE_PATH/src/universe/autoware/benchmarking/perception_benchmark_tool/benchmarking_result/gt.bin
```

This command will run the perception stack with waymo. We will get the ground truth and prediction files in the file
paths we give as arguments to the node.

Lidar point clouds and camera images are encoded in the .tfrecord file. It may take about ~60-90 seconds to decode
the data back for each segment file.

4- Install Waymo Open Dataset toolkit for metric computation:

Follow given command or instruction provided by the Waymo: <https://github.com/waymo-research/waymo-open-dataset/blob/master/docs/quick_start.md>

```bash
git clone https://github.com/waymo-research/waymo-open-dataset.git waymo-od
cd waymo-od
git checkout remotes/origin/master
sudo apt-get install --assume-yes pkg-config zip g++ zlib1g-dev unzip python3 python3-pip
BAZEL_VERSION=3.1.0
wget https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
sudo bash bazel-${BAZEL_VERSION}-installer-linux-x86_64.sh
sudo apt install build-essential
./configure.sh
bazel clean
bazel build waymo_open_dataset/metrics/tools/compute_tracking_metrics_main
```

5- Evaluate tracking result

```bash
bazel-bin/waymo_open_dataset/metrics/tools/compute_tracking_metrics_main \
/$YOUR_AUTOWARE_PATH/src/universe/autoware/benchmarking/perception_benchmark_tool/benchmarking_result/predictions.bin \
/$YOUR_AUTOWARE_PATH/src/universe/autoware/benchmarking/perception_benchmark_tool/benchmarking_result/gt.bin
```

## Result

The evaluation result of the Perception pipeline on the waymo dataset is presented below.

In our perception pipeline, we assumed the width and length of pedestrians to be 1 meter.
Waymo Dataset has so strict tracking evaluation IoU scores. (Vehicle: 0.7 , Pedestrian and Cyclist: 0.5).
For this reason with given default values, pedestrians and cyclists do not match our perception pipeline.

For more realistic results for pedestrians, we need to change the bounding box description of pedestrians.

### lidar_centerpoint

![Screenshot from 2022-05-09 01-10-48](https://user-images.githubusercontent.com/12658936/167317879-fd1777c3-e7d4-4492-9108-673f9a2f774b.png)

### lidar_apollo_instance_segmentation
![Screenshot from 2022-05-09 02-05-03](https://user-images.githubusercontent.com/12658936/167319631-aa8a7f01-88d9-4db0-8e30-ae5013359a12.png)

