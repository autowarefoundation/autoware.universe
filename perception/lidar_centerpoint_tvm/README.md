# lidar_centerpoint_tvm

## Design

### Usage

lidar_centerpoint_tvm is a package for detecting dynamic 3D objects using TVM compiled centerpoint module for different backends.

#### Neural network

This package will not build without a neural network for its inference.
The network is provided by the neural_networks_provider package.
See its design page for more information on how to enable downloading pre-compiled networks (by setting the `DOWNLOAD_ARTIFACTS` cmake variable), or how to handle user-compiled networks.

#### Backend

The backend used for the inference can be selected by setting the `lidar_centerpoint_tvm_BACKEND` cmake variable.
The current available options are `llvm` for a CPU backend, and `vulkan` for a GPU backend.
It defaults to `llvm`.

### Inputs / Outputs

to be added

### Bounding Box

The lidar segmentation node establishes a bounding box for the detected obstacles.
The `L-fit` method of fitting a bounding box to a cluster is used for that.

## Reference

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020).

[2] Lang, Alex H., et al. "Pointpillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://github.com/tianweiy/CenterPoint>

[4] <https://github.com/Abraham423/CenterPoint>

[5] <https://github.com/open-mmlab/OpenPCDet>

## Related issues

<!-- Required -->

- #908: Run Lidar Centerpoint with TVM
