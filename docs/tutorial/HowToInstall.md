# Installation steps

> Note: If the CUDA or TensorRT frameworks have already been installed, we strongly recommend uninstalling them first.

1. Set up the Autoware repository
```sh
mkdir -p ~/workspace
cd ~/workspace
git clone git@github.com:tier4/autoware.proj.git
cd autoware.proj
```

2. Run the setup script to install CUDA, cuDNN 8, OSQP, ROS 2 and TensorRT 7, entering 'y' when prompted (this step will take around 45 minutes)
```sh
./setup_ubuntu20.04.sh
```

3. Build the source code (this will take around 15 minutes)
```sh
source ~/.bashrc
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests
```
> Several modules will report stderr output, but these are just warnings and can be safely ignored.
