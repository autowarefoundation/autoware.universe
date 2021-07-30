# Installation steps

> Note: If the CUDA or TensorRT frameworks have already been installed, we strongly recommend uninstalling them first.

1. Set up the Autoware repository

```sh
mkdir -p ~/workspace
cd ~/workspace
git clone git@github.com:tier4/autoware.proj.git
cd autoware.proj
mkdir src
vcs import src < autoware.proj.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

2. Run the setup script to install CUDA, cuDNN 8, OSQP, ROS 2 and TensorRT 7, entering 'y' when prompted (this step will take around 45 minutes)

```sh
./setup_ubuntu20.04.sh
```

3. Build the source code (this will take around 15 minutes)

```sh
source ~/.bashrc
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> Several modules will report stderr output, but these are just warnings and can be safely ignored.

## Updating workspace

When you update your workspace after installation, please follow the steps below.

```sh
cd autoware.proj
git pull

# Usually you can ignore this step, but note that it sometimes causes errors.
rm -rf src
mkdir src
rm -rf build/ install/ log/

vcs import src < autoware.proj.repos
vcs pull src
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
