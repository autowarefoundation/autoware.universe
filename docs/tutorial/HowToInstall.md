# Installation steps

> Note: If the CUDA or TensorRT frameworks have already been installed, we strongly recommend uninstalling them first.

We use [vcstool](https://github.com/dirk-thomas/vcstool) to setup the workspace. Please install it first if it's not installed yet.

```sh
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update && sudo apt-get install python3-vcstool
```

1. Set up the Autoware repository

   ```sh
   mkdir -p ~/workspace
   cd ~/workspace
   git clone git@github.com:tier4/autoware.proj.git
   cd autoware.proj
   mkdir src
   vcs import src < autoware.proj.repos
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
