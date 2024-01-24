# Autoware Jetson Installation Guide

![Autoware](https://user-images.githubusercontent.com/63835446/158918717-58d6deaf-93fb-47f9-891d-e242b02cba7b.png)

This documentation covers `Local Installation of Autoware` on [NVIDIA Jetson Devices](https://developer.nvidia.com/embedded/jetson-developer-kits) and testing the installation with `Autoware Planning Simulation`.

> **Note 1**: Installation was tested with [Jetson Orin Nano] (on [Nigel](https://youtu.be/UVIShZuZmpg?si=rcPk0l3ea3gm9eAH)) and [Jetson Xavier NX] (on [F1TENTH](https://f1tenth.org/)) running [L4T (or Jetson Linux)](https://developer.nvidia.com/embedded/jetson-linux) with Ubuntu 20.04 (with ROS 1 Noetic, ROS 2 Foxy & ROS 2 Galactic).

> **Note 2**: A dedicated `Autoware_WS` was created on the Jetson-Device to organize `autoware_maps` and `autoware_local` installation (with ROS 2 Galactic).

## Official Documentation:

- For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

- For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

- For the official Autoware installation guide, see [Autoware Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/).
  - [Autoware Local (Source) Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
  - [Autoware Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)
    - [Quick Start Version Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation-prebuilt/)
    - [Development Version Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation-devel/)

- For the official Autoware planning simulation guide, see [Planning Simulation Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/).

- For the official ROS 2 Galactic installation guide, see [ROS 2 Galactic Installation (Ubuntu 20.04)](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

- For more information on NVIDIA® Jetson™ developer kits, see [Jetson Developer Kits Page](https://developer.nvidia.com/embedded/jetson-developer-kits).

- For more information on NVIDIA® Jetson™ modules, see [Jetson Modules Page](https://developer.nvidia.com/embedded/jetson-modules).
 
- For more information on NVIDIA® Jetson™ software, see [Jetson Software Page](https://developer.nvidia.com/embedded/develop/software).

---

## Install ROS 2 Galactic:
#### (Approximate Time Investment: 0.5 Hours)

Refer to the official [ROS 2 Galactic Installation (Ubuntu 20.04)](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) guide to install ROS 2 Galactic Desktop on your Jetson-Device.

## Set Up Autoware Development Environment:
#### (Approximate Time Investment: 0.5 Hours)

1. Create a dedicated workspace for Autoware called `Autoware_WS` on Jetson-Device to organize different Autoware installations, maps, data, etc., and move to the directory.
```bash
user@jetson-device:~$ mkdir -p Autoware_WS
user@jetson-device:~$ cd Autoware_WS
```

2. Clone the [`autowarefoundation/autoware`](https://github.com/autowarefoundation/autoware.git) repository into `Autoware_WS`, rename it to `autoware_local` (to differentiate it from `autoware_docker` installation, if any) and move to the directory.
```bash
user@jetson-device:~$ git clone https://github.com/autowarefoundation/autoware.git
user@jetson-device:~$ sudo mv ~/Autoware_WS/autoware ~/Autoware_WS/autoware_local
user@jetson-device:~$ cd autoware_local
```

3. Install the required dependencies (Autoware uses [`Ansible`](https://www.ansible.com/) scripts to automate dependency and configuration management).
```bash
user@jetson-device:~$ ./setup-dev-env.sh --no-nvidia --no-cuda-drivers # On certain systems, modifying any existing NVIDIA libraries can break things!
user@jetson-device:~$ # Setting up the build environment can take up to 1 hour.
                      # >  Are you sure you want to run setup? [y/N]
                      y
user@jetson-device:~$ # [Warning] Some Autoware components depend on the CUDA, cuDNN and TensorRT NVIDIA libraries which have end-user license agreements that should be reviewed before installation.
                      # Install NVIDIA libraries? [y/N]
                      N # On certain systems, modifying any existing NVIDIA libraries can break things!
```
> **Note:** The NVIDIA library and CUDA driver installation are disabled as they are already installed with the L4T JetPack SDK. If you force installation here, it may potentially mess up the kernel and cause errors at bootup. You will need to reflash the Jetson-Device if this happens.


4. Create `autoware_map` directory within `Autoware_WS` to store map files, and download & unzip `sample-map-planning` (later used for planning simulation) within this directory.
```bash
user@jetson-device:~$ cd .. # cd ~/Autoware_WS
user@jetson-device:~$ mkdir -p autoware_map
user@jetson-device:~$ gdown -O ~/Autoware_WS/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
user@jetson-device:~$ unzip -d ~/Autoware_WS/autoware_map ~/Autoware_WS/autoware_map/sample-map-planning.zip
```

## Set Up Autoware Workspace:
#### (Approximate Time Investment: >6.0 Hours - Depends on the Jetson-Device)

1. Create an `src` directory within the `autoware_local` workspace and clone `autoware` repositories into it (Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces).
```bash
user@jetson-device:~$ cd Autoware_WS/autoware_local/
user@jetson-device:~$ mkdir src
user@jetson-device:~$ vcs import src < autoware.repos
```

2. Install the required dependencies (Autoware uses [`rosdep`](https://github.com/ros-infrastructure/rosdep) to manage dependencies).
```bash
user@jetson-device:~$ sudo apt update
user@jetson-device:~$ rosdep update
user@jetson-device:~$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
> **Note:** You can ignore the `Invalid version` errors (if any) during `rosdep` installation process.

3. Create a swapfile (Originally from [Autoware Troubleshooting Section](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues)).

   Building Autoware requires a lot of memory. Jetson-Device can crash during the build process because of insufficient memory. To avoid this problem, 16-64 GB of swap should be configured (depending upon the underlying computing platform).

   Optional: Check the current swap size
   ```bash
   free -h
   ```

   Remove the current swapfile (if one exists)
   ```bash
   sudo swapoff /swapfile
   # If there is an error (swapoff: /swapfile: swapoff failed: No such file or directory)
   sudo swapoff -a

   # ONLY necessary if no error encountered with earlier command
   sudo rm /swapfile
   ```
   
   Create a new swapfile
   ```bash
   sudo fallocate -l 64G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

   Optional: Check if the change is reflected
   ```bash
   free -h
   ```

4. Build the workspace (Autoware uses [colcon](https://github.com/colcon) to build workspaces).
```bash
user@jetson-device:~$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> **Note 1:** You can ignore the `stderr` warnings (if any) during the `colcon` build process.

> **Note 2:** If you get build errors (packages fail to build) and the system is hanging for too long, try increasing the swap size.

> **Note 3:** For more advanced build options, refer to the [colcon documentation](https://colcon.readthedocs.io/).

> **Note 4:** Use `colcon build --packages-select <package_name> --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` to re-build only specific packages instead of (re)building the entire workspace to avoid large (re)build times.

## Test Autoware Installation with Planning Simulation:

1. Source the `setup.*sh` (e.g., `setup.bash`) files of ROS distribution (if not already done) and your workspace:
```bash
user@jetson-device:~$ source /opt/ros/galactic/setup.bash
user@jetson-device:~$ source ~/Autoware_WS/autoware_local/install/setup.bash
```
> **Note:** You can write the above lines to the `~/.bashrc` file so that it is automatically executed when a new terminal instance is created.

2. Launch the `planning_simulator` with the `sample-map-planning` map, `sample_vehicle` vehicle, and `sample_sensor_kit` sensor kit.
```
user@jetson-device:~$ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=~/Autoware_WS/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

3. Tinker around with the `planning_simulator` and explore the depths of Autoware stack!
   ![Autoware-Planning-Simulation](https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/Autoware-Planning-Simulation/Autoware-Planning-Simulation.gif)
> **Note:** Check out the official [Planning Simulation Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/) for more details.
