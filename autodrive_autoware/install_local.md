# Autoware Local Installation Guide

![Autoware](https://user-images.githubusercontent.com/63835446/158918717-58d6deaf-93fb-47f9-891d-e242b02cba7b.png)

This documentation covers `Docker Installation of Autoware` and testing the installation with `Autoware Planning Simulation`.

> **Note 1**: Installation was tested with Host-PC running Ubuntu 20.04 (with ROS 1 Noetic, ROS 2 Foxy & ROS 2 Galactic) and Docker-Container running Ubuntu 22.04 (with ROS 2 Humble).

> **Note 2**: A dedicated `Autoware_WS` was created on Host-PC to organize `autoware_maps`, `autoware_docker` installation (with ROS 2 Humble) and `autoware_local` installation (with ROS 2 Galactic).

> **Note 3**: Pay close attention to `user@host-pc` (when executing commands from Host-PC) and `container-id` (when executing commands from Docker-Container).

## Official Documentation:

- For Autoware's general documentation, see [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/).

- For detailed documents of Autoware Universe components, see [Autoware Universe Documentation](https://autowarefoundation.github.io/autoware.universe/).

- For the official Autoware installation guide, see [Autoware Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/).
  - [Autoware Local (Source) Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
  - [Autoware Docker Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)
    - [Quick Start Version Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation-prebuilt/)
    - [Development Version Installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation-devel/)

- For the official Autoware planning simulation guide, see [Planning Simulation Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/).

- For the official Docker installation guide, see [Get Docker](https://docs.docker.com/get-docker/), and particularly for Ubuntu, see [Docker Ubuntu Installation](https://docs.docker.com/desktop/install/ubuntu/).

---

## Set Up Autoware Development Environment:

1. Create a dedicated workspace for Autoware called `Autoware_WS` on Host-PC to organize different Autoware installations, maps, data, etc., and move to the directory.
```bash
user@host-pc:~$ mkdir -p Autoware_WS
user@host-pc:~$ cd Autoware_WS
```

2. Clone the [`autowarefoundation/autoware`](https://github.com/autowarefoundation/autoware.git) repository into `Autoware_WS`, rename it to `autoware_docker` (to differentiate it from `autoware_local` installation, if any) and move to the directory.
```bash
user@host-pc:~$ git clone https://github.com/autowarefoundation/autoware.git
user@host-pc:~$ sudo mv ~/Autoware_WS/autoware ~/Autoware_WS/autoware_docker
user@host-pc:~$ cd autoware_docker
```

3. Install the required dependencies (Autoware uses [`Ansible`](https://www.ansible.com/) scripts to automate dependency and configuration management).
```bash
user@host-pc:~$ ./setup-dev-env.sh docker # --no-nvidia --no-cuda-drivers (for installation without NVIDIA libraries & CUDA drivers)
user@host-pc:~$ # Setting up the build environment can take up to 1 hour.
                        # >  Are you sure you want to run setup? [y/N]
                        y
user@host-pc:~$ # [Warning] Some Autoware components depend on the CUDA, cuDNN and TensorRT NVIDIA libraries which have end-user license agreements that should be reviewed before installation.
                        # Install NVIDIA libraries? [y/N]
                        N # On certain systems, modifying any existing NVIDIA libraries can break things!
                        y # If you are confident about your system dependencies, you may choose to proceed with the installation of NVIDIA libraries.
```

4. Create `autoware_map` directory within `Autoware_WS` to store map files, and download & unzip `sample-map-planning` (later used for planning simulation) within this directory.
```bash
user@host-pc:~$ cd .. # cd ~/Autoware_WS
user@host-pc:~$ mkdir -p autoware_map
user@host-pc:~$ gdown -O ~/Autoware_WS/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
user@host-pc:~$ unzip -d ~/Autoware_WS/autoware_map ~/Autoware_WS/autoware_map/sample-map-planning.zip
```

5. Pull [`autowarefoundation/autoware-universe`](https://github.com/autowarefoundation/autoware/pkgs/container/autoware-universe) image from [GitHub Packages](https://github.com/features/packages).
```bash
user@host-pc:~$ docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

## Set Up Autoware Workspace:
1. Run the pulled Docker image as a container (Autoware uses [`rocker`](https://github.com/osrf/rocker) to run Docker images with customized local support).
```bash
user@host-pc:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume ~/Autoware_WS/autoware_docker --volume ~/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

2. Create an `src` directory within the `autoware_docker` workspace and clone `autoware` repositories into it (Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces).
```bash
user@container-id:~$ cd Autoware_WS/autoware_docker/
user@container-id:~$ mkdir src
user@container-id:~$ vcs import src < autoware.repos
```

3. Install the required dependencies (Autoware uses [`rosdep`](https://github.com/ros-infrastructure/rosdep) to manage dependencies).
```bash
user@container-id:~$ sudo apt update
user@container-id:~$ rosdep update
user@container-id:~$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
> **Note:** You can ignore the `Invalid version` errors (if any) during `rosdep` installation process.

4. Build the workspace (Autoware uses [colcon](https://github.com/colcon) to build workspaces).
```bash
user@container-id:~$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
> **Note 1:** You can ignore the `stderr` warnings (if any) during the `colcon` build process.

> **Note 2:** For more advanced build options, refer to the [colcon documentation](https://colcon.readthedocs.io/).

> **Note 3:** Use `colcon build --packages-select <package_name> --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` to re-build only specific packages instead of (re)building the entire workspace to avoid large (re)build times.

## Test Autoware Installation with Planning Simulation:

1. Run the pulled Docker image as a container (if not already running).
```bash
user@host-pc:~$ rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume ~/Autoware_WS/autoware_docker --volume ~/Autoware_WS/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

2. Source the `setup.*sh` (e.g., `setup.bash`) file of your workspace:
```bash
user@container-id:~$ source ~/Autoware_WS/autoware_docker/install/setup.bash
```
> **Note:** You can write the above line to the `~/.bashrc` file so that it is automatically executed when a new terminal instance is created.

3. Launch the `planning_simulator` with the `sample-map-planning` map, `sample_vehicle` vehicle, and `sample_sensor_kit` sensor kit.
```
user@container-id:~$ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=~/Autoware_WS/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

4. Tinker around with the `planning_simulator` and explore the depths of Autoware stack!
   ![Autoware-Planning-Simulation](https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/Autoware-Planning-Simulation/Autoware-Planning-Simulation.gif)
> **Note:** Check out the official [Planning Simulation Documentation](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/) for more details.

## Generally Helpful Docker Tips:
1. To access the container while it is running, execute the following command in a new terminal window to start a new bash session inside the container:
```bash
$ docker exec -it <container_name> bash
```

2. To exit the bash session(s), simply execute:
```bash
$ exit
```

3. To kill the container, execute the following command:
```bash
$ docker kill <container_name>
```

4. To remove the container, simply execute:
```bash
$ docker rm <container_name>
```

5. Running or caching multiple docker images, containers, volumes, and networks can quickly consume a lot of disk space. Hence, it is always a good idea to frequently check docker disk utilization:
```bash
$ docker system df
```

6. To avoid utilizing a lot of disk space, it is a good idea to frequently purge docker resources such as images, containers, volumes, and networks that are unused or dangling (i.e. not tagged or associated with a container). There are several ways with many options to achieve this, please refer to appropriate documentation. The easiest way (but a potentially dangerous one) is to use a single command to clean up all the docker resources (dangling or otherwise):
```bash
$ docker system prune -a
```
