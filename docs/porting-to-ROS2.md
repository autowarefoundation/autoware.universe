Porting ROS1 code to ROS2
=======================

# Frederik porting `shift_decider`

## Setting up the environment


I use the Autoware.Auto ADE with ros dashing to pull in all dependencies of 
https://github.com/tier4/AutowareArchitectureProposal

    cd ~/ade-home/
    cd AutowareAuto
    git checkout master
    ade start  --enter

**All commands that follow are to be entered in ADE!**

Checkout the `AutowareArchitectureProposal` repo and start with the master branch

    mkdir -p src
    vcs import src < autoware.proj.repos

Now the repository `github.com:tier4/autoware.iv.universe.git` that contains `shift_decider` is there   

    cd src/
    grep -R shift_decider

Now create a new branch for the port to be merged in the upstream `ros2` branch and start porting!


## Tools to help porting

### ros2-migration-tools
Following the instructions at https://github.com/awslabs/ros2-migration-tools
    pip3 install parse_cmake
    git clone https://github.com/awslabs/ros2-migration-tools.git
    wget https://github.com/llvm/llvm-project/releases/download/llvmorg-10.0.0/clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz
    tar xaf clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04.tar.xz 
    cp -r clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04/lib/libclang.so ros2-migration-tools/clang/
    cp -r clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04/lib/libclang.so.10 ros2-migration-tools/clang/
    cp -r clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04/include/ ros2-migration-tools/clang/clang
    
The package needs to be **built with ROS1**. I followed http://wiki.ros.org/noetic/Installation/Ubuntu outside of ADE

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-melodic-ros-base

In https://github.com/awslabs/ros2-migration-tools#setup-the-ros1-packages, it instructs me to compile a ros1 package with colcon to get started.

    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

And that fails. I thought `colcon` is only for ROS2 so it should only work after porting, not before. I retried with `catkin_make` but also ran into issues there
```
frederik.beaujean@frederik-beaujean-01:~/ade-home/AutowareArchitectureProposal$ catkin_make --source src/autoware/autoware.iv/control/shift_decider/ -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
Base path: /home/frederik.beaujean/ade-home/AutowareArchitectureProposal
Source space: /home/frederik.beaujean/ade-home/AutowareArchitectureProposal/src/autoware/autoware.iv/control/shift_decider
Build space: /home/frederik.beaujean/ade-home/AutowareArchitectureProposal/build
Devel space: /home/frederik.beaujean/ade-home/AutowareArchitectureProposal/devel
Install space: /home/frederik.beaujean/ade-home/AutowareArchitectureProposal/install
####
#### Running command: "cmake /home/frederik.beaujean/ade-home/AutowareArchitectureProposal/src/autoware/autoware.iv/control/shift_decider -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCATKIN_DEVEL_PREFIX=/home/frederik.beaujean/ade-home/AutowareArchitectureProposal/devel -DCMAKE_INSTALL_PREFIX=/home/frederik.beaujean/ade-home/AutowareArchitectureProposal/install -G Unix Makefiles" in "/home/frederik.beaujean/ade-home/AutowareArchitectureProposal/build"
####
CMake Error: The source "/home/frederik.beaujean/ade-home/AutowareArchitectureProposal/src/autoware/autoware.iv/control/shift_decider/CMakeLists.txt" does not match the source "/home/frederik.beaujean/ade-home/AutowareArchitectureProposal/src/CMakeLists.txt" used to generate cache.  Re-run cmake with a different source directory.
Invoking "cmake" failed
```
According to an expert:
>  You should be able to compile it with colcon, cause it works for both ROS 1 and ROS 2 code. You are getting the same error with catkin so it's probably something related to ROS 1 and the build instructions.

# Nikolai porting `vehicle_cmd_gate`

The basic setup is the same as Fred, I also used the AutowareAuto ADE.

The best source on migrating is the [migration guide](https://index.ros.org/doc/ros2/Contributing/Migration-Guide/), but the steps I followed are:

## Copying the ROS1 code
Branch off `ros2` and `git checkout control/vehicle_cmd_gate`.

## Rewriting package.xml
The migration guide covers this well. See [here](https://www.ros.org/reps/rep-0149.html) for a reference of the most recent version of this format.

## Rewriting CMakeLists.txt
Pretty straightforward by following the example of the already ported `simple_planning_simulator` package and the [pub-sub tutorial](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/#cpppubsub).

## ROS -> RCLCPP
Search-and-replace "ros" with "rclcpp" and adjust everything that needs adjusting. From here on I constantly ran `colcon build --packages-up-to vehicle_cmd_gate` and looked at the first error.

## Replacing std_msgs
ROS2 forces you to define semantically meaningful wrappers around primitive (number) types. It was luckily pretty clear which package they should belong to.

## Changing the namespaces and header files for generated message types
An additional `::msg` needed to be inserted between the package namespace and the class name.

The included headers similarly had to touched up with an extra `/msg` in the path and needed to be converted to `snake_case` – Sublime Text has a handy "Case Conversion" package for that. Unfortunately that gave a pretty cryptic error. Turns out _two_ files are being generated: One for C types (`.h` headers) and and one for CPP types (`.hpp` headers). So don't forget to change `.h` to `.hpp` too.

## Inheriting from Node instead of NodeHandle members
That's where differences start to show – I decided to make the `VehicleCmdGate` a `Node` even though the filename would suggest that `vehicle_cmd_gate_node.cpp` would be it. That's because it has publishers, subscribers, and logging. It previously had _two_ NodeHandles, a public one and a private one (`"~"`). The public one was unused and could be removed. Private nodes are not supported in ROS2, so I simply made it public, but that is an area that needs to be brought up in review.

## Latched topics
For each latched publisher, I just used `transient_local` QoS on the publisher, and we will need to do the same on all subscribers to that topic.

## Timing issues
The differences didn't matter here so much. The only thing I'm not so sure about is how to port a `ros::Timer`. In my understanding, `rclcpp::WallTimer` doesn't allow for using the `/clock` topic so it's not equivalent, i.e. the timer doesn't stop when simulation time stops.

## Parameters
It's not strictly necessary, but you probably want to make sure the filename is `xyz.param.yaml`. Then come two steps:

### Adjust code
    double vel_lim;
    pnh_.param<double>("vel_lim", vel_lim, 25.0);

becomes

    const double vel_lim = declare_parameter("vel_lim", 25.0);

which is equivalent to

    const double vel_lim = declare_parameter<double>("vel_lim", 25.0);

## Adjust param file
Two levels of hierarchy need to be added around the parameters themselves:

    <node name or /**>:
      ros__parameters:
        <params>

Also, ROS1 didn't have a problem when you specify an integer, e.g. `28` for a `double` parameter, but ROS2 does:

    [vehicle_cmd_gate-1] terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterTypeException'
    [vehicle_cmd_gate-1]   what():  parameter 'vel_lim' has invalid type: expected [double] got [integer]

Best to just change `28` to `28.0` in the param file. See also [this issue](https://github.com/ros2/rclcpp/issues/979).


## Launch file
There is a [migration guide](https://index.ros.org/doc/ros2/Tutorials/Launch-files-migration-guide/). One thing it doesn't mention is that the `.launch` file also needs to be renamed to `.launch.xml`.