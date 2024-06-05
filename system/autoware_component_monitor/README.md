# autoware_component_monitor

## Running

The `autoware_component_monitor` package allows monitoring system usage of component containers.
The composable node inside the package is attached to a component container, and it published CPU and memory usage of
the container.

To attach the node into a component container, you can load the composable node directly in your launch file:

```xml

<launch>
    <group>
        <push-ros-namespace namespace="your_namespace"/>
        ...

        <load_composable_node target="$(var container_name)">
            <composable_node pkg="autoware_component_monitor" plugin="autoware::component_monitor::ComponentMonitor"
                             name="component_monitor"/>
        </load_composable_node>

        ...
    </group>
</launch>
```

Or, you can include `component_monitor.launch.xml` in you launch file with the container name:

```xml

<launch>
    <group>
        <push-ros-namespace namespace="your_namespace"/>
        ...

        <include file="$(find-pkg-share autoware_component_monitor)/launch/component_monitor.launch.xml">
            <arg name="container_name" value="your_container"/>
        </include>

        ...
    </group>
</launch>
```

## How does monitoring works

The package uses the `top` command under the hood. `top -b -n 1 -p PID` command is tried to run at 10 Hz to get
the system usage of the process.

- `-b` activates the batch mode. By default, `top` doesn't exit and prints to stdout periodically. Batch mode allows
  exiting the program.
- `-n` number of times should `top` prints the system usage in batch mode.
- `-p` specifies the PID of the process to monitor.

Here is a sample output:

```text
top - 16:41:52 up  6:19,  1 user,  load average: 1,25, 2,06, 2,57
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s):  6,4 us,  4,3 sy,  0,0 ni, 89,4 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
MiB Mem :  63996,3 total,   4007,9 free,  31087,8 used,  28900,6 buff/cache
MiB Swap:  38147,0 total,  38129,2 free,     17,8 used.  30132,2 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   2795 meb       20   0 2645556   1,1g  40440 S  20,0   1,8  33:13.16 awesome
```

We get 5th, 8th, and 9th fields from the last line, which are RES, %CPU, and %MEM, respectively.
