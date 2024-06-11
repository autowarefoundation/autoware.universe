# autoware_component_monitor

The `autoware_component_monitor` package allows monitoring system usage of component containers.
The composable node inside the package is attached to a component container, and it publishes CPU and memory usage of
the container.

## Running

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

Or, you can include `component_monitor.launch.xml` in your launch file with the container name:

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

The package uses the `top` command under the hood. `top -b -n 1 -E k -p PID` command is tried to run at 10 Hz to get
the system usage of the process.

- `-b` activates the batch mode. By default, `top` doesn't exit and prints to stdout periodically. Batch mode allows
  exiting the program.
- `-n` number of times should `top` prints the system usage in batch mode.
- `-p` specifies the PID of the process to monitor.
- `-E k` changes the memory unit in the summary section to KiB.

Here is a sample output:

```text
top - 13:57:26 up  3:14,  1 user,  load average: 1,09, 1,10, 1,04
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s):  0,0 us,  0,8 sy,  0,0 ni, 99,2 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
KiB Mem : 65532208 total, 35117428 free, 17669824 used, 12744956 buff/cache
KiB Swap: 39062524 total, 39062524 free,        0 used. 45520816 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   3352 meb       20   0 2905940   1,2g  39292 S   0,0   2,0  23:24.01 awesome
```

We get 5th, 8th, and 9th fields from the last line, which are RES, %CPU, and %MEM, respectively.
