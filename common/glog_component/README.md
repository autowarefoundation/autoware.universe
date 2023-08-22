# glog_component

This package provides the `glog` feature as a ros2 component library. This is used to dynamically load the `glog` feature with container.

## Example

When you load the `glog_component` in container, the launch file can be like below:

```py
glog_component = ComposableNode(
    package="glog_component",
    plugin="glog_component::GlogComponent",
    name="glog_component",
)

container = ComposableNodeContainer(
    name="my_container",
    namespace="",
    package="rclcpp_components",
    executable=LaunchConfiguration("container_executable"),
    composable_node_descriptions=[
        component1,
        component2,
        glog_component,
    ],
)
```