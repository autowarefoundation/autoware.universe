# awf_2d_overlay_vehicle

Plugin for displaying 2D overlays over the RViz2 3D scene.

Based on the [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)
package, which is currently only released for ROS1, under the 3-Clause BSD license.

## Text Overlay

![Screenshot showing the robot velocity as an overlay above the RViz 3D Scene, as well as the expanded properties of the plugin](doc/screenshot_vel_overlay.png)

Both the text itself and formatting options for the text overlay are specified in
the [OverlayText.msg message type](https://github.com/teamspatzenhirn/awf_2d_overlay_vehicle/blob/main/rviz_2d_overlay_msgs/msg/OverlayText.msg)
.

### Alignment and Positioning

To allow easy positioning of the overlay along the edges of the rviz window, and to support multiple/dynamic window
sizes, the position is given by offsets from the respective border.
Depending on whether the `horizontal_alignment` is `LEFT`, `RIGHT` or `CENTER`,
the `horizontal_distance` field sets the distance to the left or right border, or the offset from center.

For `LEFT` and `RIGHT` alignment, a distance of zero means that the text is aligned to the border without any gap,
a positive distance moves the overlay towards the center.

For `CENTER` alignment, a distance of zero means completely centered, positive values move the overlay towards the
bottom right of the window.

`TOP` and `BOTTOM` for the vertical alignment work just like `LEFT` and `RIGHT` in the horizontal case.

### Using a string topic

A simple coverter node (`rviz2d_from_string_node`) is provided which can covert `std_msgs/msg/String` to `rviz_2d_overlay_msgs/msg/OverlayText`. The working principle is simple, it subscribes to a `String` topic, publishes the content as an `OverlayText` and the other proeries can be set from ROS parameters or by overtaking it in RViz2.

A launch file which runs this node and sets the parameters may look something like:

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='awf_2d_overlay_vehicle',
            executable='string_to_overlay_text',
            name='string_to_overlay_text_1',
            output='screen',
            parameters=[
                {"string_topic": "chatter"},
                {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            ],
        ),
    ])

```

In case a `/chatter` topic is needed this can be published with a single command:

```py
ros2 topic pub /chatter std_msgs/String "data: Hello world"
```

## Circular Gauge Overlay

![Screenshot showing the PieChartDisplay, a circular gauge](doc/screenshot_PieChartDisplay.png)

The `PieChartDisplay` is a rather boring pie chart, as it only displays a single value.
`PieChartDisplay` and "Circular Gauge" are used synonymously in this package.
The gauge allows displaying a
[std_msgs/Float32](https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Float32.msg).
Formatting and positioning, as well as setting the maximum value is only possible in the display options inside rviz.
