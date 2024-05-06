# geometry_msgs

This package provides messages for common geometric primitives such as points, vectors, and poses. These primitives are designed to provide a common data type and facilitate interoperability throughout the system.

For more information about ROS 2 interfaces, see [docs.ros.org](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html).

## Messages (.msg)
* [Accel](msg/Accel.msg): Expresses acceleration in free space broken into its linear and angular parts.
* [AccelStamped](msg/AccelStamped.msg): An accel with reference coordinate frame and timestamp.
* [AccelWithCovariance](msg/AccelWithCovariance.msg): Acceleration in free space with uncertainty.
* [AccelWithCovarianceStamped](msg/AccelWithCovarianceStamped.msg): An estimated accel with reference coordinate frame and timestamp.
* [Inertia](msg/Inertia.msg): Expresses the inertial properties of a link.
* [InertiaStamped](msg/InertiaStamped.msg): An Inertia with reference coordinate frame and timestamp.
* [Point32](msg/Point32.msg): The position of a 3-dimensional point in free space, with 32-bit fields.
* [Point](msg/Point.msg): The position of a 3-dimensional point in free space.
* [PointStamped](msg/PointStamped.msg): Point with reference coordinate frame and timestamp.
* [Polygon](msg/Polygon.msg): A specification of a polygon where the first and last points are assumed to be connected.
* [PolygonStamped](msg/PolygonStamped.msg): A Polygon with reference coordinate frame and timestamp.
* [Pose2D](msg/Pose2D.msg): **Deprecated as of Foxy and will potentially be removed in any following release.**
* [PoseArray](msg/PoseArray.msg): An array of poses with a header for global reference.
* [Pose](msg/Pose.msg): A representation of pose in free space, composed of position and orientation.
* [PoseStamped](msg/PoseStamped.msg): A Pose with reference coordinate frame and timestamp.
* [PoseWithCovariance](msg/PoseWithCovariance.msg): A pose in free space with uncertainty.
* [PoseWithCovarianceStamped](msg/PoseWithCovarianceStamped.msg): An estimated pose with a reference coordinate frame and timestamp.
* [Quaternion](msg/Quaternion.msg): An orientation in free space in quaternion form.
* [QuaternionStamped](msg/QuaternionStamped.msg): An orientation with reference coordinate frame and timestamp.
* [Transform](msg/Transform.msg): The transform between two coordinate frames in free space.
* [TransformStamped](msg/TransformStamped.msg): A transform from coordinate frame header.frame_id to the coordinate frame child_frame_id.
* [Twist](msg/Twist.msg): Velocity in 3-dimensional free space broken into its linear and angular parts.
* [TwistStamped](msg/TwistStamped.msg): A twist with reference coordinate frame and timestamp.
* [TwistWithCovariance](msg/TwistWithCovariance.msg): Velocity in 3-dimensional free space with uncertainty.
* [TwistWithCovarianceStamped](msg/TwistWithCovarianceStamped.msg): An estimated twist with reference coordinate frame and timestamp.
* [Vector3](msg/Vector3.msg): Represents a vector in 3-dimensional free space.
* [Vector3Stamped](msg/Vector3Stamped.msg): Represents a Vector3 with reference coordinate frame and timestamp.
* [Wrench](msg/Wrench.msg): Represents force in free space, separated into its linear and angular parts.
* [WrenchStamped](msg/WrenchStamped.msg): A wrench with reference coordinate frame and timestamp.


## Quality Declaration
This package claims to be in the **Quality Level 1** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.
