from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import numpy as np
from perception_benchmark_tool.benchmark_tools.math_utils import build_affine
from perception_benchmark_tool.benchmark_tools.math_utils import decompose_affine
from perception_benchmark_tool.benchmark_tools.math_utils import rotation_matrix_to_euler_angles
from perception_benchmark_tool.benchmark_tools.math_utils import transform_to_affine
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import tf_transformations


def create_point_cloud_mgs(frame_id, lidar_frame, ros_time):
    msg = PointCloud2()
    msg.header.stamp = ros_time.to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(lidar_frame)
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = False
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    # We pass `data` directly to we avoid using `data` setter.
    # Otherwise ROS2 converts data to `array.array` which slows down as it copies memory internally.
    # Both, `bytearray` and `array.array`, implement Python buffer protocol, so we should not see unpredictable
    # behavior.
    # deepcode ignore W0212: Avoid conversion from `bytearray` to `array.array`.
    msg._data = lidar_frame
    return msg


def create_image_msgs(frame_id, camera_frame, ros_time):
    cv_bridge = CvBridge()
    image_mgs = cv_bridge.cv2_to_imgmsg(cvim=camera_frame, encoding="rgb8")
    image_mgs.header.stamp = ros_time.to_msg()
    image_mgs.header.frame_id = frame_id
    return image_mgs


def create_camera_info(frame_id, camera_calibration, ros_time):
    camera_info = CameraInfo()
    # cam = P2 * RO_rect * Tr_velo_to_cam * velo
    camera_info.header.frame_id = frame_id
    camera_info.header.stamp = ros_time.to_msg()
    camera_info.width = camera_calibration["width"]
    camera_info.height = camera_calibration["height"]
    camera_info.p = (
        np.array(camera_calibration["vehicle_to_image"]).astype(float).flatten().tolist()
    )
    return camera_info


def make_transform_stamped(header_frame_id, child_frame_id, lidar_transform, ros_time):
    rot_mat = lidar_transform[0:3, 0:3]
    [rx, ry, rz] = rotation_matrix_to_euler_angles(rot_mat)
    static_transform_stamped = TransformStamped()
    static_transform_stamped.header.stamp = ros_time.to_msg()
    static_transform_stamped.header.frame_id = header_frame_id
    static_transform_stamped.child_frame_id = child_frame_id
    static_transform_stamped.transform.translation.x = float(lidar_transform[0, 3])
    static_transform_stamped.transform.translation.y = float(lidar_transform[1, 3])
    static_transform_stamped.transform.translation.z = float(lidar_transform[2, 3])
    quat = tf_transformations.quaternion_from_euler(rx, ry, rz)
    static_transform_stamped.transform.rotation.x = quat[0]
    static_transform_stamped.transform.rotation.y = quat[1]
    static_transform_stamped.transform.rotation.z = quat[2]
    static_transform_stamped.transform.rotation.w = quat[3]

    return static_transform_stamped


def do_transform_pose(pose: Pose, transform: TransformStamped) -> Pose:
    """
    Transform a `Pose` using a given `TransformStamped`. This method is used to share the tranformation done in `do_transform_pose_stamped()` and `do_transform_pose_with_covariance_stamped()`.

    :param pose: The pose
    :param transform: The transform
    :returns: The transformed pose
    """
    quaternion, point = decompose_affine(
        np.matmul(
            transform_to_affine(transform),
            build_affine(
                translation=[pose.position.x, pose.position.y, pose.position.z],
                rotation=[
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                ],
            ),
        )
    )
    res = Pose()
    res.position.x = point[0]
    res.position.y = point[1]
    res.position.z = point[2]
    res.orientation.w = quaternion[0]
    res.orientation.x = quaternion[1]
    res.orientation.y = quaternion[2]
    res.orientation.z = quaternion[3]
    return res


# PoseStamped
def do_transform_pose_stamped(pose: PoseStamped, transform: TransformStamped) -> PoseStamped:
    """
    Transform a `PoseStamped` using a given `TransformStamped`.

    :param pose: The stamped pose
    :param transform: The transform
    :returns: The transformed pose stamped
    """
    res = PoseStamped()
    res.pose = do_transform_pose(pose.pose, transform)
    res.header = transform.header
    return res
