import math
from typing import Iterable
from typing import Optional
from typing import Tuple

from geometry_msgs.msg import TransformStamped
import numpy as np


def is_rotation_matrix(R):
    r_transpose = np.transpose(R)
    should_be_identity = np.dot(r_transpose, R)
    identity = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(identity - should_be_identity)
    return n < 1e-6


def rotation_matrix_to_euler_angles(R):
    assert is_rotation_matrix(R)

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def euler_from_quaternion(x, y, z, w):
    # Convert a quaternion into euler angles (roll, pitch, yaw),
    # roll is rotation around x in radians (counterclockwise),
    # pitch is rotation around y in radians (counterclockwise),
    # yaw is rotation around z in radians (counterclockwise)

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def cart_to_homo(mat):
    ret = np.eye(4)
    if mat.shape == (3, 3):
        ret[:3, :3] = mat
    elif mat.shape == (3, 4):
        ret[:3, :] = mat
    else:
        raise ValueError(mat.shape)
    return ret


def build_affine(
    rotation: Optional[Iterable] = None, translation: Optional[Iterable] = None
) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.

    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = get_mat_from_quat(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine


def transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a `TransformStamped` to a affine matrix.

    :param transform: The transform that should be converted
    :returns: The affine transform
    """
    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return build_affine(transform_rotation_matrix, transform_translation)


def get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a rotation matrix.This method is based on quat2mat from https://github.com  f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 , since that library is not available via rosdep.

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [
            [1.0 - (yYZ[0] + zZ), xXYZ[1] - wXYZ[2], xXYZ[2] + wXYZ[1]],
            [xXYZ[1] + wXYZ[2], 1.0 - (xXYZ[0] + zZ), yYZ[1] - wXYZ[0]],
            [xXYZ[2] - wXYZ[1], yYZ[1] + wXYZ[0], 1.0 - (xXYZ[0] + yYZ[0])],
        ]
    )


def get_quat_from_mat(rot_mat: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to a quaternion. This method is a copy of mat2quat from https://github.com f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L150 , since that library is not available via rosdep.Method from Bar-Itzhack, Itzhack Y. (2000), "New method for extracting the quaternion from a rotation matrix", AIAA Journal of Guidance, Control and Dynamics 23(6):1085-1087 (Engineering Note), ISSN 0731-5090.

    :param rot_mat: A roatation matrix
    :returns: An quaternion
    """
    # Decompose rotation matrix
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = rot_mat.flat
    # Create matrix
    K = (
        np.array(
            [
                [Qxx - Qyy - Qzz, 0, 0, 0],
                [Qyx + Qxy, Qyy - Qxx - Qzz, 0, 0],
                [Qzx + Qxz, Qzy + Qyz, Qzz - Qxx - Qyy, 0],
                [Qyz - Qzy, Qzx - Qxz, Qxy - Qyx, Qxx + Qyy + Qzz],
            ]
        )
        / 3.0
    )
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector and reorder to w,x,y,z
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Invert quaternion if w is negative (results in positive w)
    if q[0] < 0:
        q *= -1
    return q


def decompose_affine(affine: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose an affine transformation into a quaternion and the translation.

    :param affine: The affine transformation matrix
    :returns: The quaternion and the translation array
    """
    return get_quat_from_mat(affine[:3, :3]), affine[:3, 3]
