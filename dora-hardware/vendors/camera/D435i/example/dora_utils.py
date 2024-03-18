from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.metrics import pairwise_distances


def distance_vertex(left_vertix: np.array, right_vertix: np.array) -> np.array:
    return np.linalg.norm(left_vertix[:2] - right_vertix[:2])


def distance_points(left_point: np.array, right_point: np.array) -> np.array:
    return np.linalg.norm(left_point - right_point)


def closest_vertex(vertices: np.array, point: np.array) -> Tuple[int, np.array]:
    assert (
        vertices.shape[1] == point.shape[1]
    ), "vertice has more coordinate than point"
    argmin_vertice = pairwise_distances(vertices, point).argmin()

    min_vertice = vertices[argmin_vertice]

    return (argmin_vertice, min_vertice)


def get_projection_matrix(position: np.array):
    """Creates a transformation matrix to convert points in the 3D world
    coordinate space with respect to the object.
    Use the transform_points function to transpose a given set of points
    with respect to the object.
    Args:
        position (np.array): [x, y, z, stir, yaw, roll]
    Returns:
        A 4x4 numpy matrix which represents the transformation matrix.
    """
    matrix = np.identity(4)
    [x, y, z, rx, ry, rz, rw] = position
    [roll, pitch, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
        "xyz", degrees=False
    )

    cy = np.cos((yaw))
    sy = np.sin((yaw))
    cr = np.cos((roll))
    sr = np.sin((roll))
    cp = np.cos((pitch))
    sp = np.sin((pitch))
    matrix[:3, 3] = [x, y, z]
    matrix[0, 0] = cp * cy
    matrix[0, 1] = cy * sp * sr - sy * cr
    matrix[0, 2] = -1 * (cy * sp * cr + sy * sr)
    matrix[1, 0] = sy * cp
    matrix[1, 1] = sy * sp * sr + cy * cr
    matrix[1, 2] = cy * sr - sy * sp * cr
    matrix[2, 0] = sp
    matrix[2, 1] = -1 * (cp * sr)
    matrix[2, 2] = cp * cr
    return matrix


def to_world_coordinate(points: np.array, matrix: np.array) -> np.array:
    """Internal function to transform the points according to the
    given matrix. This function either converts the points from
    coordinate space relative to the transform to the world coordinate
    space (using self.matrix), or from world coordinate space to the
    space relative to the transform (using inv(self.matrix))
    Args:
        points: An n by 3 numpy array, where each row is the
            (x, y, z) coordinates of a point.
        matrix: The matrix of the transformation to apply.
    Returns:
        An n by 3 numpy array of transformed points.
    """
    # Needed format: [[X0,..Xn],[Y0,..Yn],[Z0,..Zn]].
    # So let's transpose the point matrix.
    points = points.T

    # Add 1s row: [[X0..,Xn],[Y0..,Yn],[Z0..,Zn],[1,..1]]
    points = np.append(points, np.ones((1, points.shape[1])), axis=0)

    # Point transformation (depends on the given matrix)
    points = np.dot(matrix, points)

    # Get all but the last row in array form.
    points = np.asarray(points[0:3].T).astype(np.float32)

    return points


def get_extrinsic_matrix(transform):
    """Converts a Transform from the camera coordinate space to the
    Unreal coordinate space.
    The camera space is defined as:
        +x to right, +y to down, +z into the screen.
    The unreal coordinate space is defined as:
        +x into the screen, +y to right, +z to up.
    Args:
        transform (:py:class:`~pylot.utils.Transform`): The transform to
            convert to Unreal coordinate space.
    Returns:
        :py:class:`~pylot.utils.Transform`: The given transform after
            transforming to the Unreal coordinate space.
    """

    to_unreal_transform = np.array(
        # [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
        [[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
    )
    return np.dot(transform, to_unreal_transform)


def get_intrinsic_matrix(width: int, height: int, fov: float):
    """Creates the intrinsic matrix for a camera with the given
    parameters.
    Args:
        width (int): The width of the image returned by the camera.
        height (int): The height of the image returned by the camera.
        fov (float): The field-of-view of the camera.
    Returns:
        :py:class:`numpy.ndarray`: A 3x3 intrinsic matrix of the camera.
    """

    k = np.identity(3)
    # We use width - 1 and height - 1 to find the center column and row
    # of the image, because the images are indexed from 0.

    # Center column of the image.
    k[0, 2] = (width - 1) / 2.0
    # Center row of the image.
    k[1, 2] = (height - 1) / 2.0
    # Focal length.
    k[0, 0] = k[1, 1] = (width - 1) / (2.0 * np.tan(fov * np.pi / 360.0))
    return k


def location_to_camera_view(
    location: np.array, intrinsic_matrix, inv_extrinsic_matrix
):
    """Converts the given 3D vector to the view of the camera using
    the extrinsic and the intrinsic matrix.
    Args:
        location = [[x, y, z]]
        extrinsic_matrix: The extrinsic matrix of the camera.
    Returns:
        :py:class:`.Vector3D`: An instance with the coordinates converted
        to the camera view.
    """
    if len(location) == 0:
        return np.array([])
    position_vector = np.hstack((location, np.ones((location.shape[0], 1))))
    position_vector = position_vector.T

    # Transform the points to the camera in 3D.
    transformed_3D_pos = np.dot(inv_extrinsic_matrix, position_vector)

    # Transform the points to 2D.
    position_2D = np.dot(intrinsic_matrix, transformed_3D_pos[:3])

    # Normalize the 2D points.
    if not position_2D[2].all():
        print("could not inverse to camera image")
        return np.array([])

    location_2D = np.array(
        [
            (position_2D[0] / position_2D[2]),
            (position_2D[1] / position_2D[2]),
            (position_2D[2]),
        ]
    )
    return location_2D


def local_points_to_camera_view(location: np.array, intrinsic_matrix):
    """Converts the given 3D vector to the view of the camera using
    the extrinsic and the intrinsic matrix.
    Args:
        location = [[x, y, z]]
        extrinsic_matrix: The extrinsic matrix of the camera.
    Returns:
        :py:class:`.Vector3D`: An instance with the coordinates converted
        to the camera view.
    """
    if len(location) == 0:
        return np.array([])

    # Transform the points to 2D.
    position_2D = np.dot(intrinsic_matrix, location.T)

    # Normalize the 2D points.
    if not position_2D[2].all():
        print("could not inverse to camera image")
        return np.array([])

    location_2D = np.array(
        [
            (position_2D[0] / position_2D[2]),
            (position_2D[1] / position_2D[2]),
            (position_2D[2]),
        ]
    )
    return location_2D


LABELS = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]
