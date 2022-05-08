import numpy as np
import tensorflow as tf
from waymo_open_dataset import dataset_pb2
from waymo_open_dataset import dataset_pb2 as open_dataset
from waymo_open_dataset.utils import range_image_utils
from waymo_open_dataset.utils import transform_utils


def decode_camera_calibration(frame):
    camera_calibrations = []

    for camera_calibration in frame.context.camera_calibrations:
        camera_name = dataset_pb2.CameraName.Name.Name(camera_calibration.name)
        extrinsic = np.array(camera_calibration.extrinsic.transform).reshape(4, 4)
        cam_intrinsic = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        cam_intrinsic[0, 0] = camera_calibration.intrinsic[0]
        cam_intrinsic[1, 1] = camera_calibration.intrinsic[1]
        cam_intrinsic[0, 2] = camera_calibration.intrinsic[2]
        cam_intrinsic[1, 2] = camera_calibration.intrinsic[3]
        cam_intrinsic[2, 2] = 1
        width = camera_calibration.width
        height = camera_calibration.height

        # Swap the axes around
        axes_transformation = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])

        vehicle_to_image = np.matmul(
            cam_intrinsic, np.matmul(axes_transformation, np.linalg.inv(extrinsic))
        )

        camera_info = {
            "cam_to_vehicle": extrinsic,
            "cam_intrinsic": cam_intrinsic,
            "width": width,
            "height": height,
            "vehicle_to_image": vehicle_to_image,
        }

        camera_calibration = [camera_name, camera_info]
        camera_calibrations.append(camera_calibration)

    return camera_calibrations


def get_decoded_point_cloud(frame):
    range_images = {}
    camera_projections = {}
    range_image_top_pose = None

    for laser in frame.lasers:

        if len(laser.ri_return1.range_image_compressed) > 0:
            range_image_str_tensor = tf.io.decode_compressed(
                laser.ri_return1.range_image_compressed, "ZLIB"
            )
            ri = open_dataset.MatrixFloat()
            ri.ParseFromString(bytearray(range_image_str_tensor.numpy()))
            range_images[laser.name] = [ri]

            if laser.name == open_dataset.LaserName.TOP:
                range_image_top_pose_str_tensor = tf.io.decode_compressed(
                    laser.ri_return1.range_image_pose_compressed, "ZLIB"
                )
                range_image_top_pose = open_dataset.MatrixFloat()
                range_image_top_pose.ParseFromString(
                    bytearray(range_image_top_pose_str_tensor.numpy())
                )

            camera_projection_str_tensor = tf.io.decode_compressed(
                laser.ri_return1.camera_projection_compressed, "ZLIB"
            )
            cp = open_dataset.MatrixInt32()
            cp.ParseFromString(bytearray(camera_projection_str_tensor.numpy()))
            camera_projections[laser.name] = [cp]

        if len(laser.ri_return2.range_image_compressed) > 0:
            range_image_str_tensor = tf.io.decode_compressed(
                laser.ri_return2.range_image_compressed, "ZLIB"
            )
            ri = open_dataset.MatrixFloat()
            ri.ParseFromString(bytearray(range_image_str_tensor.numpy()))
            range_images[laser.name].append(ri)

            camera_projection_str_tensor = tf.io.decode_compressed(
                laser.ri_return2.camera_projection_compressed, "ZLIB"
            )
            cp = open_dataset.MatrixInt32()
            cp.ParseFromString(bytearray(camera_projection_str_tensor.numpy()))
            camera_projections[laser.name].append(cp)

    calibrations = sorted(frame.context.laser_calibrations, key=lambda c: c.name)
    points = []

    frame_pose = tf.convert_to_tensor(np.reshape(np.array(frame.pose.transform), [4, 4]))
    # [H, W, 6]
    range_image_top_pose_tensor = tf.reshape(
        tf.convert_to_tensor(range_image_top_pose.data), range_image_top_pose.shape.dims
    )
    # [H, W, 3, 3]
    range_image_top_pose_tensor_rotation = transform_utils.get_rotation_matrix(
        range_image_top_pose_tensor[..., 0],
        range_image_top_pose_tensor[..., 1],
        range_image_top_pose_tensor[..., 2],
    )
    range_image_top_pose_tensor_translation = range_image_top_pose_tensor[..., 3:]
    range_image_top_pose_tensor = transform_utils.get_transform(
        range_image_top_pose_tensor_rotation, range_image_top_pose_tensor_translation
    )
    for c in calibrations:
        range_image = range_images[c.name][0]

        if len(c.beam_inclinations) == 0:
            beam_inclinations = range_image_utils.compute_inclination(
                tf.constant([c.beam_inclination_min, c.beam_inclination_max]),
                height=range_image.shape.dims[0],
            )
        else:
            beam_inclinations = tf.constant(c.beam_inclinations)

        beam_inclinations = tf.reverse(beam_inclinations, axis=[-1])
        extrinsic = np.reshape(np.array(c.extrinsic.transform), [4, 4])

        range_image_tensor = tf.reshape(
            tf.convert_to_tensor(range_image.data), range_image.shape.dims
        )
        pixel_pose_local = None
        frame_pose_local = None

        if c.name == open_dataset.LaserName.TOP:
            pixel_pose_local = range_image_top_pose_tensor
            pixel_pose_local = tf.expand_dims(pixel_pose_local, axis=0)
            frame_pose_local = tf.expand_dims(frame_pose, axis=0)
        range_image_mask = range_image_tensor[..., 0] > 0
        range_image_cartesian = range_image_utils.extract_point_cloud_from_range_image(
            tf.expand_dims(range_image_tensor[..., 0], axis=0),
            tf.expand_dims(extrinsic, axis=0),
            tf.expand_dims(tf.convert_to_tensor(beam_inclinations), axis=0),
            pixel_pose=pixel_pose_local,
            frame_pose=frame_pose_local,
        )

        range_image_cartesian = tf.squeeze(range_image_cartesian, axis=0)
        points_tensor = tf.gather_nd(range_image_cartesian, tf.where(range_image_mask))

        laser = [dataset_pb2.LaserName.Name.Name(c.name), points_tensor.numpy()]
        points.append(laser)

    return points


def decode_static_tf(frame):
    lidars_transforms = {}
    cameras_transforms = {}

    for laser_calibration in frame.context.laser_calibrations:
        laser_name = dataset_pb2.LaserName.Name.Name(laser_calibration.name)
        extrinsic = np.array(laser_calibration.extrinsic.transform).reshape((4, 4))
        lidars_transforms[f"{laser_name}_LASER_EXTRINSIC"] = extrinsic
    for camera_calibration in frame.context.camera_calibrations:
        camera_name = dataset_pb2.CameraName.Name.Name(camera_calibration.name)
        extrinsic = (np.array(camera_calibration.extrinsic.transform)).reshape((4, 4))
        cameras_transforms[f"{camera_name}_CAM_EXTRINSIC"] = extrinsic

    return lidars_transforms, cameras_transforms


def decode_vehicle_pose(frame, frame_id, trans_t_zero_inverse):
    mat_trans_frame = np.array(
        [
            (
                frame.pose.transform[0],
                frame.pose.transform[1],
                frame.pose.transform[2],
                frame.pose.transform[3],
            ),
            (
                frame.pose.transform[4],
                frame.pose.transform[5],
                frame.pose.transform[6],
                frame.pose.transform[7],
            ),
            (
                frame.pose.transform[8],
                frame.pose.transform[9],
                frame.pose.transform[10],
                frame.pose.transform[11],
            ),
            (0, 0, 0, 1),
        ]
    )
    # Define T inverse of odom:
    if frame_id == 0:
        trans_t_zero_inverse = np.linalg.inv(mat_trans_frame)

    mat_trans_frame = np.dot(trans_t_zero_inverse, mat_trans_frame)
    return mat_trans_frame, trans_t_zero_inverse


def extract_dataset_from_tfrecord(path):
    gpus = tf.config.list_physical_devices("GPU")
    if gpus:
        try:
            # Currently, memory growth needs to be the same across GPUs
            for gpu in gpus:
                tf.config.experimental.set_memory_growth(gpu, True)
            logical_gpus = tf.config.list_logical_devices("GPU")
            print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
        except RuntimeError as e:
            # Memory growth must be set before GPUs have been initialized
            print(e)

    dataset = tf.data.TFRecordDataset(path, compression_type="")
    extracted_dataset = []
    trans_t_zero_inverse = np.array([0])

    lidars_transforms = {}
    cameras_transforms = {}

    for frame_id, data in enumerate(dataset):
        data_dict = {}
        frame = open_dataset.Frame()
        frame.ParseFromString(bytearray(data.numpy()))

        if frame_id == 1:
            lidars_transforms, cameras_transforms = decode_static_tf(frame)

        for image in frame.images:
            cam_name_str = dataset_pb2.CameraName.Name.Name(image.name)
            data_dict[f"{cam_name_str}_IMAGE"] = tf.io.decode_jpeg(image.image).numpy()

        for laser_name, point in get_decoded_point_cloud(frame):
            data_dict[f"{laser_name}_LASER"] = point

        for camera_name, camera_calibration in decode_camera_calibration(frame):
            data_dict[f"{camera_name}_CAM_INFO"] = camera_calibration

        data_dict["VEHICLE_POSE"], trans_t_zero_inverse = decode_vehicle_pose(
            frame, frame_id, trans_t_zero_inverse
        )

        data_dict["TIMESTAMP_MICRO"] = frame.timestamp_micros
        data_dict["FRAME_CONTEXT_NAME"] = frame.context.name
        data_dict["GT_OBJECTS"] = frame.laser_labels

        extracted_dataset.append(data_dict)

    return extracted_dataset, lidars_transforms, cameras_transforms
