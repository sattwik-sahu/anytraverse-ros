from datetime import timedelta

import depthai
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from numpy import typing as npt
from rclpy.time import Time as ROSTime
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo


def depthai_timestamp_to_ros(timestamp: timedelta) -> Time:
    total_ns: int = int(timestamp.total_seconds() * 1e9)
    return ROSTime(
        seconds=total_ns // int(1e9),
        nanoseconds=total_ns % int(1e9),
    ).to_msg()


def build_camera_info(
    calib: depthai.CalibrationHandler,
    socket: depthai.CameraBoardSocket,
    width: int,
    height: int,
    frame_id: str,
) -> CameraInfo:
    """
    Build a CameraInfo message from DepthAI calibration.

    Args:
        calib (depthai.CalibrationHandler): DepthAI calibration handler.
        socket (depthai.CameraBoardSocket): Camera socket.
        width (int): Image width.
        height (int): Image height.
        frame_id (str): Frame ID for ROS message.

    Returns:
        CameraInfo:
        Populated CameraInfo message.
    """
    msg: CameraInfo = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height

    K: npt.NDArray[np.float64] = np.array(
        calib.getCameraIntrinsics(socket, width, height)
    )
    D: npt.NDArray[np.float64] = np.array(calib.getDistortionCoefficients(socket))

    msg.k = K.flatten().tolist()

    if len(D) <= 5:
        msg.distortion_model = "plumb_bob"
        msg.d = D[:5].tolist()
    else:
        msg.distortion_model = "rational_polynomial"
        msg.d = D.tolist()

    msg.r = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    fx: float = float(K[0, 0])
    fy: float = float(K[1, 1])
    cx: float = float(K[0, 2])
    cy: float = float(K[1, 2])

    msg.p = [
        fx,
        0.0,
        cx,
        0.0,
        0.0,
        fy,
        cy,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    return msg


def set_stereo_preset(stereo: depthai.node.StereoDepth) -> None:
    """
    Apply a density preset compatible across DepthAI v3 builds.

    Args:
        stereo (depthai.Node.StereoDepth): StereoDepth node instance.
    """
    stereo.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.ROBOTICS)
    stereo.setLeftRightCheck(True)


def transformation_matrix_to_transform_stamped_msg(
    tf_matrix: npt.NDArray[np.float64],
) -> TransformStamped:
    msg = TransformStamped()

    msg.transform.translation.x = tf_matrix[0, 3]
    msg.transform.translation.y = tf_matrix[1, 3]
    msg.transform.translation.z = tf_matrix[2, 3]

    qx, qy, qz, qw = Rotation.from_matrix(matrix=tf_matrix[:3, :3]).as_quat().flatten()
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw

    return msg


def transform_stamped_msg_to_transformation_matrix(
    msg: TransformStamped,
) -> npt.NDArray[np.float64]:
    tf_matrix = np.eye(4, dtype=np.float64)

    tf_matrix[:3, :3] = Rotation.from_quat(
        [
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ]
    ).as_matrix()

    tf_matrix[0, 3] = msg.transform.translation.x
    tf_matrix[1, 3] = msg.transform.translation.y
    tf_matrix[2, 3] = msg.transform.translation.z

    return tf_matrix


def pose_to_transformation_matrix(
    q: list[float], t: list[float]
) -> npt.NDArray[np.float64]:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quat=q).as_matrix()
    matrix[:3, 3] = t
    return matrix
