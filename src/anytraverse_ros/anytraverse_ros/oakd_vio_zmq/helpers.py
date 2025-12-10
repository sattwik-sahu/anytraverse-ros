import numpy as np
from scipy.spatial.transform import Rotation as R

from anytraverse_ros.oakd_vio_zmq._typing import TransformationMatrix


def get_zmq_uri(name: str) -> str:
    return f"ipc:///tmp/{name}"


def create_transformation_matrix(
    qw: float, qx: float, qy: float, qz: float, tx: float, ty: float, tz: float
) -> TransformationMatrix:
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = R.from_quat(
        [qw, qx, qy, qz], scalar_first=True
    ).as_matrix()
    transform_matrix[:3, 3] = np.array([tx, ty, tz])
    return transform_matrix
