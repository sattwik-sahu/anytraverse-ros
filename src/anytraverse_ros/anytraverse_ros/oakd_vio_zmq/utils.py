from anytraverse_ros.oakd_vio_zmq._typing import (
    RGBD_VIO_Metadata,
    RGBD_VIO_Message,
    NDArrayMetadata,
    Image,
    Pointcloud,
    TransformationMatrix,
)
import numpy as np
from numpy import typing as npt
import ormsgpack


def create_ndarray_metadata(array: npt.NDArray) -> NDArrayMetadata:
    """
    Creates metadata for a numpy ndarray.

    Args:
        array (npt.NDArray): Input numpy array to extract metadata from.

    Returns:
        NDArrayMetadata: Metadata object containing the shape and dtype of the input array.
    """
    return NDArrayMetadata(shape=array.shape, dtype=str(array.dtype))


def create_metadata(
    rgb: Image, pointcloud: Pointcloud, transform: TransformationMatrix
) -> RGBD_VIO_Metadata:
    """
    Creates an RGBD VIO message metadata.

    Args:
        rgb (Image): The RGB image. Shape: `(H, W, 3)`
        pointcloud (Pointcloud): The pointcloud aligned to the image pixels.
            Shape: `(N, 3)`
        transform (TransformationMatrix): The camera to world transformation
            in matrix form. Shape: `(4, 4)`

    Returns:
        RGBD_VIO_Metadata: The RGBD VIO metadata dict constructed from the
            above arguments.

    ## Notes:
        - `H` = Height of the image
        - `W` = Width of the image
        - `N` = Number of points in the pointcloud
    """
    return RGBD_VIO_Metadata(
        rgb=create_ndarray_metadata(array=rgb),
        pointcloud=create_ndarray_metadata(array=pointcloud),
        transform=create_ndarray_metadata(array=transform),
    )


def parse_ndarray_buffer(frame: bytes, metadata: NDArrayMetadata) -> npt.NDArray:
    return np.frombuffer(frame, dtype=metadata["dtype"]).reshape(metadata["shape"])


def parse_message(
    metadata_bytes: bytes,
    rgb_frame: bytes,
    pointcloud_frame: bytes,
    transform_frame: bytes,
) -> RGBD_VIO_Message:
    metadata: RGBD_VIO_Metadata = ormsgpack.unpackb(metadata_bytes)
    return RGBD_VIO_Message(
        rgb=parse_ndarray_buffer(frame=rgb_frame, metadata=metadata["rgb"]),
        pointcloud=parse_ndarray_buffer(
            frame=pointcloud_frame, metadata=metadata["pointcloud"]
        ),
        transform=parse_ndarray_buffer(
            frame=transform_frame, metadata=metadata["transform"]
        ),
    )
