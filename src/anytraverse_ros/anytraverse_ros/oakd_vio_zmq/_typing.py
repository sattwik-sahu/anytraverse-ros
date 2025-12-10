from dataclasses import dataclass
from typing import TypedDict

import numpy as np
from numpy import typing as npt

Float = np.float64
"""A 64-bit floating point number"""

Image = npt.NDArray[np.uint8]
"""An image where each pixel is represented by 8-bit colors"""

Pointcloud = npt.NDArray[Float]
"""A pointcloud containing points as 64-bit floating point numbers"""

TransformationMatrix = npt.NDArray[Float]
"""The transformation matrix from current pose to starting pose"""

DepthMap = npt.NDArray[Float]


class NDArrayMetadata(TypedDict):
    """Metadata for a numpy NDArray"""

    shape: tuple[int, ...]
    dtype: str


class RGBD_VIO_Metadata(TypedDict):
    """Metadata for an RGBD VIO message"""

    rgb: NDArrayMetadata
    depth: NDArrayMetadata
    pointcloud: NDArrayMetadata
    transform: NDArrayMetadata


@dataclass
class RGBD_VIO_Message:
    rgb: Image
    depth: DepthMap
    pointcloud: Pointcloud
    transform: TransformationMatrix
