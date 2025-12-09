import threading
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from numpy import typing as npt
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header

from anytraverse_ros.oakd_vio_zmq.subscribe import Subscriber


R_CAM_ROBOT = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])


class OakdVIO_Node(Node):
    """
    Publishes:
    - Compressed RGB image
    - PointCloud2
    - CameraInfo
    - TF transform
    """

    def __init__(self) -> None:
        super().__init__(node_name="oakd_vio_node", namespace="/oakd")

        fast_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # -------------------------
        # Publishers
        # -------------------------
        self._rgb_pub = self.create_publisher(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            fast_qos,
        )
        self._caminfo_pub = self.create_publisher(
            CameraInfo,
            "/camera/color/camera_info",
            fast_qos,
        )
        self._pcl_pub = self.create_publisher(
            PointCloud2,
            "/camera/points",
            fast_qos,
        )
        self._tf_pub = self.create_publisher(
            TransformStamped,
            "/tf",
            fast_qos,
        )

        self._bridge = CvBridge()
        self._has_data = False
        self._lock = threading.Lock()

        # -------------------------
        # ZMQ thread
        # -------------------------
        self._oakd_sub = Subscriber(stream_name="oakd")
        self._oakd_sub.connect()
        self._thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._thread.start()

        # Pre-create PointCloud2 message
        self._pcl_msg = PointCloud2(
            height=1,
            width=640 * 400,
            fields=[
                PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate("xyz")
            ],
            is_bigendian=False,
            point_step=12,
            row_step=12 * 640 * 400,
            is_dense=True,
        )

        # Camera intrinsic defaults (replace with real OAK-D values)
        self.fx = 650.0
        self.fy = 650.0
        self.cx = 320.0
        self.cy = 200.0
        self.dist = [0.0, 0.0, 0.0, 0.0, 0.0]

    def _unsubscribe_sensor(self) -> None:
        self._thread.join()
        self._oakd_sub.close()

    # ============================================================
    # SENSOR LOOP
    # ============================================================
    def _sensor_loop(self) -> None:
        while True:
            sensor_data = self._oakd_sub.get_next()

            if sensor_data is not None:
                self._has_data = True

                # Convert to robot frame
                points = sensor_data.pointcloud * 1e-3  # mm to m
                points = np.stack([points[:, 2], -points[:, 0], -points[:, 1]]).T

                self._publish_data(
                    rgb=sensor_data.rgb,
                    points=points,
                    # points=sensor_data.pointcloud * 1e-3,
                    # points=(sensor_data.pointcloud @ R_CAM_ROBOT.T).T * 1e-3,  # mm to m
                    T=sensor_data.transform,
                )
            else:
                self._has_data = False

    # ============================================================
    # PUBLISHING FUNCTION
    # ============================================================
    def _publish_data(
        self,
        rgb: npt.NDArray[np.uint8],
        points: npt.NDArray[np.float32],
        T: npt.NDArray[np.float32],
    ) -> None:
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id="camera_link")

        # -------------------------
        # COMPRESSED RGB IMAGE
        # -------------------------
        img_msg = self._bridge.cv2_to_compressed_imgmsg(rgb, dst_format="jpg")
        img_msg.header = header
        self._rgb_pub.publish(img_msg)

        # -------------------------
        # CAMERA INFO
        # -------------------------
        height, width, _ = rgb.shape

        cam_info = CameraInfo()
        cam_info.header = header
        cam_info.width = width
        cam_info.height = height
        cam_info.distortion_model = "rational_polynomial"
        cam_info.d = self.dist

        cam_info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.p = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        self._caminfo_pub.publish(cam_info)

        # -------------------------
        # POINT CLOUD
        # -------------------------
        self._pcl_msg.header = header
        self._pcl_msg.data = points.tobytes()  # ZERO COPY
        self._pcl_pub.publish(self._pcl_msg)

        # -------------------------
        # TF TRANSFORM
        # -------------------------
        tf_msg = TransformStamped()
        tf_msg.header = Header(stamp=now, frame_id="world")
        tf_msg.child_frame_id = "camera_link"

        Rmat = T[:3, :3]
        tvec = T[:3, 3]
        quat = R.from_matrix(Rmat).as_quat()

        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[1])
        tf_msg.transform.translation.z = float(tvec[2])

        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        self._tf_pub.publish(tf_msg)


# ============================================================
# MAIN
# ============================================================
def main():
    rclpy.init()
    node = OakdVIO_Node()
    rclpy.spin(node)
    node.destroy_node()
    node._unsubscribe_sensor()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
