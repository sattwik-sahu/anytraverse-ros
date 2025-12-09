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
import math

from anytraverse_ros.oakd_vio_zmq.subscribe import Subscriber


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
            CompressedImage, "/camera/color/image_raw/compressed", fast_qos
        )
        self._caminfo_pub = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", fast_qos
        )
        self._pcl_pub = self.create_publisher(PointCloud2, "/camera/points", fast_qos)
        self._tf_pub = self.create_publisher(TransformStamped, "/tf", fast_qos)

        self._bridge = CvBridge()
        self._has_data = False
        self._lock = threading.Lock()
        self._running = True

        # OAK-D Pro W camera intrinsics (IMX378 at 640x400)
        # Calculated from HFOV=95°, VFOV=72° using: f = w * 0.5 / tan(FOV/2)
        self.fx = 640 * 0.5 / math.tan(math.radians(95.0 / 2))  # ≈434 pixels
        self.fy = 400 * 0.5 / math.tan(math.radians(72.0 / 2))  # ≈434 pixels
        self.cx = 320.0  # width / 2
        self.cy = 200.0  # height / 2
        self.dist = [0.0, 0.0, 0.0, 0.0, 0.0]

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

        self.get_logger().info(
            f"OAK-D Pro W Node initialized. Camera intrinsics: "
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

    def _unsubscribe_sensor(self) -> None:
        self._running = False
        self._thread.join()
        self._oakd_sub.close()

    # ============================================================
    # SENSOR LOOP
    # ============================================================
    def _sensor_loop(self) -> None:
        while self._running:
            try:
                sensor_data = self._oakd_sub.get_next()

                if sensor_data is not None:
                    with self._lock:
                        self._has_data = True

                    # Convert to robot frame
                    points = sensor_data.pointcloud * 1e-3  # mm to m
                    points = np.stack([points[:, 2], -points[:, 0], -points[:, 1]]).T

                    # Validate pointcloud data
                    if points.size > 0 and np.isfinite(points).all():
                        self._publish_data(
                            rgb=sensor_data.rgb,
                            points=points,
                            T=sensor_data.transform,
                        )
                    else:
                        self.get_logger().warn("Received invalid pointcloud data")
                else:
                    with self._lock:
                        self._has_data = False
                    # Small sleep to prevent busy-waiting when no data available
                    # This helps with jerkiness
                    import time

                    time.sleep(0.001)  # 1ms

            except Exception as e:
                self.get_logger().error(f"Error in sensor loop: {e}")
                import time

                time.sleep(0.01)  # Brief pause on error

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
        try:
            img_msg = self._bridge.cv2_to_compressed_imgmsg(rgb, dst_format="jpg")
            img_msg.header = header
            self._rgb_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing RGB image: {e}")

        # -------------------------
        # CAMERA INFO
        # -------------------------
        try:
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
        except Exception as e:
            self.get_logger().error(f"Error publishing camera info: {e}")

        # -------------------------
        # POINT CLOUD
        # -------------------------
        try:
            # Ensure points are contiguous in memory and correct dtype
            points_contiguous = np.ascontiguousarray(points, dtype=np.float32)

            self._pcl_msg.header = header
            self._pcl_msg.width = points_contiguous.shape[0]
            self._pcl_msg.row_step = 12 * points_contiguous.shape[0]
            self._pcl_msg.data = points_contiguous.tobytes()
            self._pcl_pub.publish(self._pcl_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing pointcloud: {e}")

        # -------------------------
        # TF TRANSFORM
        # -------------------------
        try:
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
        except Exception as e:
            self.get_logger().error(f"Error publishing TF: {e}")


# ============================================================
# MAIN
# ============================================================
def main():
    rclpy.init()
    node = OakdVIO_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._unsubscribe_sensor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
