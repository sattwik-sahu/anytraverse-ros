import math
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from numpy import typing as npt
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header
import cv2
from tf2_ros import TransformBroadcaster

from oakd_ros.subscribe import Subscriber


class OakdVIO_Node(Node):
    """
    Publishes:
    - Compressed RGB image
    - Compressed Depth image (PNG 16-bit)
    - CameraInfo
    - TF
    """

    def __init__(self) -> None:
        super().__init__(node_name="oakd_vio_node", namespace="/oakd")

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # -------------------------
        # Publishers
        # -------------------------
        self._rgb_pub = self.create_publisher(
            CompressedImage, "/camera/color/image_raw/compressed", qos_profile
        )
        self._caminfo_pub = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", qos_profile
        )

        # COMPRESSED DEPTH - fixed topic name
        self._depth_pub = self.create_publisher(
            CompressedImage, "/camera/depth/image_raw/compressedDepth", qos_profile
        )

        # self._tf_pub = self.create_publisher(TransformStamped, "/tf", fast_qos)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._running = True

        # Camera intrinsics (OAK-D Pro W)
        self.fx = 640 * 0.5 / math.tan(math.radians(95.0 / 2))
        self.fy = 400 * 0.5 / math.tan(math.radians(72.0 / 2))
        self.cx = 320.0
        self.cy = 200.0
        self.dist = [0.0] * 5

        # ZMQ reading thread
        self._oakd_sub = Subscriber(stream_name="oakd")
        self._oakd_sub.connect()
        self._thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"OAK-D Pro W Node initialized. fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx}, cy={self.cy}"
        )

    def _unsubscribe_sensor(self):
        self._running = False
        self._thread.join()
        self._oakd_sub.close()

    # ============================================================
    # SENSOR LOOP
    # ============================================================
    def _sensor_loop(self):
        while self._running:
            try:
                sensor_data = self._oakd_sub.get_next()
                if sensor_data is None:
                    import time

                    time.sleep(0.001)
                    continue

                self._publish_data(
                    rgb=sensor_data.rgb,
                    depth_mm=sensor_data.depth,
                    T=sensor_data.transform.astype(np.float32),
                )

            except Exception as e:
                self.get_logger().error(f"Sensor loop error: {e}")

    # ============================================================
    # PUBLISH DATA
    # ============================================================
    def _publish_data(
        self,
        rgb: npt.NDArray[np.uint8],
        depth_mm: npt.NDArray[np.uint16],
        T: npt.NDArray[np.float32],
    ):
        now_msg = self.get_clock().now().to_msg()
        header = Header(stamp=now_msg, frame_id="camera_optical_link")

        # -------------------------
        # RGB COMPRESSED
        # -------------------------
        try:
            rgb_msg = self._bridge.cv2_to_compressed_imgmsg(rgb, dst_format="jpg")
            rgb_msg.header = header
            self._rgb_pub.publish(rgb_msg)
        except Exception as e:
            self.get_logger().error(f"RGB publish error: {e}")

        # -------------------------
        # CAMERA INFO
        # -------------------------
        try:
            h, w = rgb.shape[:2]
            cam_info = CameraInfo()
            cam_info.header = header
            cam_info.width = w
            cam_info.height = h
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
            self.get_logger().error(f"Camera info publish error: {e}")

        # -------------------------
        # COMPRESSED DEPTH (PNG 16-bit)
        # -------------------------
        try:
            # Ensure depth is contiguous and correct dtype
            if not depth_mm.flags["C_CONTIGUOUS"]:
                depth_mm = np.ascontiguousarray(depth_mm)

            # Ensure it's uint16
            if depth_mm.dtype != np.uint16:
                self.get_logger().warning(
                    f"Depth dtype is {depth_mm.dtype}, converting to uint16"
                )
                depth_mm = depth_mm.astype(np.uint16)

            # Encode depth as 16-bit PNG
            success, png_buffer = cv2.imencode(
                ".png", depth_mm, [cv2.IMWRITE_PNG_COMPRESSION, 3]
            )

            if not success:
                self.get_logger().error("Failed to encode depth image as PNG")
                return

            # Create CompressedImage message
            depth_msg = CompressedImage()
            depth_msg.header = header
            depth_msg.format = "16UC1; compressedDepth png"
            depth_msg.data = png_buffer.tobytes()

            self._depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(
                f"Depth publish error: {e}", throttle_duration_sec=1.0
            )

        # -------------------------
        # TF
        # -------------------------
        try:
            tf = TransformStamped()
            tf.header = Header(stamp=now_msg, frame_id="odom")  # Or 'map'/'odom'
            tf.child_frame_id = "camera_link"

            Rmat = T[:3, :3]
            t = T[:3, 3]
            quat = R.from_matrix(Rmat).as_quat()

            tf.transform.translation.x = float(t[0])
            tf.transform.translation.y = float(t[1])
            tf.transform.translation.z = float(t[2])

            tf.transform.rotation.x = float(quat[0])
            tf.transform.rotation.y = float(quat[1])
            tf.transform.rotation.z = float(quat[2])
            tf.transform.rotation.w = float(quat[3])

            self._tf_broadcaster.sendTransform(tf)
        except Exception as e:
            self.get_logger().error(f"TF publish error: {e}")


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
