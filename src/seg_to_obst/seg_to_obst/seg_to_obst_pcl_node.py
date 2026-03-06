import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField, Image
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class SegmentationToObstaclePointCloudNode(Node):
    """The obstacle cloud projector node"""

    _CAMERA_OPTICAL_FRAME_ID_PARAM: str = "camera_optical_frame_id"
    _TRAVERSABILITY_THRESHOLD_PARAM: str = "traversability_threshold"

    def __init__(self) -> None:
        super().__init__(node_name="obstacle_pcl_node")

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Declare parameters
        self.declare_parameter(name=self._TRAVERSABILITY_THRESHOLD_PARAM, value=0.5)
        self.declare_parameter(
            name=self._CAMERA_OPTICAL_FRAME_ID_PARAM, value="camera_rgb_optical_frame"
        )
        self._trav_thresh: float = (
            self.get_parameter(self._TRAVERSABILITY_THRESHOLD_PARAM)
            .get_parameter_value()
            .double_value
        )
        self._optical_frame_id: str = (
            self.get_parameter(name=self._CAMERA_OPTICAL_FRAME_ID_PARAM)
            .get_parameter_value()
            .string_value
        )

        # Create subscribers
        self._depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_raw"
        )
        self._trav_sub = message_filters.Subscriber(self, Image, "/trav_map")
        self._camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/camera/depth/camera_info"
        )

        # Sync the topics
        self._topic_sync = message_filters.ApproximateTimeSynchronizer(
            [self._depth_sub, self._trav_sub, self._camera_info_sub],
            queue_size=10,
            slop=0.1,
        )

        # What to do when topics synced?
        self._topic_sync.registerCallback(self._sync_callback)

        # CV bridge for reading image topics
        self._bridge = CvBridge()

        # Obstacle pointcloud publisher
        self._obst_pcl_pub = self.create_publisher(
            PointCloud2, "/obstacle_points", qos_profile=qos_profile
        )

    def _get_camera_intrinsics(
        self, camera_info_msg: CameraInfo
    ) -> tuple[float, float, float, float]:
        fx, cx, fy, cy = [camera_info_msg.p[j] for j in (0, 2, 5, 6)]
        return fx, cx, fy, cy

    def _sync_callback(
        self,
        depth_msg: Image,
        trav_msg: Image,
        camera_info_msg: CameraInfo,
    ) -> None:
        try:
            # ==== 1. Decode Depth Image correctly ====
            # CvBridge knows how to handle the raw bit-depth (16-bit or 32-bit)
            # 'passthrough' keeps the original encoding (usually 16UC1 for mm)
            depth_img = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )

            # Validate depth image
            if depth_img is None:
                self.get_logger().warn("Failed to decode depth image")
                return

            # Convert depth from mm (uint16) to m (float32)
            # If your camera already sends float32 (meters), this handles both.
            if depth_img.dtype == np.uint16:
                depth_img_m = depth_img.astype(np.float32) * 1e-3
            else:
                depth_img_m = depth_img.astype(np.float32)

            # ==== 2. Read the traversability map ====
            # Using standard Image conversion now
            trav_map_uint8 = self._bridge.imgmsg_to_cv2(
                trav_msg, desired_encoding="passthrough"
            )

            # Handle Grayscale vs Color shapes
            if len(trav_map_uint8.shape) == 3:
                # If RGB, AnyTraverse puts data in all channels, so taking [0] is fine
                trav_map_uint8 = trav_map_uint8[:, :, 0]

            # Normalize to 0.0 - 1.0 range
            trav_map = trav_map_uint8.astype(np.float32) / 255.0

        except Exception as e:
            self.get_logger().error(f"Error processing images: {e}")
            return

        # ==== Vectorized pixel reprojection ====
        # Get the camera intrinsic parameters
        fx, cx, fy, cy = self._get_camera_intrinsics(camera_info_msg=camera_info_msg)

        # Create grid of pixel coordinates (u, v)
        # NOTE (u, v) <- (grid(w), grid(h))
        h, w = depth_img_m.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

        crop_height = int(h * 0.05)  # 5% of height
        trav_map[-crop_height:, :] = 1.0

        # Mask for obstacle points (pixel coords)
        obst_mask = (
            (depth_img_m > 0.25) & (depth_img_m < 10.0) & (trav_map < self._trav_thresh)
        )

        # Get the obstacle pixel coordinates and corresponding depth
        z_obst = depth_img_m[obst_mask]
        u_obst = u_grid[obst_mask]
        v_obst = v_grid[obst_mask]

        # Unproject obstacle points
        x_obst = (u_obst - cx) * z_obst / fx
        y_obst = (v_obst - cy) * z_obst / fy

        # ==== Construct the pointcloud message and publish ====
        obst_pcl_msg = PointCloud2()

        # Metadata
        obst_pcl_msg.header = depth_msg.header
        obst_pcl_msg.header.frame_id = self.get_parameter(
            self._CAMERA_OPTICAL_FRAME_ID_PARAM
        ).value
        obst_pcl_msg.height = 1
        obst_pcl_msg.width = x_obst.shape[0]

        # Fields
        obst_pcl_msg.fields = [
            PointField(name=axis, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, axis in enumerate("xyz")
        ]

        # More metadata
        obst_pcl_msg.is_bigendian = False
        obst_pcl_msg.point_step = 12  # 3 * float32 (4 bytes)
        obst_pcl_msg.row_step = obst_pcl_msg.point_step * x_obst.shape[0]
        obst_pcl_msg.is_dense = True

        # Stack x, y, z coords, convert to bytes
        obst_points = np.column_stack((x_obst, y_obst, z_obst)).astype(dtype=np.float32)
        obst_points = obst_points[::2]
        obst_pcl_msg.data = obst_points.tobytes()

        # Publish the obstacle pointcloud!
        self._obst_pcl_pub.publish(obst_pcl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationToObstaclePointCloudNode()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
