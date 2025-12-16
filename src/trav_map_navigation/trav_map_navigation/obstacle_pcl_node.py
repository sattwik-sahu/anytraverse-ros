import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ObstaclePointCloudNode(Node):
    """The obstacle cloud projector node"""

    def __init__(self) -> None:
        super().__init__(node_name="obstacle_cloud_node")

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Declare parameters
        self.declare_parameter("traversability_threshold", 0.5)
        self._trav_thresh: float = (
            self.get_parameter("traversability_threshold").value or 0.5
        )

        # Create subscribers
        self._depth_sub = message_filters.Subscriber(
            self, CompressedImage, "/camera/depth/image_raw/compressedDepth"
        )
        self._trav_sub = message_filters.Subscriber(self, CompressedImage, "/trav_map")
        self._camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/camera/color/camera_info"
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
        depth_msg: CompressedImage,
        trav_msg: CompressedImage,
        camera_info_msg: CameraInfo,
    ) -> None:
        try:
            # ==== Decode the depth image ====
            # Read compressed depth bytes (:. uint8)
            depth_np = np.frombuffer(depth_msg.data, np.uint8)

            # Decode bytes to original depth map
            depth_img = cv2.imdecode(depth_np, cv2.IMREAD_UNCHANGED)

            # Validate depth image
            if depth_img is None:
                self.get_logger().warn("Failed to decode depth image")
                return

            # Convert depth from mm to m
            depth_img_m = depth_img.astype(np.float32) * 1e-3

            # ==== Read the traversability map ====
            # This returns uint8 (0-255)
            trav_map_uint8 = self._bridge.compressed_imgmsg_to_cv2(trav_msg)

            # Handle Grayscale vs Color shapes
            if len(trav_map_uint8.shape) == 3:
                trav_map_uint8 = trav_map_uint8[:, :, 0]  # Take one channel if BGR

            # Normalize to 0.0 - 1.0 range
            trav_map = trav_map_uint8.astype(np.float32) / 255.0
        except Exception as e:
            self.get_logger().error(f"Error processing images...\n{e}")
            return

        # ==== Vectorized pixel reprojection ====
        # Get the camera intrinsic parameters
        fx, cx, fy, cy = self._get_camera_intrinsics(camera_info_msg=camera_info_msg)

        # Create grid of pixel coordinates (u, v)
        # NOTE (u, v) <- (grid(w), grid(h))
        h, w = depth_img_m.shape
        u_grid, v_grid = np.meshgrid(np.arange(w), np.arange(h))

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
        obst_pcl_msg.header.frame_id = "camera_optical_link"
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
    node = ObstaclePointCloudNode()

    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
