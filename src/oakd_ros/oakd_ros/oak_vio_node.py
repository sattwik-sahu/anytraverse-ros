#!/usr/bin/env python3
"""
ROS 2 node that streams color data from an OAK-D Pro W using the DepthAI v3 API and RTAB-Map SLAM.

Published topics:
    /oak/camera/image_raw   (sensor_msgs/Image, bgr8, rectified RGB frame)
    /oak/camera/camera_info (sensor_msgs/CameraInfo)         <- RGB intrinsics
    /oak/depth/image_raw    (sensor_msgs/Image, 16UC1, mm, aligned to RGB)
    /oak/depth/camera_info  (sensor_msgs/CameraInfo)
    /oak/imu/data           (sensor_msgs/Imu)

Published TF:
    odom -> base_link       (Calculated using the on-device tracked RGB camera pose and
                             static extrinsics, corrected so base_link starts at the origin)
"""

import threading

import depthai as dai
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

RGB_SOCKET = dai.CameraBoardSocket.CAM_A
LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C


def image_msg(frame: np.ndarray, encoding: str, frame_id: str, stamp) -> Image:
    frame = np.ascontiguousarray(frame)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if frame.ndim == 2 else frame.shape[2]
    msg.step = frame.shape[1] * frame.itemsize * channels
    msg.data = frame.tobytes()
    return msg


def camera_info_msg(
    calib: "dai.CalibrationHandler", socket, width, height, frame_id
) -> CameraInfo:
    K = calib.getCameraIntrinsics(socket, width, height)
    D = [float(v) for v in calib.getDistortionCoefficients(socket)]

    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.k = [float(v) for row in K for v in row]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [
        msg.k[0],
        msg.k[1],
        msg.k[2],
        0.0,
        msg.k[3],
        msg.k[4],
        msg.k[5],
        0.0,
        msg.k[6],
        msg.k[7],
        msg.k[8],
        0.0,
    ]
    if len(D) > 5 and any(abs(v) > 1e-9 for v in D[5:]):
        msg.distortion_model = "rational_polynomial"
        msg.d = D[:8] if len(D) >= 8 else D
    else:
        msg.distortion_model = "plumb_bob"
        msg.d = D[:5]
    return msg


class OakDNode(Node):
    def __init__(self):
        super().__init__("oak_vio_node")

        # Parametrized properties
        self.declare_parameters(
            namespace="",
            parameters=[
                ("rgb_size", [640, 480]),
                ("mono_size", [640, 400]),
                ("fps", 30),
                ("imu_hz", 200),
                ("camera_optical_frame_id", "oak_rgb_camera_optical_frame"),
                ("imu_frame_id", "oak_imu_frame"),
                ("base_frame_id", "base_link"),
                ("odom_frame_id", "odom"),
            ],
        )

        # Retrieve parameter values
        self.rgb_size: tuple[int, int] = tuple(self.get_parameter("rgb_size").value)
        self.mono_size: tuple[int, int] = tuple(self.get_parameter("mono_size").value)
        self.fps: int = self.get_parameter("fps").value
        self.imu_hz: int = self.get_parameter("imu_hz").value

        self.camera_optical_frame_id: str = self.get_parameter(
            "camera_optical_frame_id"
        ).value
        self.imu_frame_id: str = self.get_parameter("imu_frame_id").value
        self.base_frame_id: str = self.get_parameter("base_frame_id").value
        self.odom_frame_id: str = self.get_parameter("odom_frame_id").value

        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.camera_pub = self.create_publisher(
            Image, "oak/camera/image_raw", sensor_qos
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "oak/camera/camera_info", sensor_qos
        )
        self.depth_pub = self.create_publisher(Image, "oak/depth/image_raw", sensor_qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "oak/depth/camera_info", sensor_qos
        )
        self.imu_pub = self.create_publisher(Imu, "oak/imu/data", sensor_qos)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._stop = threading.Event()
        self._threads = []
        self._last_tf_warn_time = 0.0

        self._build_and_start_pipeline()

    # ------------------------------------------------------------------
    def _safe_publish(self, publisher, msg):
        if self._stop.is_set() or not rclpy.ok():
            return
        try:
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"Dropped publish during shutdown: {e}")

    # ------------------------------------------------------------------
    def _build_and_start_pipeline(self):
        self.pipeline = dai.Pipeline()
        platform = self.pipeline.getDefaultDevice().getPlatform()
        self.get_logger().info(f"Connected to OAK device, platform: {platform}")

        # Nodes
        rgb = self.pipeline.create(dai.node.Camera).build(
            RGB_SOCKET, sensorFps=self.fps
        )
        left = self.pipeline.create(dai.node.Camera).build(
            LEFT_SOCKET, sensorFps=self.fps
        )
        right = self.pipeline.create(dai.node.Camera).build(
            RIGHT_SOCKET, sensorFps=self.fps
        )
        stereo = self.pipeline.create(dai.node.StereoDepth)
        imu = self.pipeline.create(dai.node.IMU)
        odom = self.pipeline.create(dai.node.BasaltVIO)
        slam = self.pipeline.create(dai.node.RTABMapSLAM)

        params = {
            "RGBD/CreateOccupancyGrid": "true",
            "Grid/3D": "true",
            "Rtabmap/SaveWMState": "true",
        }
        slam.setParams(params)

        # Stereo configuration
        stereo.setExtendedDisparity(False)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setRectifyEdgeFillColor(0)
        stereo.enableDistortionCorrection(True)
        stereo.initialConfig.setLeftRightCheckThreshold(10)
        stereo.setDepthAlign(RGB_SOCKET)
        stereo.setOutputSize(*self.rgb_size)

        # IMU Configuration
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], self.imu_hz
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        # Output setups
        rgbOut = rgb.requestOutput(
            self.rgb_size,
            type=dai.ImgFrame.Type.BGR888i,
            fps=self.fps,
            resizeMode=dai.ImgResizeMode.CROP,
            enableUndistortion=True,
        )

        rgbHostOut = rgb.requestOutput(
            self.rgb_size,
            type=dai.ImgFrame.Type.BGR888i,
            fps=self.fps,
            resizeMode=dai.ImgResizeMode.CROP,
            enableUndistortion=True,
        )

        leftOut = left.requestOutput(self.mono_size, fps=self.fps)
        rightOut = right.requestOutput(self.mono_size, fps=self.fps)

        # Links for VIO
        leftOut.link(stereo.left)
        rightOut.link(stereo.right)
        stereo.syncedLeft.link(odom.left)
        stereo.syncedRight.link(odom.right)
        imu.out.link(odom.imu)

        # Links for SLAM
        rgbOut.link(slam.rect)
        stereo.depth.link(slam.depth)
        odom.transform.link(slam.odom)

        # Output queues
        self.camera_queue = rgbHostOut.createOutputQueue(maxSize=8, blocking=False)
        self.depth_queue = slam.passthroughDepth.createOutputQueue(
            maxSize=8, blocking=False
        )

        self.imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)
        self.transform_queue = slam.transform.createOutputQueue(
            maxSize=20, blocking=False
        )

        self.pipeline.start()

        device = self.pipeline.getDefaultDevice()
        calib = device.readCalibration()

        w, h = self.rgb_size
        self._camera_info_msg = camera_info_msg(
            calib,
            RGB_SOCKET,
            w,
            h,
            self.camera_optical_frame_id,
        )
        self._depth_info_msg = camera_info_msg(
            calib,
            RGB_SOCKET,
            w,
            h,
            self.camera_optical_frame_id,
        )

        self._threads = [
            threading.Thread(target=self._camera_loop, daemon=True),
            threading.Thread(target=self._depth_loop, daemon=True),
            threading.Thread(target=self._imu_loop, daemon=True),
            threading.Thread(target=self._transform_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    # ------------------------------------------------------------------
    def _camera_loop(self):
        while not self._stop.is_set() and self.pipeline.isRunning() and rclpy.ok():
            try:
                camera_frame = self.camera_queue.get()
            except Exception:
                continue
            if camera_frame is None:
                continue
            stamp = self.get_clock().now().to_msg()
            self._safe_publish(
                self.camera_pub,
                image_msg(
                    camera_frame.getFrame(), "bgr8", self.camera_optical_frame_id, stamp
                ),
            )
            self._camera_info_msg.header.stamp = stamp
            self._safe_publish(self.camera_info_pub, self._camera_info_msg)

    def _depth_loop(self):
        while not self._stop.is_set() and self.pipeline.isRunning() and rclpy.ok():
            try:
                depth_frame = self.depth_queue.get()
            except Exception:
                continue
            if depth_frame is None:
                continue
            stamp = self.get_clock().now().to_msg()
            self._safe_publish(
                self.depth_pub,
                image_msg(
                    depth_frame.getFrame(), "16UC1", self.camera_optical_frame_id, stamp
                ),
            )
            self._depth_info_msg.header.stamp = stamp
            self._safe_publish(self.depth_info_pub, self._depth_info_msg)

    def _imu_loop(self):
        while not self._stop.is_set() and self.pipeline.isRunning() and rclpy.ok():
            try:
                data = self.imu_queue.get()
            except Exception:
                continue
            if data is None:
                continue
            stamp = self.get_clock().now().to_msg()
            for packet in data.packets:
                accel = packet.acceleroMeter
                gyro = packet.gyroscope

                msg = Imu()
                msg.header.stamp = stamp
                msg.header.frame_id = self.imu_frame_id
                msg.linear_acceleration.x = float(accel.x)
                msg.linear_acceleration.y = float(accel.y)
                msg.linear_acceleration.z = float(accel.z)
                msg.angular_velocity.x = float(gyro.x)
                msg.angular_velocity.y = float(gyro.y)
                msg.angular_velocity.z = float(gyro.z)
                msg.orientation_covariance[0] = -1.0
                self._safe_publish(self.imu_pub, msg)

    def _transform_loop(self):
        while not self._stop.is_set() and self.pipeline.isRunning() and rclpy.ok():
            try:
                transform = self.transform_queue.get()
            except Exception:
                continue
            if transform is None:
                continue
            stamp = self.get_clock().now().to_msg()

            translation = transform.getTranslation()
            quat = transform.getQuaternion()
            qx, qy, qz, qw = self._quat_components(quat)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id

            try:
                # 1. Lookup static transform: T_cam_opt^base directly from the TF tree
                tf_base_to_cam = self.tf_buffer.lookup_transform(
                    self.camera_optical_frame_id, self.base_frame_id, rclpy.time.Time()
                )

                t_b2c = tf_base_to_cam.transform.translation
                q_b2c = tf_base_to_cam.transform.rotation

                # Represent T_cam_opt^base
                r_cam_base = R.from_quat([q_b2c.x, q_b2c.y, q_b2c.z, q_b2c.w])
                t_cam_base = np.array([t_b2c.x, t_b2c.y, t_b2c.z])

                # Apply correction from DepthAI IMU convention to REP 103
                r_imu_ros = R.from_quat([0.5, -0.5, 0.5, -0.5])
                r_cam_base = r_imu_ros * r_cam_base
                t_cam_base = r_imu_ros.apply(t_cam_base)

                # Compute inverse transform: T_base^cam_opt
                r_base_cam = r_cam_base.inv()
                t_base_cam = -r_base_cam.apply(t_cam_base)

                # 2. Extract raw tracked camera pose in RDF optical coordinates directly from VSLAM node
                r_odom_cam_opt = R.from_quat([qx, qy, qz, qw])
                t_odom_cam_opt = np.array([translation.x, translation.y, translation.z])

                # 3. Apply the similarity transformation directly.
                # Because T_cam_opt^base already incorporates the RDF-to-ROS rotation internally,
                # manual pre-multiplication is omitted.
                # T_odom'^base = T_base^cam_opt * T_odom_cam_opt * T_cam_opt^base
                r_12 = r_base_cam * r_odom_cam_opt
                t_12 = r_base_cam.apply(t_odom_cam_opt) + t_base_cam

                r_odom_base = r_12 * r_cam_base
                t_odom_base = r_12.apply(t_cam_base) + t_12

                qx_out, qy_out, qz_out, qw_out = r_odom_base.as_quat()

                tf_msg.child_frame_id = self.base_frame_id
                tf_msg.transform.translation.x = float(t_odom_base[0])
                tf_msg.transform.translation.y = float(t_odom_base[1])
                tf_msg.transform.translation.z = float(t_odom_base[2])
                tf_msg.transform.rotation.x = float(qx_out)
                tf_msg.transform.rotation.y = float(qy_out)
                tf_msg.transform.rotation.z = float(qz_out)
                tf_msg.transform.rotation.w = float(qw_out)

            except Exception as e:
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                if now_sec - self._last_tf_warn_time > 5.0:
                    self.get_logger().warning(
                        f"Could not calculate odom->base transform: {e}. "
                        f"Falling back to publishing raw odom->camera transform.",
                    )
                    self._last_tf_warn_time = now_sec

                tf_msg.child_frame_id = self.camera_optical_frame_id
                tf_msg.transform.translation.x = float(translation.x)
                tf_msg.transform.translation.y = float(translation.y)
                tf_msg.transform.translation.z = float(translation.z)
                tf_msg.transform.rotation.x = qx
                tf_msg.transform.rotation.y = qy
                tf_msg.transform.rotation.z = qz
                tf_msg.transform.rotation.w = qw

            if self._stop.is_set() or not rclpy.ok():
                break
            try:
                self.tf_broadcaster.sendTransform(tf_msg)
            except Exception as e:
                self.get_logger().debug(f"Dropped TF broadcast during shutdown: {e}")

    @staticmethod
    def _quat_components(q):
        for names in (("qx", "qy", "qz", "qw"), ("x", "y", "z", "w")):
            if all(hasattr(q, n) for n in names):
                return tuple(float(getattr(q, n)) for n in names)
        raise AttributeError(
            "Could not read quaternion fields from dai.TransformData.getQuaternion()"
        )

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        try:
            self.pipeline.stop()
            self.pipeline.wait()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    try:
        with rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO):
            node = OakDNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
