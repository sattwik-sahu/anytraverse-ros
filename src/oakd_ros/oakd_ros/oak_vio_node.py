import depthai
import numpy as np
import rclpy
from cv_bridge.core import CvBridge
from numpy import typing as npt
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.timer import Timer
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener

from oakd_ros.utils import (
    build_camera_info,
    pose_to_transformation_matrix,
    set_stereo_preset,
    transform_stamped_msg_to_transformation_matrix,
    transformation_matrix_to_transform_stamped_msg,
)


class OakVIONode(Node):
    """
    ROS2 node starting OAKD camera with BasaltVIO and RTAB for loop closure.
    """

    def __init__(self) -> None:
        super().__init__(node_name="oakd_vio_node")

        # Create QoS profile
        _qos_profile: QoSProfile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Create parameters
        self.declare_parameter("image_fps", 30)  # type: ignore
        self.declare_parameter("imu_freq", 200)  # type: ignore
        self.declare_parameter("image_width", 640)  # type: ignore
        self.declare_parameter("image_height", 400)  # type: ignore
        self.declare_parameter(
            "camera_optical_frame_id",
            "oak_rgb_camera_optical_frame",  # type: ignore
        )
        self.declare_parameter("imu_frame_id", "oak_imu_frame")  # type: ignore
        self.declare_parameter("odom_child_frame_id", "oak_left_camera_frame")  # type: ignore
        self.declare_parameter("base_frame_id", "base_link")  # type: ignore
        self._image_fps: int = (
            self.get_parameter("image_fps").get_parameter_value().integer_value
        )
        self._imu_freq: int = (
            self.get_parameter("imu_freq").get_parameter_value().integer_value
        )
        self._image_resolution: tuple[int, int] = (
            self.get_parameter("image_width").get_parameter_value().integer_value,
            self.get_parameter("image_height").get_parameter_value().integer_value,
        )
        self._camera_optical_frame_id: str = (
            self.get_parameter("camera_optical_frame_id")
            .get_parameter_value()
            .string_value
        )
        self._imu_frame_id: str = (
            self.get_parameter("imu_frame_id").get_parameter_value().string_value
        )
        self._odom_child_frame_id: str = (
            self.get_parameter("odom_child_frame_id").get_parameter_value().string_value
        )
        self._base_frame_id: str = (
            self.get_parameter("base_frame_id").get_parameter_value().string_value
        )

        # Create CV bridge
        self._cv_bridge: CvBridge = CvBridge()

        # Create the publishers
        self._rgb_publisher: Publisher = self.create_publisher(
            msg_type=Image, topic="/oak/rgb/image_raw", qos_profile=_qos_profile
        )
        self._rgb_camera_info_publisher: Publisher = self.create_publisher(
            msg_type=CameraInfo, topic="/oak/rgb/camera_info", qos_profile=_qos_profile
        )
        self._depth_publisher: Publisher = self.create_publisher(
            msg_type=Image, topic="/oak/depth/image_raw", qos_profile=_qos_profile
        )
        self._depth_camera_info_publisher: Publisher = self.create_publisher(
            msg_type=CameraInfo,
            topic="/oak/depth/camera_info",
            qos_profile=_qos_profile,
        )
        self._imu_publisher: Publisher = self.create_publisher(
            msg_type=Imu, topic="/oak/imu/data", qos_profile=_qos_profile
        )

        # Create the tf stuff
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(buffer=self._tf_buffer, node=self)
        self._tf_broadcaster = TransformBroadcaster(node=self)

        # Setup the pipeline
        self._setup_depthai_pipeline()

        # Create callback groups for parallel execution
        self._camera_callback_group = MutuallyExclusiveCallbackGroup()
        self._imu_callback_group = MutuallyExclusiveCallbackGroup()
        self._odom_callback_group = MutuallyExclusiveCallbackGroup()

        # Create timers with callbacks to publish data
        self._camera_timer: Timer = self.create_timer(
            timer_period_sec=1.0 / (2 * self._image_fps),
            callback=self._camera_callback,
            callback_group=self._camera_callback_group,
        )
        self._imu_timer: Timer = self.create_timer(
            timer_period_sec=1.0 / self._imu_freq,
            callback=self._imu_callback,
            callback_group=self._imu_callback_group,
        )
        self._odom_timer: Timer = self.create_timer(
            timer_period_sec=1.0 / self._image_fps,
            callback=self._odom_callback,
            callback_group=self._odom_callback_group,
        )

        self._pipeline.start()

    def _setup_depthai_pipeline(self) -> None:
        """Setup the depthai pipeline for the node."""

        # Setup device, pipeline and calibration
        self._device: depthai.Device = depthai.Device()
        self._pipeline: depthai.Pipeline = depthai.Pipeline(defaultDevice=self._device)
        self._calibration: depthai.CalibrationHandler = self._device.readCalibration()

        # RGB camera nodes
        rgb_camera_node = self._pipeline.create(depthai.node.Camera).build(
            boardSocket=depthai.CameraBoardSocket.CAM_A, sensorFps=self._image_fps
        )
        rgb_camera_output = rgb_camera_node.requestOutput(
            size=self._image_resolution,
            type=depthai.ImgFrame.Type.BGR888p,
            fps=self._image_fps,
        )
        self._rgb_queue = rgb_camera_output.createOutputQueue(maxSize=4, blocking=False)

        # Setup stereo depth
        camera_left = self._pipeline.create(depthai.node.Camera).build(
            boardSocket=depthai.CameraBoardSocket.CAM_B, sensorFps=self._image_fps
        )
        camera_right = self._pipeline.create(depthai.node.Camera).build(
            boardSocket=depthai.CameraBoardSocket.CAM_C, sensorFps=self._image_fps
        )
        camera_left_out = camera_left.requestFullResolutionOutput(
            type=depthai.ImgFrame.Type.GRAY8, fps=self._image_fps
        )
        camera_right_out = camera_right.requestFullResolutionOutput(
            type=depthai.ImgFrame.Type.GRAY8, fps=self._image_fps
        )
        stereo = self._pipeline.create(depthai.node.StereoDepth)
        set_stereo_preset(stereo=stereo)
        stereo.setSubpixel(enable=True)
        stereo.setExtendedDisparity(enable=False)
        stereo.setRectifyEdgeFillColor(color=0)
        stereo.enableDistortionCorrection(True)
        stereo.initialConfig.setLeftRightCheck(enable=True)

        # Align depth with the RGB output
        stereo.setDepthAlign(camera=depthai.CameraBoardSocket.CAM_A)
        width, height = self._image_resolution
        stereo.setOutputSize(width=width, height=height)
        camera_left_out.link(input=stereo.left)
        camera_right_out.link(input=stereo.right)
        self._depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        # Setup IMU
        imu = self._pipeline.create(depthai.node.IMU)
        imu.enableIMUSensor(
            sensors=[
                depthai.IMUSensor.ACCELEROMETER_CALIBRATED,
                depthai.IMUSensor.GYROSCOPE_CALIBRATED,
            ],
            reportRate=self._imu_freq,
        )
        imu.setBatchReportThreshold(batchReportThreshold=1)
        imu.setMaxBatchReports(maxBatchReports=10)
        self._imu_queue = imu.out.createOutputQueue(maxSize=20, blocking=False)

        # Camera info msg
        self._rgb_info_msg = build_camera_info(
            calib=self._calibration,
            socket=depthai.CameraBoardSocket.CAM_A,
            width=width,
            height=height,
            frame_id=self._camera_optical_frame_id,
        )
        self._depth_info_msg = build_camera_info(
            calib=self._calibration,
            socket=depthai.CameraBoardSocket.CAM_A,
            width=width,
            height=height,
            frame_id=self._camera_optical_frame_id,
        )

        # Setup BasaltVIO and RTAB SLAM nodes
        odom = self._pipeline.create(depthai.node.BasaltVIO)
        slam = self._pipeline.create(depthai.node.RTABMapSLAM)
        params = {
            "RGBD/CreateOccupancyGrid": "true",
            "Grid/3D": "true",
            "Rtabmap/SaveWMState": "true",
        }
        slam.setParams(params)
        stereo.syncedLeft.link(odom.left)
        stereo.syncedRight.link(odom.right)
        stereo.rectifiedLeft.link(slam.rect)
        imu.out.link(odom.imu)
        odom.transform.link(slam.odom)
        self._slam_output_queue = slam.transform.createOutputQueue(
            maxSize=10, blocking=False
        )

    # def _camera_callback(self) -> None:
    #     pass
    #
    # def _imu_callback(self) -> None:
    #     pass
    #
    # def _odom_callback(self) -> None:
    #     pass

    def _camera_callback(self) -> None:
        if not self._pipeline.isRunning():
            self.get_logger().error("DepthAI pipeline has stopped working!")
            rclpy.shutdown()
            return

        self._process_rgb()
        self._process_depth()

    def _imu_callback(self) -> None:
        if not self._pipeline.isRunning():
            return

        self._process_imu()

    def _odom_callback(self) -> None:
        if not self._pipeline.isRunning():
            return

        self._process_odom()

    def _process_rgb(self) -> None:
        # tryGetAll() fetches all frames accumulated since the last poll
        frames = self._rgb_queue.tryGetAll()
        for frame in frames:
            ros_timestamp = self.get_clock().now().to_msg()
            cv_image: npt.NDArray[np.uint8] = frame.getCvFrame()  # type: ignore

            # Construct messages
            image_msg: Image = self._cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_msg.header.stamp = ros_timestamp
            image_msg.header.frame_id = self._camera_optical_frame_id
            self._rgb_info_msg.header.stamp = ros_timestamp

            # Publish messages
            self._rgb_publisher.publish(msg=image_msg)
            self._rgb_camera_info_publisher.publish(msg=self._rgb_info_msg)

    def _process_depth(self) -> None:
        # tryGetAll() fetches all frames accumulated since the last poll
        frames = self._depth_queue.tryGetAll()
        for frame in frames:
            ros_timestamp = self.get_clock().now().to_msg()
            depth_np: npt.NDArray[np.uint16] = frame.getFrame()  # type: ignore

            # Construct messages
            depth_msg = self._cv_bridge.cv2_to_imgmsg(depth_np, encoding="16UC1")
            depth_msg.header.stamp = ros_timestamp
            depth_msg.header.frame_id = self._camera_optical_frame_id
            self._depth_info_msg.header.stamp = ros_timestamp

            # Publish messages
            self._depth_publisher.publish(msg=depth_msg)
            self._depth_camera_info_publisher.publish(msg=self._depth_info_msg)

    def _process_imu(self) -> None:
        # tryGetAll() completely drains the queue, fixing the 400Hz bottleneck
        imu_data_list = self._imu_queue.tryGetAll()

        for imu_data in imu_data_list:
            for packet in imu_data.packets:  # type: ignore
                ros_timestamp = self.get_clock().now().to_msg()
                accel = packet.acceleroMeter
                gyro = packet.gyroscope

                # Construct message
                imu_msg: Imu = Imu()
                imu_msg.header.stamp = ros_timestamp
                imu_msg.header.frame_id = self._imu_frame_id
                imu_msg.linear_acceleration.x = float(accel.x)
                imu_msg.linear_acceleration.y = float(accel.y)
                imu_msg.linear_acceleration.z = float(accel.z)
                imu_msg.angular_velocity.x = float(gyro.x)
                imu_msg.angular_velocity.y = float(gyro.y)
                imu_msg.angular_velocity.z = float(gyro.z)
                imu_msg.orientation_covariance[0] = -1.0
                imu_msg.linear_acceleration_covariance[0] = -1.0
                imu_msg.angular_velocity_covariance[0] = -1.0

                # Publish message
                self._imu_publisher.publish(msg=imu_msg)

    def _process_odom(self) -> None:
        transforms: list[depthai.TransformData] = self._slam_output_queue.tryGetAll()  # type: ignore

        for transform in transforms:
            t = transform.getTranslation()
            q = transform.getQuaternion()

            tf_base_child = transform_stamped_msg_to_transformation_matrix(
                msg=self._tf_buffer.lookup_transform(
                    self._base_frame_id,
                    self._odom_child_frame_id,
                    self.get_clock().now(),
                )
            )
            tf_odom_child = pose_to_transformation_matrix(
                q=[q.qx, q.qy, q.qz, q.qw], t=[t.x, t.y, t.z]
            )
            tf_odom_base = tf_odom_child @ np.linalg.inv(tf_base_child)
            msg = transformation_matrix_to_transform_stamped_msg(tf_odom_base)
            msg.header.stamp = self.get_clock().now()
            msg.header.frame_id = self._base_frame_id
            msg.child_frame_id = self._odom_child_frame_id
            self._tf_broadcaster.sendTransform(transform=msg)


def main():
    try:
        with rclpy.init():
            node = OakVIONode()
            rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
