import depthai
import numpy as np
import rclpy
from numpy import typing as npt
from rclpy.node import Node as Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.timer import Timer
from sensor_msgs.msg import CameraInfo, Image, Imu

from oakd_ros.utils import (
    build_camera_info,
    set_stereo_preset,
)

from cv_bridge import CvBridge


class OakdCameraNode(Node):
    FPS: int = 30
    IMU_RATE: int = 400
    IMAGE_WIDTH: int = 640
    IMAGE_HEIGHT: int = 480
    OPTICAL_FRAME_ID: str = "oakd_rgb_optical_frame"
    IMU_FRAME_ID: str = "oakd_imu_frame"

    def __init__(self) -> None:
        super().__init__(node_name="oakd_node")

        # Create QoS Profile
        _qos_profile: QoSProfile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # Create the CV Bridge
        self._cv_bridge: CvBridge = CvBridge()

        # Publishers
        self._image_pub: Publisher = self.create_publisher(
            msg_type=Image, topic="/oakd/rgb/image_raw", qos_profile=_qos_profile
        )
        self._camera_info_pub: Publisher = self.create_publisher(
            msg_type=CameraInfo,
            topic="/oakd/rgb/camera_info",
            qos_profile=_qos_profile,
        )
        self._depth_pub: Publisher = self.create_publisher(
            msg_type=Image, topic="/oakd/depth/image_raw", qos_profile=_qos_profile
        )
        self._depth_camera_info_pub: Publisher = self.create_publisher(
            msg_type=CameraInfo,
            topic="/oakd/depth/camera_info",
            qos_profile=_qos_profile,
        )
        self._imu_pub: Publisher = self.create_publisher(
            msg_type=Imu, topic="/oakd/imu/data", qos_profile=_qos_profile
        )

        # Setup the pipeline
        self._setup_pipeline()

        # Create timer
        self._timer: Timer = self.create_timer(
            timer_period_sec=1.0 / self.FPS, callback=self._poll
        )

    def _setup_pipeline(self) -> None:
        self._device: depthai.Device = depthai.Device()
        self._calibration: depthai.CalibrationHandler = self._device.readCalibration()
        self._pipeline: depthai.Pipeline = depthai.Pipeline(defaultDevice=self._device)

        # RGB camera nodes
        rgb_camera = self._pipeline.create(depthai.node.Camera).build(
            boardSocket=depthai.CameraBoardSocket.CAM_A, sensorFps=self.FPS
        )
        rgb_out = rgb_camera.requestOutput(
            size=(self.IMAGE_WIDTH, self.IMAGE_HEIGHT),
            type=depthai.ImgFrame.Type.BGR888p,
            fps=self.FPS,
        )
        self._rgb_queue = rgb_out.createOutputQueue(maxSize=4, blocking=False)

        # Stereo depth
        camera_left = self._pipeline.create(depthai.node.Camera).build(
            boardSocket=depthai.CameraBoardSocket.CAM_B, sensorFps=self.FPS
        )
        camera_right = self._pipeline.create(depthai.node.Camera).build(
            boardSocket=depthai.CameraBoardSocket.CAM_C, sensorFps=self.FPS
        )
        camera_left_out = camera_left.requestFullResolutionOutput(
            type=depthai.ImgFrame.Type.GRAY8, fps=self.FPS
        )
        camera_right_out = camera_right.requestFullResolutionOutput(
            type=depthai.ImgFrame.Type.GRAY8, fps=self.FPS
        )
        stereo = self._pipeline.create(depthai.node.StereoDepth)
        set_stereo_preset(stereo=stereo)
        stereo.setSubpixel(enable=False)
        stereo.setExtendedDisparity(enable=False)
        stereo.setDepthAlign(camera=depthai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(width=self.IMAGE_WIDTH, height=self.IMAGE_HEIGHT)
        camera_left_out.link(input=stereo.left)
        camera_right_out.link(input=stereo.right)
        self._depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        # IMU
        imu = self._pipeline.create(depthai.node.IMU)
        imu.enableIMUSensor(
            sensors=[
                depthai.IMUSensor.ACCELEROMETER_RAW,
                depthai.IMUSensor.GYROSCOPE_RAW,
            ],
            reportRate=self.IMU_RATE,
        )
        imu.setBatchReportThreshold(batchReportThreshold=1)
        imu.setMaxBatchReports(maxBatchReports=10)
        self._imu_queue = imu.out.createOutputQueue(maxSize=20, blocking=False)
        self._pipeline.start()

        # Camera info msg
        self._rgb_info_msg = build_camera_info(
            calib=self._calibration,
            socket=depthai.CameraBoardSocket.CAM_A,
            width=self.IMAGE_WIDTH,
            height=self.IMAGE_HEIGHT,
            frame_id=self.OPTICAL_FRAME_ID,
        )
        self._depth_info_msg = build_camera_info(
            calib=self._calibration,
            socket=depthai.CameraBoardSocket.CAM_A,
            width=self.IMAGE_WIDTH,
            height=self.IMAGE_HEIGHT,
            frame_id=self.OPTICAL_FRAME_ID,
        )

    def _poll(self) -> None:
        if not self._pipeline.isRunning():
            self.get_logger().error("DepthAI pipeline has stopped working!")
            rclpy.shutdown()
        else:
            self._process_rgb()
            self._process_depth()
            self._process_imu()

    def _process_rgb(self) -> None:
        frame: depthai.ImgFrame | None = self._rgb_queue.tryGet()  # type: ignore
        if frame is not None:
            ros_timestamp = self.get_clock().now().to_msg()
            cv_image: npt.NDArray[np.uint8] = frame.getCvFrame()  # type: ignore

            # Construct messages
            image_msg: Image = self._cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_msg.header.stamp = ros_timestamp
            image_msg.header.frame_id = self.OPTICAL_FRAME_ID
            self._rgb_info_msg.header.stamp = ros_timestamp

            # Publish messages
            self._image_pub.publish(msg=image_msg)
            self._camera_info_pub.publish(msg=self._rgb_info_msg)

    def _process_depth(self) -> None:
        frame: depthai.ImgFrame | None = self._depth_queue.tryGet()  # type: ignore
        if frame is not None:
            ros_timestamp = self.get_clock().now().to_msg()
            depth_np: npt.NDArray[np.uint16] = frame.getFrame()  # type: ignore

            # Construct messages
            depth_msg = self._cv_bridge.cv2_to_imgmsg(depth_np, encoding="16UC1")
            depth_msg.header.stamp = ros_timestamp
            depth_msg.header.frame_id = self.OPTICAL_FRAME_ID
            self._depth_info_msg.header.stamp = ros_timestamp

            # Publish messages
            self._depth_pub.publish(msg=depth_msg)
            self._depth_camera_info_pub.publish(msg=self._depth_info_msg)

    def _process_imu(self) -> None:
        imu_data: depthai.IMUData | None = self._imu_queue.tryGet()  # type: ignore

        if imu_data is not None:
            for packet in imu_data.packets:
                ros_timestamp = self.get_clock().now().to_msg()
                accel = packet.acceleroMeter
                gyro = packet.gyroscope

                # Construct message
                imu_msg: Imu = Imu()
                imu_msg.header.stamp = ros_timestamp
                imu_msg.header.frame_id = self.IMU_FRAME_ID
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
                self._imu_pub.publish(msg=imu_msg)

    def destroy_node(self) -> None:
        """Stop pipeline and clean up."""
        try:
            self._pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    oakd_node: OakdCameraNode = OakdCameraNode()
    rclpy.spin(node=oakd_node)

    oakd_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
