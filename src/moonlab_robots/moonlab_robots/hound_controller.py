import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rhino import init_hound as init_hound_controller
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class HoundController(Node):
    """The controller node for the Hound robot, built at MOON Lab"""

    def __init__(self) -> None:
        super().__init__(node_name="hound_controller")

        # QoS Profile
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 1. Get connection string parameter
        self.declare_parameter(name="connection_str", value="/dev/ttyUSB0")

        # NOTE: Added .string_value to extract the actual string from the ROS Parameter object
        connection_str = (
            self.get_parameter("connection_str").get_parameter_value().string_value
        )

        self.get_logger().info(f"Connecting to Hound hardware at: {connection_str}")

        # 2. Initialize the hardware interface
        try:
            self._controller = init_hound_controller(connection_str=connection_str)  # type: ignore
            self.get_logger().info("Hound hardware initialized successfully.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect to Hound hardware: {e}")
            raise e

        # 3. Create Subscriber to /cmd_vel
        self._cmd_vel_sub = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self._send_command,
            qos_profile=qos_profile,
        )
        self.get_logger().info(f"Subscribed to {self._cmd_vel_sub.topic}")

    def _send_command(self, msg: Twist) -> None:
        """Callback that runs every time a Twist message is received."""
        # Optional: Log the incoming command for debugging
        # self.get_logger().debug(f"Rx Twist: Lin={msg.linear.x:.2f}, Ang={msg.angular.z:.2f}")

        try:
            # Map ROS Twist messages to the robot's hardware API
            # Note: You were inverting angular.z in your snippet (-msg.angular.z),
            # ensuring this aligns with your robot's steering direction (Left vs Right).
            feedback = self._controller.send_velocity_cmd(
                throttle=msg.linear.x,
                steering=-msg.angular.z,
            )

            # Log the robot feedback values
            self.get_logger().info(
                f"PWM values: throttle={feedback['throttle_pwm']} | steering={feedback['steering_pwm']}"
            )

            if msg.linear.x == msg.angular.z == 0:
                self._controller.apply_brake()
            else:
                self._controller.release_brake()
        except Exception as e:
            self.get_logger().warn(f"Hardware communication error: {e}")


def main(args=None):
    # Standard ROS 2 Python node entry point
    rclpy.init(args=args)

    node = HoundController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow cleanly killing the node with Ctrl+C
        pass
    except Exception as e:
        print(f"Node terminated with error: {e}")
    finally:
        # Cleanup when shutting down
        if "node" in locals():
            # optional: node._controller.stop() # Good practice to stop motors on exit
            node.destroy_node()
        node._controller.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
