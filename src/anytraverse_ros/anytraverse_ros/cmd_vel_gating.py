import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelGating(Node):
    def __init__(self) -> None:
        super().__init__(node_name="cmd_vel_gating_node", namespace="/anytraverse")

        # QoS Settings
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._gated_cmd_vel_pub = self.create_publisher(
            msg_type=Twist, topic="/anytraverse/cmd_vel", qos_profile=qos_profile
        )

        self._hoc_flag = False

        self._cmd_vel_sub = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self._cmd_vel_callback,
            qos_profile=10,
        )

        self._hoc_req_sub = self.create_subscription(
            msg_type=Bool,
            topic="/anytraverse/hoc_req",
            callback=self._hoc_req_callback,
            qos_profile=qos_profile,
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        if self._hoc_flag:
            self._gated_cmd_vel_pub.publish(Twist())
        else:
            self._gated_cmd_vel_pub.publish(msg)

    def _hoc_req_callback(self, msg: Bool) -> None:
        self._hoc_flag = msg.data

        # SAFETY CRITICAL: If Human Operator Call is required,
        # stop immediately. Don't wait for the next cmd_vel input.
        if self._hoc_flag:
            stop_msg = Twist()
            self._gated_cmd_vel_pub.publish(stop_msg)
            self.get_logger().warn("HOC Requested: Robot HALTED.")


def main():
    rclpy.init()
    node = CmdVelGating()
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        node.get_logger().fatal(f"{ex}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
