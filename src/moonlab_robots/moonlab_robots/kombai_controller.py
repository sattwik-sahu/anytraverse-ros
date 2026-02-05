import zmq
from abc import ABC, abstractmethod
from typing_extensions import override
import json
import time
from typing import TypedDict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class RobotCommand(TypedDict):
    velocity: tuple[float, float]
    yaw_speed: float


class ZMQPublisher[TMessage](ABC):
    """
    A class-based ZeroMQ Publisher using PUB socket.

    Attributes:
        address (str): The IPC or TCP address to bind to.
        context (zmq.Context): The ZeroMQ context.
        socket (zmq.Socket): The PUB socket.
    """

    def __init__(self, address: str = "tcp:") -> None:
        """
        Initializes the ZMQPublisher.

        Args:
            address (str): Address to bind the publisher to.
        """
        self.address = "tcp://*:5555"
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 10)  # Optional: High-water mark for queuing
        self.socket.bind(self.address)
        time.sleep(0.5)

    @abstractmethod
    def serialize_message(self, message: TMessage) -> str:
        pass

    def send(self, topic: str, message: TMessage) -> None:
        """
        Sends a message with a topic prefix.

        Args:
            topic (str): Topic string used by subscribers to filter.
            message (str): The message to send.
        """
        serialized_message = self.serialize_message(message=message)
        full_msg = f"{topic} {serialized_message}"
        print(f"[SOCKET] > {full_msg}")
        self.socket.send_string(full_msg)

    def close(self) -> None:
        """Closes the socket and terminates the context."""
        self.socket.close()
        self.context.term()


class UnitreeZMQPublisher(ZMQPublisher[RobotCommand]):
    def __init__(self) -> None:
        super().__init__(address="anytraverse")

    @override
    def serialize_message(self, message: RobotCommand) -> str:
        return json.dumps(message)


class KombaiControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("kombai_controller_node")

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._subscription = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self._cmd_vel_callback,
            qos_profile=qos_profile,
        )
        self._subscription  # prevent unused variable warning

        self._publisher = UnitreeZMQPublisher()
        self.get_logger().info("Kombai Controller Node has been started.")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        command: RobotCommand = {
            "velocity": (msg.linear.x, msg.linear.y),
            "yaw_speed": msg.angular.z,
        }
        self.get_logger().info(f"Sending command: {command}")
        self._publisher.send(topic="cmd_vel", message=command)

    def destroy_node(self) -> None:
        self._publisher.close()
        super().destroy_node()


def main():
    rclpy.init()
    kombai_controller_node = KombaiControllerNode()
    try:
        rclpy.spin(node=kombai_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        kombai_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
