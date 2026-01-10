import asyncio
import threading
from threading import Thread

import cv2
import rclpy
import torch
import websockets
from anytraverse import build_pipeline_from_paper
from anytraverse.utils.state import TraversalState
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Header, String


class AnyTraverseNode(Node):
    """The AnyTraverse node."""

    def __init__(self) -> None:
        super().__init__(node_name="anytraverse_node", namespace="/anytraverse")

        # Fast QoS
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Build the AnyTraverse pipeline
        self.get_logger().info("Building AnyTraverse pipeline")
        self._anytraverse = build_pipeline_from_paper(
            init_traversabilty_preferences={"grass": 1.0},
            ref_scene_similarity_threshold=0.80,
            roi_uncertainty_threshold=0.3,
            roi_x_bounds=(0.25, 0.75),
            roi_y_bounds=(0.67, 0.95),
        )

        # CV bridge
        self._bridge = CvBridge()

        # Subscribe to the image topic
        self._image_sub = self.create_subscription(
            msg_type=CompressedImage,
            topic="/camera/color/image_raw/compressed",
            qos_profile=qos_profile,
            callback=self._image_callback,
        )

        # Publishers for the AnyTraverse output topics
        self._trav_map_pub = self.create_publisher(
            msg_type=CompressedImage,
            topic="/anytraverse/trav_map",
            qos_profile=qos_profile,
        )
        self._unc_map_pub = self.create_publisher(
            msg_type=CompressedImage,
            topic="/anytraverse/unc_map",
            qos_profile=qos_profile,
        )
        self._roi_trav_pub = self.create_publisher(
            msg_type=Float32, topic="/anytraverse/roi/trav", qos_profile=qos_profile
        )
        self._roi_unc_pub = self.create_publisher(
            msg_type=Float32, topic="/anytraverse/roi/unc", qos_profile=qos_profile
        )
        self._trav_state_pub = self.create_publisher(
            msg_type=String, topic="/anytraverse/state", qos_profile=qos_profile
        )
        self._hoc_req_pub = self.create_publisher(
            msg_type=Bool, topic="/anytraverse/hoc_req", qos_profile=qos_profile
        )
        self._trav_pref_pub = self.create_publisher(
            msg_type=String, topic="/anytraverse/trav_pref", qos_profile=qos_profile
        )

        # Image message buffer for performance boost
        self._latest_msg = None
        self._busy = False
        self._lock = threading.Lock()

    def _image_callback(self, msg):
        img = self._bridge.compressed_imgmsg_to_cv2(msg)

        with self._lock:
            self._latest_msg = (img, msg.header)

        if not self._busy:
            self._run_inference_async()

    def _run_inference_async(self):
        self._busy = True
        Thread(target=self._inference_worker, daemon=True).start()

    def _inference_worker(self):
        while True:
            with self._lock:
                if self._latest_msg is None:
                    self._busy = False
                    return  # No more frames to process

                img, header = self._latest_msg
                self._latest_msg = None

            # --- RUN INFERENCE ---
            with torch.inference_mode():
                output = self._anytraverse.step(PILImage.fromarray(img))

            # --- PUBLISH ---
            self._publish(
                trav_map=output.traversability_map,
                unc_map=output.uncertainty_map,
                roi_trav=output.roi_traversability,
                roi_unc=output.roi_uncertainty,
                trav_state=output.traversal_state.name,
                hoc_req=output.traversal_state is not TraversalState.OK,
                incoming_header=header,
            )

    def _publish(
        self,
        trav_map: torch.Tensor,
        unc_map: torch.Tensor,
        roi_trav: float,
        roi_unc: float,
        trav_state: str,
        hoc_req: bool,
        incoming_header: Header,
    ) -> None:
        def to_uint8_img(t):
            img = t.detach().cpu().numpy()
            if img.ndim == 2:
                img = (img * 255).clip(0, 255).astype("uint8")
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            elif img.ndim == 3 and img.shape[0] in (1, 3):
                img = img.transpose(1, 2, 0)
                img = (img * 255).clip(0, 255).astype("uint8")
            else:
                img = (img * 255).clip(0, 255).astype("uint8")
            return img

        # Convert traversability and uncertainty maps to numpy
        trav_np = to_uint8_img(trav_map)
        unc_np = to_uint8_img(unc_map)

        # Compress the traversability and uncertainty maps
        trav_map_msg = self._bridge.cv2_to_compressed_imgmsg(trav_np, dst_format="jpg")
        unc_map_msg = self._bridge.cv2_to_compressed_imgmsg(unc_np, dst_format="jpg")
        trav_map_msg.header = incoming_header
        unc_map_msg.header = incoming_header

        # Publish maps
        self._trav_map_pub.publish(trav_map_msg)
        self._unc_map_pub.publish(unc_map_msg)

        # ROI specific statistics
        # 1. Traversability in ROI
        roi_trav_msg = Float32()
        roi_trav_msg.data = roi_trav
        self._roi_trav_pub.publish(roi_trav_msg)

        # 2. Uncertainty in ROI
        roi_unc_msg = Float32()
        roi_unc_msg.data = roi_unc
        self._roi_unc_pub.publish(roi_unc_msg)

        # Traversal state (unseen scene/unknown object/ok)
        trav_state_msg = String()
        trav_state_msg.data = trav_state
        self._trav_state_pub.publish(trav_state_msg)

        # Is a human operator call required? (if state not ok)
        hoc_req_msg = Bool()
        hoc_req_msg.data = hoc_req
        self._hoc_req_pub.publish(hoc_req_msg)

        # Publish the traversability preferences
        # These are updated through human operator calls and memory hits
        trav_pref_msg = String()
        trav_pref_msg.data = str(self._anytraverse.traversability_preferences)
        self._trav_pref_pub.publish(trav_pref_msg)


async def websocket_handler(node: AnyTraverseNode, websocket):
    async for message in websocket:
        if message == "ok":
            # node._anytraverse.human_call(human_input="")
            node._anytraverse.register_scene()
        else:
            try:
                node._anytraverse.human_call(human_input=message)
            except Exception:
                print("Invalid syntax for human operator call.")


async def websocket_main(node):
    async with websockets.serve(
        lambda ws: websocket_handler(node, ws), "0.0.0.0", 7777
    ):
        # run forever
        await asyncio.Future()


def main():
    rclpy.init()
    node = AnyTraverseNode()

    # Run WebSocket server in a background event loop thread
    def ws_thread_func():
        asyncio.run(websocket_main(node))

    ws_thread = threading.Thread(target=ws_thread_func, daemon=True)
    ws_thread.start()

    # Run ROS2 normally
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
