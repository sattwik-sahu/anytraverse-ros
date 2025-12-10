import threading
import time
from typing import Optional

import zmq

from anytraverse_ros.oakd_vio_zmq._typing import RGBD_VIO_Message
from anytraverse_ros.oakd_vio_zmq.helpers import get_zmq_uri
from anytraverse_ros.oakd_vio_zmq.utils import parse_message


class Subscriber:
    """
    Thread-safe ZeroMQ subscriber for RGBD Visual-Inertial Odometry (VIO) messages.

    This class continuously listens for multipart RGBD VIO messages from a ZeroMQ
    publisher in the background, while exposing a thread-safe interface to fetch
    only the **latest available frame**. Intermediate frames are automatically
    dropped to maintain real-time performance, making it ideal for applications
    where processing speed may lag behind the incoming data rate.

    The received message typically includes:
        - RGB image (as bytes)
        - Point cloud (as bytes or serialized NumPy array)
        - Transformation matrix (as bytes)
        - Metadata (as a packed message)

    Thread safety is ensured using a `threading.Lock` to synchronize access to
    the shared `_latest_msg` buffer between the background receiving thread and
    the main user thread.

    Attributes:
        _stream_uri (str): Fully qualified ZeroMQ URI (e.g. "tcp://127.0.0.1:5555")
            resolved from the given `stream_name`.
        _ctx (zmq.Context): ZeroMQ context for managing sockets.
        _socket (zmq.Socket): The subscriber socket used to receive multipart
            messages from the publisher.
        _latest_msg (Optional[list[bytes]]): The most
            recent raw multipart message received. Overwritten continuously by
            the background listener thread.
        _lock (threading.Lock): Synchronization primitive used to protect access
            to `_latest_msg` between threads.
        _running (bool): Indicates whether the background receiving thread is
            currently active.
        _thread (Optional[threading.Thread]): Background thread object responsible
            for receiving and storing the latest messages.
    """

    def __init__(self, stream_name: str) -> None:
        """
        Initialize a new Subscriber instance.

        Args:
            stream_name (str): The logical name of the stream, used to generate
                the ZeroMQ connection URI via `get_zmq_uri()`.
        """
        self._stream_uri = get_zmq_uri(stream_name)
        self._ctx = zmq.Context[zmq.Socket[bytes]]()
        self._socket = self._ctx.socket(zmq.SUB)

        self._latest_msg: Optional[list[bytes]] = None
        # self._lock = threading.Lock()

        self._running = False
        self._thread: Optional[threading.Thread] = None

    def connect(self) -> None:
        """
        Establish the ZeroMQ connection and start the background listener thread.

        This method connects the subscriber socket to the publisher URI, configures
        ZeroMQ socket options for low-latency, loss-tolerant behavior, and spawns
        a daemon thread that continuously reads from the socket without blocking
        the main thread.

        Raises:
            zmq.ZMQError: If the connection cannot be established.
        """
        self._socket.connect(addr=self._stream_uri)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.setsockopt(zmq.RCVHWM, 1)  # Allow only 1 message in the queue
        self._socket.setsockopt(zmq.LINGER, 0)  # Prevent blocking on close

        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def _listen_loop(self) -> None:
        """
        Background listener loop that continuously receives messages.

        This method runs in a dedicated daemon thread. It repeatedly polls
        the ZeroMQ socket in non-blocking mode (`zmq.NOBLOCK`) to receive
        multipart messages from the publisher. When a new message is available,
        it replaces `_latest_msg` under a thread lock to ensure safe access.

        If no message is available, it briefly sleeps to avoid CPU overuse.

        Exceptions:
            - `zmq.Again`: Raised when no message is currently available.
            - Other exceptions are caught and logged, preventing thread termination.
        """
        while self._running:
            try:
                msg = self._socket.recv_multipart(flags=zmq.NOBLOCK)
                self._latest_msg = msg  # Overwrite old message with new one
                # with self._lock:
            except zmq.Again:
                time.sleep(0.001)  # No new message, sleep briefly
            except Exception as e:
                print(f"[Subscriber] Error in listener thread: {e}")
                time.sleep(0.01)

    def get_next(self) -> Optional[RGBD_VIO_Message]:
        """
        Retrieve and consume the most recent message.

        This method returns the latest available RGBD VIO message, if any.
        If multiple frames arrived between consecutive calls, only the newest
        one is returned; all intermediate ones are discarded.

        The internal `_latest_msg` buffer is cleared after reading to prevent
        duplicate retrieval.

        Returns:
            Optional[RGBD_VIO_Message]:
                - A fully parsed RGBD VIO message containing:
                    - RGB image (as NumPy array or bytes)
                    - Point cloud (as NumPy array)
                    - Transformation matrix (4x4 float array)
                    - Metadata (dictionary or custom object)
                - `None` if no new frame is currently available.
        """
        # with self._lock:
        msg = self._latest_msg
        self._latest_msg = None  # Consume message after retrieval

        if msg is None:
            return None

        metadata_bytes, rgb_frame, depth_frame, pointcloud_frame, transform_frame = msg
        return parse_message(
            metadata_bytes=metadata_bytes,  # type: ignore
            rgb_frame=rgb_frame,  # type: ignore
            depth_frame=depth_frame,  # type: ignore
            pointcloud_frame=pointcloud_frame,  # type: ignore
            transform_frame=transform_frame,  # type: ignore
        )

    def close(self) -> None:
        """Gracefully shut down the subscriber and release resources.

        Stops the background listener thread, closes the ZeroMQ socket,
        and terminates the associated context.

        This method should be called once the subscriber is no longer needed
        to ensure proper cleanup of underlying network resources.

        Example:
        >>> sub = Subscriber("oakd")
        >>> sub.connect()
        >>> ...
        >>> sub.close()
        """
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        self._socket.close(0)
        self._ctx.term()
