"""Camera frames for the page: the latest gz.msgs.Image of each camera,
subscribed only while someone watches it (Gazebo renders a camera only while
its topic has subscribers), JPEG-encoded once per frame for every viewer."""
import asyncio
import threading
import time
from collections import deque

import cv2
import numpy as np

RGB_INT8 = 3  # gz.msgs.PixelFormatType
R_FLOAT32 = 13
JPEG_QUALITY = 80


def to_bgr(msg, depth_range):
    """A gz.msgs.Image as an 8-bit BGR picture: colour as it is, depth [m]
    coloured near (warm) to far (cool) over depth_range, no return black."""
    if msg.pixel_format_type == RGB_INT8:
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if msg.pixel_format_type == R_FLOAT32:
        depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        near, far = depth_range
        valid = np.isfinite(depth) & (depth > near) & (depth < far)
        # Log scale: a metre matters more close by than at 30 m.
        scaled = np.log(np.clip(depth, near, far) / near) / np.log(far / near)
        picture = cv2.applyColorMap((255 * (1 - scaled)).astype(np.uint8), cv2.COLORMAP_TURBO)
        picture[~valid] = 0
        return picture
    raise ValueError(f"unsupported pixel format {msg.pixel_format_type}")


class Feed:
    """One camera. on_image runs on gz-transport threads; the async side
    waits with next_frame. watch/unwatch count viewers and call
    subscribe(feed) / unsubscribe(feed) on the first and last one.

    The first picture after subscribing is dropped: the RGB-D camera's first
    colour image after a time without colour subscribers is all black
    (measured in gz-sensors 8; depth is not affected)."""

    def __init__(self, name, topic, subscribe, unsubscribe, depth_range=(0.1, 40.0)):
        self.name = name
        self.topic = topic
        self._subscribe = subscribe
        self._unsubscribe = unsubscribe
        self.depth_range = depth_range
        self.viewers = 0
        self._skip = 0
        self._lock = threading.Lock()
        self._msg = None
        self.seq = 0
        self._times = deque(maxlen=64)
        self._waiters = []  # (loop, future)
        self._jpeg = (0, None)
        self._encode_lock = threading.Lock()

    def watch(self):
        self.viewers += 1
        if self.viewers == 1:
            with self._lock:
                self._skip = 1
            self._subscribe(self)

    def unwatch(self):
        self.viewers -= 1
        if self.viewers == 0:
            self._unsubscribe(self)

    def on_image(self, msg):
        with self._lock:
            if self._skip:
                self._skip -= 1
                return
            self._msg = msg
            self.seq += 1
            self._times.append(time.monotonic())
            waiters, self._waiters = self._waiters, []
        for loop, future in waiters:
            loop.call_soon_threadsafe(_resolve, future)

    def fps(self, now=None):
        """Frames received over the last second."""
        now = time.monotonic() if now is None else now
        with self._lock:
            return sum(1 for t in self._times if now - t <= 1.0)

    async def next_frame(self, after, timeout):
        """Wait until a frame newer than seq `after` arrives; returns its seq,
        or the current seq on timeout."""
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        with self._lock:
            if self.seq > after:
                return self.seq
            self._waiters.append((loop, future))
        try:
            await asyncio.wait_for(future, timeout)
        except asyncio.TimeoutError:
            with self._lock:
                self._waiters = [w for w in self._waiters if w[1] is not future]
        return self.seq

    def jpeg(self):
        """(seq, JPEG bytes) of the latest frame, encoded once; (0, None) before the first."""
        with self._encode_lock:
            with self._lock:
                seq, msg = self.seq, self._msg
            if msg is None or self._jpeg[0] == seq:
                return self._jpeg
            ok, data = cv2.imencode(".jpg", to_bgr(msg, self.depth_range), [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                self._jpeg = (seq, data.tobytes())
            return self._jpeg


def _resolve(future):
    if not future.done():
        future.set_result(None)
