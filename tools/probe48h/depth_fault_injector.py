#!/usr/bin/env python3
"""
H3: depth "black-hole" fault injector -- topic-level only, no cuVSLAM/
nvblox core touched.

Subscribes the raw depth topic (/depth_camera by default, see
docs/probe48h/PHASE0.md 0a), republishes on a NEW topic
(/depth_camera/faulted by default). For Corrupted-condition runs, point
nvblox.launch.py's remap target at the faulted topic instead of the raw
one (a launch-arg/config change, not a code change) -- cuVSLAM keeps
reading its own separate stereo input, unaffected either way.

Only 32FC1 (float32 meters, Gazebo's standard depth-camera encoding) is
supported -- corrupting bytes of an unrecognized encoding without
checking would silently produce garbage, so an unexpected encoding logs
an error and passes the frame through unmodified rather than guessing.

Fault: a square block of NaN depth values in one corner, toggleable and
positionable.

Usage: depth_fault_injector.py [--enabled] [--corner top-left|top-right|
       bottom-left|bottom-right] [--block-frac 0.25]
       [--in /depth_camera] [--out /depth_camera/faulted]
"""
import argparse
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
NAN_F32 = struct.pack("<f", float("nan"))


class DepthFaultInjector(Node):
    def __init__(self, args) -> None:
        super().__init__("depth_fault_injector")
        self.enabled = args.enabled
        self.corner = args.corner
        self.block_frac = args.block_frac
        self.pub = self.create_publisher(Image, args.out, SENSOR_QOS)
        self.create_subscription(Image, args.in_topic, self._cb, SENSOR_QOS)
        self.warned_encoding = False

    def _cb(self, msg: Image) -> None:
        if not self.enabled or msg.encoding != "32FC1":
            if msg.encoding != "32FC1" and not self.warned_encoding:
                self.get_logger().error(
                    f"unsupported encoding '{msg.encoding}' (only 32FC1 handled) "
                    f"-- passing frames through unmodified"
                )
                self.warned_encoding = True
            self.pub.publish(msg)
            return

        data = bytearray(msg.data)
        step = msg.step
        w, h = msg.width, msg.height
        bw = max(1, int(w * self.block_frac))
        bh = max(1, int(h * self.block_frac))
        x0 = 0 if "left" in self.corner else w - bw
        y0 = 0 if "top" in self.corner else h - bh

        for row in range(y0, y0 + bh):
            row_off = row * step
            for col in range(x0, x0 + bw):
                off = row_off + col * 4
                data[off:off + 4] = NAN_F32

        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = bytes(data)
        self.pub.publish(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--enabled", action="store_true")
    ap.add_argument("--corner", default="top-left",
                     choices=["top-left", "top-right", "bottom-left", "bottom-right"])
    ap.add_argument("--block-frac", type=float, default=0.25)
    ap.add_argument("--in", dest="in_topic", default="/depth_camera")
    ap.add_argument("--out", default="/depth_camera/faulted")
    args = ap.parse_args()

    rclpy.init()
    node = DepthFaultInjector(args)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
