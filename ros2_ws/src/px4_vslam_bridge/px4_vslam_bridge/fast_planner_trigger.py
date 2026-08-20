#!/usr/bin/env python3
"""
Phase 7: reliably trigger Fast-Planner's KinoReplanFSM out of WAIT_TARGET.

kino_replan_fsm.cpp's waypointCallback sets trigger_=true on receipt of
any nav_msgs/Path on /waypoint_generator/waypoints, ignoring its actual
content when target_type_==PRESET_TARGET (our flight_type=2 config uses
the launch file's baked-in waypoint0_x/y/z instead) -- so this only needs
poses[0].position.z >= -0.1 to pass its one sanity check.

Waits for an actual subscriber match before publishing, unlike a plain
`ros2 topic pub --once` (a one-shot publisher that sends immediately
after creation, with no guarantee the DDS discovery handshake with
fast_planner_node's subscription has completed yet -- confirmed as the
real cause of a silent trigger failure that led to a live flight test
chasing a stale trajectory, see fast_planner_bridge.py's docstring).
"""

import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

DISCOVERY_TIMEOUT_S = 10.0


class FastPlannerTrigger(Node):
    def __init__(self) -> None:
        super().__init__("fast_planner_trigger")
        self.pub = self.create_publisher(Path, "/waypoint_generator/waypoints", 1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastPlannerTrigger()

    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while node.pub.get_subscription_count() == 0:
        if time.monotonic() > deadline:
            node.get_logger().error(
                f"No subscriber appeared on /waypoint_generator/waypoints within "
                f"{DISCOVERY_TIMEOUT_S}s -- is fast_planner_node running?"
            )
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    pose = PoseStamped()
    pose.pose.position.z = 0.0
    msg.poses = [pose]
    node.pub.publish(msg)
    node.get_logger().info(
        f"Trigger published ({node.pub.get_subscription_count()} subscriber(s) matched)."
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
