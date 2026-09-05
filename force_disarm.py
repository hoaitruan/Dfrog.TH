#!/usr/bin/env python3
"""
Sends PX4's force-disarm command (VEHICLE_CMD_COMPONENT_ARM_DISARM,
param1=0.0 disarm, param2=21196.0 force-magic) so the vehicle can be
disarmed while still airborne/hovering, since this project has no
autoland/RTL step -- a normal disarm request is refused by PX4 while
flying. Used to end a supervised test flight cleanly after the planned
trajectory completes and holds.

BLOCKS until arming_state==DISARMED is actually confirmed via
/fmu/out/vehicle_status_v4 (retrying the disarm command every
RETRY_INTERVAL_S if it hasn't taken effect yet), up to TIMEOUT_S. Exits
0 once confirmed, exits 1 if the timeout is hit without confirmation --
this was previously fire-and-forget (publish 3x, sleep, exit 0
unconditionally), which let a run_gate.sh cleanup report success even
when the disarm command hadn't actually landed yet (root cause of the
false-alarm halt in the G3.3 gate sweep, run 5/100 -- see
flight_test_log.html). Confirming here, not just detecting the failure
downstream, is the actual fix.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus

PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Matches the QoS fast_planner_bridge.py already uses to subscribe to
# /fmu/out/vehicle_status_v4 -- PX4's uXRCE-DDS output topics are
# best-effort, a RELIABLE subscriber would never match.
SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

TIMEOUT_S = 10.0
RETRY_INTERVAL_S = 1.5
POLL_TIMEOUT_S = 0.2


def send_disarm(node, pub):
    msg = VehicleCommand()
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    msg.param1 = 0.0
    msg.param2 = 21196.0
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    for _ in range(3):
        pub.publish(msg)
        time.sleep(0.2)


def main() -> int:
    rclpy.init()
    node = Node("force_disarm")
    pub = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PUB_QOS)

    last_arming_state = {"value": None}

    def status_cb(msg: VehicleStatus) -> None:
        last_arming_state["value"] = msg.arming_state

    node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", status_cb, SUB_QOS)
    time.sleep(1.0)  # let publisher/subscriber match before sending

    send_disarm(node, pub)

    t_start = time.time()
    t_last_retry = t_start
    confirmed = False
    while time.time() - t_start < TIMEOUT_S:
        rclpy.spin_once(node, timeout_sec=POLL_TIMEOUT_S)
        if last_arming_state["value"] == VehicleStatus.ARMING_STATE_DISARMED:
            confirmed = True
            break
        if time.time() - t_last_retry >= RETRY_INTERVAL_S:
            node.get_logger().warn(
                f"still not confirmed disarmed (last arming_state={last_arming_state['value']}) "
                f"-- resending disarm command"
            )
            send_disarm(node, pub)
            t_last_retry = time.time()

    elapsed = time.time() - t_start
    if confirmed:
        node.get_logger().info(f"force-disarm CONFIRMED (arming_state=DISARMED) after {elapsed:.1f}s")
    else:
        node.get_logger().error(
            f"force-disarm NOT CONFIRMED after {elapsed:.1f}s "
            f"(last arming_state={last_arming_state['value']}, "
            f"None means vehicle_status_v4 was never received)"
        )

    node.destroy_node()
    rclpy.shutdown()
    return 0 if confirmed else 1


if __name__ == "__main__":
    sys.exit(main())
