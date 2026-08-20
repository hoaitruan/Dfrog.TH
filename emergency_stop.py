#!/usr/bin/env python3
"""
Emergency stop -- last resort, for when fast_planner_bridge is
unresponsive or the graceful stop (SIGINT/SIGTERM to the bridge -> it
requests AUTO.LAND -> disarms itself) isn't appropriate or hasn't worked.
This is motors-off, not a graceful land: the vehicle drops from whatever
altitude it's at when this runs.

Ordering is the safety-critical part, not an afterthought. fast_planner_
bridge.py re-requests ARM every control-loop tick (20Hz, ~50ms period)
whenever arming_state != ARMED (fast_planner_bridge.py, _control_loop's
arm-request branch). An external disarm command sent while the bridge is
still alive races that loop -- confirmed live, this is exactly what
caused the 26m excursion incident: the disarm and the bridge's next
_arm() call landed within ~50ms of each other and arming_state flapped
mid-flight.

SIGKILL is used here instead of SIGTERM specifically because SIGKILL
cannot be intercepted by any signal handler -- including the graceful-
stop handler now installed in the bridge for the normal case -- and
terminates the process at the OS level, atomically. There is no window
in which the bridge's timer can fire one more time after SIGKILL is
delivered. This script waits for CONFIRMED process death (polling
`pgrep`, not just "sent the signal and moved on") before sending the
force-disarm command, so there is no race window at all: not "probably
fast enough", but verified-impossible for the bridge to still be running
when the disarm lands.

Usage (inside the container, as admin):
  python3 /workspaces/isaac_ros-dev/emergency_stop.py
"""
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand

# Matches fast_planner_bridge.py's own publisher QoS for /fmu/in/vehicle_command
# (RELIABLE/VOLATILE) -- confirmed this is what PX4's subscriber actually
# expects; an earlier attempt with BEST_EFFORT/TRANSIENT_LOCAL silently
# failed to take effect.
PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

BRIDGE_KILL_CONFIRM_TIMEOUT_S = 5.0


def kill_bridge_and_confirm_dead() -> None:
    result = subprocess.run(["pgrep", "-f", "fast_planner_bridge"], capture_output=True, text=True)
    pids = [p for p in result.stdout.split() if p.strip()]
    if not pids:
        print("no fast_planner_bridge process found -- nothing to kill, proceeding to disarm")
        return

    for pid in pids:
        print(f"SIGKILL -> pid {pid}")
        subprocess.run(["kill", "-KILL", pid])

    deadline = time.time() + BRIDGE_KILL_CONFIRM_TIMEOUT_S
    while time.time() < deadline:
        result = subprocess.run(["pgrep", "-f", "fast_planner_bridge"], capture_output=True, text=True)
        if not result.stdout.strip():
            print("confirmed: fast_planner_bridge process(es) are gone (re-arm loop cannot fire again)")
            return
        time.sleep(0.05)

    print(
        f"ABORT: fast_planner_bridge still alive after {BRIDGE_KILL_CONFIRM_TIMEOUT_S}s "
        "of SIGKILL -- refusing to send force-disarm while the re-arm loop might still "
        "be running. This should not be possible (SIGKILL is uncatchable) -- investigate."
    )
    sys.exit(1)


def force_disarm() -> None:
    rclpy.init()
    node = Node("emergency_stop")
    pub = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PUB_QOS)
    time.sleep(1.0)  # let the publisher match before sending

    msg = VehicleCommand()
    msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    msg.param1 = 0.0       # ARMING_ACTION_DISARM
    msg.param2 = 21196.0   # PX4's force-arm/disarm magic number (Commander.cpp:1012)
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True

    for _ in range(5):
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        pub.publish(msg)
        time.sleep(0.1)

    node.get_logger().info("force-disarm sent (bridge confirmed dead first -- no re-arm race possible)")
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    print("=== EMERGENCY STOP: SIGKILL bridge -> confirm dead -> force-disarm ===")
    print("This is motors-off, not a graceful land. The vehicle will drop.")
    kill_bridge_and_confirm_dead()
    force_disarm()
    print("emergency_stop.py complete -- verify arming_state externally.")


if __name__ == "__main__":
    main()
