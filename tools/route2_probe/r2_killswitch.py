#!/usr/bin/env python3
"""
Route 2 closed-loop probe safety watchdog. This is the FIRST project
script that runs while EKF2_EV_CTRL is nonzero -- a genuine runaway
(estimator diverges, PX4 chases the phantom error with real thrust, per
the original Phase 6 incident) is possible by construction, since that
is exactly the phenomenon under test.

Extends hard_killswitch.py's proven, incident-tested design (kept
byte-identical: same thresholds, same kill-bridge-then-disarm ordering
and confirmation logic -- see that file's own docstring for the full
rationale) with ONE new trigger this probe specifically needs:
divergence between the EKF's fused position estimate and Gazebo ground
truth. hard_killswitch.py alone only catches the estimate running away in
absolute terms (altitude/horizontal-from-origin/descent-rate); it cannot
detect the estimate disagreeing with reality while both still look like
"reasonable" numbers in isolation -- which is exactly the Phase 6 failure
mode (EKF2 fusing a drifted cuVSLAM pose that read as a plausible
position, not an obviously insane one).

Frame note: /fmu/out/vehicle_local_position_v1 is NED (PX4 native);
/ground_truth/odom is ENU (see ground_truth_tf.py). Conversion used here
(NED_x=ENU_y, NED_y=ENU_x, NED_z=-ENU_z), origin-aligned at the first
ground-truth sample, was validated live before this file was trusted:
residual stayed ~0.03-0.06m flat across a real flight with 3.6m of
lateral and 4.7m of vertical displacement (would have grown with
displacement had the axes been swapped or sign-flipped) -- see
flight_test_log.html, Route 2 setup entry.

On ANY breach (geofence, as before, OR new: divergence), this is DATA,
not merely a failure to suppress: writes a one-line JSON breach record to
--breach-file (if given) BEFORE acting, so the calling batch script can
label the run "runaway_captured" rather than just seeing a disarm.
"""
import argparse
import json
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand
from nav_msgs.msg import Odometry

SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=5)
PUB_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1)

# Unchanged from hard_killswitch.py -- see its own docstring for the
# real-incident derivation of every one of these numbers.
ALTITUDE_LIMIT_M = 8.0
HORIZONTAL_LIMIT_M = 15.0
ALTITUDE_LOWER_LIMIT_M = -0.5
DESCENT_RATE_LIMIT_MPS = 1.5
DESCENT_RATE_GUARD_MIN_ALTITUDE_M = 1.0
ARMED = 2
DISARMED = 1

# NEW for Route 2: est-vs-ground-truth horizontal divergence. 10m per the
# task spec's own example threshold -- comfortably inside
# HORIZONTAL_LIMIT_M=15m (so a genuine large divergence trips THIS check
# first, giving a more specific "divergence" label rather than only ever
# seeing the coarser absolute-position geofence fire).
DIVERGENCE_LIMIT_M = 10.0

BRIDGE_PGREP_PATTERN = "fast_planner_bridge"
BRIDGE_KILL_POLL_PERIOD_S = 0.01
BRIDGE_KILL_CONFIRM_TIMEOUT_S = 5.0


class R2Killswitch(Node):
    def __init__(self, breach_file):
        super().__init__("r2_killswitch")
        self.breach_file = breach_file
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PUB_QOS)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._pos_cb, SUB_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, SUB_QOS)
        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_cb, SUB_QOS)

        self.armed = None
        self.triggered = False
        self.breach_time = None
        self.disarm_sent_time = None
        self.disarm_confirmed_time = None

        self.gt = None       # latest (x,y,z) ENU
        self.gt0 = None      # ENU position at first sample (origin)

        self.get_logger().info(
            f"r2_killswitch armed: ALTITUDE_LIMIT_M={ALTITUDE_LIMIT_M} "
            f"HORIZONTAL_LIMIT_M={HORIZONTAL_LIMIT_M} "
            f"ALTITUDE_LOWER_LIMIT_M={ALTITUDE_LOWER_LIMIT_M} "
            f"DESCENT_RATE_LIMIT_MPS={DESCENT_RATE_LIMIT_MPS} "
            f"DIVERGENCE_LIMIT_M={DIVERGENCE_LIMIT_M} "
            f"(bridge-kill-before-disarm, pattern='{BRIDGE_PGREP_PATTERN}', breach_file={breach_file})"
        )

    def _gt_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.gt = (p.x, p.y, p.z)
        if self.gt0 is None:
            self.gt0 = self.gt

    def _divergence_m(self, est_x, est_y):
        if self.gt is None or self.gt0 is None:
            return None
        gx, gy, _ = self.gt
        gx0, gy0, _ = self.gt0
        gt_ned_x = gy - gy0
        gt_ned_y = gx - gx0
        return ((est_x - gt_ned_x) ** 2 + (est_y - gt_ned_y) ** 2) ** 0.5

    def _kill_bridge_and_confirm_dead(self) -> None:
        result = subprocess.run(["pgrep", "-f", BRIDGE_PGREP_PATTERN], capture_output=True, text=True)
        pids = [p for p in result.stdout.split() if p.strip()]
        if not pids:
            self.get_logger().warn("no fast_planner_bridge process found -- proceeding straight to disarm.")
            return
        for pid in pids:
            subprocess.run(["kill", "-KILL", pid])
        self.get_logger().warn(f"SIGKILL sent to bridge PID(s): {pids}")
        deadline = time.monotonic() + BRIDGE_KILL_CONFIRM_TIMEOUT_S
        while time.monotonic() < deadline:
            result = subprocess.run(["pgrep", "-f", BRIDGE_PGREP_PATTERN], capture_output=True, text=True)
            if not result.stdout.strip():
                elapsed_ms = (time.monotonic() - self.breach_time) * 1000.0
                self.get_logger().warn(f"bridge confirmed dead. breach->bridge_dead latency = {elapsed_ms:.1f} ms")
                return
            time.sleep(BRIDGE_KILL_POLL_PERIOD_S)
        self.get_logger().error(
            f"bridge NOT confirmed dead after {BRIDGE_KILL_CONFIRM_TIMEOUT_S}s -- "
            "proceeding to force-disarm anyway (contested disarm beats none)."
        )

    def _send_force_disarm(self):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0
        msg.param2 = 21196.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _status_cb(self, msg: VehicleStatus) -> None:
        prev = self.armed
        self.armed = msg.arming_state
        if self.triggered and prev != DISARMED and self.armed == DISARMED and self.disarm_confirmed_time is None:
            self.disarm_confirmed_time = time.monotonic()
            latency_ms = (self.disarm_confirmed_time - self.breach_time) * 1000.0
            self.get_logger().warn(f"CUTOFF CONFIRMED: arming_state==DISARMED. latency = {latency_ms:.1f} ms")

    def _write_breach_record(self, breach_kind, breach_value, limit, unit, altitude):
        if not self.breach_file:
            return
        record = {
            "event": "runaway_captured",
            "breach_kind": breach_kind,
            "breach_value": breach_value,
            "limit": limit,
            "unit": unit,
            "altitude_m": altitude,
            "t_monotonic": self.breach_time,
        }
        try:
            with open(self.breach_file, "w") as f:
                json.dump(record, f)
        except OSError as e:
            self.get_logger().error(f"could not write breach file {self.breach_file}: {e}")

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        altitude = -msg.z
        horiz = (msg.x ** 2 + msg.y ** 2) ** 0.5
        descent_rate = msg.vz
        divergence = self._divergence_m(msg.x, msg.y)

        if not self.triggered:
            breach = None
            if altitude > ALTITUDE_LIMIT_M:
                breach = ("altitude_high", altitude, ALTITUDE_LIMIT_M, ">", "m")
            elif altitude < ALTITUDE_LOWER_LIMIT_M:
                breach = ("altitude_low", altitude, ALTITUDE_LOWER_LIMIT_M, "<", "m")
            elif horiz > HORIZONTAL_LIMIT_M:
                breach = ("horizontal", horiz, HORIZONTAL_LIMIT_M, ">", "m")
            elif (
                altitude > DESCENT_RATE_GUARD_MIN_ALTITUDE_M
                and descent_rate > DESCENT_RATE_LIMIT_MPS
            ):
                breach = ("descent_rate", descent_rate, DESCENT_RATE_LIMIT_MPS, ">", "m/s")
            elif divergence is not None and divergence > DIVERGENCE_LIMIT_M:
                breach = ("divergence", divergence, DIVERGENCE_LIMIT_M, ">", "m")

            if breach is not None:
                self.triggered = True
                self.breach_time = time.monotonic()
                breach_kind, breach_value, limit, cmp, unit = breach
                self.get_logger().error(
                    f"BREACH DETECTED: {breach_kind}={breach_value:.3f}{unit} {cmp} "
                    f"limit={limit}{unit} (altitude={altitude:.3f}m, divergence={divergence}) -- "
                    f"killing bridge before disarm"
                )
                self._write_breach_record(breach_kind, breach_value, limit, unit, altitude)
                self._kill_bridge_and_confirm_dead()
                for _ in range(5):
                    self._send_force_disarm()
                self.disarm_sent_time = time.monotonic()
                send_latency_ms = (self.disarm_sent_time - self.breach_time) * 1000.0
                self.get_logger().warn(f"force-disarm published, breach->sent latency = {send_latency_ms:.1f} ms")
        else:
            if self.disarm_confirmed_time is None and (time.monotonic() - self.disarm_sent_time) < 3.0:
                self._send_force_disarm()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--breach-file", default=None, help="Path to write a one-line JSON breach record to, if triggered.")
    args = ap.parse_args()

    rclpy.init()
    node = R2Killswitch(args.breach_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
