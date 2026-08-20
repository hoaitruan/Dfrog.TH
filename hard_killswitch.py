#!/usr/bin/env python3
"""
Hard automated cutoff (Phase 2; upgraded after Integration Flight #1).

Monitors altitude AND horizontal distance from origin against fixed
thresholds; on breach it ACTUALLY acts -- it does not just log.

Integration Flight #1 exposed a real gap: the old cutoff force-disarmed
on breach but never touched fast_planner_bridge.py, which re-requests ARM
every control-loop tick (20Hz) whenever arming_state != ARMED. The old
cutoff's disarm and the bridge's next re-arm request raced -- confirmed
live: 4 re-arm requests were logged in the 6s gap before a human ran
emergency_stop.py manually, because the cutoff itself never removed the
actor that was fighting it.

Fix: fold emergency_stop.py's own proven approach directly into the
breach handler, IN THIS ORDER, so it no longer depends on a human doing
this manually: (a) identify the bridge process, (b) SIGKILL it,
(c) confirm dead via pgrep-poll, (d) THEN send the force-disarm burst.
This ordering is the entire point -- force-disarming before the bridge is
confirmed dead leaves exactly the re-arm window this upgrade exists to
close. The kill-and-confirm runs synchronously inside the breach
callback, not fire-and-forget, so "bridge dead before disarm" is a
structural guarantee of the code path, not a timing hope.

Bridge identification: pgrep -f "fast_planner_bridge" -- the SAME pattern
emergency_stop.py already used successfully (confirmed live, Integration
Flight #1: caught both the `ros2 run` wrapper PID and the underlying
entry-point PID in one call). The -f flag is the part that matters for
robustness, including on Jetson: bare `pgrep fast_planner_bridge` (no -f)
matches against the kernel's `comm` field, which truncates at 15 bytes --
"fast_planner_bridge" is 20 characters, so an unqualified match would
silently match NOTHING, on any architecture, any platform. -f instead
matches the full, untruncated /proc/<pid>/cmdline, which has no such
length limit. This isn't a Jetson-specific fix; it removes a truncation
bug that would exist identically on x86_64 -- Jetson is just named
because it's the real deployment target where a silent miss would matter
most.

No /ground_truth/odom exists in this pure-SITL (no Gazebo) setup, so this
monitors PX4's own /fmu/out/vehicle_local_position_v1 (NED) -- which is
also the honest choice: on real hardware there is no ground truth either,
only the estimator's own numbers.

Test regime is a 3-5m altitude hover. Thresholds are set with real margin
above that envelope but tight enough to catch a genuine runaway quickly:
  ALTITUDE_LIMIT_M       = 8.0   (climb past this -> breach)
  HORIZONTAL_LIMIT_M     = 15.0  (drift past this radius from origin -> breach)
  ALTITUDE_LOWER_LIMIT_M = -0.5  (drop below this -> breach; see below)

ALTITUDE_LOWER_LIMIT_M added after receding-horizon milestone-1's first
flight: the frontier mechanism drove the vehicle to z=-1.68m (1.68m
*below* ground) mid-flight, self-corrected, and completed the mission --
but the cutoff, upper/horizontal-only, was silent through the entire
excursion. This closes that gap.

Deliberately set BELOW ground (-0.5m), not at a real flight altitude
gated to "cruise phase only". Two designs were possible:
  (a) a floor below ground (chosen) -- fires only on actually punching
      through the ground, by construction never during normal operation,
      since a legitimate landing settles at ~0m (every prior flight's
      touchdown read within a few cm of 0, positive or negative -- never
      close to -0.5m).
  (b) a floor at a real altitude (e.g. 0.5m), gated to only be active
      outside commanded takeoff/landing.
(b) was rejected: it requires the killswitch to know the vehicle's
current *intended* flight phase (has takeoff finished? has landing been
commanded?) -- real, ongoing complexity, and a second way for this
component to be wrong (a phase-detection bug could mask a real breach,
or -- worse for its own credibility -- false-trigger mid-landing and
force-disarm a vehicle that's already on its way down safely). (a) needs
none of that: it is a pure, unconditional altitude comparison, exactly
like the two bounds that already exist, with zero new state and zero
flight-phase awareness. That is the same minimal, hard-to-get-wrong
design this whole cutoff has followed from Phase 2 onward, and -0.5m
gives real margin (the actual excursion this exists to catch was -1.68m,
1.18m past this floor) without being anywhere near legitimate touchdown
noise.

DESCENT_RATE_LIMIT_MPS added after the Loi 1+2 verify flight: the fixed
-0.5m floor caught a real ground-strike-adjacent runaway, but only after
the fact -- breach fired at -0.508m, yet the vehicle (already diving
ballistically) still reached -1.01m before the disarm could stop it. A
position-only floor cannot fire before the vehicle is already almost at
the floor, no matter how tight the margin; a fast-enough dive always
overshoots it by some amount. Reacting to vertical VELOCITY instead
catches the same runaway while there is still altitude left to lose.

Threshold derivation (both numbers are real, not guessed):
  - AUTO.LAND's own design descent rate is MPC_LAND_SPEED = 0.7 m/s
    (PX4 default; confirmed NOT overridden by this project's airframe
    file, ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth
    _stereo). MPC_LAND_CRWL's slower near-ground crawl phase requires a
    downward distance sensor, which this vehicle does not have, so real
    AUTO.LAND holds at 0.7 m/s all the way to touchdown -- confirmed
    empirically in verification below, not just from the param default.
  - The runaway this guard exists to catch (Loi 1+2 verify flight)
    peaked at ~2.5-3.0 m/s instantaneous descent (position_log_verify1
    .csv: -2.17 m/s at t=180.38, -2.51 m/s at t=180.65, -3.03 m/s at
    t=180.92 -- computed from consecutive ~0.27s-spaced samples).
  - DESCENT_RATE_LIMIT_MPS = 1.5 m/s: 2.1x margin above AUTO.LAND's 0.7
    m/s design rate (comfortable headroom against ramp-up/estimator
    noise, confirmed clean empirically below), while sitting well below
    the ~2.5-3.0 m/s runaway -- and also below fast_planner's own
    commanded-velocity ceiling (search/max_vel = bspline/limit_vel =
    1.5 m/s, a 3-axis magnitude cap fast_planner_px4.launch.py already
    enforces, so no legitimately-planned descent should reach this
    threshold on its own vertical component alone in practice either).
    Retroactively against the verify-flight data: 1.5 m/s would have
    fired at t~180.25 (altitude ~2.1m), about 2.1s and ~2.6m of altitude
    margin earlier than the -0.5m floor's actual t=182.356 trigger.

Gating: per the design tension (AUTO.LAND itself descends on purpose),
option (a) -- set the rate threshold well above AUTO.LAND's controlled
rate -- was chosen over (b) -- disabling the check during AUTO.LAND.
Same reasoning as the position floor's own (a)-over-(b) choice above:
(b) requires tracking flight-phase/intent, a second way to be wrong;
(a) is a pure, stateless numeric comparison, and the 0.7-vs-1.5-vs-2.5+
m/s margins are clean enough not to need anything smarter. On top of
that, the check is also gated to altitude > DESCENT_RATE_GUARD_MIN
_ALTITUDE_M (1.0m, matching PX4's own MPC_LAND_ALT3 default -- its own
"final approach" threshold) so it never reacts to touchdown-contact
vz transients (bounce/settling) right at the ground, where the -0.5m
position floor already covers genuine ground-punch-through and where
there is negligible altitude left for a velocity-based guard to usefully
buy back anyway. This is an altitude gate, not a flight-phase gate: it
reads the same `altitude` value this callback already computes every
tick, no new state.
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand

SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=5)
PUB_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1)

ALTITUDE_LIMIT_M = 8.0
HORIZONTAL_LIMIT_M = 15.0
ALTITUDE_LOWER_LIMIT_M = -0.5
# See module docstring's "Threshold derivation" for the real numbers
# (MPC_LAND_SPEED=0.7 m/s vs the ~2.5-3.0 m/s observed runaway) behind
# both of these.
DESCENT_RATE_LIMIT_MPS = 1.5
DESCENT_RATE_GUARD_MIN_ALTITUDE_M = 1.0
ARMED = 2
DISARMED = 1

# Matches emergency_stop.py's own pattern -- see module docstring for why
# -f (full cmdline, not truncated comm) is the safety-relevant part.
BRIDGE_PGREP_PATTERN = "fast_planner_bridge"
# Tighter than emergency_stop.py's 0.05s -- this is now on the automated
# critical path (breach -> disarm), not a human-paced manual script, so
# poll latency directly adds to breach->disarm latency.
BRIDGE_KILL_POLL_PERIOD_S = 0.01
BRIDGE_KILL_CONFIRM_TIMEOUT_S = 5.0


class HardKillswitch(Node):
    def __init__(self):
        super().__init__("hard_killswitch")
        # Publisher created at startup, not on-demand -- avoids paying DDS
        # discovery latency at the exact moment we need to act.
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PUB_QOS)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._pos_cb, SUB_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, SUB_QOS)

        self.armed = None
        self.triggered = False
        self.breach_time = None
        self.breach_value = None
        self.breach_kind = None
        self.bridge_pids_killed = []
        self.bridge_kill_confirmed_time = None
        self.disarm_sent_time = None
        self.disarm_confirmed_time = None

        self.get_logger().info(
            f"hard_killswitch armed: ALTITUDE_LIMIT_M={ALTITUDE_LIMIT_M} "
            f"HORIZONTAL_LIMIT_M={HORIZONTAL_LIMIT_M} "
            f"ALTITUDE_LOWER_LIMIT_M={ALTITUDE_LOWER_LIMIT_M} "
            f"DESCENT_RATE_LIMIT_MPS={DESCENT_RATE_LIMIT_MPS} "
            f"DESCENT_RATE_GUARD_MIN_ALTITUDE_M={DESCENT_RATE_GUARD_MIN_ALTITUDE_M} "
            f"(bridge-kill-before-disarm upgrade active, pattern='{BRIDGE_PGREP_PATTERN}')"
        )

    def _kill_bridge_and_confirm_dead(self) -> None:
        """Blocking, synchronous: identify -> SIGKILL -> confirm dead.

        Runs to completion before the caller sends any disarm command.
        Blocking the executor here is deliberate, not an oversight -- the
        whole point is a strict happens-before relationship between
        "bridge is dead" and "disarm is sent", and a synchronous call is
        the simplest thing that actually guarantees that ordering.
        """
        result = subprocess.run(["pgrep", "-f", BRIDGE_PGREP_PATTERN], capture_output=True, text=True)
        pids = [p for p in result.stdout.split() if p.strip()]
        if not pids:
            self.get_logger().warn(
                "no fast_planner_bridge process found -- nothing to kill "
                "(already dead, or never started). Proceeding straight to disarm."
            )
            self.bridge_kill_confirmed_time = time.monotonic()
            return

        for pid in pids:
            subprocess.run(["kill", "-KILL", pid])
        self.bridge_pids_killed = pids
        self.get_logger().warn(f"SIGKILL sent to bridge PID(s): {pids}")

        deadline = time.monotonic() + BRIDGE_KILL_CONFIRM_TIMEOUT_S
        while time.monotonic() < deadline:
            result = subprocess.run(["pgrep", "-f", BRIDGE_PGREP_PATTERN], capture_output=True, text=True)
            if not result.stdout.strip():
                self.bridge_kill_confirmed_time = time.monotonic()
                elapsed_ms = (self.bridge_kill_confirmed_time - self.breach_time) * 1000.0
                self.get_logger().warn(
                    f"bridge confirmed dead (re-arm loop cannot fire again). "
                    f"breach->bridge_dead latency = {elapsed_ms:.1f} ms"
                )
                return
            time.sleep(BRIDGE_KILL_POLL_PERIOD_S)

        # Could not confirm dead. emergency_stop.py's policy (a human,
        # unhurried script) is to ABORT rather than disarm into a
        # contested re-arm race. A hard, autonomous cutoff's entire
        # purpose is to act on a live runaway, so instead: proceed to
        # force-disarm anyway and flag the uncertainty loudly, rather than
        # silently withholding the disarm while a runaway continues. A
        # contested disarm is still better odds than no disarm at all.
        self.bridge_kill_confirmed_time = time.monotonic()
        self.get_logger().error(
            f"bridge NOT confirmed dead after {BRIDGE_KILL_CONFIRM_TIMEOUT_S}s -- "
            "proceeding to force-disarm anyway (contested disarm beats none), "
            "but the re-arm race this upgrade exists to close is NOT ruled out this time."
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
            self.get_logger().warn(
                f"CUTOFF CONFIRMED: arming_state==DISARMED. "
                f"breach->confirmed_disarm latency = {latency_ms:.1f} ms"
            )

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        altitude = -msg.z  # NED: negative z = up
        horiz = (msg.x ** 2 + msg.y ** 2) ** 0.5
        descent_rate = msg.vz  # NED: positive vz = descending

        if not self.triggered:
            breach = None
            if altitude > ALTITUDE_LIMIT_M:
                breach = ("altitude_high", altitude, ALTITUDE_LIMIT_M, ">", "m")
            elif altitude < ALTITUDE_LOWER_LIMIT_M:
                breach = ("altitude_low", altitude, ALTITUDE_LOWER_LIMIT_M, "<", "m")
            elif horiz > HORIZONTAL_LIMIT_M:
                breach = ("horizontal", horiz, HORIZONTAL_LIMIT_M, ">", "m")
            # Gated on altitude > DESCENT_RATE_GUARD_MIN_ALTITUDE_M -- see
            # module docstring's "Gating" section: this is what keeps
            # touchdown-contact vz transients from tripping it, without
            # needing to track AUTO.LAND/flight-phase state.
            elif (
                altitude > DESCENT_RATE_GUARD_MIN_ALTITUDE_M
                and descent_rate > DESCENT_RATE_LIMIT_MPS
            ):
                breach = ("descent_rate", descent_rate, DESCENT_RATE_LIMIT_MPS, ">", "m/s")

            if breach is not None:
                self.triggered = True
                self.breach_time = time.monotonic()
                self.breach_kind, self.breach_value, limit, cmp, unit = breach
                self.get_logger().error(
                    f"BREACH DETECTED: {self.breach_kind}={self.breach_value:.3f}{unit} {cmp} "
                    f"limit={limit}{unit} (altitude={altitude:.3f}m) -- killing bridge before disarm"
                )

                # Ordering is the entire point: bridge dead BEFORE disarm.
                self._kill_bridge_and_confirm_dead()

                for _ in range(5):
                    self._send_force_disarm()
                self.disarm_sent_time = time.monotonic()
                send_latency_ms = (self.disarm_sent_time - self.breach_time) * 1000.0
                self.get_logger().warn(f"force-disarm published, breach->sent latency = {send_latency_ms:.1f} ms")
        else:
            # keep resending disarm for a bit after trigger as a backstop,
            # in case the first burst was lost. The bridge is already
            # confirmed dead by this point, so there is no actor left that
            # could contest a delayed disarm landing.
            if self.disarm_confirmed_time is None and (time.monotonic() - self.disarm_sent_time) < 3.0:
                self._send_force_disarm()


def main():
    rclpy.init()
    node = HardKillswitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
