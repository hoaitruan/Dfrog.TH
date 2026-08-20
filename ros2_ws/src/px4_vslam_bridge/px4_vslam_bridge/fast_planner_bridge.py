#!/usr/bin/env python3
"""
Phase 7: bridges Fast-Planner's traj_server output into PX4 Offboard.

Same arm/offboard/climb boilerplate as reactive_esdf_avoidance.py and
minimal_offboard.py (proven pattern throughout this project). After the
climb, this node relays quadrotor_msgs/PositionCommand from
/planning/pos_cmd (published by plan_manage's traj_server, itself driven
by fast_planner_node's kinodynamic-replan + B-spline optimization) into
PX4 TrajectorySetpoint messages.

Velocity + acceleration feedforward (added after a second real flight
incident -- see kino_replan_fsm.cpp's REPLAN_TRAJ case): every replan
after the first evaluates the *previous* B-spline at the current time
(`info->position_traj_.evaluateDeBoorT(t_cur)`) as its new starting
state, NOT real vehicle odometry. This is standard for receding-horizon
planners, but it only stays valid if the real vehicle tracks the
commanded trajectory tightly -- and position-only control (this
project's established pattern through Phase 6) doesn't track tightly
enough. Real tracking error compounded across replan cycles until the
commanded trajectory diverged catastrophically from the real vehicle's
position (confirmed: altitude swung from -5m up through +0.3m, i.e. the
ground, before being stopped by disarming). Feeding PositionCommand's own
velocity/acceleration through to TrajectorySetpoint (which already
carries this data -- computed specifically for this purpose) closes that
gap, matching what Fast-Planner's own reference architecture assumes.

Independent 20Hz setpoint-streaming timer, decoupled from
/planning/pos_cmd's own publish rate: PX4 requires continuous
OffboardControlMode heartbeats or it exits Offboard, so this always
re-publishes the LAST received command rather than only on receipt --
matching reactive_esdf_avoidance.py's fast-loop/slow-update split, for
the same reason (never let an upstream node's publish cadence become a
flight-safety dependency).

Freshness check (added after a real flight incident): traj_server keeps
publishing PositionCommand at a steady rate even when fast_planner_node's
FSM has fallen back to WAIT_TARGET with no live trajectory -- it just
holds/replays whatever B-spline it last received, with no signal
distinguishing "actively tracking a fresh plan" from "stale/no plan".
During testing, a re-trigger publish (`ros2 topic pub --once`, a
one-shot publisher with no wait for subscriber discovery -- a classic
DDS race) silently failed to reach fast_planner_node, so the FSM never
left WAIT_TARGET while traj_server kept serving an old trajectory from
an earlier, unrelated test run. The bridge had no way to tell the
difference and chased the mismatched setpoint, causing a real erratic,
unsafe excursion (stopped by disarming). Now: if no PositionCommand has
arrived within POS_CMD_MAX_AGE_S, treat the target as stale and hold
current position instead of trusting it.
"""

import math
import os
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand

from px4_vslam_bridge.frame_transforms import enu_to_ned_position

# RELATIVE to home (captured at arm time), not an absolute world point --
# was CLIMB_TARGET_NED = (0.0, 0.0, -5.0), an absolute NED point. In a
# long-running container the drone can be left sitting far from true
# origin by an earlier test; an absolute climb target then becomes an
# untested large diagonal step, which is exactly what caused a real
# runaway (incident #5, ~47m and ~44m position error, force-disarmed
# twice) -- fixed at the time by restarting the whole simulation to force
# a fresh spawn at origin, not by fixing this. This is the actual
# structural fix: climb straight up 5m from wherever the drone actually
# is when it arms, regardless of where that happens to be.
#
# GROUND_TEST_ZERO_CLIMB_Z_NED (kill-path verification only, 2026-08-13):
# when set, overrides the z offset so the "climb target" equals home
# itself -- the bridge arms and holds in place, never commanding an
# altitude change, so the re-arm-fight and both kill paths can be
# exercised on the ground with no possibility of liftoff. Unset (the
# normal case) leaves the real -5.0 climb behavior untouched.
_ground_test_z = os.getenv("GROUND_TEST_ZERO_CLIMB_Z_NED")
if _ground_test_z is not None:
    CLIMB_TARGET_OFFSET_NED = (0.0, 0.0, float(_ground_test_z))
else:
    CLIMB_TARGET_OFFSET_NED = (0.0, 0.0, -5.0)
CLIMB_ARRIVAL_TOLERANCE_M = 0.3

# traj_server publishes on a steady internal timer regardless of whether
# fast_planner_node has a live plan -- generous enough to tolerate normal
# replan-cycle gaps, tight enough to catch "FSM fell back to WAIT_TARGET,
# no one is updating this anymore" within a fraction of a second.
POS_CMD_MAX_AGE_S = 0.5

# Graceful-stop kill path (added after the 26m runaway incident,
# 2026-08-13). Root cause: this node re-requests ARM every control-loop
# tick (20Hz) whenever arming_state != ARMED (see _control_loop below).
# An external disarm sent while this process is still alive loses that
# race within ~50ms -- confirmed live. The old shutdown path (relying on
# Python's default SIGINT->KeyboardInterrupt, no SIGTERM handler at all)
# just let the process die, abruptly cutting the setpoint stream with no
# attempt to hand control back to the autopilot first.
#
# Fix: SIGINT/SIGTERM now set a flag instead of killing anything. The
# control loop checks that flag FIRST, before the arm-request branch can
# ever run again, and drives an explicit state machine: request AUTO.LAND
# (PX4-native, flies itself once accepted -- no companion setpoints
# needed) while continuing to hold position so the offboard-loss failsafe
# doesn't race our own explicit mode switch; once AUTO.LAND is confirmed,
# cease all offboard publishing (safe -- the offboard-loss failsafe only
# applies while user_intended_mode is OFFBOARD, confirmed via PX4 source);
# wait for disarm (COM_DISARM_LAND auto-disarms ~2s after landing is
# detected) with a redundant explicit plain-disarm backstop; only then
# exit. This is the "graceful" tier -- for an unresponsive bridge, use the
# separate emergency_stop.py (SIGKILL + confirmed-dead + force-disarm),
# not this path.
NAV_STATE_AUTO_LAND = 18
ARMING_STATE_DISARMED = 1
LAND_CMD_RESEND_PERIOD_TICKS = 5     # 0.25s @ 20Hz
DISARM_BACKSTOP_PERIOD_TICKS = 40    # 2s @ 20Hz


class FastPlannerBridge(Node):
    def __init__(self) -> None:
        super().__init__("fast_planner_bridge")

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", pub_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", pub_qos)

        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._pos_cb, sub_qos
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, sub_qos
        )
        self.create_subscription(
            PositionCommand, "/planning/pos_cmd", self._pos_cmd_cb, 10
        )

        self._local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._pos_received = False
        self._tick = 0
        self._tick_enter = 0
        self._confirmed_armed = False
        self._climbed = False

        self._micro_waypoint_ned = None  # (x, y, z, yaw)
        self._last_pos_cmd_time = None
        self._home_ned = None  # captured once, at the moment OFFBOARD+ARMED is confirmed

        # Graceful-stop state machine -- see module-level comment above
        # POS_CMD_MAX_AGE_S. None until a shutdown signal arrives.
        self._shutdown_requested = False
        self._stop_state = None  # "REQUEST_LAND" | "WAIT_FOR_DISARM" | "DONE"
        signal.signal(signal.SIGINT, self._handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self._handle_shutdown_signal)

        auto_start = os.getenv("OFFBOARD_AUTO_START", "").lower() in {"1", "true", "yes", "y"}
        self._start_allowed = auto_start
        if not auto_start:
            threading.Thread(target=self._wait_for_enter, daemon=True).start()

        self._timer = self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info(
            f"fast_planner_bridge ready -- will capture home at arm time and "
            f"climb {CLIMB_TARGET_OFFSET_NED} NED relative to it, "
            "then follow /planning/pos_cmd from Fast-Planner"
        )
        self.get_logger().info(">>> Press ENTER to arm and fly (or set OFFBOARD_AUTO_START=1) <<<")

    def _handle_shutdown_signal(self, signum, frame) -> None:
        if self._shutdown_requested:
            return  # already stopping, ignore a repeat signal
        self._shutdown_requested = True
        self._stop_state = "REQUEST_LAND"
        self.get_logger().warn(
            f"Shutdown signal {signum} received -- ceasing ARM requests, "
            "commanding AUTO.LAND, will disarm and exit once landed. "
            "If this doesn't complete and the vehicle needs to come down "
            "NOW, use emergency_stop.py instead of repeating this signal."
        )

    def _wait_for_enter(self) -> None:
        try:
            input()
        except EOFError:
            return
        self._start_allowed = True
        self.get_logger().info("ENTER pressed -- arming and taking off!")

    def _climb_target_ned(self) -> tuple:
        """Absolute NED climb target = home (captured at arm time) + the
        configured relative offset. Only valid after _home_ned is set
        (i.e. after OFFBOARD+ARMED is confirmed) -- nothing calls this
        before then."""
        hx, hy, hz = self._home_ned
        ox, oy, oz = CLIMB_TARGET_OFFSET_NED
        return (hx + ox, hy + oy, hz + oz)

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._local_position = msg
        self._pos_received = True

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def _pos_cmd_cb(self, msg: PositionCommand) -> None:
        x, y, z = enu_to_ned_position(msg.position.x, msg.position.y, msg.position.z)
        vx, vy, vz = enu_to_ned_position(msg.velocity.x, msg.velocity.y, msg.velocity.z)
        ax, ay, az = enu_to_ned_position(
            msg.acceleration.x, msg.acceleration.y, msg.acceleration.z
        )
        # Fast-Planner yaw is ENU (CCW from +X/East); PX4 yaw is NED (CW
        # from +X/North). North=ENU's +Y, so NED_yaw = pi/2 - ENU_yaw,
        # matching the same East/North swap as enu_to_ned_position. The
        # rate is the negated derivative of that same relation.
        yaw_ned = math.pi / 2.0 - msg.yaw
        yawspeed_ned = -msg.yaw_dot
        self._micro_waypoint_ned = {
            "pos": (x, y, z),
            "vel": (vx, vy, vz),
            "acc": (ax, ay, az),
            "yaw": yaw_ned,
            "yawspeed": yawspeed_ned,
        }
        self._last_pos_cmd_time = self.get_clock().now()

    def _control_loop(self) -> None:
        self._tick += 1

        # Structural guarantee, not a hope: once a shutdown signal has
        # been received, this branch runs INSTEAD of everything below --
        # the arm-request branch further down literally cannot execute
        # again after this point.
        if self._shutdown_requested:
            self._graceful_stop_step()
            return

        if not self._pos_received:
            if self._tick % 40 == 0:
                self.get_logger().warn(
                    f"[tick {self._tick}] No data on /fmu/out/vehicle_local_position_v1.",
                )
            self._publish_hold_at_current()
            return

        if not self._start_allowed:
            self._publish_hold_at_current()
            return

        self._tick_enter += 1
        if self._tick_enter < 50:  # 2.5s pre-stream before mode switch
            self._publish_hold_at_current()
            return

        nav_state = self._vehicle_status.nav_state
        arming_state = self._vehicle_status.arming_state
        OFFBOARD_MODE, ARMED = 14, 2

        if nav_state != OFFBOARD_MODE:
            self._set_offboard_mode()
            self._publish_hold_at_current()
            if self._tick_enter % 20 == 0:
                self.get_logger().info(f"Requesting OFFBOARD... nav_state={nav_state}")
            return

        if arming_state != ARMED:
            self._arm()
            self._publish_hold_at_current()
            if self._tick_enter % 20 == 0:
                self.get_logger().info(f"Requesting ARM... arming_state={arming_state}")
            return

        if not self._confirmed_armed:
            self._confirmed_armed = True
            # Capture home HERE, at confirmed-armed time, not at node
            # startup -- the vehicle can sit disarmed for an arbitrary
            # time (and in principle drift, though it shouldn't while
            # disarmed) between this node starting and actually arming.
            self._home_ned = (
                self._local_position.x,
                self._local_position.y,
                self._local_position.z,
            )
            climb_target = self._climb_target_ned()
            self.get_logger().info(
                f"OFFBOARD + ARMED confirmed -- home_ned={self._home_ned}, "
                f"climbing to {climb_target}."
            )

        if not self._climbed:
            x, y, z = self._climb_target_ned()
            self._publish_trajectory_setpoint(x, y, z, yaw=0.0)
            dx = x - self._local_position.x
            dy = y - self._local_position.y
            dz = z - self._local_position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < CLIMB_ARRIVAL_TOLERANCE_M:
                self._climbed = True
                self._micro_waypoint_ned = {
                    "pos": self._climb_target_ned(),
                    "vel": None,
                    "acc": None,
                    "yaw": 0.0,
                    "yawspeed": None,
                }
                self.get_logger().info(f"Climb complete (dist={dist:.2f}m) -- following Fast-Planner.")
            if self._tick % 20 == 0:
                self.get_logger().info(
                    f"[CLIMBING] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                    f"{self._local_position.z:.2f}) dist={dist:.2f}m"
                )
            return

        if self._micro_waypoint_ned is None:
            self._publish_hold_at_current()
            return

        # Stale-plan guard -- see module docstring for the real incident
        # this fixes. _last_pos_cmd_time is None only right after the
        # climb, before any real PositionCommand has arrived yet; in that
        # case _micro_waypoint_ned is still the (safe, fixed) climb
        # target, not stale data, so it's fine to use directly.
        age_s = (
            (self.get_clock().now() - self._last_pos_cmd_time).nanoseconds / 1e9
            if self._last_pos_cmd_time is not None
            else 0.0
        )
        stale = self._last_pos_cmd_time is not None and age_s > POS_CMD_MAX_AGE_S
        if stale:
            self._publish_hold_at_current()
            if self._tick % 20 == 0:
                self.get_logger().warn(
                    f"STALE /planning/pos_cmd (age={age_s:.2f}s > {POS_CMD_MAX_AGE_S}s) -- "
                    "holding current position instead of following it.",
                )
            return

        wp = self._micro_waypoint_ned
        x, y, z = wp["pos"]
        self._publish_trajectory_setpoint(
            x, y, z, wp["yaw"], vel=wp["vel"], acc=wp["acc"], yawspeed=wp["yawspeed"]
        )

        if self._tick % 20 == 0:
            self.get_logger().info(
                f"[FOLLOWING] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                f"{self._local_position.z:.2f}) target_ned=({x:.2f},{y:.2f},{z:.2f}) "
                f"pos_cmd_age={age_s:.2f}s"
            )

    def _graceful_stop_step(self) -> None:
        arming_state = self._vehicle_status.arming_state
        nav_state = self._vehicle_status.nav_state

        if self._stop_state == "REQUEST_LAND":
            # Keep the setpoint stream alive here -- ceasing it before PX4
            # has actually switched modes would race the offboard-loss
            # failsafe (COM_OF_LOSS_T=1.0s, confirmed via PX4 source) against
            # our own explicit mode-switch request. Hold in place instead.
            self._publish_hold_at_current()
            if nav_state == NAV_STATE_AUTO_LAND:
                self._stop_state = "WAIT_FOR_DISARM"
                self.get_logger().info(
                    "AUTO.LAND confirmed -- ceasing offboard setpoints, "
                    "handing off to the autopilot."
                )
                return
            if self._tick % LAND_CMD_RESEND_PERIOD_TICKS == 0:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=4.0, param3=6.0
                )
                self.get_logger().info(f"Requesting AUTO.LAND... nav_state={nav_state}")
            return

        if self._stop_state == "WAIT_FOR_DISARM":
            # No more publishing here -- offboard-loss failsafe logic only
            # applies while user_intended_mode is OFFBOARD (PX4 source,
            # checkModeFallback), which is no longer true once AUTO.LAND
            # was confirmed above. Nothing further from this node can
            # trigger it.
            if arming_state == ARMING_STATE_DISARMED:
                self._stop_state = "DONE"
                self.get_logger().info("Disarmed -- graceful stop complete, shutting down.")
                rclpy.shutdown()
                return
            # COM_DISARM_LAND auto-disarms ~2s after landing is detected;
            # this is a redundant explicit backstop, not the primary path.
            if self._tick % DISARM_BACKSTOP_PERIOD_TICKS == 0:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
                )
                self.get_logger().info(
                    f"Waiting for disarm (backstop disarm sent)... arming_state={arming_state}"
                )
            return

    def _publish_hold_at_current(self) -> None:
        if not self._pos_received:
            self._publish_trajectory_setpoint(math.nan, math.nan, math.nan, math.nan)
            return
        self._publish_trajectory_setpoint(
            self._local_position.x, self._local_position.y, self._local_position.z, 0.0
        )

    def _offboard_mode_msg(self, vel_acc_valid: bool) -> OffboardControlMode:
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        # Must match whether THIS tick's TrajectorySetpoint actually carries
        # real (non-NaN) velocity/acceleration -- claiming a field valid to
        # PX4 while sending NaN in it is undefined behavior, not "ignored".
        # Caused a real runaway during the climb phase (position-only, so
        # vel/acc are NaN) when this was unconditionally True: see
        # fast_planner_bridge_test.log incident, 2026-08-10.
        msg.velocity = vel_acc_valid
        msg.acceleration = vel_acc_valid
        msg.attitude = False
        msg.body_rate = False
        return msg

    def _publish_trajectory_setpoint(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        vel: tuple | None = None,
        acc: tuple | None = None,
        yawspeed: float | None = None,
    ) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float(v) for v in vel] if vel is not None else [math.nan] * 3
        msg.acceleration = [float(a) for a in acc] if acc is not None else [math.nan] * 3
        msg.yaw = float(yaw)
        msg.yawspeed = float(yawspeed) if yawspeed is not None else math.nan
        # Published atomically with the matching heartbeat, right before
        # the setpoint it describes -- see _offboard_mode_msg's comment.
        vel_acc_valid = vel is not None and acc is not None
        self.offboard_pub.publish(self._offboard_mode_msg(vel_acc_valid))
        self.traj_pub.publish(msg)

    def _arm(self) -> None:
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _set_offboard_mode(self) -> None:
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _send_vehicle_command(self, command: int, **kwargs) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastPlannerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
