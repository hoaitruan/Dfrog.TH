"""NED<->ENU and FRD<->FLU conversions.

Ported from px4_ros_com's frame_transforms.cpp/h (MAVROS-derived, the
reference PX4 uses for its own ROS 2 examples) -- same module as
~/Drone/ros2_ws/src/wall_avoid/wall_avoid/frame_transforms.py, duplicated
here since this is a separate container/workspace.

Quaternions are (w, x, y, z), Hamiltonian convention, matching px4_msgs.
"""

import math

# Rz(pi/2) * Rx(pi), see frame_transforms.h NED_ENU_Q. Self-inverse.
NED_ENU_Q = (0.0, 0.70710678118, 0.70710678118, 0.0)

# Rx(pi): FRD body <-> FLU body. Also self-inverse.
AIRCRAFT_BASELINK_Q = (0.0, 1.0, 0.0, 0.0)


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def ned_to_enu_position(x, y, z):
    """(North, East, Down) -> (East, North, Up). Self-inverse."""
    return (y, x, -z)


enu_to_ned_position = ned_to_enu_position


def px4_to_ros_orientation(q_ned_frd):
    """PX4 attitude (aircraft/FRD to NED) -> ROS attitude (baselink/FLU to ENU)."""
    return quat_mult(quat_mult(NED_ENU_Q, q_ned_frd), AIRCRAFT_BASELINK_Q)


def ros_to_px4_orientation(q_enu_flu):
    """Inverse of px4_to_ros_orientation (same two-step composition, self-inverse)."""
    return quat_mult(quat_mult(NED_ENU_Q, q_enu_flu), AIRCRAFT_BASELINK_Q)


def yaw_from_quat(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
