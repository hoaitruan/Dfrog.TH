# Phase 7: minimal ROS2 port of the ROS1 quadrotor_msgs/Bspline message,
# needed by RohitPawar2406/Fast-Planner-ROS2 (which has no official
# quadrotor_msgs ROS2 package of its own -- see plan_manage's
# kino_replan_fsm.cpp/traj_server.cpp for the exact field usage this
# was built from). Represents a piecewise-polynomial B-spline trajectory:
# position control points + knot vector, plus a separate 1D yaw B-spline.

int32 order
builtin_interfaces/Time start_time
int32 traj_id
geometry_msgs/Point[] pos_pts
float64[] knots
float64[] yaw_pts
float64 yaw_dt
