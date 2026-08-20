# Phase 7: minimal ROS2 port of the ROS1 quadrotor_msgs/PositionCommand
# message, needed by RohitPawar2406/Fast-Planner-ROS2's traj_server.cpp
# (converts a Bspline trajectory into per-tick setpoints for the flight
# controller). Field set built from traj_server.cpp's actual usage.

std_msgs/Header header

uint8 TRAJECTORY_STATUS_EMPTY=0
uint8 TRAJECTORY_STATUS_READY=1
uint8 TRAJECTORY_STATUS_COMPLETED=2
uint8 TRAJECTORY_STATUS_ABORT=3

int32 trajectory_id
uint8 trajectory_flag

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

float64 yaw
float64 yaw_dot

float64[3] kx
float64[3] kv
