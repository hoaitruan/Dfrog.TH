#!/usr/bin/env python3
"""
Task 5: pre-flight health check. Refuses (non-zero exit) unless:
  1. /fmu/* topics are present (PX4<->ROS2 DDS bridge genuinely up).
  2. Every pipeline process runs as `admin`, not root (root silently
     receives zero DDS data -- confirmed real incident, see README's
     "Operational gotchas").
  3. The ESDF service returns non-empty real data for frame_id: "map"
     (nvblox is actually mapping something, not just alive).
  4. The vehicle spawned near true origin (/fmu/out/vehicle_local_position_v1)
     -- the CLIMB_TARGET_OFFSET_NED fix (Task 3) makes an absolute climb
     safe regardless, but a drone far from origin before arming is itself
     a sign something upstream (a leftover process, a stale world) is
     wrong, worth catching here rather than discovering mid-flight.

Run inside the container. Exits 0 and prints "HEALTH CHECK: PASS" only if
all four pass; otherwise prints exactly which check failed and exits 1.
"""
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nvblox_msgs.srv import EsdfAndGradients
from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import VehicleLocalPosition

# PX4's /fmu/out/* topics publish BEST_EFFORT -- the default RELIABLE
# subscription QoS is incompatible and silently receives nothing (no
# error, just zero messages). Same fix already applied in
# fast_planner_bridge.py's own /fmu/out subscriptions.
PX4_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

ORIGIN_TOLERANCE_M = 2.0
REQUIRED_FMU_TOPICS = [
    "/fmu/out/vehicle_status_v4",
    "/fmu/out/vehicle_local_position_v1",
]
# Process name substrings that must be running as admin (checked via ps).
REQUIRED_ADMIN_PROCESSES = [
    "px4_sitl_default/bin/px4",
    "MicroXRCEAgent",
    "ground_truth_tf",
    "nvblox_container",
    "visual_slam_launch_container",
]


def fail(msg: str) -> None:
    print(f"HEALTH CHECK: FAIL -- {msg}")
    sys.exit(1)


def check_admin_processes() -> None:
    ps = subprocess.run(["ps", "-eo", "user,args"], capture_output=True, text=True).stdout
    for name in REQUIRED_ADMIN_PROCESSES:
        matches = [line for line in ps.splitlines() if name in line]
        if not matches:
            fail(f"expected process not running: {name}")
        for line in matches:
            user = line.split(None, 1)[0]
            if user != "admin":
                fail(f"process '{name}' running as '{user}', not admin: {line.strip()}")
    print("  [1/4] all required processes running as admin: OK")


def check_fmu_topics(node: Node) -> None:
    # DDS discovery is async -- a just-created node's graph view can be
    # incomplete for the first moment. Give it a couple seconds and retry
    # rather than fail on a transient empty read.
    import time
    topics: set = set()
    t0 = time.time()
    while time.time() - t0 < 5.0:
        topics = {name for name, _ in node.get_topic_names_and_types()}
        if all(t in topics for t in REQUIRED_FMU_TOPICS):
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    missing = [t for t in REQUIRED_FMU_TOPICS if t not in topics]
    if missing:
        fail(f"missing /fmu topics (PX4<->DDS bridge not up): {missing}")
    print("  [2/4] /fmu/* topics present: OK")


def check_esdf_nonempty(node: Node) -> None:
    cli = node.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")
    if not cli.wait_for_service(timeout_sec=5.0):
        fail("nvblox get_esdf_and_gradient service not available")
    req = EsdfAndGradients.Request()
    req.use_aabb = True
    req.update_esdf = True
    req.frame_id = "map"
    req.aabb_min_m = Point(x=-3.0, y=-3.0, z=-3.0)
    req.aabb_size_m = Vector3(x=6.0, y=6.0, z=6.0)
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    resp = future.result()
    if resp is None or not resp.success:
        fail("ESDF query failed (success=False or timed out)")
    data = resp.esdf_and_gradients.data
    non_sentinel = sum(1 for v in data if v > -999.0)
    if non_sentinel == 0:
        fail("ESDF query returned zero real (non-sentinel) voxels -- nvblox isn't seeing anything")
    print(f"  [3/4] ESDF non-empty: OK ({non_sentinel} real voxels)")


def check_spawn_near_origin(node: Node) -> None:
    result = {}

    def cb(msg: VehicleLocalPosition) -> None:
        result["pos"] = (msg.x, msg.y, msg.z)

    sub = node.create_subscription(
        VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", cb, PX4_OUT_QOS
    )
    import time
    t0 = time.time()
    while time.time() - t0 < 5.0 and "pos" not in result:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if "pos" not in result:
        fail("no data on /fmu/out/vehicle_local_position_v1 within 5s")
    x, y, z = result["pos"]
    dist = (x**2 + y**2 + z**2) ** 0.5
    if dist > ORIGIN_TOLERANCE_M:
        fail(
            f"vehicle at ({x:.2f},{y:.2f},{z:.2f}), {dist:.2f}m from origin "
            f"(tolerance {ORIGIN_TOLERANCE_M}m) -- restart PX4/Gazebo for a fresh spawn"
        )
    print(f"  [4/4] spawn near origin: OK ({dist:.2f}m from origin)")


def main() -> None:
    rclpy.init()
    node = Node("health_check")
    print("Running pre-flight health check...")
    check_admin_processes()
    check_fmu_topics(node)
    check_esdf_nonempty(node)
    check_spawn_near_origin(node)
    print("HEALTH CHECK: PASS")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
