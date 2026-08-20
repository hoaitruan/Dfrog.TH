#!/usr/bin/env python3
"""
Phase 7: Fast-Planner (kinodynamic replan + B-spline optimization) wired
into our own pipeline, replacing kino_replan_launch.xml's reference setup
(a Foxy/ROS1-simulator XML launch file) with our real sensors.

Differences from the reference launch file:
  - sdf_map/pose_type=2 (ODOMETRY), not 1 (POSE_STAMPED): our
    ground_truth_tf.py already publishes nav_msgs/Odometry on
    /ground_truth/odom (GPS-based, see the airframe file's EKF2_EV_CTRL=0
    comment for why this uses PX4's own reliable estimate rather than
    cuVSLAM). POSE_STAMPED would need a new adapter node for no benefit.
  - /sdf_map/depth remapped to our real bridged depth camera
    (/depth_camera, 32FC1 meters -- sdf_map.cpp explicitly handles this
    encoding, converting to CV_16UC1 via k_depth_scaling_factor).
  - No waypoint_generator node: that package lives under the fork's
    (skipped) src/uav_simulator, not src/fast_planner. flight_type=2
    (global waypoints baked into launch params) sidesteps needing it --
    same effect as flight_type=1's "2D Nav Goal" for an unattended sim
    flight test.
  - sdf_map/frame_id="map": our own ground_truth_tf.py already publishes
    a working "map" frame (PX4-GPS-based); no need to replicate the
    reference launch's map->world->simulator identity-transform chain.

map_size/local_update_range sized for vio_test.sdf's obstacle field (8
ring pillars at 2.5m radius spanning z=0-10m, ~16 other markers within
about a 6m radius, hover altitude ~5m) -- generous margin, not tuned
tightly.
"""

import launch
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    odom_topic = "/ground_truth/odom"

    traj_server = Node(
        package="plan_manage",
        executable="traj_server",
        name="traj_server",
        output="screen",
        remappings=[
            ("/position_cmd", "/planning/pos_cmd"),
            ("/odom_world", odom_topic),
        ],
        parameters=[{"traj_server/time_forward": 1.5, "use_sim_time": True}],
    )

    fast_planner_node = Node(
        package="plan_manage",
        executable="fast_planner_node",
        name="fast_planner_node",
        output="screen",
        remappings=[
            ("/odom_world", odom_topic),
            ("/sdf_map/odom", odom_topic),
            ("/sdf_map/depth", "/depth_camera"),
        ],
        parameters=[
            {
                "use_sim_time": True,
                "planner_node/planner": 1,
                # fsm
                "fsm/flight_type": 2,
                "fsm/thresh_replan": 1.5,
                "fsm/thresh_no_replan": 2.0,
                "fsm/waypoint_num": 1,
                # TEMPORARY for receding-horizon milestone-1's first FLIGHT
                # test. B is straight ahead (+y, the camera's own facing
                # direction, confirmed across every prior flight) and FAR --
                # 8.0m, well past nvblox's 5.0m max integration range
                # (nvblox_base.yaml:75). Deliberately NO obstacle, NO
                # occlusion -- this is the exact goal that failed under pure
                # global planning (permanently unobserved -> NO_PATH loop,
                # the original bug this whole milestone exists to fix), now
                # being retried under receding-horizon planning: plan to the
                # observed frontier, fly, reveal more, replan, repeat.
                "fsm/waypoint0_x": 0.0,
                "fsm/waypoint0_y": 8.0,
                "fsm/waypoint0_z": 5.0,
                # sdf_map
                "sdf_map/resolution": 0.1,
                "sdf_map/map_size_x": 20.0,
                "sdf_map/map_size_y": 20.0,
                "sdf_map/map_size_z": 12.0,
                "sdf_map/local_update_range_x": 5.5,
                "sdf_map/local_update_range_y": 5.5,
                "sdf_map/local_update_range_z": 4.5,
                # Collision-margin fix (goalflight2->3): the pillar-03 collision
                # was NOT a perception gap -- DEBUG_OBSTACLE_CHECK showed it
                # observed=1 with a correctly-converging ESDF the entire
                # approach. Root cause: this value (0.199) inflates the mapped
                # obstacle surface by ONLY ~0.2m before any distance is even
                # measured, and search/margin (below) adds only 0.1m more on
                # top of THAT -- neither term ever accounted for the vehicle's
                # own physical radius (x500-class quad, prop-tip radius
                # ~0.3m), so a "safe" planned point could still have the
                # vehicle's own body overlapping the obstacle. Raised to 0.35
                # to bake vehicle-radius clearance directly into the ESDF
                # itself, so every consumer (search/margin, checkTrajCollision
                # via manager/clearance_threshold, the optimizer's
                # optimization/dist0) automatically inherits it.
                "sdf_map/obstacles_inflation": 0.35,
                "sdf_map/local_bound_inflate": 0.0,
                "sdf_map/local_map_margin": 50,
                "sdf_map/ground_height": 0.0,
                # goalflight20/21: under real system load, fast_planner_node
                # (single-threaded) was observed stalling for multiple
                # SECONDS at a time -- the executor-stall guard in
                # kino_replan_fsm.cpp makes those stalls safe (hold in
                # place), but doesn't reduce how OFTEN they happen. The
                # corridor-tier ESDF fetch response (onEsdfResponse in
                # sdf_map.cpp) processes ~2.1M voxels synchronously on
                # that same single thread every cycle -- its own DEBUG_TIMING
                # showed 200-232ms even at its FASTEST, the single most
                # expensive periodic thing this node does. Stretched from
                # the code default (1.5s) to 4.0s: corridor is the FAR-field
                # tier (long-range look-ahead), not what immediate
                # collision-avoidance depends on -- that's the near tier,
                # left untouched at its own 0.25s default (declared
                # separately, not overridden here) so close-range map
                # freshness near obstacles is unaffected. Trades slightly
                # staler far-field data for a ~60% cut in how often the
                # single most expensive synchronous block runs.
                "sdf_map/esdf_corridor_fetch_period_s": 4.0,
                # camera intrinsics -- StereoIMU/model.sdf: horizontal_fov
                # 1.518rad, 640x480 -> fx=fy=337.357, cx=320, cy=240 (see
                # the fixed K matrix on /stereo/left/camera_info, Phase 3).
                # OakD-Lite's depth sensor uses its own fov (1.274rad) --
                # recomputed: fx=fy = (640/2) / tan(1.274/2) = 481.06
                "sdf_map/cx": 320.0,
                "sdf_map/cy": 240.0,
                "sdf_map/fx": 481.06,
                "sdf_map/fy": 481.06,
                # depth filter -- our depth is 32FC1 meters, converted to
                # CV_16UC1 via k_depth_scaling_factor (sdf_map.cpp handles
                # TYPE_32FC1 explicitly). 1000.0 -> mm, matching a typical
                # real depth sensor's integer encoding.
                "sdf_map/use_depth_filter": True,
                "sdf_map/depth_filter_tolerance": 0.15,
                "sdf_map/depth_filter_maxdist": 15.0,
                "sdf_map/depth_filter_mindist": 0.2,
                "sdf_map/depth_filter_margin": 2,
                "sdf_map/k_depth_scaling_factor": 1000.0,
                "sdf_map/skip_pixel": 2,
                # local fusion (occupancy log-odds)
                "sdf_map/p_hit": 0.65,
                "sdf_map/p_miss": 0.35,
                "sdf_map/p_min": 0.12,
                "sdf_map/p_max": 0.90,
                "sdf_map/p_occ": 0.80,
                "sdf_map/min_ray_length": 0.3,
                "sdf_map/max_ray_length": 15.0,
                "sdf_map/esdf_slice_height": 5.0,
                "sdf_map/visualization_truncate_height": 11.0,
                "sdf_map/virtual_ceil_height": 12.0,
                "sdf_map/show_occ_time": False,
                "sdf_map/show_esdf_time": False,
                "sdf_map/pose_type": 2,  # ODOMETRY
                "sdf_map/frame_id": "map",
                # planner manager
                # Lỗi 3 Part B: max_vel/max_acc reduced a notch (1.5->1.2,
                # 1.0->0.8, ~20% each) across every coupled limit below
                # (manager/search/optimization all plan against the SAME
                # nominal limits; bspline/limit_* is the post-hoc
                # feasibility check on the optimized result -- all four
                # must move together or the optimizer targets one number
                # while the feasibility check enforces another). Doesn't
                # fix the tracking-divergence gap itself (that's Lỗi 3
                # Part A, the watchdog above) -- this asks less of the
                # controller in the first place, so whatever tracking
                # error exists has more margin to be absorbed before it
                # becomes the kind of divergence Lỗi 3/the ground-strike
                # floor have to catch.
                "manager/max_vel": 1.2,
                "manager/max_acc": 0.8,
                "manager/max_jerk": 4.0,
                # Combined thrust-vector budget (m/s^2), NOT another per-
                # axis limit -- see NonUniformBspline::checkFeasibility's
                # comment for what this actually constrains. Derived from
                # the watchdog flight's own .ulg telemetry, not a guess:
                # hover_thrust_estimate measured ~0.7465 for this exact
                # SITL airframe (valid almost the entire flight), and PX4's
                # own internal model treats normalized thrust as linear in
                # specific force with hover_thrust as the calibration
                # point -- max specific thrust = g / hover_thrust ~= 13.14
                # m/s^2 (thrust-to-weight ~= 1.34). At the fast-descent
                # event this whole fix exists for, real telemetry showed
                # thrust setpoint already AT that ceiling (magnitude 1.0)
                # while carrying a real 38.9deg tilt -- confirming this
                # ceiling is real, not theoretical. This value is 12.5, a
                # 20% margin below the measured 13.14 physical max,
                # reserving headroom for tracking error and disturbances
                # rather than planning trajectories right up to the edge
                # the airframe was just caught at.
                "manager/thrust_accel_limit": 12.5,
                "manager/dynamic_environment": 0,
                "manager/local_segment_length": 6.0,
                # Was 0.2 -- same under-margined root cause as
                # sdf_map/obstacles_inflation above. Raised to 0.3 so
                # checkTrajCollision's runtime "still safe" trigger matches
                # the hard search margin (0.35 inflation + 0.3 here = ~0.65m
                # real clearance from the true obstacle surface).
                "manager/clearance_threshold": 0.3,
                "manager/control_points_distance": 0.5,
                "manager/use_geometric_path": False,
                "manager/use_kinodynamic_path": True,
                "manager/use_topo_path": False,
                "manager/use_optimization": True,
                # kinodynamic path searching
                "search/max_tau": 0.6,
                "search/init_max_tau": 0.8,
                "search/max_vel": 1.2,
                "search/max_acc": 0.8,
                # Same combined thrust-vector budget as manager/thrust_accel_limit
                # (12.5 m/s^2, 20% margin below the measured 13.14 m/s^2
                # physical max -- see that param's own comment for the
                # full derivation). Search-level pruning, not just the
                # post-optimization feasibility check: without this, the
                # search kept succeeding on every-axis-within-max_acc_
                # candidates that STILL failed the coupled thrust check
                # downstream, every single time, with no way to try
                # anything gentler -- confirmed live (64 consecutive
                # rejections, one verify flight, zero progress toward the
                # goal). Both values must move together.
                "search/thrust_accel_limit": 12.5,
                "search/w_time": 10.0,
                "search/horizon": 7.0,
                "search/lambda_heu": 5.0,
                "search/resolution_astar": 0.1,
                "search/time_resolution": 0.8,
                # Was 0.2 -- see sdf_map/obstacles_inflation above for the
                # full root-cause writeup. Raised to 0.3 (stacks with the
                # 0.35 inflation for ~0.65m hard clearance from the true
                # obstacle surface, comfortably past the ~0.3m vehicle
                # prop-tip radius).
                "search/margin": 0.3,
                "search/allocate_num": 100000,
                "search/check_num": 5,
                # trajectory optimization
                "optimization/lambda1": 10.0,
                "optimization/lambda2": 5.0,
                "optimization/lambda3": 0.00001,
                "optimization/lambda4": 0.01,
                "optimization/lambda5": 0.0,
                "optimization/lambda6": 0.0,
                "optimization/lambda7": 100.0,
                # Was 0.4 -- see sdf_map/obstacles_inflation above for the
                # full root-cause writeup. Raised to 0.5 so the optimizer's
                # soft push-away target (0.35 inflation + 0.5 = ~0.85m from
                # the true surface) clears the kStartStateDivergenceTol /
                # kTrackingDivergenceHoldTol watchdog tolerance (0.75m) --
                # previously that tolerance was LARGER than the entire
                # planned obstacle clearance, so "acceptable" tracking noise
                # alone could erase the whole safety margin.
                "optimization/dist0": 0.5,
                "optimization/max_vel": 1.2,
                "optimization/max_acc": 0.8,
                # Third and last leg of the same combined thrust-vector
                # budget (search/thrust_accel_limit prunes the discrete
                # search primitives; this shapes what the continuous
                # optimizer smooths them into -- the optimizer was the
                # missing piece: search-level pruning alone still produced
                # the identical "Reallocate ratio: 1.55533" rejection on
                # re-verify, because the search's path is only an initial
                # guess the optimizer is free to reshape past the budget
                # again). All three (manager/search/optimization) must
                # move together.
                "optimization/thrust_accel_limit": 12.5,
                "optimization/algorithm1": 15,
                "optimization/algorithm2": 11,
                "optimization/max_iteration_num1": 2,
                "optimization/max_iteration_num2": 300,
                "optimization/max_iteration_num3": 200,
                "optimization/max_iteration_num4": 200,
                "optimization/max_iteration_time1": 0.0001,
                "optimization/max_iteration_time2": 0.005,
                "optimization/max_iteration_time3": 0.003,
                "optimization/max_iteration_time4": 0.003,
                "optimization/order": 3,
                "bspline/limit_vel": 1.2,
                "bspline/limit_acc": 0.8,
                "bspline/limit_ratio": 1.1,
            }
        ],
    )

    return launch.LaunchDescription([traj_server, fast_planner_node])
