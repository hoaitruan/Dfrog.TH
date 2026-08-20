#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.hpp>
#include <plan_env/obj_predictor.hpp>
#include <plan_env/sdf_map.h>
#include "quadrotor_msgs/msg/bspline.hpp"
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.hpp>

#include <fast_planner/fast_planner.h>

using std::vector;

class FastPlanner;
class FastPlannerManager;

namespace fast_planner {

class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  rclcpp::Node::SharedPtr nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(rclcpp::Node::SharedPtr node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  /* replan retry backoff -- exec_timer_ fires every 10ms unconditionally,
  and GEN_NEW_TRAJ/REPLAN_TRAJ previously called the (potentially
  expensive) kinodynamic search again on every single tick after a
  failure, with no delay. Measured during Milestone 1 verification: a
  persistently-failing search retried ~every 10-20ms, and the nvblox
  fetch timers it competes with on this single-threaded executor were
  measurably degraded during that storm (near-field fetch cadence dropped
  to ~80% of its configured rate, corridor to ~89%, with gaps up to ~12x
  their configured period) -- not a full deadlock, but real, measured
  executor interference that a failing search has no business causing.
  consecutive_replan_failures_ grows the wait between attempts
  (exponential, capped) after each failure and resets to 0 on success, so
  a failing search yields the executor back to the fetch timers between
  attempts instead of hammering the same stale map data. */
  int consecutive_replan_failures_ = 0;
  rclcpp::Time next_replan_attempt_time_;
  bool replanBackoffReady();
  void onReplanFailure();
  void onReplanSuccess();

  // Lỗi 3: continuous tracking-divergence watchdog cooldown -- see
  // checkCollisionCallback's watchdog block for the full rationale.
  // Default-constructed (epoch 0), so the very first EXEC_TRAJ tick is
  // always eligible to trigger a hold if divergence already exceeds
  // tolerance at that point.
  rclcpp::Time next_watchdog_hold_allowed_time_;
  void triggerTrackingDivergenceHold();

  // Executor-stall guard (goalflight19 root cause): the Lỗi 3 watchdog
  // above catches POSITION divergence, but only gets to run once
  // execFSMCallback's own 10ms timer actually fires -- both timers share
  // ONE thread (rclcpp::spin(), single-threaded executor), so if the
  // whole node gets descheduled under system load (observed live: a 4s
  // gap with ZERO log lines from this node, not one slow callback, a
  // total blackout), the open-loop spline belief (t_cur = now() -
  // start_time_) silently runs 4 real seconds ahead of a vehicle that
  // never moved, and by the time any tick finally runs again, REPLAN_TRAJ's
  // own re-ground fires on an already-3m+ divergence and commits a
  // "catch up" trajectory from a belief that's already stale -- this is
  // what produced the violent snap-and-drop that crashed goalflight19,
  // not a bad decision, a decision made on stale time. Checking wall-clock
  // gap directly, at the top of execFSMCallback, catches the STALL ITSELF
  // (any tick's actual dt) rather than waiting for its downstream
  // position-divergence symptom to grow large enough to trip the
  // existing watchdog -- converts a 4s blackout into an immediate hold
  // the moment normal ticking resumes, instead of one big trajectory
  // commit first.
  bool have_last_exec_tick_time_ = false;
  rclcpp::Time last_exec_tick_time_;

  // Odom-source staleness guard (goalflight24 root cause): the odom SOURCE
  // (ground_truth_tf) can itself get starved under the same host-load
  // pressure the executor-stall guard above exists to handle -- when it
  // does, odom_pos_ keeps holding whatever position was last received,
  // with no way for anything downstream to tell it's gone stale.
  // triggerTrackingDivergenceHold() trusted it unconditionally and pinned
  // a hold to a position that was really ~6s old (0.53m altitude logged
  // while the real vehicle, per PX4 telemetry, was at 4.75m) -- the
  // bridge then chased that phantom target down, tripped the
  // descent-rate killswitch, and the resulting force-disarm free-fell
  // the vehicle into the ground. Tracking wall-clock RECEIPT time (not
  // the message's own header stamp -- this only needs to catch a
  // genuinely stalled source, not clock-sync nuances) lets
  // triggerTrackingDivergenceHold() refuse to act on a reading that's no
  // longer credibly "real right now."
  bool have_last_odom_recv_time_ = false;
  rclcpp::Time last_odom_recv_time_;

  // Multi-threaded executor rework (goalflight27 livelock) -- see
  // SDFMap::map_cb_group_'s declaration for the full incident. exec_timer_,
  // safety_timer_, odom_sub_, and waypoint_sub_ all run in THIS group so
  // they stay mutually exclusive with each other (same no-overlap
  // guarantee execFSMCallback/checkCollisionCallback always had) while
  // running independently of SDFMap's own group on a MultiThreadedExecutor
  // -- flight control can no longer be blocked by map population or
  // visualization work, however loaded the host is.
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state -- the TRUE
                                                                   // mission goal, set only by
                                                                   // waypointCallback/checkCollision
                                                                   // Callback's legitimate
                                                                   // substitution. Never written by
                                                                   // frontier logic.
  // Receding-horizon milestone 1: what actually gets planned to each
  // cycle. Pass-through to end_pt_ once the true goal is observed-and-
  // free; otherwise the ray-marched frontier point. Recomputed fresh
  // every GEN_NEW_TRAJ/REPLAN_TRAJ cycle -- not persisted across cycles,
  // so it can never go stale the way a cached value could.
  Eigen::Vector3d frontier_pt_;
  int current_wp_;

  // Ray-marches from `from` toward `goal` (fixed ~sdf_map/resolution
  // steps), stopping at the first point that's neither within the
  // start-state exemption radius nor confirmed observed-and-free.
  // Returns that boundary, pulled back by one step and floored at the
  // exemption radius (see .cpp for the full rationale -- this is the
  // milestone-1 target substitution, kept out of planner_manager.cpp/
  // kinodynamic_astar.cpp entirely, which stay goal-agnostic).
  Eigen::Vector3d computeFrontier(const Eigen::Vector3d& from, const Eigen::Vector3d& goal);

  // Milestone-2 extension (goalflight2/3/4 pillar_03 collisions): the
  // straight single-ray march above targets a point immediately adjacent
  // to whatever obstacle sits directly on the line to `goal` -- fine when
  // nothing is in the way, but for an obstacle centered ON that line
  // (pillar_03 sits exactly between the spawn point and B), the ray is
  // blocked at nearly the same distance on every replan cycle, so the
  // search is repeatedly handed a target hugging the obstacle's near
  // face with no incentive to detour around it. This helper factors out
  // the single-ray march so computeFrontier can try several yaw-rotated
  // candidate rays and pick whichever clears farthest. Returns the
  // achievable distance along `dir` from `from` (same pull-back/floor
  // convention as computeFrontier itself).
  double marchRay(const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double max_dist);

  // Hysteresis state for the yaw-fan in computeFrontier(): the fan picks
  // whichever candidate direction clears farthest, recomputed from
  // scratch every cycle with no memory of the previous choice. Real
  // incident (goalflight13): two replan cycles 7s apart, with the
  // vehicle having made real forward progress in between, picked
  // frontier targets 5.8m apart in x -- a full flip from a left-side
  // detour to a right-side one -- because the ESDF had updated just
  // enough between cycles to nudge which candidate's clearance edged out
  // the other. The FSM commanded that reversal as a real setpoint change;
  // the vehicle couldn't track it and lost altitude control.
  //
  // Stored as an ANGLE OFFSET from the current direct-to-goal line, NOT
  // an absolute world-frame direction -- goalflight16 caught the latter
  // approach live: persisting an absolute vector and re-marching it from
  // an ever-moving `from` let the reachable distance (capped at
  // total_dist = current distance to goal) grow every cycle the vehicle
  // moved further along it, a runaway feedback loop with no anchor back
  // to the goal (frontier_pt walked from y=8 out to y=12.4, real
  // distance-from-origin climbing straight to the 15m horizontal
  // killswitch limit). An angle offset re-applied to the FRESH
  // direct-to-goal direction every cycle can't do that: it's
  // structurally capped at total_dist like every other candidate, and
  // rotates back toward the goal as the vehicle approaches it, the same
  // way the un-persisted fan already did before hysteresis was added --
  // this only adds "prefer the side already committed to," not a new
  // failure mode.
  bool have_last_frontier_angle_ = false;
  double last_frontier_angle_deg_ = 0.0;

  /* ROS utils */
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // rclcpp::Publisher replan_pub_, new_pub_, bspline_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_ ;
  rclcpp::Publisher<quadrotor_msgs::msg::Bspline>::SharedPtr  bspline_pub_;

  //std::shared_ptr<FastPlanner> nh1;

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback();
  void checkCollisionCallback();
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
  KinoReplanFSM(/* args */);
  ~KinoReplanFSM();

  void init(std::shared_ptr<FastPlanner> nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
