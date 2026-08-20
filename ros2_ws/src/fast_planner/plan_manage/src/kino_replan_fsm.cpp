#include "plan_manage/kino_replan_fsm.h"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cmath>

namespace fast_planner {

KinoReplanFSM::KinoReplanFSM() {
   std::cout << "KinoReplanFSM hit ! " << std::endl;
}

KinoReplanFSM::~KinoReplanFSM() {
  std::cout << "KinoReplanFSM ended ! " << std::endl;
}

void KinoReplanFSM::init(std::shared_ptr<FastPlanner> nh) {
  node_ = nh;
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  // Crash fix (Lỗi 3 watchdog, found on its first real flight): a
  // default-constructed rclcpp::Time is implicitly wall-clock typed
  // (RCL_SYSTEM_TIME). node_->now() below (use_sim_time:=true) returns
  // RCL_ROS_TIME. checkCollisionCallback's watchdog compares
  // next_watchdog_hold_allowed_time_ against node_->now() unconditionally
  // once divergence first exceeds threshold -- comparing two rclcpp::Time
  // values with different clock types throws std::runtime_error("can't
  // compare times with different time sources"), uncaught, taking the
  // whole node down (SIGABRT). Seeding it here with the node's own clock
  // -- the same clock node_->now() will always read from -- means the
  // mismatched-clock-type state this default construction produces is
  // never actually compared against.
  next_watchdog_hold_allowed_time_ = nh->now();

  nh->declare_parameter("fsm/flight_type", -1);
  nh->declare_parameter("fsm/thresh_replan", -1.0);
  nh->declare_parameter("fsm/thresh_no_replan", -1.0);
  nh->declare_parameter("fsm/waypoint_num", -1);

  nh->get_parameter("fsm/flight_type", target_type_);
  nh->get_parameter("fsm/thresh_replan", replan_thresh_);
  nh->get_parameter("fsm/thresh_no_replan", no_replan_thresh_);
  nh->get_parameter("fsm/waypoint_num", waypoint_num_);

  //waypoints_.resize(waypoint_num_);
  nh->declare_parameter("fsm/waypoint0_x", -1.0);
  nh->declare_parameter("fsm/waypoint0_y", -1.0);
  nh->declare_parameter("fsm/waypoint0_z", -1.0);
  nh->declare_parameter("fsm/waypoint1_x", -1.0);
  nh->declare_parameter("fsm/waypoint1_y", -1.0);
  nh->declare_parameter("fsm/waypoint1_z", -1.0);
  nh->declare_parameter("fsm/waypoint2_x", -1.0);
  nh->declare_parameter("fsm/waypoint2_y", -1.0);
  nh->declare_parameter("fsm/waypoint2_z", -1.0);

  nh->get_parameter("fsm/waypoint0_x", waypoints_[0][0]);
  nh->get_parameter("fsm/waypoint0_y", waypoints_[0][1]);
  nh->get_parameter("fsm/waypoint0_z", waypoints_[0][2]);

  nh->get_parameter("fsm/waypoint1_x", waypoints_[1][0]);
  nh->get_parameter("fsm/waypoint1_y", waypoints_[1][1]);
  nh->get_parameter("fsm/waypoint1_z", waypoints_[1][2]);

  nh->get_parameter("fsm/waypoint2_x", waypoints_[2][0]);
  nh->get_parameter("fsm/waypoint2_y", waypoints_[2][1]);
  nh->get_parameter("fsm/waypoint2_z", waypoints_[2][2]);

  //////////////////////// DOUBTS - NEXT 3 lines
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  // See control_cb_group_'s declaration in kino_replan_fsm.h (goalflight27
  // livelock) for the full rationale.
  control_cb_group_ = nh->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec_timer_ = nh->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KinoReplanFSM::execFSMCallback, this), control_cb_group_);
  safety_timer_ = nh->create_wall_timer(std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::checkCollisionCallback, this), control_cb_group_);

  rclcpp::SubscriptionOptions control_sub_opts;
  control_sub_opts.callback_group = control_cb_group_;
  waypoint_sub_ = nh->create_subscription<nav_msgs::msg::Path>(
      "/waypoint_generator/waypoints", 1, std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1), control_sub_opts);
  odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 1, std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1), control_sub_opts);

  replan_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  new_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  bspline_pub_ = nh->create_publisher<quadrotor_msgs::msg::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses[0].pose.position.z < -0.1) {
    return;
  }

  RCUTILS_LOG_INFO("Triggered!");
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  std::cout << "DEBugging waypoints .....................................ok done ! ";
  std::cout << end_pt_ << std::endl;

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
  last_odom_recv_time_ = node_->now();
  have_last_odom_recv_time_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  int pre_s = static_cast<int>(exec_state_);
  exec_state_ = new_state;
  // RCLCPP_INFO(this->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
  RCUTILS_LOG_INFO("[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
}

void KinoReplanFSM::printFSMExecState() {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  //RCLCPP_INFO(this->get_logger(), "[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
  RCUTILS_LOG_INFO("[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
}

void KinoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) RCUTILS_LOG_INFO("no odom");
    if (!trigger_) RCUTILS_LOG_INFO("wait for goal");
    fsm_num = 0;
  }

  // Executor-stall guard -- see last_exec_tick_time_'s declaration
  // comment (kino_replan_fsm.h) for the full incident this addresses.
  // Nominal period is 10ms (exec_timer_); kStallGapS is ~30x that, well
  // clear of ordinary scheduling jitter, but small next to the multi-
  // SECOND blackout that actually caused damage -- this is meant to
  // catch a stall in its first few hundred ms, long before the spline
  // belief has drifted far enough for anything downstream to notice.
  // Skipped while there's nothing to hold relative to (no odom yet, or
  // no trajectory has ever been produced) -- matches the same guards
  // the Lỗi 3 watchdog and REPLAN_TRAJ's empty-spline guard already use.
  {
    const rclcpp::Time now = node_->now();
    if (have_last_exec_tick_time_) {
      const double gap_s = (now - last_exec_tick_time_).seconds();
      const double kStallGapS = 0.3;
      LocalTrajData* info = &planner_manager_->local_data_;
      if (gap_s > kStallGapS && have_odom_ && info->position_traj_.getControlPoint().rows() > 0 &&
          (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ || exec_state_ == GEN_NEW_TRAJ)) {
        RCUTILS_LOG_WARN(
            "EXECUTOR STALL: %.3fs gap since the last FSM tick (nominal 0.01s) -- holding in "
            "place before trusting whatever the open-loop spline belief drifted to.",
            gap_s);
        triggerTrackingDivergenceHold();
        exec_state_ = EXEC_TRAJ;
        last_exec_tick_time_ = now;
        return;
      }
    }
    have_last_exec_tick_time_ = true;
    last_exec_tick_time_ = now;
  }

  switch (exec_state_) {
    case INIT:{
      if (!have_odom_ || !trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;}

    case WAIT_TARGET:{
      if (!have_target_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;}

    case GEN_NEW_TRAJ:{
      if (!replanBackoffReady()) break;
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_(0) = atan2(odom_orient_.toRotationMatrix()(1, 0), odom_orient_.toRotationMatrix()(0, 0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      // Receding-horizon milestone 1: decide this cycle's actual planning
      // target. Pass-through to the true goal once it's confirmed
      // observed AND clearly free (reusing checkCollisionCallback's own
      // 0.3m "safely clear" threshold, not a second magic number) --
      // frontier computation becomes a no-op at that point. Otherwise,
      // ray-march to the observed frontier. No new FSM state: frontier_pt_
      // is a per-cycle value, recomputed fresh every time, never a
      // transition edge.
      {
        auto sdf_map = planner_manager_->edt_environment_->sdf_map_;
        const bool goal_ready = sdf_map->isObserved(end_pt_) && sdf_map->getDistance(end_pt_) > 0.3;
        frontier_pt_ = goal_ready ? end_pt_ : computeFrontier(start_pt_, end_pt_);
        // TEMPORARY, milestone-1 static verification only.
        RCUTILS_LOG_INFO(
            "DEBUG_FRONTIER pass_through=%d end_pt=(%.3f,%.3f,%.3f) frontier_pt=(%.3f,%.3f,%.3f) "
            "start_pt=(%.3f,%.3f,%.3f)",
            goal_ready ? 1 : 0, end_pt_(0), end_pt_(1), end_pt_(2), frontier_pt_(0), frontier_pt_(1),
            frontier_pt_(2), start_pt_(0), start_pt_(1), start_pt_(2));
      }

      if (callKinodynamicReplan()) {
        onReplanSuccess();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        onReplanFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;}
      case EXEC_TRAJ: {
            /* determine if need to replan */
            LocalTrajData* info     = &planner_manager_->local_data_;
            // node_->now() (respects use_sim_time), not an unattached
            // rclcpp::Clock().now() -- an unattached wall-clock now() minus
            // a sim-time start_time_ produced a massively-overshooting
            // t_cur, making every trajectory look instantly finished.
            rclcpp::Time time_now = node_->now();
            double         t_cur    = (time_now - info->start_time_).seconds();
            t_cur                   = min(info->duration_, t_cur);

            Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

            // Segment-boundary vs. goal-reached, kept distinct. The
            // kinodynamic search can end a LOCAL segment via
            // REACH_HORIZON (search/horizon, 7.0m by default) well short
            // of end_pt_ for goals farther than one segment can cover --
            // kinodynamicReplan() still returns success for that shorter
            // segment, so info->duration_ is only the length of THIS
            // segment, not "distance remaining to the mission goal" (see
            // kinodynamic_astar.cpp's REACH_HORIZON/REACH_END distinction,
            // discarded by the time it reaches planner_manager.cpp). The
            // original check here declared the whole mission complete the
            // instant this segment's own playback time ran out, with no
            // check against end_pt_ at all -- this is what caused the
            // "stops short of the goal" bug for any goal beyond one
            // segment's reach. kGoalReachedTol reinstates exactly the
            // check this line's own dead comment had already sketched out
            // (0.5m) and never wired in.
            const double kGoalReachedTol = 0.5;
            if (t_cur > info->duration_ - 1e-2) {
              if ((end_pt_ - pos).norm() < kGoalReachedTol) {
                std::cout << "Reached goal (within " << kGoalReachedTol << "m) -- mission complete. t_cur="
                          << t_cur << " duration=" << info->duration_ << std::endl;
                have_target_ = false;
                changeFSMExecState(WAIT_TARGET, "FSM");
                return;
              } else {
                std::cout << "Segment finished (" << (end_pt_ - pos).norm()
                          << "m still short of end_pt_) -- replanning toward the real goal, not stopping."
                          << std::endl;
                changeFSMExecState(REPLAN_TRAJ, "FSM");
                return;
              }

            } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
              //std::cout << "near end" << std::endl;
              return;

            } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
              // std::cout << "near start" << std::endl;
              return;

            } else {
              changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
          }
    case REPLAN_TRAJ:{
      if (!replanBackoffReady()) break;
      LocalTrajData* info     = &planner_manager_->local_data_;

      // Defense-in-depth, not the primary fix (that's the exec_state_
      // gate in checkCollisionCallback's "goal near collision" branch,
      // above) -- this is a backstop against evaluating an
      // uninitialized trajectory by any OTHER path that might reach
      // REPLAN_TRAJ without a prior successful plan. A default-
      // constructed NonUniformBspline (NonUniformBspline() {}) never
      // sets control_points_, so getControlPoint().rows() == 0 reliably
      // detects "no plan has ever succeeded yet" without needing to
      // touch evaluateDeBoor's own indexing.
      if (info->position_traj_.getControlPoint().rows() == 0) {
        RCUTILS_LOG_WARN(
            "REPLAN_TRAJ entered with no prior successful trajectory -- "
            "falling back to GEN_NEW_TRAJ instead of evaluating an empty spline.");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        break;
      }

      // node_->now(), not an unattached rclcpp::Clock().now() -- this is
      // the exact mechanism behind this project's historically-documented
      // open-loop-replanning divergence incident: t_cur here feeds
      // evaluateDeBoorT(t_cur) below to compute the next replan's start
      // state. A wall-clock-vs-sim-clock mismatched t_cur means that
      // start state was being evaluated at a nonsensical point on the
      // previous B-spline, not the real "current time along the
      // trajectory" the receding-horizon replan logic assumes.
      rclcpp::Time time_now = node_->now();
      double         t_cur    = (time_now - info->start_time_).seconds();

      Eigen::Vector3d spline_start_pt = info->position_traj_.evaluateDeBoorT(t_cur);

      // Sanity cross-check, not a replacement: REPLAN_TRAJ's start-state is
      // normally this open-loop spline evaluation (kept for trajectory
      // continuity -- unconditionally snapping to odom_pos_ every cycle
      // would inject real tracking/odom noise into the planning start-state
      // on every single replan, this project's own historically-documented
      // "incident #3" open-loop-divergence failure mode in reverse). But an
      // UNCHECKED open-loop belief is exactly what let the checkTrajCollision
      // replan-storm's real-vehicle tracking drift go unnoticed and
      // uncorrected (root cause of the -1.68m ground excursion): the only
      // path back to real odom was start_pt_ = odom_pos_ at GEN_NEW_TRAJ
      // (:157 above), which runs ONLY after a search failure -- so a
      // persistently-succeeding-but-diverging REPLAN_TRAJ loop never
      // re-grounded. kStartStateDivergenceTol=0.75m: normal tracking error
      // observed across prior flights tops out ~0.1-0.3m (2.5x margin above
      // that noise floor, so ordinary tracking jitter never trips this and
      // forces an unnecessary discontinuity); the incident's actual
      // divergence was ~4m (spline belief ~4.88m vs real odom ~0.56m) --
      // 0.75m catches that kind of drift more than 5x earlier, while it's
      // still small, instead of only detecting it after a search failure
      // (which, per this investigation, can be many replan cycles later).
      const double kStartStateDivergenceTol = 0.75;
      const double start_state_divergence = (spline_start_pt - odom_pos_).norm();
      // TEMPORARY, verify-flight instrumentation only: unconditional per-cycle
      // record of the divergence value and whether the snap below fires,
      // regardless of outcome -- the WARN a few lines down only logs on
      // trigger, which can't show the full distribution (e.g. "never got
      // close" vs "repeatedly grazed the threshold").
      RCUTILS_LOG_INFO(
          "DEBUG_STARTSTATE divergence=%.3f snapped=%d spline=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f)",
          start_state_divergence, start_state_divergence > kStartStateDivergenceTol ? 1 : 0,
          spline_start_pt(0), spline_start_pt(1), spline_start_pt(2), odom_pos_(0), odom_pos_(1),
          odom_pos_(2));
      if (start_state_divergence > kStartStateDivergenceTol) {
        RCUTILS_LOG_WARN(
            "REPLAN_TRAJ start-state divergence %.3fm exceeds %.2fm tolerance "
            "(spline belief=(%.3f,%.3f,%.3f) real odom=(%.3f,%.3f,%.3f)) -- "
            "re-grounding start-state to real odometry.",
            start_state_divergence, kStartStateDivergenceTol, spline_start_pt(0), spline_start_pt(1),
            spline_start_pt(2), odom_pos_(0), odom_pos_(1), odom_pos_(2));
        // Full snap to real state, not a blend -- a partial blend would
        // still seed the new plan with a known-untrustworthy component of
        // the old, already-diverged belief, undermining the very check
        // just performed. Mirrors GEN_NEW_TRAJ's own real-state block
        // (:157-161) exactly: position and velocity are real fields off
        // /odom_world; acceleration has no real-odometry equivalent
        // (nav_msgs/Odometry carries no accel field) so it's zeroed, same
        // as GEN_NEW_TRAJ already does -- leaving accel spline-evaluated
        // here while pos/vel snap to odom would seed the optimizer with a
        // physically-incoherent state (an acceleration computed for a
        // different position/velocity than the one actually being used).
        start_pt_  = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
        start_yaw_(0) = atan2(odom_orient_.toRotationMatrix()(1, 0), odom_orient_.toRotationMatrix()(0, 0));
        start_yaw_(1) = start_yaw_(2) = 0.0;
      } else {
        start_pt_  = spline_start_pt;
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];
      }

      // Same pass-through-or-frontier decision as GEN_NEW_TRAJ, recomputed
      // fresh against THIS cycle's start_pt_ (the receding-horizon point,
      // not the original mission start) -- see that case's comment for
      // the full rationale. No new FSM state here either.
      {
        auto sdf_map = planner_manager_->edt_environment_->sdf_map_;
        const bool goal_ready = sdf_map->isObserved(end_pt_) && sdf_map->getDistance(end_pt_) > 0.3;
        frontier_pt_ = goal_ready ? end_pt_ : computeFrontier(start_pt_, end_pt_);
        // TEMPORARY, milestone-1 static verification only.
        RCUTILS_LOG_INFO(
            "DEBUG_FRONTIER pass_through=%d end_pt=(%.3f,%.3f,%.3f) frontier_pt=(%.3f,%.3f,%.3f) "
            "start_pt=(%.3f,%.3f,%.3f)",
            goal_ready ? 1 : 0, end_pt_(0), end_pt_(1), end_pt_(2), frontier_pt_(0), frontier_pt_(1),
            frontier_pt_(2), start_pt_(0), start_pt_(1), start_pt_(2));
      }

      std_msgs::msg::Empty replan_msg;
      replan_pub_->publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        onReplanSuccess();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        onReplanFailure();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

bool KinoReplanFSM::replanBackoffReady() {
  if (consecutive_replan_failures_ == 0) return true;
  return node_->now() >= next_replan_attempt_time_;
}

void KinoReplanFSM::onReplanFailure() {
  // Exponential backoff, capped: 20ms * 2^(failures-1), capped at 1s.
  // failures=1 -> 20ms, 2 -> 40ms, 3 -> 80ms, ... saturating at 1s. Base
  // case (first failure) still allows a near-immediate retry since a
  // single transient miss is common and cheap; the cap keeps a
  // persistently-failing search from ever going fully silent for more
  // than 1s at a time, so it still gets a fair, if infrequent, chance to
  // notice newly-fetched map data.
  consecutive_replan_failures_ = std::min(consecutive_replan_failures_ + 1, 10);
  const double backoff_s = std::min(0.02 * std::pow(2.0, consecutive_replan_failures_ - 1), 1.0);
  next_replan_attempt_time_ = node_->now() + rclcpp::Duration::from_seconds(backoff_s);
}

void KinoReplanFSM::onReplanSuccess() {
  consecutive_replan_failures_ = 0;
}

void KinoReplanFSM::checkCollisionCallback() {
  LocalTrajData* info = &planner_manager_->local_data_;

  // TEMPORARY, verify-flight instrumentation only: settles the open
  // question from the goal-flight collision -- was the obstacle the
  // vehicle hit already observed by the map at the time (occlusion
  // theorem correctly waiting on data it didn't have yet), or was it
  // observed-and-blocked already, with the planner routing through it
  // anyway (a real collision-avoidance gap)? A post-hoc query against the
  // live map couldn't answer this -- the local map window had already
  // re-centered on the resting position and aged the historical region
  // out by the time this was checked. Logged unconditionally, every tick,
  // so it can be correlated against the real odom trajectory afterward
  // regardless of which moment turns out to matter. pillar_03's position
  // (0, 2.5, 5) is specific to this test world's vio_test.sdf, not a
  // general mechanism -- this is a diagnostic for one open question, not
  // a permanent addition.
  {
    const Eigen::Vector3d kPillar03(0.0, 2.5, 5.0);
    auto sdf_map = planner_manager_->edt_environment_->sdf_map_;
    bool p3_observed = sdf_map->isObserved(kPillar03);
    double p3_dist = sdf_map->getDistance(kPillar03);
    double vehicle_dist_to_p3 = (odom_pos_ - kPillar03).norm();
    RCUTILS_LOG_INFO(
        "DEBUG_OBSTACLE_CHECK pillar03_observed=%d pillar03_esdf_dist=%.3f "
        "vehicle_dist_to_pillar03=%.3f odom=(%.3f,%.3f,%.3f)",
        p3_observed ? 1 : 0, p3_dist, vehicle_dist_to_p3, odom_pos_(0), odom_pos_(1), odom_pos_(2));
  }

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    // Gate on isObserved(end_pt_): evaluateCoarseEDT -> sdf_map_->getDistance
    // cannot tell "confirmed blocked" from "never observed" (both read
    // esdf_unknown_value_, ~0). Without this gate, dist<=0.3 fires
    // continuously for any goal nvblox simply hasn't seen yet -- not a
    // hypothetical, this is the exact mechanism that silently substituted
    // end_pt_ away from the real goal and truncated a real flight 2.02m
    // short. Only evaluate/substitute when the true goal is confirmed
    // observed AND blocked, never when it's merely not-yet-seen.
    if (edt_env->sdf_map_->isObserved(end_pt_) && dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;

        // Root cause of a real crash (SIGABRT): this branch used to force
        // REPLAN_TRAJ unconditionally, with no check of exec_state_ --
        // unlike the other two REPLAN_TRAJ-forcing branches in this same
        // function (both already gated on exec_state_ == EXEC_TRAJ, see
        // above and checkTrajCollision's branch below). REPLAN_TRAJ
        // (execFSMCallback's REPLAN_TRAJ case) unconditionally reads
        // planner_manager_->local_data_ and calls evaluateDeBoorT on its
        // position_traj_ -- valid only once a plan has actually
        // succeeded (GEN_NEW_TRAJ or REPLAN_TRAJ's own success path, both
        // of which transition to EXEC_TRAJ). If this fired while still in
        // GEN_NEW_TRAJ (no plan ever produced yet -- exactly what happens
        // when end_pt_ itself sits in unobserved space, so this branch
        // fires every 50ms with nothing better to switch to), local_data_
        // .position_traj_ is a default-constructed, empty NonUniformBspline
        // -- evaluateDeBoorT then indexes into its empty control-point/knot
        // arrays, hitting an Eigen out-of-bounds assertion. Gating this the
        // same way the other two branches already are fixes it at the
        // actual invalid-transition root cause, not just the empty-spline
        // symptom. When NOT in EXEC_TRAJ, GEN_NEW_TRAJ's own retry loop
        // (with its own backoff) is already retrying toward the same
        // end_pt_ regardless -- this branch forcing an extra, redundant
        // transition was never load-bearing for that retry behavior.
        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

        std_msgs::msg::Empty emt;
        replan_pub_->publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      RCUTILS_LOG_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }

  /* ---------- Lỗi 3: continuous tracking-divergence watchdog ---------- */
  // Lỗi 1/2 fixed the planner's own BELIEF, at replan BOUNDARIES only:
  // checkTrajCollision (above) stopped false-triggering on unobserved
  // space, and REPLAN_TRAJ's start-state now re-grounds to real odom when
  // it diverges from the spline by more than kStartStateDivergenceTol.
  // Neither watches the real vehicle BETWEEN boundaries, while a
  // trajectory is actively executing -- and a real flight showed exactly
  // that gap: altitude fell ~3m in ~4.4s (~0.68 m/s average divergence
  // growth) while the commanded plan stayed flat, undetected until the
  // -0.5m ground-strike floor caught it, far too late to be a useful
  // planning signal.
  //
  // Gated on EXEC_TRAJ, same as checkTrajCollision above -- meaningless
  // otherwise (no trajectory is being executed in any other state).
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ && info->position_traj_.getControlPoint().rows() > 0) {
    rclcpp::Time time_now = node_->now();
    double       t_cur    = (time_now - info->start_time_).seconds();
    t_cur                 = std::min(info->duration_, std::max(0.0, t_cur));

    Eigen::Vector3d commanded_pos = info->position_traj_.evaluateDeBoorT(t_cur);

    // Threshold: reuses REPLAN_TRAJ's own kStartStateDivergenceTol value
    // (0.75m) -- same physical meaning (normal tracking noise tops out
    // ~0.1-0.3m across prior flights, this is a ~2.5x margin above that
    // floor). What differs isn't the threshold, it's the CADENCE: the
    // REPLAN_TRAJ check only fires at a replan boundary, which can be
    // seconds apart; this runs every 50ms, this function's own tick rate.
    // At the incident's own observed divergence rate (~0.68 m/s), a real
    // divergence crosses 0.75m in ~1.1s of continuous drift -- caught
    // here within a tick or two of that, instead of waiting for whatever
    // replan boundary comes next (or, absent this check entirely, the
    // -0.5m ground-strike floor).
    const double kTrackingDivergenceHoldTol = 0.75;
    const double tracking_divergence = (commanded_pos - odom_pos_).norm();

    // TEMPORARY, verify-flight instrumentation only: unconditional
    // per-cycle record, same pattern as REPLAN_TRAJ's DEBUG_STARTSTATE.
    RCUTILS_LOG_INFO(
        "DEBUG_WATCHDOG divergence=%.3f commanded=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f)",
        tracking_divergence, commanded_pos(0), commanded_pos(1), commanded_pos(2), odom_pos_(0),
        odom_pos_(1), odom_pos_(2));

    if (tracking_divergence > kTrackingDivergenceHoldTol && node_->now() >= next_watchdog_hold_allowed_time_) {
      // Response: HOLD, not a forward replan. A forward replan still
      // commands the vehicle further through an optimizer that assumes it
      // can track the result -- if tracking itself is what's failing,
      // asking for MORE commanded motion isn't obviously safer than
      // asking for less. Holding publishes a short, stationary trajectory
      // pinned to the REAL odom position (not the diverged commanded
      // one), giving the controller a bounded window to actually catch up
      // before anything asks it to move again.
      RCUTILS_LOG_WARN(
          "Lỗi 3 WATCHDOG: tracking divergence %.3fm exceeds %.2fm tolerance "
          "(commanded=(%.3f,%.3f,%.3f) real odom=(%.3f,%.3f,%.3f)) -- holding in place, "
          "not replanning forward.",
          tracking_divergence, kTrackingDivergenceHoldTol, commanded_pos(0), commanded_pos(1),
          commanded_pos(2), odom_pos_(0), odom_pos_(1), odom_pos_(2));
      triggerTrackingDivergenceHold();

      // Cooldown, not a one-shot latch: without this, a still-diverged
      // vehicle would retrigger a brand new hold on literally the next
      // 50ms tick, each one restarting the settle window from zero and
      // starving EXEC_TRAJ's own logic (execFSMCallback) of the chance to
      // ever reach its "t_cur > duration_" branch -- which is what
      // naturally hands back off to REPLAN_TRAJ once the hold's short
      // duration elapses (see triggerTrackingDivergenceHold()'s own
      // comment for why that duration is 1.5s). Matches that duration so
      // the earliest a second hold can trigger is right as the first one
      // would otherwise have handed off anyway.
      const double kHoldCooldownS = 1.5;
      next_watchdog_hold_allowed_time_ = node_->now() + rclcpp::Duration::from_seconds(kHoldCooldownS);
    }
  }
}

void KinoReplanFSM::triggerTrackingDivergenceHold() {
  // Odom-source staleness guard -- see header comment (goalflight24
  // crash). odom_pos_ is only trustworthy as "real, right now" if it was
  // actually received recently; if the odom source itself stalled,
  // publishing a hold pinned to it would pin to a phantom position
  // instead of a real one -- exactly what caused the crash. Skip
  // publishing entirely rather than guess: whatever trajectory is
  // already active keeps running untouched, which is strictly safer
  // than committing to a reading that might be several seconds old.
  const double kOdomStaleS = 0.5;
  if (!have_last_odom_recv_time_ ||
      (node_->now() - last_odom_recv_time_).seconds() > kOdomStaleS) {
    RCUTILS_LOG_WARN(
        "ODOM STALENESS GUARD: skipping tracking-divergence hold -- last odom received %.3fs "
        "ago (limit %.2fs), can't trust it as 'real, right now'.",
        have_last_odom_recv_time_ ? (node_->now() - last_odom_recv_time_).seconds() : -1.0,
        kOdomStaleS);
    return;
  }

  LocalTrajData* info = &planner_manager_->local_data_;

  // A uniform B-spline whose control points are ALL EQUAL is exactly
  // constant everywhere (partition of unity), so this evaluates to
  // hold_pos at every t -- built directly via setUniformBspline, NOT
  // planner_manager_->kinodynamicReplan(): asking the optimizer for a
  // "replan to the same point" would very likely hit its own <0.2m
  // "Close goal" refusal (planner_manager.cpp, start_pt==end_pt here by
  // construction), and even if it didn't, running the full search+
  // optimization pipeline is exactly the "more commanded motion through
  // an optimizer that assumes trackable output" this hold exists to
  // avoid -- this needs to be structurally incapable of commanding
  // forward motion, not just unlikely to.
  //
  // order=3 (matches every other spline in this project), interval=0.5s,
  // 6 control points -> duration = (rows-order)*interval = 1.5s, per
  // NonUniformBspline::setUniformBspline's own uniform-knot formula.
  // Matches kHoldCooldownS in checkCollisionCallback.
  const Eigen::Vector3d hold_pos  = odom_pos_;  // real position, not the diverged commanded one
  const int             kHoldOrder    = 3;
  const double          kHoldInterval = 0.5;
  const int             kHoldCtrlPts  = 6;

  Eigen::MatrixXd pos_ctrl(kHoldCtrlPts, 3);
  for (int i = 0; i < kHoldCtrlPts; ++i) pos_ctrl.row(i) = hold_pos.transpose();

  info->position_traj_.setUniformBspline(pos_ctrl, kHoldOrder, kHoldInterval);
  info->velocity_traj_     = info->position_traj_.getDerivative();
  info->acceleration_traj_ = info->velocity_traj_.getDerivative();
  info->start_pos_         = info->position_traj_.evaluateDeBoorT(0.0);
  info->duration_          = info->position_traj_.getTimeSum();
  info->start_time_        = node_->now();
  info->traj_id_ += 1;

  const double hold_yaw =
      atan2(odom_orient_.toRotationMatrix()(1, 0), odom_orient_.toRotationMatrix()(0, 0));
  Eigen::MatrixXd yaw_ctrl(kHoldCtrlPts, 1);
  for (int i = 0; i < kHoldCtrlPts; ++i) yaw_ctrl(i, 0) = hold_yaw;
  info->yaw_traj_.setUniformBspline(yaw_ctrl, kHoldOrder, kHoldInterval);
  info->yawdot_traj_    = info->yaw_traj_.getDerivative();
  info->yawdotdot_traj_ = info->yawdot_traj_.getDerivative();

  // Same publish shape as callKinodynamicReplan() -- traj_server rebuilds
  // its own NonUniformBspline purely from pos_pts+knots+order (and
  // yaw_pts+yaw_dt), it has no idea this came from a hold instead of a
  // real replan.
  quadrotor_msgs::msg::Bspline bspline;
  bspline.order      = kHoldOrder;
  bspline.start_time = info->start_time_;
  bspline.traj_id    = info->traj_id_;

  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  for (int i = 0; i < pos_pts.rows(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = pos_pts(i, 0);
    pt.y = pos_pts(i, 1);
    pt.z = pos_pts(i, 2);
    bspline.pos_pts.push_back(pt);
  }

  Eigen::VectorXd knots = info->position_traj_.getKnot();
  for (int i = 0; i < knots.rows(); ++i) bspline.knots.push_back(knots(i));

  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
  for (int i = 0; i < yaw_pts.rows(); ++i) bspline.yaw_pts.push_back(yaw_pts(i, 0));
  bspline.yaw_dt = info->yaw_traj_.getInterval();

  bspline_pub_->publish(bspline);

  RCUTILS_LOG_WARN(
      "Lỗi 3 WATCHDOG: published %.2fs hold trajectory pinned to real odom (%.3f,%.3f,%.3f), "
      "traj_id=%d.",
      info->duration_, hold_pos(0), hold_pos(1), hold_pos(2), info->traj_id_);
}

double KinoReplanFSM::marchRay(const Eigen::Vector3d& from, const Eigen::Vector3d& dir, double max_dist) {
  auto sdf_map = planner_manager_->edt_environment_->sdf_map_;
  const double step = 0.1;  // matches sdf_map/resolution (fast_planner_px4.launch.py)

  // Matches the start-state exemption radius's own default (0.3m,
  // esdf_start_state_clearing_radius_m_ in sdf_map.cpp) -- the near-field
  // blind spot is a KNOWN structural fact (Task C's diagnosis: 0%
  // observed within 0.3m of the camera), not a sign of real unexplored
  // space, so the march treats it as unconditionally passable regardless
  // of isObserved(), exactly like getDistance/getInflateOccupancy already
  // do for collision-checking. This is also what structurally rules out
  // edge case (a) -- "first step already unobserved" -- from ever
  // occurring: step 0 (=`from` itself) is always at distance 0 from
  // `from`, always inside this radius, always passable by construction.
  // Hardcoded (not read from SDFMap, which doesn't expose the value
  // publicly) -- matches the same already-established pattern as
  // checkCollisionCallback's own `dist <= 0.3` magic number, right above.
  const double kExemptionRadius = 0.3;

  double last_good_dist = 0.0;  // `from` is always "good" (dist=0, within its own exemption)
  for (double d = step; d <= max_dist + 1e-6; d += step) {
    const double clamped_d = std::min(d, max_dist);
    const Eigen::Vector3d p = from + dir * clamped_d;
    const bool within_exemption = clamped_d <= kExemptionRadius;
    const bool observed_free = sdf_map->isObserved(p) && sdf_map->getInflateOccupancy(p) == 0;
    if (!(within_exemption || observed_free)) break;
    last_good_dist = clamped_d;
  }

  // Pull back by one step (consistent with the existing inflation
  // margin's spirit -- don't target the exact last-known-good voxel,
  // leave a cushion), then floor at kExemptionRadius so the result can
  // never land closer to `from` than that -- which in turn guarantees it
  // clears kinodynamicReplan()'s own "Close goal" refusal (planner_
  // manager.cpp:151, <0.2m) with 0.1m of real margin, handling edge case
  // (b). Finally capped at max_dist so the floor can never push the
  // frontier past the real goal when the goal itself is closer than
  // kExemptionRadius (the pass-through gate should normally have already
  // caught that case, but this keeps the march itself safe even if
  // called directly).
  double frontier_dist = std::max(last_good_dist - step, kExemptionRadius);
  frontier_dist = std::min(frontier_dist, max_dist);
  return frontier_dist;
}

Eigen::Vector3d KinoReplanFSM::computeFrontier(const Eigen::Vector3d& from, const Eigen::Vector3d& goal) {
  const Eigen::Vector3d delta = goal - from;
  const double total_dist = delta.norm();
  if (total_dist < 1e-3) return from;  // degenerate: already at goal, nothing to march

  const Eigen::Vector3d dir = delta / total_dist;

  // Yaw-only rotation of the CURRENT direct-to-goal direction -- every
  // candidate, including the hysteresis "continuity" one below, is
  // rebuilt from this fresh `dir` every call, never from a stored
  // absolute vector (see last_frontier_angle_deg_'s declaration comment
  // for why: an absolute vector let goalflight16 run away toward the
  // 15m horizontal killswitch limit).
  auto angleToDir = [&](double deg) {
    const double rad = deg * M_PI / 180.0;
    const double cos_a = std::cos(rad), sin_a = std::sin(rad);
    Eigen::Vector3d d(dir(0) * cos_a - dir(1) * sin_a, dir(0) * sin_a + dir(1) * cos_a, dir(2));
    d.normalize();
    return d;
  };

  double best_dist = marchRay(from, dir, total_dist);
  Eigen::Vector3d best_dir = dir;
  double best_angle_deg = 0.0;

  // goalflight2/3/4 root cause: pillar_03 sits directly on the line from
  // spawn to B, so the direct ray above is blocked at nearly the same
  // point every replan cycle -- the search is then handed a target
  // hugging the obstacle's near face, with no incentive to detour, and
  // three consecutive flights collided right there (DEBUG_OBSTACLE_CHECK/
  // DEBUG_WATCHDOG both confirmed the obstacle was mapped throughout; the
  // gap was never perception, it was this single-ray targeting). If the
  // direct ray is blocked well short of the real goal (not just "close
  // to goal because goal is close"), also try rays fanned out in yaw
  // around it and keep whichever clears farthest -- a cheap stand-in for
  // full topological path search (use_topo_path exists but is a much
  // larger, separate lift) that's enough to route around a single
  // obstacle centered on the direct line.
  const double kBlockedMargin = 0.5;  // meters short of total_dist to count as "actually blocked"
  if (total_dist - best_dist > kBlockedMargin) {
    // Yaw-only fan: every obstacle in this world (the 8-pillar ring) is
    // a vertical column spanning the full flight altitude range, so
    // lateral (yaw) detours are what's structurally available --
    // pitching the ray up/down buys nothing against a pillar that's
    // solid top to bottom.
    static const double kCandidateAnglesDeg[] = {15, -15, 30, -30, 45, -45, 60, -60};
    for (double deg : kCandidateAnglesDeg) {
      const Eigen::Vector3d cand_dir = angleToDir(deg);
      const double cand_dist = marchRay(from, cand_dir, total_dist);
      if (cand_dist > best_dist) {
        best_dist = cand_dist;
        best_dir = cand_dir;
        best_angle_deg = deg;
      }
    }

    // Hysteresis (goalflight13 root cause): the argmax above is
    // recomputed from scratch every cycle -- nothing stops it from
    // flipping to the opposite side of the obstacle between two cycles
    // if the ESDF updates just enough to nudge which candidate edges out
    // the other. Real incident: two cycles 7s apart, real forward
    // progress in between, frontier target flipped 5.8m in x (a full
    // left-detour <-> right-detour reversal); the FSM commanded that
    // reversal as a real setpoint change and the vehicle lost altitude
    // control trying to track it. Fix: once committed to a detour side,
    // require a genuinely better candidate (not just the nominal
    // argmax) before abandoning it -- re-evaluate the previous cycle's
    // chosen ANGLE, reapplied to the CURRENT direct-to-goal direction
    // (not a stale absolute vector), and only switch away if the new
    // best beats it by more than kFrontierSwitchMargin.
    const double kFrontierSwitchMargin = 1.5;  // meters -- must clear meaningfully farther, not marginally
    if (have_last_frontier_angle_) {
      const Eigen::Vector3d continuity_dir = angleToDir(last_frontier_angle_deg_);
      const double continuity_dist = marchRay(from, continuity_dir, total_dist);
      if (best_dist - continuity_dist < kFrontierSwitchMargin) {
        best_dist = continuity_dist;
        best_dir = continuity_dir;
        best_angle_deg = last_frontier_angle_deg_;
      }
    }
    have_last_frontier_angle_ = true;
    last_frontier_angle_deg_ = best_angle_deg;
  } else {
    // Direct ray is clear (or the fan was never entered) -- no detour
    // side is currently committed to, so nothing to persist. Next time
    // the direct ray IS blocked, the fan starts fresh with no stale
    // preference from a now-irrelevant obstacle passed long ago.
    have_last_frontier_angle_ = false;
  }

  return from + best_dir * best_dist;
}

bool KinoReplanFSM::callKinodynamicReplan() {
  // frontier_pt_, not end_pt_ -- the receding-horizon target for THIS
  // cycle (either a pass-through to the true goal, or the ray-marched
  // observed frontier; see GEN_NEW_TRAJ/REPLAN_TRAJ, both of which
  // compute it fresh before calling this). planner_manager_->
  // kinodynamicReplan() itself stays goal-agnostic -- it has no idea
  // whether this is the real goal or an intermediate point.
  auto plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, frontier_pt_, end_vel_);
  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    quadrotor_msgs::msg::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_->publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0,0, 0.0, 1), true, 0.2, Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

}  // namespace fast_planner
