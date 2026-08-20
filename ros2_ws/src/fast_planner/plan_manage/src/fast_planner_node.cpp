
#include <fast_planner/fast_planner.h>
#include <plan_manage/kino_replan_fsm.h>

using namespace fast_planner;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastPlanner>();

  node->declare_parameter<float>("bspline/limit_vel", 0);
  node->declare_parameter<float>("bspline/limit_acc", 0);
  node->declare_parameter<float>("bspline/limit_ratio", 0);

  node->declare_parameter<int>("planner_node/planner", -1);
  rclcpp::Parameter planner_param = node->get_parameter("planner_node/planner"); 

  int planner = planner_param.as_int();
  // RCLCPP_INFO(node->get_logger(), "The planner value is (int) : %s",
  //               planner_param.value_to_string().c_str());

  // # Initialize the kino Class
  std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>();
  
  if (planner == 1) {
      kino_replan->init(node);
  } else if (planner == 2) {
      std::cout << ("TOPO Commented for now ");
  }

  // REVERTED (goalflight28, same night as the attempt): a MultiThreadedExecutor
  // was tried here to let SDFMap's map work (vis_timer_/ESDF fetch/esdf_client_,
  // all in map_cb_group_) run concurrently with KinoReplanFSM's control work
  // (exec_timer_/safety_timer_/odom_sub_, all in control_cb_group_) instead of
  // serializing on one thread. It DID eliminate the executor-stall livelock
  // (0 stalls across the whole test flight, versus 100+ before) -- but it also
  // introduced a new, worse failure: kinodynamic search started failing on
  // nearly every attempt, including trivial ~0.3m hops in open space nowhere
  // near an obstacle (logged: start=(0.06,0.28,5) -> goal=(0.06,0.58,5),
  // "kinodynamic search fail!", repeatedly). SDFMap's buffers
  // (occupancy_buffer_inflate_/distance_buffer_all_/local_bound_min_/max_) are
  // plain vectors written in-place by onEsdfResponse() with no synchronization
  // against concurrent reads from the search thread -- safe from crashes
  // (fixed-size, no reallocation) but NOT safe against reading a
  // partway-through-update state (e.g. local_bound_ recentered before that
  // cycle's buffer fill catches up, defaulting nearby cells back to
  // "occupied" under the unknown->occupied policy right when search reads
  // them). That's a real concurrency bug, not something to leave running on a
  // flight-safety-relevant node -- reverted back to the single-threaded
  // executor (proven safe, if slow-under-load, per the report's known
  // limitation) rather than iterate on a race condition live. The
  // control_cb_group_/map_cb_group_ wiring is left in place elsewhere
  // (harmless no-op under a single-threaded executor) so a future attempt
  // that adds proper synchronization around SDFMap's buffers doesn't have to
  // redo the callback-group plumbing.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
