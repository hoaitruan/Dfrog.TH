/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "plan_env/sdf_map.h"
#include "rcutils/logging_macros.h"

#define current_img_ md_.depth_image_[image_cnt_ & 1]
#define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void SDFMap::initMap(std::shared_ptr<FastPlanner> node) {
  node_ = node;
  RCLCPP_INFO(node->get_logger(), "Entered sdf entry point ! ");

  // See map_cb_group_'s declaration in sdf_map.h (goalflight27 livelock)
  // for the full rationale -- every timer/subscription/client this map
  // owns below is created against this group so a MultiThreadedExecutor
  // can run them off the flight-control thread entirely.
  map_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  node->declare_parameter<float>("sdf_map/resolution", 0.5);
  node->declare_parameter<float>("sdf_map/map_size_x", 0.5);
  node->declare_parameter<float>("sdf_map/map_size_y", 0.5);
  node->declare_parameter<float>("sdf_map/map_size_z", 0.5);
  node->declare_parameter<float>("sdf_map/local_update_range_x", 0.5);
  node->declare_parameter<float>("sdf_map/local_update_range_y", 0.5);
  node->declare_parameter<float>("sdf_map/local_update_range_z", 0.5);
  node->declare_parameter<float>("sdf_map/obstacles_inflation", 0.5);
  node->declare_parameter<float>("sdf_map/local_bound_inflate", 0.5);
  node->declare_parameter<int>("sdf_map/local_map_margin", 5);
  node->declare_parameter<float>("sdf_map/ground_height", 0.5);
  node->declare_parameter<float>("sdf_map/cx", 0.5);
  node->declare_parameter<float>("sdf_map/cy", 0.5);
  node->declare_parameter<float>("sdf_map/fx", 0.5);
  node->declare_parameter<float>("sdf_map/fy", 0.5);
  node->declare_parameter<bool>("sdf_map/use_depth_filter", "true");
  node->declare_parameter<float>("sdf_map/depth_filter_tolerance", 0.5);
  node->declare_parameter<float>("sdf_map/depth_filter_maxdist", 0.5);
  node->declare_parameter<float>("sdf_map/depth_filter_mindist", 0.5);
  node->declare_parameter<int>("sdf_map/depth_filter_margin", 1);
  node->declare_parameter<float>("sdf_map/k_depth_scaling_factor", 0.5);
  node->declare_parameter<int>("sdf_map/skip_pixel", 0.5);
  node->declare_parameter<float>("sdf_map/p_hit", 0.5);
  node->declare_parameter<float>("sdf_map/p_miss", 0.5);
  node->declare_parameter<float>("sdf_map/p_min", 0.5);
  node->declare_parameter<float>("sdf_map/p_max", 0.5);
  node->declare_parameter<float>("sdf_map/p_occ", 0.5);
  node->declare_parameter<float>("sdf_map/min_ray_length", 0.5);
  node->declare_parameter<float>("sdf_map/max_ray_length", 0.5);
  node->declare_parameter<float>("sdf_map/esdf_slice_height", 0.5);
  node->declare_parameter<float>("sdf_map/visualization_truncate_height", 0.5);
  node->declare_parameter<float>("sdf_map/virtual_ceil_height", 0.5);
  node->declare_parameter<bool>("sdf_map/show_occ_time", false);
  node->declare_parameter<bool>("sdf_map/show_esdf_time", 0.5);
  node->declare_parameter<int>("sdf_map/pose_type", 0);
  node->declare_parameter<string>("sdf_map/frame_id", "map");
  node->declare_parameter<float>("sdf_map/esdf_unknown_value", 0.0);
  node->declare_parameter<float>("sdf_map/esdf_near_field_radius", 1.5);
  node->declare_parameter<float>("sdf_map/esdf_near_fetch_period_s", 0.25);
  node->declare_parameter<float>("sdf_map/esdf_corridor_fetch_period_s", 1.5);
  node->declare_parameter<float>("sdf_map/esdf_corridor_half_extent", 3.2);
  node->declare_parameter<float>("sdf_map/esdf_start_state_clearing_radius_m", 0.3);

  /* get parameter */
  double x_size, y_size, z_size;
  node->get_parameter_or("sdf_map/resolution", mp_.resolution_, -1.0);
  RCLCPP_INFO(node->get_logger(), "Resolution value: %f", mp_.resolution_);
  node->get_parameter_or("sdf_map/map_size_x", x_size, -1.0);
  node->get_parameter_or("sdf_map/map_size_y", y_size, -1.0);
  node->get_parameter_or("sdf_map/map_size_z", z_size, -1.0);
  node->get_parameter_or("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node->get_parameter_or("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node->get_parameter_or("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node->get_parameter_or("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node->get_parameter_or("sdf_map/fx", mp_.fx_, -1.0);
  node->get_parameter_or("sdf_map/fy", mp_.fy_, -1.0);
  node->get_parameter_or("sdf_map/cx", mp_.cx_, -1.0);
  node->get_parameter_or("sdf_map/cy", mp_.cy_, -1.0);

  node->get_parameter_or("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
  node->get_parameter_or("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node->get_parameter_or("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node->get_parameter_or("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node->get_parameter_or("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node->get_parameter_or("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node->get_parameter_or("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  node->get_parameter_or("sdf_map/p_hit", mp_.p_hit_, 0.70);
  node->get_parameter_or("sdf_map/p_miss", mp_.p_miss_, 0.35);
  node->get_parameter_or("sdf_map/p_min", mp_.p_min_, 0.12);
  node->get_parameter_or("sdf_map/p_max", mp_.p_max_, 0.97);
  node->get_parameter_or("sdf_map/p_occ", mp_.p_occ_, 0.80);
  node->get_parameter_or("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node->get_parameter_or("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node->get_parameter_or("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  node->get_parameter_or("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node->get_parameter_or("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  node->get_parameter_or("sdf_map/show_occ_time", mp_.show_occ_time_, false);
  node->get_parameter_or("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
  node->get_parameter_or("sdf_map/pose_type", mp_.pose_type_, 1);

  node->get_parameter_or("sdf_map/frame_id", mp_.frame_id_, string("world"));
  node->get_parameter_or("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  node->get_parameter_or("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
  node->get_parameter_or("sdf_map/ground_height", mp_.ground_height_, 1.0);
  node->get_parameter_or("sdf_map/esdf_unknown_value", mp_.esdf_unknown_value_, 0.0);
  node->get_parameter_or("sdf_map/esdf_near_field_radius", mp_.esdf_near_field_radius_, 1.5);
  node->get_parameter_or("sdf_map/esdf_near_fetch_period_s", mp_.esdf_near_fetch_period_s_, 0.25);
  node->get_parameter_or(
      "sdf_map/esdf_corridor_fetch_period_s", mp_.esdf_corridor_fetch_period_s_, 1.5);
  node->get_parameter_or(
      "sdf_map/esdf_corridor_half_extent", mp_.esdf_corridor_half_extent_, 3.2);
  node->get_parameter_or(
      "sdf_map/esdf_start_state_clearing_radius_m",
      mp_.esdf_start_state_clearing_radius_m_, 0.3);

  // Initialize other parameters
    mp_.local_bound_inflate_ = std::max(mp_.resolution_, mp_.local_bound_inflate_);
    mp_.resolution_inv_ = 1 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    mp_.prob_hit_log_ = logit(mp_.p_hit_);
    mp_.prob_miss_log_ = logit(mp_.p_miss_);
    mp_.clamp_min_log_ = logit(mp_.p_min_);
    mp_.clamp_max_log_ = logit(mp_.p_max_);
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);
    mp_.unknown_flag_ = 0.01;

    cout << "hit: " << mp_.prob_hit_log_ << endl;
    cout << "miss: " << mp_.prob_miss_log_ << endl;
    cout << "min log: " << mp_.clamp_min_log_ << endl;
    cout << "max: " << mp_.clamp_max_log_ << endl;
    cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

    for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

    mp_.map_min_idx_ = Eigen::Vector3i::Zero();
    mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();



    // Initialize buffers
    mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
    mp_.resolution_inv_ = 1 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    mp_.prob_hit_log_ = logit(mp_.p_hit_);
    mp_.prob_miss_log_ = logit(mp_.p_miss_);
    mp_.clamp_min_log_ = logit(mp_.p_min_);
    mp_.clamp_max_log_ = logit(mp_.p_max_);
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);
    mp_.unknown_flag_ = 0.01;

    cout << "hit: " << mp_.prob_hit_log_ << endl;
    cout << "miss: " << mp_.prob_miss_log_ << endl;
    cout << "min log: " << mp_.clamp_min_log_ << endl;
    cout << "max: " << mp_.clamp_max_log_ << endl;
    cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

    for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

    mp_.map_min_idx_ = Eigen::Vector3i::Zero();
    mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

    // initialize data buffers

    int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

    md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
    md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
    // Default 1 = occupied. REVISED after a measured failure: unknown
    // space was originally left "not occupied" (0) so the discrete
    // kinodynamic A* search -- which has no third "unknown" state to use
    // in this buffer -- could still path through never-yet-observed
    // space. That let a real trajectory fly straight through an
    // unobserved section of a real pillar (Task 2c far-goal test).
    // "Never plan through space not confirmed free" is the correct
    // general policy, not a compromise to relax for convenience -- a far,
    // largely-unobserved goal now correctly yields no path (or a path
    // only to the observed frontier) instead of a false one. Safely
    // progressing into unobserved space (frontier/receding-horizon
    // planning) is real, separate work -- see ARCHITECTURE.md's
    // Milestone 2 note.
    md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 1);

    md_.distance_buffer_ = vector<double>(buffer_size, 10000);
    md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
    // Initialized to esdf_unknown_value_ (now 0.0, the same boundary
    // value a just-touching-an-obstacle voxel reads) -- see the
    // esdf_unknown_value_ declaration in sdf_map.h for the full
    // rationale and the incident that changed it from 0.2.
    md_.distance_buffer_all_ = vector<double>(buffer_size, mp_.esdf_unknown_value_);
    // Zero-initialized (unobserved) -- matches distance_buffer_all_'s own
    // "nothing seen yet" starting state. Written alongside it in
    // onEsdfResponse() once real nvblox data starts arriving.
    md_.observed_buffer_ = vector<char>(buffer_size, 0);

    md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
    md_.count_hit_ = vector<short>(buffer_size, 0);
    md_.flag_rayend_ = vector<char>(buffer_size, -1);
    md_.flag_traverse_ = vector<char>(buffer_size, -1);

    md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
    md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
    md_.raycast_num_ = 0;

    md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
    md_.proj_points_cnt = 0;


    // Initialize subscribers and publishers
    //
    // Path A Milestone 1: the depth_sub_/pose_sub_/odom_sub_ message_filters
    // synced subscription block (previously wired here to depthPoseCallback/
    // depthOdomCallback, which raycast raw depth into occupancy_buffer_/
    // distance_buffer_all_) is deliberately NOT wired up any more --
    // distance_buffer_all_ and occupancy_buffer_inflate_ are now populated
    // from nvblox's ESDF instead, see esdf_client_ and the
    // esdf_near_fetch_timer_/esdf_corridor_fetch_timer_ callbacks below.
    // depthPoseCallback/depthOdomCallback/projectDepthImage/raycastProcess/
    // updateOccupancyCallback/updateESDFCallback bodies are left in the
    // source, unused, rather than deleted, so the previous path stays
    // readable/revertible. Consequence: has_first_depth_ never becomes
    // true, so odomCallback's `if (has_first_depth_) return` guard never
    // fires and it keeps tracking camera_pos_/has_odom_ from
    // indep_odom_sub_ indefinitely -- this is what the nvblox fetch below
    // uses to center its AABB.
    //
    // if (mp_.pose_type_ == POSE_STAMPED) { ... } else if (mp_.pose_type_ == ODOMETRY) { ... }
    // -- intentionally removed, see comment above.

    rclcpp::SubscriptionOptions map_sub_opts;
    map_sub_opts.callback_group = map_cb_group_;
    indep_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/cloud", 10, std::bind(&SDFMap::cloudCallback, this, std::placeholders::_1),
        map_sub_opts);
    indep_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/sdf_map/odom", 10, std::bind(&SDFMap::odomCallback, this, std::placeholders::_1),
        map_sub_opts);
    subscription_ = node->create_subscription<std_msgs::msg::String>(
        "/sdf/topic_subs", 10, std::bind(&SDFMap::topic_callback, this, std::placeholders::_1),
        map_sub_opts);

    RCLCPP_INFO(node->get_logger(), "Entered sdf entry point  subscriber check .....! ");

    map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/occupancy", 10);
    map_inf_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/occupancy_inflate", 10);
    esdf_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/esdf", 10);
    update_range_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
        "/sdf_map/update_range", 10);
    unknown_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/unknown", 10);
    confirmed_obstacle_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/confirmed_obstacle", 10);
    depth_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sdf_map/depth_cloud", 10);

    // Initialize timers
    // occ_timer_/esdf_timer_ (raycasting-driven) intentionally not created
    // -- see the subscriber-wiring comment above. Replaced by the nvblox
    // fetch timers below.
    //
    // goalflight20-22: this timer's own callback (visCallback -> publishMap/
    // publishMapInflate/publishUnknown/publishConfirmedObstacle, four FULL
    // local_bound_min_..max_ volume iterations, ~1.09M voxels each at the
    // current local_update_range_ of 5.5x5.5x4.5m/0.1m resolution -- ESDF is
    // the only one of the five that's a 2D slice) runs on the SAME single
    // thread as execFSMCallback's safety-critical 10ms tick. Before the
    // local_bound_min_/max_ fix (recentering it on the drone so the
    // visualization topics show real data instead of one fixed corner
    // voxel forever), these loops were a no-op -- local_bound was stuck at
    // a single voxel. Fixing the DATA made the loops real, and at 50ms
    // that's ~4.4M voxel visits every 50ms, indefinitely, competing
    // directly with flight control for the one thread both run on -- a
    // strong candidate contributor to the executor-stall pattern observed
    // live (goalflight20-22), though not isolated in controlled
    // conditions against the other confirmed contender (real system
    // load: rclone/Gazebo/RViz2 all measured spiking independently in the
    // same window). A human watching a debug point cloud has no use
    // for flight-control-rate (20Hz) refreshes -- 2Hz is already more than
    // eye can follow for this kind of display -- so slowing this alone
    // recovers most of the budget without touching anything safety-
    // relevant (map POPULATION -- onEsdfResponse -- is on its own timers,
    // untouched by this).
    vis_timer_ = node->create_wall_timer(
        500ms, std::bind(&SDFMap::visCallback, this), map_cb_group_);

    // nvblox ESDF source (Path A, Milestone 1). Two-tier fetch, both
    // min-pooling into the same uniform-resolution distance_buffer_all_/
    // occupancy_buffer_inflate_ (Fast-Planner's grid has one resolution;
    // near-field and corridor differ only in AABB extent and fetch rate,
    // never in the resolution they write into). Async call_async() +
    // add_done_callback(), never a blocking wait -- fast_planner_node runs
    // rclcpp::spin(node), the plain single-threaded executor, so a
    // blocking call here would deadlock it against itself (the Phase 5
    // nvblox-server deadlock's client-side mirror). in_flight guards
    // prevent an overlapping request queuing up behind a slow one.
    esdf_client_ = node->create_client<nvblox_msgs::srv::EsdfAndGradients>(
        "/nvblox_node/get_esdf_and_gradient", rmw_qos_profile_services_default, map_cb_group_);
    esdf_near_fetch_timer_ = node->create_wall_timer(
        std::chrono::duration<double>(mp_.esdf_near_fetch_period_s_),
        std::bind(&SDFMap::esdfNearFetchCallback, this), map_cb_group_);
    esdf_corridor_fetch_timer_ = node->create_wall_timer(
        std::chrono::duration<double>(mp_.esdf_corridor_fetch_period_s_),
        std::bind(&SDFMap::esdfCorridorFetchCallback, this), map_cb_group_);

    // TEMPORARY, Task 2c verification only -- see debugQueryCallback().
    debug_query_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
        "/sdf_map/debug_query_point", 10,
        std::bind(&SDFMap::debugQueryCallback, this, std::placeholders::_1), map_sub_opts);

    // Initialize variables
    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
    md_.esdf_need_update_ = false;
    md_.has_first_depth_ = false;
    md_.has_odom_ = false;
    md_.has_cloud_ = false;
    md_.image_cnt_ = 0;

    md_.esdf_time_ = 0.0;
    md_.fuse_time_ = 0.0;
    md_.update_num_ = 0;
    md_.max_esdf_time_ = 0.0;
    md_.max_fuse_time_ = 0.0;

    rand_noise_ = std::uniform_real_distribution<double>(-0.2, 0.2);
    rand_noise2_ = std::normal_distribution<double>(0, 0.2);
    std::random_device rd;
    eng_ = std::default_random_engine(rd());
}

void SDFMap::resetBuffer() {
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_[dim]];
  double z[mp_.map_voxel_num_[dim] + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "what?");
        }
      }

  rclcpp::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2] +
                                            y * mp_.map_voxel_num_[2] + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

int SDFMap::setCacheOccupancy(const Eigen::Vector3d& pos, int occ) {
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage() {
  md_.proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; v++) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++) {
        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  } else {
    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else {
      Eigen::Vector3d pt_cur, pt_world;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_) {
          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;
        }
      }
    }
  }

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void SDFMap::raycastProcess() {
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // determine the local bounding box for updating ESDF
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty()) {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

      for (int z = min_cut_m(2); z < min_cut(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

      for (int y = min_cut_m(1); y < min_cut(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void SDFMap::visCallback() {
  // Jetson-load note: on real hardware this runs headless -- nobody has
  // RViz2 open watching a debug point cloud mid-flight. Each of the five
  // publish*() functions below now early-returns via a subscriber-count
  // check (see each one's own guard) rather than doing its full
  // local_bound_min_..max_ voxel iteration (~1.09M voxels apiece)
  // unconditionally every 500ms -- on a dev workstation that was
  // measurable waste (§24); on a Jetson Orin Nano's weaker CPU, during an
  // actual unattended flight with zero viewers, it's the difference
  // between ~5.5M wasted voxel visits/cycle and effectively zero. The
  // guard is per-publisher, not a single flag, so plugging RViz2 back in
  // for a ground-station check still gets full-fidelity output on
  // whichever topics are actually open, no config change needed.
  publishMap();
  publishMapInflate(false);
  // publishUpdateRange();
  publishESDF();

  publishUnknown();
  publishConfirmedObstacle();
  // publishDepth();
}

void SDFMap::updateOccupancyCallback() {
  if (!md_.occ_need_update_) return;

  /* update occupancy */
  rclcpp::Time t1, t2;
  t1 = node_->now();

  projectDepthImage();
  raycastProcess();

  if (md_.local_updated_) clearAndInflateLocalMap();

  t2 = node_->now();

  md_.fuse_time_ += (t2 - t1).seconds();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).seconds());

  if (mp_.show_occ_time_)
    RCUTILS_LOG_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf",(t2 - t1).seconds(),md_.fuse_time_ / md_.update_num_,md_.max_fuse_time_);


  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;
  md_.local_updated_ = false;
}

void SDFMap::updateESDFCallback() {
  if (!md_.esdf_need_update_) return;

  /* esdf */
  rclcpp::Time t1, t2;
  t1 = node_->now();

  updateESDF3d();

  t2 = node_->now();

  md_.esdf_time_ += (t2 - t1).seconds();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).seconds());

  if (mp_.show_esdf_time_)
    RCUTILS_LOG_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).seconds(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}

// ============================================================
// nvblox ESDF source (Path A, Milestone 1)
// ============================================================
//
// Replaces updateOccupancyCallback()/updateESDFCallback() above as the
// source for distance_buffer_all_ and occupancy_buffer_inflate_ -- the
// only two buffers any query consumer (getDistance, getDistWithGradTrilinear,
// getInflateOccupancy, and everything in bspline_opt/ and path_searching/
// that calls through them) actually reads. Confirmed by a full grep of
// every sdf_map_-> / edt_environment_-> call site across bspline_opt/,
// path_searching/, plan_manage/, plan_env/ during the Task 2 design pass --
// no third buffer exists.
//
// Grid alignment: nvblox's ESDF response uses the same row-major,
// x-slowest index order as toAddress() below (confirmed by reading
// esdf_and_gradients_conversions.cu's MultiArrayLayout construction
// against toAddress()'s formula -- both are
// x*(ny*nz) + y*nz + z), and the same corner-origin/center-at-+0.5
// convention as posToIndex()/indexToPos() (confirmed against the .srv
// comment and indexToPos()'s `(id+0.5)*resolution + origin`). No transpose
// or reorder is needed, only an origin subtract and a resolution ratio.
//
// Sign convention: nvblox's SignedDistanceFunctor is positive outside an
// obstacle, negated only `if (is_inside)`; Fast-Planner's own
// updateESDF3d() combines distance_buffer_/distance_buffer_neg_ the same
// way (positive outward distance, negated only for cells that were
// themselves occupied). Confirmed by reading both implementations --
// nvblox's raw distance values are used here unchanged, no sign flip.
//
// Unknown-space handling and the esdf_unknown_value_/obstacles_inflation_
// relationship are documented at their point of use below.

void SDFMap::esdfNearFetchCallback() {
  if (!md_.has_odom_) return;
  const Eigen::Vector3d half_extent(
      mp_.esdf_near_field_radius_, mp_.esdf_near_field_radius_, mp_.esdf_near_field_radius_);
  fetchEsdfBlock(half_extent, &esdf_near_in_flight_);
}

void SDFMap::esdfCorridorFetchCallback() {
  if (!md_.has_odom_) return;
  const Eigen::Vector3d half_extent(
      mp_.esdf_corridor_half_extent_, mp_.esdf_corridor_half_extent_,
      mp_.esdf_corridor_half_extent_);
  fetchEsdfBlock(half_extent, &esdf_corridor_in_flight_);
}

void SDFMap::fetchEsdfBlock(const Eigen::Vector3d& half_extent, bool* in_flight_flag) {
  // in_flight guard: never let a second request for this tier queue up
  // behind a slow one (same pattern as reactive_esdf_avoidance.py's
  // _esdf_query_in_flight, Phase 6). The two tiers guard independently, so
  // a slow corridor fetch never blocks the near-field tier or vice versa.
  if (*in_flight_flag) return;
  if (!esdf_client_->service_is_ready()) return;

  auto request = std::make_shared<nvblox_msgs::srv::EsdfAndGradients::Request>();
  request->update_esdf = true;
  request->visualize_esdf = false;
  request->use_aabb = true;
  // Must match nvblox's own global_frame ("map") exactly, or nvblox
  // returns an empty grid with a warning rather than guessing -- confirmed
  // during the mission audit (Section B1). Reusing mp_.frame_id_ (already
  // configured to "map" in fast_planner_px4.launch.py) rather than
  // hardcoding, so the two can't drift apart independently.
  request->frame_id = mp_.frame_id_;
  request->aabb_min_m.x = md_.camera_pos_(0) - half_extent(0);
  request->aabb_min_m.y = md_.camera_pos_(1) - half_extent(1);
  request->aabb_min_m.z = md_.camera_pos_(2) - half_extent(2);
  request->aabb_size_m.x = 2.0 * half_extent(0);
  request->aabb_size_m.y = 2.0 * half_extent(1);
  request->aabb_size_m.z = 2.0 * half_extent(2);

  // NOTE: start-state false-occupancy fix attempted here via nvblox's
  // spheres_to_clear_center_m/radius_m (clearTsdfInsideShapes) and
  // reverted -- confirmed via direct raw-ESDF-value inspection that this
  // clears occupied voxels back to the UNOBSERVED sentinel (-1000), not
  // to a confirmed-free value, so it's a no-op against Fast-Planner's own
  // unknown->occupied policy (both read identically as "unknown"). The
  // function with the correct semantics, Mapper::markUnobservedTsdfFree-
  // InsideRadius(), exists in nvblox_core but isn't wired to any ROS-level
  // interface (no param, no service field) -- using it needs new nvblox_
  // ros-side plumbing, not just a client-side change here. Left
  // unimplemented pending that design decision.

  *in_flight_flag = true;
  // Async send + callback, never a blocking wait (spin_until_future_complete
  // etc.) -- fast_planner_node runs plain rclcpp::spin(node), the
  // single-threaded executor (confirmed: fast_planner_node.cpp:32, no
  // MultiThreadedExecutor anywhere in plan_manage/). A blocking call here
  // would deadlock this node against its own single thread -- the
  // client-side mirror of the Phase 5 nvblox-server deadlock.
  esdf_client_->async_send_request(
      request,
      [this, in_flight_flag](
          rclcpp::Client<nvblox_msgs::srv::EsdfAndGradients>::SharedFuture future) {
        this->onEsdfResponse(future, in_flight_flag);
      });
}

void SDFMap::onEsdfResponse(
    rclcpp::Client<nvblox_msgs::srv::EsdfAndGradients>::SharedFuture future,
    bool* in_flight_flag) {
  // TEMPORARY, Task 2c timing measurement only -- remove before Milestone 1
  // ships (or gate behind a param if kept).
  const rclcpp::Time t_cb_start = node_->now();

  *in_flight_flag = false;
  auto response = future.get();
  if (!response->success) {
    // Includes the frame-mismatch case nvblox itself warns and returns
    // empty for -- not fatal, just skip this update and try again next tick.
    return;
  }

  const float src_res_f = response->voxel_size_m;
  if (src_res_f <= 0.0f) return;
  const double src_res = static_cast<double>(src_res_f);

  const auto& dims = response->esdf_and_gradients.layout.dim;
  if (dims.size() != 3) return;
  const int nx = static_cast<int>(dims[0].size);
  const int ny = static_cast<int>(dims[1].size);
  const int nz = static_cast<int>(dims[2].size);
  const auto& data = response->esdf_and_gradients.data;
  if (nx <= 0 || ny <= 0 || nz <= 0 ||
      static_cast<int>(data.size()) != nx * ny * nz) {
    return;
  }

  const Eigen::Vector3d src_origin(
      response->origin_m.x, response->origin_m.y, response->origin_m.z);

  // Resolution mismatch is real (nvblox measured at 0.05m, Fast-Planner
  // configured at 0.1m sdf_map/resolution -- exactly 2x, confirmed live
  // during the Task 2 design pass), not assumed 1:1. pool_factor is
  // computed from the actual response, not hardcoded, so this keeps
  // working if either resolution is retuned later.
  const int pool_factor = std::max(1, static_cast<int>(std::round(mp_.resolution_ / src_res)));

  const Eigen::Vector3d aabb_min = src_origin;
  const Eigen::Vector3d aabb_max =
      src_origin + Eigen::Vector3d(nx * src_res, ny * src_res, nz * src_res);

  Eigen::Vector3i dst_min_idx, dst_max_idx;
  posToIndex(aabb_min, dst_min_idx);
  posToIndex(aabb_max, dst_max_idx);
  boundIndex(dst_min_idx);
  boundIndex(dst_max_idx);

  for (int dx = dst_min_idx(0); dx <= dst_max_idx(0); ++dx) {
    for (int dy = dst_min_idx(1); dy <= dst_max_idx(1); ++dy) {
      for (int dz = dst_min_idx(2); dz <= dst_max_idx(2); ++dz) {
        Eigen::Vector3d dst_pos;
        indexToPos(Eigen::Vector3i(dx, dy, dz), dst_pos);

        const Eigen::Vector3i src_base(
            static_cast<int>(std::floor((dst_pos(0) - src_origin(0)) / src_res)),
            static_cast<int>(std::floor((dst_pos(1) - src_origin(1)) / src_res)),
            static_cast<int>(std::floor((dst_pos(2) - src_origin(2)) / src_res)));

        // Signed min-pool over the covering source block: taking the
        // minimum (not average/nearest) is the conservative choice for a
        // safety distance field -- never overstate clearance -- and
        // taking it over the *signed* value correctly biases toward "more
        // inside" when a block straddles an obstacle surface.
        double min_dist = std::numeric_limits<double>::max();
        bool any_observed = false;
        // Jetson-load note: sx*ny*nz + sy*nz was recomputed from scratch on
        // every (ix,iy,iz) triple -- algebraically the same address, just
        // rebuilt via 2 extra multiplications per inner-loop iteration for
        // no reason. Hoisted to the loop level each term is actually
        // invariant at (2.1M destination voxels x up to 8 inner iterations
        // each on the corridor tier, this is the single most expensive
        // required -- non-visualization -- callback in the system, and the
        // Jetson target's per-core FP/integer throughput is weaker than
        // this dev box's). Pure algebraic hoist, not a behavior change:
        // sx*ny*nz + sy*nz + sz == sx_off + sy_off + sz for every (sx,sy,sz).
        for (int ix = 0; ix < pool_factor; ++ix) {
          const int sx = src_base(0) - pool_factor / 2 + ix;
          if (sx < 0 || sx >= nx) continue;
          const int sx_off = sx * ny * nz;
          for (int iy = 0; iy < pool_factor; ++iy) {
            const int sy = src_base(1) - pool_factor / 2 + iy;
            if (sy < 0 || sy >= ny) continue;
            const int sxy_off = sx_off + sy * nz;
            for (int iz = 0; iz < pool_factor; ++iz) {
              const int sz = src_base(2) - pool_factor / 2 + iz;
              if (sz < 0 || sz >= nz) continue;
              const float v = data[sxy_off + sz];
              if (v <= -999.0f) continue;  // nvblox's unobserved sentinel (-1000.0 default)
              any_observed = true;
              min_dist = std::min(min_dist, static_cast<double>(v));
            }
          }
        }

        const int dst_addr = toAddress(dx, dy, dz);
        // Persists exactly the any_observed computed above -- previously
        // discarded once it picked a branch. This is the ONLY place
        // observed_buffer_ is written; isObserved() (sdf_map.h) just
        // reads it back.
        md_.observed_buffer_[dst_addr] = any_observed ? 1 : 0;
        if (any_observed) {
          md_.distance_buffer_all_[dst_addr] = min_dist;
          // Threshold reuses obstacles_inflation_ (0.199m, already
          // configured for the old raycasting path's own inflation step)
          // rather than a new parameter -- consistent with the existing
          // design intent, one fewer constant to keep in sync.
          md_.occupancy_buffer_inflate_[dst_addr] =
              (min_dist <= mp_.obstacles_inflation_) ? 1 : 0;
        } else {
          // FLIPPED after the Task 2c far-goal collision: unknown ->
          // occupied (1), not passable (0). "Never plan through space not
          // confirmed free." See esdf_unknown_value_'s declaration in
          // sdf_map.h and the occupancy_buffer_inflate_ init comment
          // above for the full incident/rationale.
          md_.distance_buffer_all_[dst_addr] = mp_.esdf_unknown_value_;
          md_.occupancy_buffer_inflate_[dst_addr] = 1;
        }
      }
    }
  }

  // Milestone-1's cloudCallback() (dead in this pipeline -- /sdf_map/cloud
  // has zero publishers; nvblox's ESDF fetch replaced it) used to be the
  // only place keeping local_bound_min_/max_ current, recentering them on
  // the drone every update. Nothing replaced that when Path A moved map
  // population to this callback -- local_bound_min_/max_ sat at their
  // zero-initialized (0,0,0)-(0,0,0) default for the entire session
  // (resetBuffer(), the function that WOULD set them to the full map
  // extent, is itself never called from anywhere either). Every publish*()
  // function (all four /sdf_map/* visualization topics: occupancy,
  // occupancy_inflate, esdf, unknown) iterates exactly that range, so all
  // four have been rendering one fixed, meaningless corner voxel,
  // regardless of drone position or real obstacles, since Path A shipped --
  // confirmed live via temporary instrumentation: considered=1/occ_true=1/
  // pushed=1 on every single tick. Recentered here (not accumulated/
  // expand-only) on camera_pos_ +/- local_update_range_, mirroring
  // cloudCallback's own dead approach, so the published maps stay a live,
  // receding window tied to the drone's CURRENT position, not a
  // monotonically-growing history -- consistent with what a receding-
  // horizon display should show.
  posToIndex(md_.camera_pos_ + mp_.local_update_range_, md_.local_bound_max_);
  posToIndex(md_.camera_pos_ - mp_.local_update_range_, md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // TEMPORARY, Task 2c timing measurement only.
  const double cb_ms = (node_->now() - t_cb_start).seconds() * 1000.0;
  const char* tier = (in_flight_flag == &esdf_near_in_flight_) ? "near" : "corridor";
  RCLCPP_INFO(
      node_->get_logger(),
      "DEBUG_TIMING tier=%s voxels=%d callback_ms=%.3f",
      tier, nx * ny * nz, cb_ms);
}

// TEMPORARY, Task 2c verification only -- remove before Milestone 1 ships.
void SDFMap::debugQueryCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  const Eigen::Vector3d pos(msg->x, msg->y, msg->z);
  const double dist = getDistance(pos);
  const int occ = getInflateOccupancy(pos);
  const bool observed = isObserved(pos);
  RCLCPP_INFO(
      node_->get_logger(),
      "DEBUG_QUERY_RESULT pos=(%.3f,%.3f,%.3f) distance_buffer_all_=%.4f getInflateOccupancy=%d isObserved=%d",
      pos(0), pos(1), pos(2), dist, occ, observed ? 1 : 0);
}

void SDFMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr img,
                               const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) {
    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);


    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(md_.depth_image_);

    // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

    /* get pose */
    md_.camera_pos_(0) = pose->pose.position.x;
    md_.camera_pos_(1) = pose->pose.position.y;
    md_.camera_pos_(2) = pose->pose.position.z;
    md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
    if (isInMap(md_.camera_pos_)) {
        md_.has_odom_ = true;
        md_.update_num_ += 1;
        md_.occ_need_update_ = true;
    } else {
        md_.occ_need_update_ = false;
    }
}

void SDFMap::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    if (md_.has_first_depth_) return;

    md_.camera_pos_(0) = odom->pose.pose.position.x;
    md_.camera_pos_(1) = odom->pose.pose.position.y;
    md_.camera_pos_(2) = odom->pose.pose.position.z;

    md_.has_odom_ = true;
}

void SDFMap::topic_callback(const std_msgs::msg::String::SharedPtr msg) 
{
    // RCLCPP_INFO(node_class_->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void SDFMap::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr img) {
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*img, latest_cloud);

    md_.has_cloud_ = true;

    if (!md_.has_odom_) {
        // std::cout << "no odom!" << std::endl;
        return;
    }

    if (latest_cloud.points.size() == 0) return;

    if (std::isnan(md_.camera_pos_(0)) || std::isnan(md_.camera_pos_(1)) || std::isnan(md_.camera_pos_(2))) return;

    this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                      md_.camera_pos_ + mp_.local_update_range_);

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    int inf_step_z = 1;

    double max_x, max_y, max_z, min_x, min_y, min_z;

    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        /* point inside update range */
        Eigen::Vector3d devi = p3d - md_.camera_pos_;
        Eigen::Vector3i inf_pt;

        if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
            fabs(devi(2)) < mp_.local_update_range_(2)) {

            /* inflate the point */
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z) {

                        p3d_inf(0) = pt.x + x * mp_.resolution_;
                        p3d_inf(1) = pt.y + y * mp_.resolution_;
                        p3d_inf(2) = pt.z + z * mp_.resolution_;

                        max_x = std::max(max_x, p3d_inf(0));
                        max_y = std::max(max_y, p3d_inf(1));
                        max_z = std::max(max_z, p3d_inf(2));

                        min_x = std::min(min_x, p3d_inf(0));
                        min_y = std::min(min_y, p3d_inf(1));
                        min_z = std::min(min_z, p3d_inf(2));

                        posToIndex(p3d_inf, inf_pt);

                        if (!isInMap(inf_pt)) continue;

                        int idx_inf = toAddress(inf_pt);

                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
                    }
        }
    }

    min_x = std::min(min_x, md_.camera_pos_(0));
    min_y = std::min(min_y, md_.camera_pos_(1));
    min_z = std::min(min_z, md_.camera_pos_(2));

    max_x = std::max(max_x, md_.camera_pos_(0));
    max_y = std::max(max_y, md_.camera_pos_(1));
    max_z = std::max(max_z, md_.camera_pos_(2));

    max_z = std::max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.esdf_need_update_ = true;
}

void SDFMap::publishMap() {
    if (map_pub_->get_subscription_count() == 0) return;
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    int lmm = mp_.local_map_margin_ / 2;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z) {
                if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height_) continue;

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_->publish(cloud_msg);
}

void SDFMap::publishMapInflate(bool all_info) {
    if (map_inf_pub_->get_subscription_count() == 0) return;
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    if (all_info) {
        int lmm = mp_.local_map_margin_;
        min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
        max_cut += Eigen::Vector3i(lmm, lmm, lmm);
    }

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
            for (int z = min_cut(2); z <= max_cut(2); ++z) {
                if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height_) continue;

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                cloud.push_back(pt);
            }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_inf_pub_->publish(cloud_msg);
}

void SDFMap::publishUnknown() {
  if (unknown_pub_->get_subscription_count() == 0) return;
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        // occupancy_buffer_ (the pre-nvblox raycasting buffer) is dead in
        // this pipeline -- Path A Milestone 1 disabled the depth_sub_/
        // pose_sub_/odom_sub_ wiring that used to update it (see
        // initSubscribersAndPublishers()'s comment), so it stays frozen at
        // its unknown-init value for every voxel, forever. That would make
        // this function mark the ENTIRE local bound "unknown" always, not
        // the real, shrinking receding-horizon frontier. observed_buffer_
        // is the live one -- written by onEsdfResponse() as real nvblox
        // data arrives, same buffer isObserved() reads (sdf_map.h) -- so
        // this now matches what "unobserved" actually means everywhere
        // else in the codebase (checkTrajCollision, computeFrontier).
        if (md_.observed_buffer_[toAddress(x, y, z)] == 0) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, cloud_msg);
  unknown_pub_->publish(cloud_msg);
}

void SDFMap::publishConfirmedObstacle() {
  if (confirmed_obstacle_pub_->get_subscription_count() == 0) return;
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        const int addr = toAddress(x, y, z);
        // The intersection, not either buffer alone: occupancy_inflate==1
        // by itself means "occupied OR never observed" (both default to
        // 1 -- see the unknown->occupied policy comment at this buffer's
        // init), so on its own it can't prove the camera saw anything.
        // observed_buffer_==1 confirms nvblox actually resolved this
        // voxel. Requiring both is exactly "the drone has verified a
        // real obstacle is here" -- the proof-of-detection signal, not a
        // cloud a viewer has to visually subtract two other displays to
        // get.
        if (md_.observed_buffer_[addr] == 0) continue;
        if (md_.occupancy_buffer_inflate_[addr] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, cloud_msg);
  confirmed_obstacle_pub_->publish(cloud_msg);
}

void SDFMap::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PCLPointCloud2 pcl_cloud;
  pcl::toPCLPointCloud2(cloud, pcl_cloud);
  pcl_conversions::fromPCL(pcl_cloud, cloud_msg);
  depth_pub_->publish(cloud_msg);
}

void SDFMap::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::msg::Marker mk;

  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = node_->now(); // Assuming node_ is your rclcpp::Node
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_->publish(mk);
}

void SDFMap::publishESDF() {
  if (esdf_pub_->get_subscription_count() == 0) return;
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = std::min(dist, max_dist);
      dist = std::max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // Convert PCL point cloud to ROS 2 message
  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *cloud_msg);

  // Publish the ROS 2 message
  esdf_pub_->publish(std::move(cloud_msg));

  // RCLCPP_INFO(node_->get_logger(), "pub esdf");
}

void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                          std::vector<Eigen::Vector3d>& slice, std::vector<Eigen::Vector3d>& grad, int sign) {
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {

      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist() {
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0) {
          // Do something here if needed
        }
      }
}

bool SDFMap::odomValid() {return md_.has_odom_;}

bool SDFMap::hasDepthObservation() {return md_.has_first_depth_;}

double SDFMap::getResolution() {return mp_.resolution_;}

Eigen::Vector3d SDFMap::getOrigin() {return mp_.map_origin_;}

int SDFMap::getVoxelNum() {
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_.map_origin_;
  size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* Interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstSharedPtr img,
                               const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  /* Get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  /* Get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
}

void SDFMap::depthCallback(const sensor_msgs::msg::Image::SharedPtr img) {
  //std::cout << "depth: " << img->header.stamp << std::endl;
}

void SDFMap::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
  //std::cout << "pose: " << pose->header.stamp << std::endl;

  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
}

// SDFMap
