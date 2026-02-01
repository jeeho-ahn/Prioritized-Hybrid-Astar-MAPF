/*****************************************************************
 * Prioritized Hybrid Astar Demo with Pushing Tasks from ReloPush
 * Refactored for Debuggability & Modularity
 *
 * 2025.11.9
 * Refactored Version
 ******************************************************************/

#include <LoadFinalSequence.h>
#include <PHAstar.h>
#include <Reeds_Shepp.h>
#include <Task.h>
#include <Visualization.h>
#include <config.h>
#include <iomanip>
#include <iostream>

// for finding parking
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>


// ==========================================
// 1. HELPER & UTILITY FUNCTIONS
// ==========================================

// --- Configuration & Initialization ---
Params initialize_params(const std::vector<FinalAllocation> &loadedSequence) {
  Params params;
  // Set analytic expansion threshold
  params.analytic_threshold = 5.0 * params.max_steer;
  // params.analytic_threshold = std::hypot(params.max_x - params.min_x,
  // params.max_y - params.min_y) * 2.0;

  // Set workspace boundaries if available
  if (!loadedSequence.empty()) {
    const auto &bound = loadedSequence[0].snapshot.parameters.boundary;
    params.min_x = bound.xMin;
    params.min_y = bound.yMin;
    params.max_x = bound.xMax;
    params.max_y = bound.yMax;
    std::cout << "[Init] Workspace set: [" << bound.xMin << ", " << bound.xMax
              << "] x [" << bound.yMin << ", " << bound.yMax << "]"
              << std::endl;
  }
  return params;
}

std::unordered_map<std::string, EntityMeta *>
initialize_entities(const std::vector<FinalAllocation> &loadedSequence) {
  std::unordered_map<std::string, EntityMeta *> entities;

  // Robot 1
  RobotMeta *robot1 = new RobotMeta;
  robot1->name = "robot1";
  robot1->type = EntityType::ROBOT;
  robot1->initial_pose = {0.5, 0.45, 0.0};
  robot1->size.front_length = 0.32;
  robot1->size.rear_length = 0.2;
  robot1->size.width = 0.3;
  robot1->min_turning_radius = 1.43;
  robot1->wheel_base = 0.4;
  robot1->speed_transit = 0.2;
  robot1->speed_transfer = 0.15;
  entities["robot1"] = robot1;

  // Robot 2
  
      RobotMeta* robot2 = new RobotMeta;
      robot2->name = "robot2";
      robot2->type = EntityType::ROBOT;
      robot2->initial_pose = {0.5, 3.0, 0.0};
      robot2->size.front_length = 0.32;
      robot2->size.rear_length = 0.2;
      robot2->size.width = 0.3;
      robot2->min_turning_radius = 1.43;
      robot2->wheel_base = 0.4;
      robot2->speed_transit = 0.2;
      robot2->speed_transfer = 0.15;
      entities["robot2"] = robot2;
  
  /*
  RobotMeta* robot3 = new RobotMeta;
  robot3->name = "robot3";
  robot3->type = EntityType::ROBOT;
  robot3->initial_pose = {0.5, 1.8, 0.0};
  robot3->size.front_length = 0.32;
  robot3->size.rear_length = 0.2;
  robot3->size.width = 0.3;
  robot3->min_turning_radius = 1.43;
  robot3->wheel_base = 0.4;
  robot3->speed_transit = 0.2;
  robot3->speed_transfer = 0.15;
  entities["robot3"] = robot3;
*/

  // Parse Objects
  if (!loadedSequence.empty()) {
    for (const auto &[name, info] : loadedSequence[0].snapshot.mo_list) {
      ObjectMeta *obj = new ObjectMeta;
      obj->name = name;
      obj->type = EntityType::OBJECT;
      obj->initial_pose = {info.x, info.y, info.nominalOrientation};
      // Assuming ObjectMeta size struct is similar
      obj->size.front_length = 0.075;
      obj->size.rear_length = 0.075;
      obj->size.width = 0.15;
      entities[name] = obj;
    }
  }
  return entities;
}

Pose calcRobotPoseFromObj(const Pose &obj_pose, const OccuRect &robot_size,
                          const OccuRect &obj_size) {
  double offset = robot_size.front_length + obj_size.rear_length + 0.1 + 0.05;
  Pose robot_pose;
  robot_pose.x = obj_pose.x - offset * std::cos(obj_pose.yaw);
  robot_pose.y = obj_pose.y - offset * std::sin(obj_pose.yaw);
  robot_pose.yaw = obj_pose.yaw;
  return robot_pose;
}

// --- Collision & Blocking Checkers (Preserved from original) ---

// (Keep your helper functions project_corners, overlaps, get_rect_axes here)
// ... [Assuming SAT helpers are defined as in original] ...

// ==========================================
// SMARTER DIAGNOSTIC FUNCTION
// ==========================================
void diagnose_planning_failure(RobotMeta *robot, const Pose &start,
                               const Pose &goal, double time,
                               TimeTable &timetable) {
  std::cerr << "\n  [Diagnostics] Analyzing failure for " << robot->name
            << " at t=" << time << "s..." << std::endl;

  auto others_map = timetable.get_poses(time);
  std::vector<std::pair<EntityMeta *, Pose>> others(others_map.begin(),
                                                    others_map.end());

  // --- Helper Lambda: Is this a valid transfer? ---
  auto is_valid_transfer = [](EntityMeta *e1, const Pose &p1, EntityMeta *e2,
                              const Pose &p2) -> bool {
    // 1. Identify Robot and Object
    RobotMeta *r =
        dynamic_cast<RobotMeta *>(e1->type == EntityType::ROBOT ? e1 : e2);
    ObjectMeta *o =
        dynamic_cast<ObjectMeta *>(e1->type == EntityType::OBJECT ? e1 : e2);
    if (!r || !o)
      return false; // Not a Robot-Object pair

    const Pose &r_pose = (e1 == r) ? p1 : p2;
    const Pose &o_pose = (e1 == o) ? p1 : p2;

    // 2. Check Orientation Alignment (should be similar for pushing)
    double yaw_diff = std::abs(r_pose.yaw - o_pose.yaw);
    while (yaw_diff > M_PI)
      yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI)
      yaw_diff += 2 * M_PI;
    if (std::abs(yaw_diff) > 0.5)
      return false; // Angle mismatch > ~30 deg

    // 3. Check Relative Position (Object should be in front)
    // Simple check: Distance should be roughly sum of half-lengths
    double dx = o_pose.x - r_pose.x;
    double dy = o_pose.y - r_pose.y;
    double dist = std::hypot(dx, dy);

    // Expected distance center-to-center approx (Front_R + Rear_O)
    // Allowing some tolerance (e.g., 0.2m)
    double expected_dist = r->size.front_length + o->size.rear_length;
    if (dist > expected_dist + 0.3 || dist < expected_dist - 0.3)
      return false;

    return true;
  };
  // ------------------------------------------------

  // 1. Check Start Pose Validity
  Corners start_c =
      get_corners(start.x, start.y, start.yaw, robot->size.front_length,
                  robot->size.rear_length, robot->size.width);
  bool start_ok = true;
  for (const auto &[ent, pose] : others) {
    if (ent == robot)
      continue;

    Corners ent_c =
        get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length,
                    ent->size.rear_length, ent->size.width);
    if (rectangles_intersect(start_c, ent_c)) {
      std::cerr << "    [FAIL] Start Pose COLLIDES with " << ent->name
                << " (Dist: " << std::hypot(start.x - pose.x, start.y - pose.y)
                << "m)" << std::endl;
      start_ok = false;
    }
  }
  if (start_ok)
    std::cerr << "    [PASS] Start Pose is collision-free." << std::endl;

  // 2. Check Goal Pose Validity
  Corners goal_c =
      get_corners(goal.x, goal.y, goal.yaw, robot->size.front_length,
                  robot->size.rear_length, robot->size.width);
  bool goal_ok = true;
  for (const auto &[ent, pose] : others) {
    if (ent == robot)
      continue;

    Corners ent_c =
        get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length,
                    ent->size.rear_length, ent->size.width);
    if (rectangles_intersect(goal_c, ent_c)) {
      std::cerr << "    [FAIL] Goal Pose COLLIDES with " << ent->name
                << std::endl;
      goal_ok = false;
    }
  }
  if (goal_ok)
    std::cerr << "    [PASS] Goal Pose is collision-free." << std::endl;

  // 3. Check Global Consistency
  std::cerr << "    [Info] Checking other entities for consistency..."
            << std::endl;
  bool global_issue = false;
  for (size_t i = 0; i < others.size(); ++i) {
    for (size_t j = i + 1; j < others.size(); ++j) {
      auto [ent1, p1] = others[i];
      auto [ent2, p2] = others[j];

      if (ent1 == robot || ent2 == robot)
        continue;

      Corners c1 = get_corners(p1.x, p1.y, p1.yaw, ent1->size.front_length,
                               ent1->size.rear_length, ent1->size.width);
      Corners c2 = get_corners(p2.x, p2.y, p2.yaw, ent2->size.front_length,
                               ent2->size.rear_length, ent2->size.width);

      if (rectangles_intersect(c1, c2)) {
        // Check if this is a valid transfer (Robot pushing Object)
        if (is_valid_transfer(ent1, p1, ent2, p2)) {
          // Valid transfer - ignore
          // std::cout << "    [Info] Ignoring contact between " << ent1->name
          // << " and " << ent2->name << " (Transferring)" << std::endl;
        } else {
          std::cerr << "    [WARN] Global Consistency: " << ent1->name
                    << " intersects " << ent2->name << "!" << std::endl;
          global_issue = true;
        }
      }
    }
  }
  if (!global_issue)
    std::cerr << "    [PASS] Global scene is consistent (ignoring transfers)."
              << std::endl;
}

// ==========================================
// UPDATED: PLAN INITIAL TRANSIT
// ==========================================
bool plan_initial_transit(
    RobotMeta *robot, const Pose &target_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params) {
  Pose current_pose = timetable.get_pose(robot, start_time);
  robot->initial_pose = current_pose; // Update meta for planner

  std::cout << "  [Transit] Planning " << robot->name << " -> ("
            << target_pose.x << ", " << target_pose.y << ", " << target_pose.yaw
            << ") starting at " << start_time << "s" << std::endl;

  PHAStar planner(robot, target_pose, &timetable, &entities, params, false, "",
                  start_time);
  auto path_res = planner.Planning_with_res(start_time);

  if (path_res.waypoints.empty()) {
    // std::cerr << "  [Error] Transit planning failed for " << robot->name <<
    // std::endl;
    std::cerr << " [Error] Transit planning failed for " << robot->name
              << " - Status: " << static_cast<int>(path_res.status)
              << ", Detail: " << path_res.failure_detail << std::endl;

    // Call the new diagnostic tool
    diagnose_planning_failure(robot, current_pose, target_pose, start_time,
                              timetable);
    if (DEBUG_VIS) { // Assuming you keep a global DEBUG_VIS toggle
      visualize_current_state(timetable, entities, params, start_time,
                              current_pose, target_pose);
      visualize_search_tree(path_res.explored_nodes, params);
    }

    return false;
  }

  if (DEBUG_VIS) {
    std::cout << "[Debug] Visualizing Plan..." << std::endl;
        
    visualize_planning_debug(
            timetable,           // Global history/future of others
            robot,  // The robot executing this plan
            path_res,              // The output plan (waypoints)
            start_time,  // Absolute start time for this plan
            current_pose,  // Start
            target_pose,   // Goal
            params
        );
  }
  

  // add final push
  double delta_t = params.final_push_distance / robot->speed_transit;
  auto final_push_pose =
      offsetPose(path_res.waypoints.back(), params.final_push_distance);
  auto final_push_wpt = Waypoint(final_push_pose);
  final_push_wpt.time = path_res.waypoints.back().time + delta_t;
  final_push_wpt.linear_velocity = path_res.waypoints.back().linear_velocity;
  path_res.waypoints.push_back(final_push_wpt);

  // Adjust relative time and register
  for (auto &wp : path_res.waypoints)
    wp.time -= start_time;

  Trajectory transit_traj;
  transit_traj.entity = robot;
  transit_traj.start_time = start_time;
  transit_traj.waypoints = path_res.waypoints;
  transit_traj.is_transfer = false;
  timetable.add_trajectory(transit_traj);

  return true;
}

// ==========================================
// UPDATED: CHECK COLLISION TRAJECTORY
// (Ensures we ignore Robot-vs-Object if Object is being pushed)
// ==========================================
bool check_collision_trajectory(const Trajectory &traj, double start_time,
                                TimeTable &timetable, const Params &params,
                                bool verbose = false) {
  if (traj.waypoints.empty())
    return false;
  double dt = 0.1;
  double duration = traj.waypoints.back().time;

  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  ObjectMeta *object = dynamic_cast<ObjectMeta *>(traj.transferred_object);

  for (double t = 0; t <= duration; t += dt) {
    double abs_t = start_time + t;
    auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
    Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                   std::get<2>(pose_tuple)};

    Corners r_corners =
        get_corners(r_pose.x, r_pose.y, r_pose.yaw, robot->size.front_length,
                    robot->size.rear_length, robot->size.width);
    auto others = timetable.get_poses(abs_t);

    // 1. Robot Body vs Others
    for (const auto &[ent, o_pose] : others) {
      if (ent == robot)
        continue;
      if (object && ent == object)
        continue;

      Corners o_corners =
          get_corners(o_pose.x, o_pose.y, o_pose.yaw, ent->size.front_length,
                      ent->size.rear_length, ent->size.width);

      if (rectangles_intersect(r_corners, o_corners)) {
        if (verbose)
          std::cout << "  [Collision] Robot vs " << ent->name
                    << " at t=" << abs_t << std::endl;
        return true;
      }
    }

    // 2. Pushed Object vs Others
    if (traj.is_transfer && object) {
      double offset = robot->size.front_length + object->size.rear_length;
      Pose o_pose = {r_pose.x + offset * std::cos(r_pose.yaw),
                     r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
      Corners obj_corners =
          get_corners(o_pose.x, o_pose.y, o_pose.yaw, object->size.front_length,
                      object->size.rear_length, object->size.width);

      for (const auto &[ent, other_p] : others) {
        if (ent == robot || ent == object)
          continue;

        // --- IGNORE RULE ---
        // "It is obvious that robot2 and the objects are in contact... ignore
        // this" If we are checking collisions for Robot 1 (traj.entity), we do
        // NOT want to flag collisions between Robot 1's Payload (O1) and Robot
        // 2 (ent). However, usually hitting another robot is bad. IF you want
        // to be extremely aggressive and ignore Other Robots entirely for the
        // payload:
        if (ent->type == EntityType::ROBOT)
          continue;

        Corners other_c = get_corners(other_p.x, other_p.y, other_p.yaw,
                                      ent->size.front_length,
                                      ent->size.rear_length, ent->size.width);
        if (rectangles_intersect(obj_corners, other_c)) {
          if (verbose)
            std::cout << "  [Collision] Pushed Object vs " << ent->name
                      << " at t=" << abs_t << std::endl;
          return true;
        }
      }
    }
  }
  return false;
}

const double INF = std::numeric_limits<double>::infinity();

struct ParkingCandidate {
  Pose pose;
  double estimated_rs_length; // Actual Reeds-Shepp free-space path length
                              // (lower-bound cost)
};

std::vector<ParkingCandidate>
generate_parking_candidates(const Pose &current_pose, RobotMeta *robot,
                            const Params &params) {
  std::vector<ParkingCandidate> candidates;

  double maxc = 1.0 / robot->min_turning_radius;
  double step_size = 0.2; // Can be coarse; only used for discretization (we
                          // ignore the full path)
  double wb = robot->wheel_base;

  auto compute_rs_length = [&](const Pose &goal) -> double {
    auto [xs, ys, yaws, ctypes, lengths, steers, directions] =
        ReedShepp::reeds_shepp_path_planning(current_pose.x, current_pose.y,
                                             current_pose.yaw, goal.x, goal.y,
                                             goal.yaw, maxc, step_size, wb);

    if (xs.empty()) {
      return INF;
    }
    double L = 0.0;
    for (double len : lengths) {
      L += std::abs(len);
    }
    return L;
  };

  // 1. Strongly prefer the robot's initial_pose (often designed to be
  // safe/clear)
  double init_L = compute_rs_length(robot->initial_pose);
  candidates.push_back({robot->initial_pose, init_L});

  // 2. Workspace corners with multiple orientations (out-of-the-way locations)
  double margin = 0.6; // Safe margin from exact bounds
  std::vector<double> corner_yaws = {0.0, M_PI / 2, M_PI, -M_PI / 2};
  std::vector<std::pair<double, double>> corner_positions = {
      {params.min_x + margin, params.min_y + margin},
      {params.max_x - margin, params.min_y + margin},
      {params.max_x - margin, params.max_y - margin},
      {params.min_x + margin, params.max_y - margin}};

  for (const auto &pos : corner_positions) {
    for (double yaw : corner_yaws) {
      Pose p{pos.first, pos.second, yaw};
      double L = compute_rs_length(p);
      candidates.push_back({p, L});
    }
  }

  // 3. Random samples as fallback (20-30 is plenty)
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dist(params.min_x + margin,
                                          params.max_x - margin);
  std::uniform_real_distribution<> y_dist(params.min_y + margin,
                                          params.max_y - margin);
  std::uniform_real_distribution<> yaw_dist(-M_PI, M_PI);

  for (int i = 0; i < 25; ++i) {
    Pose p{x_dist(gen), y_dist(gen), yaw_dist(gen)};
    double L = compute_rs_length(p);
    candidates.push_back({p, L});
  }

  // Sort by Reeds-Shepp length (lower is better). INF goes to the end.
  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b) {
              return a.estimated_rs_length < b.estimated_rs_length;
            });

  // Debug print (optional, can comment out)
  std::cout << "[Parking] Generated " << candidates.size()
            << " candidates (sorted by RS length)" << std::endl;

  return candidates;
}

// Moves a blocking robot out of the way (Updated with Safe Parking Search)
bool resolve_goal_blocking(
    RobotMeta *robot_to_plan, const Pose &desired_goal, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params, double current_time, bool check_trajectory_blocking,
    const TrajectoryPtr &traj_ptr) {
  constexpr double BLOCK_DIST_THRESHOLD =
      0.90; // [m] if anyone is closer than this -> blocking
  constexpr double BLOCK_YAW_THRESHOLD = M_PI_2; // 90 deg yaw tolerance
  constexpr double STATIONARY_THRESHOLD =
      0.20; // moved <20 cm in 4 seconds -> considered waiting
  constexpr double LOOKAHEAD_TIME = 4.0; // seconds
  constexpr double TRAJ_SAMPLE_STEP =
      0.5; // [m] sample every 50cm along trajectory for blocks
  constexpr double TRAJ_BLOCK_DIST =
      0.60; // [m] closer than this to traj point -> blocking along path

  bool cleared_any = false;

  // Helper: Check if a candidate pose collides with the active task's path or
  // goal
  auto is_candidate_safe = [&](const Pose &candidate,
                               RobotMeta *blocker) -> bool {
    // 1. Check vs Goal
    Corners c_cand = get_corners(
        candidate.x, candidate.y, candidate.yaw, blocker->size.front_length,
        blocker->size.rear_length, blocker->size.width);
    Corners c_goal =
        get_corners(desired_goal.x, desired_goal.y, desired_goal.yaw,
                    robot_to_plan->size.front_length,
                    robot_to_plan->size.rear_length, robot_to_plan->size.width);

    if (rectangles_intersect(c_cand, c_goal))
      return false;

    // 2. Check vs Trajectory (if available)
    if (traj_ptr && !traj_ptr->waypoints.empty()) {
      double duration = traj_ptr->waypoints.back().time;
      // Iterate waypoints directly or interpolate? Interpolating is safer but
      // slower. Using sparse waypoints + width check might be enough if
      // TRAJ_BLOCK_DIST covers it. Let's use the sample approach from original
      // code logic.
      for (double rel_t = 0.0; rel_t <= duration; rel_t += 0.2) { // 0.2s steps
        Pose p = TimeTable::interpolate_waypoints(traj_ptr->waypoints, rel_t);
        // Simple distance check first for speed
        double dist = std::hypot(p.x - candidate.x, p.y - candidate.y);
        if (dist < (TRAJ_BLOCK_DIST + 0.5)) { // Broad phase
          Corners c_traj = get_corners(
              p.x, p.y, p.yaw, robot_to_plan->size.front_length,
              robot_to_plan->size.rear_length, robot_to_plan->size.width);
          if (rectangles_intersect(c_cand, c_traj))
            return false;
        }
      }
    }
    return true;
  };

  // Iterate over all other entities to find blockers
  for (const auto &[name, ent] : entities) {
    if (ent->type != EntityType::ROBOT || ent == robot_to_plan)
      continue;
    RobotMeta *blocker = dynamic_cast<RobotMeta *>(ent);

    Pose now_pose = timetable.get_pose(blocker, current_time);
    Pose future_pose =
        timetable.get_pose(blocker, current_time + LOOKAHEAD_TIME);
    double moved =
        std::hypot(future_pose.x - now_pose.x, future_pose.y - now_pose.y);

    // Check 1: Blocking the Goal?
    bool is_goal_blocking = false;
    double dist_to_goal =
        std::hypot(now_pose.x - desired_goal.x, now_pose.y - desired_goal.y);
    if (dist_to_goal < BLOCK_DIST_THRESHOLD && moved < STATIONARY_THRESHOLD) {
      is_goal_blocking = true;
    }

    // Check 2: Blocking the Trajectory?
    bool is_traj_blocking = false;
    if (check_trajectory_blocking && traj_ptr && !is_goal_blocking) {
      double duration = traj_ptr->waypoints.back().time;
      for (double rel_t = 0.0; rel_t <= duration; rel_t += 0.5) {
        Pose p = TimeTable::interpolate_waypoints(traj_ptr->waypoints, rel_t);
        double dist = std::hypot(p.x - now_pose.x, p.y - now_pose.y);
        if (dist < TRAJ_BLOCK_DIST && moved < STATIONARY_THRESHOLD) {
          is_traj_blocking = true;
          break;
        }
      }
    }

    if (is_goal_blocking || is_traj_blocking) {
      std::cout << "[COLLISION RESOLUTION] " << blocker->name << " is blocking "
                << robot_to_plan->name << " ("
                << (is_goal_blocking ? "Goal" : "Path")
                << "). Finding safe parking...\n";

      // Generate candidates
      auto candidates = generate_parking_candidates(now_pose, blocker, params);
      bool resolved = false;

      for (const auto &cand : candidates) {
        // 1. Is this candidate safe from the ACTIVE robot's path/goal?
        if (!is_candidate_safe(cand.pose, blocker))
          continue;

        // 2. Can the blocker reach it? (Planner check)
        // Note: The planner will also check for collisions with STATIC
        // environment and OTHER robots (via TimeTable)
        PHAStar evader(blocker, cand.pose, &timetable, &entities, params, false,
                       "", current_time);
        auto evade_path = evader.Planning_with_res();

        if (!evade_path.waypoints.empty()) {
          for (auto &wp : evade_path.waypoints)
            wp.time -= current_time; // relative

          Trajectory evade_traj;
          evade_traj.entity = blocker;
          evade_traj.start_time = current_time;
          evade_traj.waypoints = evade_path.waypoints;
          evade_traj.is_transfer = false;

          timetable.add_trajectory(evade_traj);
          std::cout << "  -> Resolved! " << blocker->name
                    << " moving to safe spot at (" << cand.pose.x << ", "
                    << cand.pose.y << ")\n";
          resolved = true;
          cleared_any = true;
          break;
        }
      }

      if (!resolved) {
        std::cerr << "  -> CRITICAL: Could not find any safe parking spot for "
                  << blocker->name << "!\n";
      }
    }
  }

  return cleared_any;
}

// ==========================================
// 2. CORE PLANNING SUB-ROUTINES
// ==========================================

// Finds the robot that becomes free the earliest
RobotMeta *find_earliest_robot(const std::vector<RobotMeta *> &robots,
                               TimeTable &timetable, double &out_free_time) {
  RobotMeta *best_robot = nullptr;
  out_free_time = std::numeric_limits<double>::infinity();

  for (auto *robot : robots) {
    double t = timetable.get_entity_max_time(robot);
    if (t < out_free_time) {
      out_free_time = t;
      best_robot = robot;
    }
  }
  return best_robot;
}

/*
// Plans the initial transit from current pose to task start
bool plan_initial_transit(RobotMeta* robot, const Pose& target_pose, double
start_time, TimeTable& timetable, const std::unordered_map<std::string,
EntityMeta*>& entities, const Params& params)
{
    Pose current_pose = timetable.get_pose(robot, start_time);
    robot->initial_pose = current_pose; // Update meta for planner

    std::cout << "  [Transit] Planning " << robot->name << " -> ("
              << target_pose.x << ", " << target_pose.y << ") starting at " <<
start_time << "s" << std::endl;

    PHAStar planner(robot, target_pose, &timetable, &entities, params, false,
"", start_time); auto path_res = planner.Planning_with_res();

    if (path_res.waypoints.empty()) {
        std::cerr << " [Error] Transit planning failed for " << robot->name <<
std::endl; if (DEBUG_VIS) {  // Assuming you keep a global DEBUG_VIS toggle
            visualize_current_state(timetable, entities, params, start_time,
current_pose, target_pose);
        }
        return false;
    }

    // Adjust relative time and register
    for (auto& wp : path_res.waypoints) wp.time -= start_time;

    Trajectory transit_traj;
    transit_traj.entity = robot;
    transit_traj.start_time = start_time;
    transit_traj.waypoints = path_res.waypoints;
    transit_traj.is_transfer = false;
    timetable.add_trajectory(transit_traj);

    return true;
}
*/

// Attempts to find a collision-free time slot for a trajectory segment
double find_safe_start_time(Trajectory *traj, double earliest_start,
                            TimeTable &timetable, const Params &params) {
  double check_time = earliest_start;
  double step = 0.5;
  int max_retries = 200; // ~100 seconds wait limit

  for (int i = 0; i < max_retries; ++i) {
    double dummy_col_time = 0;
    if (!check_collision_trajectory(*traj, check_time, timetable, params,
                                    false)) {
      if (i > 0) {
        std::cout << "  [Delay] Delayed " << (i * step) << "s for safety."
                  << std::endl;
      }
      return check_time;
    }
    check_time += step;
  }

  std::cerr << "  [Error] Could not find safe slot after "
            << (max_retries * step) << "s wait." << std::endl;
  return -1.0; // Failure signal
}

// Generates and adds a retraction trajectory (backing up) after a push
void append_retraction(RobotMeta *robot, const Trajectory &previous_traj,
                       TimeTable &timetable) {
  if (previous_traj.waypoints.empty())
    return;

  double retract_dist = 0.15; // Meters
  double accumulated_dist = 0.0;
  size_t num_wp = previous_traj.waypoints.size();
  size_t start_idx = num_wp - 1;

  // 1. Calculate how many waypoints to backtrack
  for (int i = static_cast<int>(num_wp) - 2; i >= 0; --i) {
    double d = std::hypot(
        previous_traj.waypoints[i + 1].x - previous_traj.waypoints[i].x,
        previous_traj.waypoints[i + 1].y - previous_traj.waypoints[i].y);
    accumulated_dist += d;
    if (accumulated_dist >= retract_dist) {
      start_idx = i + 1;
      break;
    }
  }

  if (start_idx >= num_wp)
    return;

  // 2. Create reversed waypoints
  std::vector<Waypoint> retract_wp;
  for (int i = num_wp - 1; i >= static_cast<int>(start_idx); --i) {
    Waypoint wp = previous_traj.waypoints[i];
    wp.linear_velocity = -std::abs(wp.linear_velocity); // Reverse gear
    wp.steering_angle = -wp.steering_angle;             // Reverse curve
    retract_wp.push_back(wp);
  }

  // 3. Recalculate timing (assume transit speed)
  double current_time = 0.0;
  retract_wp[0].time = 0.0;
  for (size_t i = 1; i < retract_wp.size(); ++i) {
    double d = std::hypot(retract_wp[i].x - retract_wp[i - 1].x,
                          retract_wp[i].y - retract_wp[i - 1].y);
    current_time += (d / robot->speed_transit);
    retract_wp[i].time = current_time;
  }

  // 4. Add to timetable
  Trajectory retract_traj;
  retract_traj.entity = robot;
  retract_traj.start_time = timetable.get_entity_max_time(robot);
  retract_traj.waypoints = retract_wp;
  retract_traj.is_transfer = false;

  timetable.add_trajectory(retract_traj);
  std::cout << "  [Retract] Backing up " << retract_dist << "m ("
            << current_time << "s)." << std::endl;
}

// ==========================================
// 3. MAIN TASK PIPELINE
// ==========================================

// Helper function to handle the scheduling of a single path segment
void schedule_path_segment(const EdgePath &edge_path, EntityMeta *obj_meta,
                           RobotMeta *robot, TimeTable &timetable,
                           const Params &params) {
  // 1. Get current available time from the timetable
  double current_avail_time = timetable.get_entity_max_time(robot);

  // 2. Convert EdgePath to Trajectory
  // ReloPushPath2TrajPtr is defined in Task.h
  TrajectoryPtr traj =
      ReloPushPath2TrajPtr(edge_path, robot, obj_meta, current_avail_time);

  // 3. Find safe start time
  // Note: The original code had a bug where the second block passed the wrong
  // pointer to find_safe_start_time. Using traj.get() here ensures we always
  // check the CURRENT path.
  double safe_start_time =
      find_safe_start_time(traj.get(), current_avail_time, timetable, params);

  // 4. Update timestamps and add to timetable
  traj->start_time = safe_start_time;

  // CalcualteTimeStamps is defined in Entities.h
  traj->CalcualteTimeStamps(robot);

  timetable.add_trajectory(*traj);
}

void process_task_execution(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params) {
  // 1. Plan Transit to Task Start
  double robot_avail_time = timetable.get_entity_max_time(robot);
  if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time,
                            timetable, entities, params)) {
    std::cerr << "Aborting task due to transit failure." << std::endl;
    return;
  }

  // 2. ObsRelo (if exists)
  if (task.vertexChain.size() > 2) {
    // Iterate through obstacles
    // Note: verify if vertexChain indices align 1:1 with obsReloPaths indices
    for (size_t obs_ind = 1; obs_ind < task.vertexChain.size() - 1; obs_ind++) {
      std::cout << "Obstacle Relocation" << std::endl;

      // Identify the obstacle
      std::string obs_name = task.vertexChain[obs_ind].name;
      auto obs_meta = entities.at(obs_name);

      size_t push_path_idx = 0;
      size_t post_path_idx = 1;

      if (task.obsReloPaths->size() > post_path_idx) {
        // Step A: Two-point push trajectory
        schedule_path_segment(task.obsReloPaths->at(push_path_idx), obs_meta,
                              robot, timetable, params);

        // Step B: Schedule the "Post-Obs/Return" path
        schedule_path_segment(task.obsReloPaths->at(post_path_idx), obs_meta,
                              robot, timetable, params);
      } else {
        std::cerr << "Error: obsReloPaths missing required paths for index "
                  << obs_ind << std::endl;
      }
    }
  }

  // 3. Execute Edge Paths (Pushing / Relocation Segments)
  int segment_idx = 0;
  for (auto &path_ptr : task.EdgePaths) {
    segment_idx++;
    double segment_ready_time = timetable.get_entity_max_time(robot);

    // Prepare Trajectory Object
    path_ptr->entity = robot;
    path_ptr->CalcualteTimeStamps(
        robot); // Reset internal relative times based on robot constraints

    Pose segment_goal = path_ptr->waypoints.back();

    // A. Resolve Static Blockers (Goal & Trajectory)
    //    (Pass single path as vector to compatible helper)
    resolve_goal_blocking(robot, segment_goal, timetable, entities, params,
                          segment_ready_time, path_ptr->is_transfer,
                          {path_ptr});

    // todo: resolve path blocking
    ////////////////////////////

    /// ////////////

    // B. Find Valid Start Time (Collision Delay)
    std::cout << "  [Segment " << segment_idx << "] Checking schedule..."
              << std::endl;
    double safe_start_time = find_safe_start_time(
        path_ptr.get(), segment_ready_time, timetable, params);
    if (safe_start_time < 0) { // Or if waypoints.empty() after any re-plan
      std::cerr << " [Error] Segment " << segment_idx
                << " failed (permanent blockage or empty path)" << std::endl;
      if (DEBUG_VIS) {
        Pose start_pose = timetable.get_pose(
            robot, segment_ready_time); // Current at ready time
        visualize_current_state(timetable, entities, params, segment_ready_time,
                                start_pose, segment_goal);
      }
      break;
    }

    // C. Register Trajectory
    path_ptr->start_time = safe_start_time;
    path_ptr->is_transfer = path_ptr->is_transfer; // Explicit for clarity
    path_ptr->transferred_object =
        path_ptr->is_transfer ? path_ptr->transferred_object : nullptr;
    timetable.add_trajectory(*path_ptr);

    // D. Handle Retraction (if this was a push)
    if (path_ptr->is_transfer) {
      append_retraction(robot, *path_ptr, timetable);
    }
  }
}

// ==========================================
// 4. MAIN ENTRY POINT
// ==========================================

int main(int argc, char **argv) {
  // --- 1. Load Data ---
  std::string filename = std::string(CMAKE_SOURCE_DIR) +
                         "/final_sequence_o11_iros_obj11_v2.txt.b64";
  std::cout << "[System] Loading sequence: " << filename << std::endl;

  std::vector<FinalAllocation> loadedSequence =
      loadFinalSequenceFromFile(filename);
  if (loadedSequence.empty()) {
    std::cerr << "[System] Failed to load sequence." << std::endl;
    return 1;
  }

  // --- 2. Initialize Environment ---
  Params params = initialize_params(loadedSequence);
  auto entities = initialize_entities(loadedSequence);
  TimeTable timetable(0.5);
  timetable.add_initial(entities);

  // Identify Robots
  std::vector<RobotMeta *> all_robots;
  for (const auto &[name, ent] : entities) {
    if (ent->type == EntityType::ROBOT)
      all_robots.push_back(dynamic_cast<RobotMeta *>(ent));
  }

  // --- 3. Initialize Tasks ---
  std::vector<Task> tasks;
  for (const auto &fa : loadedSequence) {
    tasks.emplace_back(fa, entities);
  }
  std::cout << "[System] Initialized " << tasks.size() << " tasks."
            << std::endl;

  // --- 4. Task Allocation & Execution Loop ---
  int task_counter = 0;
  for (auto &task : tasks) {
    task_counter++;
    std::cout << "\n=== Processing Task " << task_counter << " ("
              << task.targetObject->name << ") ===" << std::endl;

    // Assign Robot
    double earliest_time;
    RobotMeta *assigned_robot = task.assignedRobot;

    if (!assigned_robot) {
      assigned_robot =
          find_earliest_robot(all_robots, timetable, earliest_time);
      if (!assigned_robot) {
        std::cerr << "[Error] No robots available." << std::endl;
        continue;
      }
      task.assignedRobot = assigned_robot;
      std::cout << "[Assign] Assigned " << assigned_robot->name
                << " (Free at t=" << std::fixed << std::setprecision(2)
                << earliest_time << "s)" << std::endl;
    }

    // Execute
    process_task_execution(assigned_robot, task, timetable, entities, params);
  }

  // --- 5. Visualization & Cleanup ---
  show_results(argc, argv, timetable, entities, params);

  for (auto &pair : entities)
    delete pair.second;
  return 0;
}
