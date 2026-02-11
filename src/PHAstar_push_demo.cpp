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
#include <CollisionUtils.h>
#include <LNSTypes.h>
#include <config.h>
#include <iomanip>
#include <iostream>

// for finding parking & LNS
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <numeric>



const bool DEBUG_VIS = false;


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

  // Robot 3
  RobotMeta* robot3 = new RobotMeta;
  robot3->name = "robot3";
  robot3->type = EntityType::ROBOT;
  robot3->initial_pose = {0.5, 4.5, 0.0};
  robot3->size.front_length = 0.32;
  robot3->size.rear_length = 0.2;
  robot3->size.width = 0.3;
  robot3->min_turning_radius = 1.43;
  robot3->wheel_base = 0.4;
  robot3->speed_transit = 0.2;
  robot3->speed_transfer = 0.15;
  entities["robot3"] = robot3;


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
// ==========================================
void diagnose_planning_failure(RobotMeta *robot, const Pose &start,
                               const Pose &goal, double time,
                               TimeTable &timetable,
                               const std::unordered_map<std::string, EntityMeta *> &entities,
                               const Params &params) {
  std::cerr << "\n  [Diagnostics] Analyzing failure for " << robot->name
            << " at t=" << time << "s..." << std::endl;

  auto others_map = timetable.get_poses(time);
  std::vector<std::pair<EntityMeta *, Pose>> others(others_map.begin(),
                                                    others_map.end());

  // --- Helper Lambda: Is this a valid transfer? ---
  // ... (existing lambda) ...

  std::cout << "    Start: (" << start.x << ", " << start.y << ", " << start.yaw << ")" << std::endl;
  std::cout << "    Goal:  (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")" << std::endl;

  // Cross-check with Planner's internal check
  PHAStar diag_planner(robot, goal, &timetable, &entities, params, false, "", time);
  // Note: start node is initialized in constructor from robot->initial_pose
  if (diag_planner.start) {
       auto col_info = diag_planner.check_collision_at(diag_planner.start.get());
       std::cout << "    [Planner Check] Start Node Collision: " << (col_info.is_valid ? "VALID" : "COLLISION") << std::endl;
       if (!col_info.is_valid) {
           std::cout << "      Reason: " << col_info.reason
                     << ", Entity: " << col_info.entity_name << std::endl;
       }

       // Also check Goal
       Node goal_node(goal.x, goal.y, goal.yaw, time + 10.0, 0, 0, nullptr, 0); // Arbitrary future time
       auto goal_info = diag_planner.check_collision_at(&goal_node);
       std::cout << "    [Planner Check] Goal Node Collision: " << (goal_info.is_valid ? "VALID" : "COLLISION") << std::endl;
       if (!goal_info.is_valid) {
           std::cout << "      Reason: " << goal_info.reason
                     << ", Entity: " << goal_info.entity_name << std::endl;
       }
  } else {
       std::cout << "    [Planner Check] Start Node is NULL (Init failed?)" << std::endl;
  }

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
// 2. CORE PLANNING SUB-ROUTINES (Moved up)
// ==========================================

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
  double step_size = 0.2;
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

  // 1. Robot's initial_pose (often designed to be safe)
  double init_L = compute_rs_length(robot->initial_pose);
  candidates.push_back({robot->initial_pose, init_L});

  // 2. Workspace corners — only 1 orientation each (facing inward)
  double margin = 0.6;
  std::vector<std::pair<Pose, double>> corner_list = {
      {{params.min_x + margin, params.min_y + margin, M_PI/4}, 0},   // bottom-left
      {{params.max_x - margin, params.min_y + margin, 3*M_PI/4}, 0}, // bottom-right
      {{params.max_x - margin, params.max_y - margin, -3*M_PI/4}, 0},// top-right
      {{params.min_x + margin, params.max_y - margin, -M_PI/4}, 0},  // top-left
  };
  for (auto &[p, L] : corner_list) {
    L = compute_rs_length(p);
    candidates.push_back({p, L});
  }

  // 3. Nearby spots at ~1.5m radius (quick moves to clear the area)
  double radius = 1.5;
  for (int i = 0; i < 8; i++) {
    double angle = i * M_PI / 4.0;
    double nx = current_pose.x + radius * std::cos(angle);
    double ny = current_pose.y + radius * std::sin(angle);
    // Clamp to workspace bounds
    nx = std::max(params.min_x + margin, std::min(params.max_x - margin, nx));
    ny = std::max(params.min_y + margin, std::min(params.max_y - margin, ny));
    Pose p{nx, ny, angle + M_PI}; // Face back toward original position
    double L = compute_rs_length(p);
    candidates.push_back({p, L});
  }

  // Sort by Reeds-Shepp length (lower is better)
  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b) {
              return a.estimated_rs_length < b.estimated_rs_length;
            });
  return candidates;
}

// ==========================================
// UPDATED: CHECK COLLISION TRAJECTORY (Legacy)
// ==========================================
bool check_collision_trajectory(const Trajectory &traj, double start_time,
                                TimeTable &timetable, const Params &params,
                                bool verbose = false) {
    // Legacy wrapper if needed, or reimplement using detailed
    if (traj.waypoints.empty()) return false;
    // ... (Use detailed or keep duplicate if lazy, better to keep simple)
    // For now I'll keep check_collision_trajectory_detailed separate and above.
    return false; // Placeholder, actually I'll implement detailed fully below
}
// Wait, I am inserting detailed checker here.

CollisionInfo check_collision_trajectory_detailed(const Trajectory &traj, double start_time,
                                TimeTable &timetable, const Params &params,
                                bool verbose = false) {
  if (traj.waypoints.empty())
    return {true, "Empty Trajectory", "", start_time};

  double dt = 0.1;
  double duration = traj.waypoints.back().time;

  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  ObjectMeta *object = dynamic_cast<ObjectMeta *>(traj.transferred_object);

  for (double t = 0; t <= duration; t += dt) {
    double abs_t = start_time + t;
    auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
    Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                   std::get<2>(pose_tuple)};

    CollisionGeometry r_geom = setup_collision_geometry(r_pose, robot->size, 1.0);
    auto others = timetable.get_poses(abs_t);

    CollisionGeometry obj_geom;
    Pose obj_pose;
    const CollisionGeometry* obj_geom_ptr = nullptr;
    const Pose* obj_pose_ptr = nullptr;

    if (traj.is_transfer && object) {
      double offset = robot->size.front_length + object->size.rear_length;
      obj_pose = {r_pose.x + offset * std::cos(r_pose.yaw),
                  r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
      obj_geom = setup_collision_geometry(obj_pose, object->size, 1.0);
      obj_geom_ptr = &obj_geom;
      obj_pose_ptr = &obj_pose;
    }

    for (const auto &[ent, o_pose] : others) {
      if (ent == robot || (object && ent == object))
        continue;
      auto collision = check_entity_collision(r_geom, r_pose, ent, o_pose, params);
      if (collision.has_collision) {
         return {false, "Robot Collision", collision.colliding_entity->name, abs_t};
      }
    }

    if (traj.is_transfer && object) {
      for (const auto &[ent, other_p] : others) {
        if (ent == robot || ent == object) continue;
        if (ent->type == EntityType::ROBOT) continue;
        auto collision = check_entity_collision(obj_geom, obj_pose, ent, other_p, params);
        if (collision.has_collision) {
           return {false, "Object Collision", collision.colliding_entity->name, abs_t};
        }
      }
    }
  }
  return {true, "Valid", "", 0.0};
}

bool relocate_blocking_robot(RobotMeta* blocker,
                             TimeTable& timetable,
                             const Params& params,
                             const std::unordered_map<std::string, EntityMeta*>& entities,
                             const Trajectory* blocked_traj_hint = nullptr) {

    double ready_time = timetable.get_entity_max_time(blocker);
    // Add small buffer to start time to ensure no conflict with previous finish
    ready_time += 0.1;

    Pose start_pose = timetable.get_pose(blocker, ready_time);
    blocker->initial_pose = start_pose;

    std::cout << "  [Relocate] Attempting to move " << blocker->name
              << " from (" << start_pose.x << ", " << start_pose.y << ")" << std::endl;

    auto candidates = generate_parking_candidates(start_pose, blocker, params);

    for (const auto& cand : candidates) {
        bool conflict = false;
        if (blocked_traj_hint) {
            Corners cand_corners = get_corners(cand.pose.x, cand.pose.y, cand.pose.yaw,
                                             blocker->size.front_length, blocker->size.rear_length, blocker->size.width);
            for (size_t i = 0; i < blocked_traj_hint->waypoints.size(); i += 5) {
                const auto& wp = blocked_traj_hint->waypoints[i];
                Corners wp_corners = get_corners(wp.x, wp.y, wp.yaw,
                                               blocker->size.front_length, blocker->size.rear_length, blocker->size.width);
                if (rectangles_intersect(cand_corners, wp_corners)) {
                    conflict = true;
                    break;
                }
            }
        }
        if (conflict) continue;

        PHAStar planner(blocker, cand.pose, &timetable, &entities, params, false, "", ready_time);

        // OPTIMIZATION: Low iteration limit for parking search.
        // Parking moves should be short. If a spot needs many iterations, skip it.
        planner.max_search_iterations = 500;

        auto res = planner.Planning_with_res(ready_time);

        if (res.status == PlanningStatus::SUCCESS) {
            Trajectory relo_traj;
            relo_traj.entity = blocker;
            relo_traj.start_time = ready_time;
            relo_traj.waypoints = res.waypoints;
             for (auto &wp : relo_traj.waypoints)
                wp.time -= ready_time;
            timetable.add_trajectory(relo_traj);
            std::cout << "  [Relocate] SUCCESS: Moved " << blocker->name
                      << " to (" << cand.pose.x << ", " << cand.pose.y << ")" << std::endl;
            return true;
        }
    }
    std::cerr << "  [Relocate] FAILED: Could not find safe parking spot for " << blocker->name << std::endl;
    return false;
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

  std::cout << "  [Transit] Planning " << robot->name << " -> Target ("
            << target_pose.x << ", " << target_pose.y << ", " << target_pose.yaw
            << ") from Start (" << current_pose.x << ", " << current_pose.y << ", " << current_pose.yaw
            << ") at " << start_time << "s" << std::endl;

  PHAStar planner(robot, target_pose, &timetable, &entities, params, false, "",
                  start_time);
  auto path_res = planner.Planning_with_res(start_time);

  // If blocked by robot, we already have a "ghost" path in path_res.waypoints
  if (path_res.status == PlanningStatus::BLOCKED_BY_ROBOT) {
      std::cout << "  [Transit] Path blocked by robot " << path_res.colliding_entity << ". Attempting relocation..." << std::endl;

      // We need a Trajectory object for relocate_blocking_robot
      Trajectory ghost_traj;
      ghost_traj.waypoints = path_res.waypoints;
      ghost_traj.entity = robot; // FIX: Prevent segfault in collision check

      if (ghost_traj.waypoints.empty()) {
          std::cerr << " [Error] Blocked path has no waypoints! Cannot relocate." << std::endl;
          path_res.status = PlanningStatus::NO_PATH_FOUND;
      } else {
          // Need waypoints to be RELATIVE to 0 for check_collision... wait
          // Actually relocate_blocking_robot and checking logic might prefer absolute?
          // check_collision_trajectory_detailed uses relative times in waypoints.
          double initial_t = ghost_traj.waypoints.front().time;
      for(auto& wp : ghost_traj.waypoints) wp.time -= initial_t;

      CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);

      if (!col_info.is_valid && !col_info.entity_name.empty()) {
          // Safeguard: Check if entity exists in map (e.g., "Boundary" might not)
          if (entities.count(col_info.entity_name)) {
              EntityMeta* collider = entities.at(col_info.entity_name);
              if (collider && collider->type == EntityType::ROBOT) {
                  RobotMeta* blocker = dynamic_cast<RobotMeta*>(collider);
                  if (relocate_blocking_robot(blocker, timetable, params, entities, &ghost_traj)) {
                       std::cout << "  [Transit] Relocation successful. Retrying plan..." << std::endl;
                       // Restore absolute times for planner if we use it again? No, we create a new one.
                       PHAStar retry_planner(robot, target_pose, &timetable, &entities, params, false, "", start_time);
                       path_res = retry_planner.Planning_with_res(start_time);
                  } else {
                       std::cerr << "  [Transit] Relocation FAILED. Discarding blocked path." << std::endl;
                       path_res.waypoints.clear();
                       path_res.status = PlanningStatus::NO_PATH_FOUND;
                  }
              }
          } else {
              std::cerr << "  [Transit] Blocked by unknown entity/boundary (" << col_info.entity_name << "). Cannot relocate." << std::endl;
              path_res.waypoints.clear();
              path_res.status = PlanningStatus::NO_PATH_FOUND;
          }
      }
  }
  }

  // Fallback for cases where standard planning finds nothing (Search exhausted)
  if (path_res.waypoints.empty()) {
     std::cerr << " [Transit] Standard planning failed. Attempting to resolve blocking robots with full Ghost Planning..." << std::endl;

     // 1. Attempt Ghost Planning (Ignore other robots)
     PHAStar ghost_planner(robot, target_pose, &timetable, &entities, params, false, "", start_time);
     ghost_planner.set_ignore_other_robots(true);
     auto ghost_res = ghost_planner.Planning_with_res(start_time);

     if (ghost_res.status == PlanningStatus::SUCCESS) {
         // 2. Identify blockers on the ghost path
         Trajectory ghost_traj;
         ghost_traj.waypoints = ghost_res.waypoints;
         ghost_traj.entity = robot; // FIX: Prevent segfault in collision check
         double initial_t = ghost_traj.waypoints.front().time;
         for(auto& wp : ghost_traj.waypoints) wp.time -= initial_t;

         CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);

         if (!col_info.is_valid && !col_info.entity_name.empty()) {
             // Safeguard: Check if entity exists in map
             if (entities.count(col_info.entity_name)) {
                 EntityMeta* collider = entities.at(col_info.entity_name);
                 if (collider && collider->type == EntityType::ROBOT) {
                     RobotMeta* blocker = dynamic_cast<RobotMeta*>(collider);

                     // Restore absolute path for hint
                     for(auto& wp : ghost_traj.waypoints) wp.time += initial_t;

                     if (relocate_blocking_robot(blocker, timetable, params, entities, &ghost_traj)) {
                          std::cout << "  [Transit] Relocation successful. Retrying plan..." << std::endl;
                          PHAStar retry_planner(robot, target_pose, &timetable, &entities, params, false, "", start_time);
                          path_res = retry_planner.Planning_with_res(start_time);
                     }
                 }
             } else {
                 std::cerr << "  [Transit] Ghost path blocked by unknown entity/boundary (" << col_info.entity_name << "). Cannot relocate." << std::endl;
             }
         }
     }
  }

  if (path_res.waypoints.empty()) {
    std::cerr << " [Error] Transit planning failed for " << robot->name
              << " - Status: " << static_cast<int>(path_res.status)
              << ", Detail: " << path_res.failure_detail << std::endl;
    diagnose_planning_failure(robot, current_pose, target_pose, start_time, timetable, entities, params);
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

// Returns list of (robot, free_time) sorted by earliest free time
std::vector<std::pair<RobotMeta*, double>> get_sorted_candidate_robots(
    const std::vector<RobotMeta *> &robots, TimeTable &timetable) {
  std::vector<std::pair<RobotMeta*, double>> candidates;
  for (auto *robot : robots) {
    double t = timetable.get_entity_max_time(robot);
    candidates.emplace_back(robot, t);
  }
  std::sort(candidates.begin(), candidates.end(),
    [](const auto& a, const auto& b) { return a.second < b.second; });
  return candidates;
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
                            TimeTable &timetable, const Params &params,
                            const std::unordered_map<std::string, EntityMeta *> &entities,
                            std::string* out_blocking_entity = nullptr) {
  double check_time = earliest_start;
  double step = 0.5;
  int max_retries = 200; // ~100 seconds wait limit

  std::string last_relocated_robot = "";
  double last_relocation_time = -100.0;
  int relocation_attempts = 0;
  const int max_relocation_attempts = 2; // Cap total relocations per call

  for (int i = 0; i < max_retries; ++i) {
    CollisionInfo col_info = check_collision_trajectory_detailed(*traj, check_time, timetable, params, false);

    if (col_info.is_valid) {
        if (i > 0) std::cout << "  [Delay] Delayed " << (i * step) << "s for safety." << std::endl;
        return check_time;
    }
    
    // Capture blocking entity
    if (out_blocking_entity && !col_info.entity_name.empty()) {
        *out_blocking_entity = col_info.entity_name;
    }

    // Handle Collision
    EntityMeta* collider = nullptr;
    if (entities.count(col_info.entity_name)) {
        collider = entities.at(col_info.entity_name);
    }

    if (collider && collider->type == EntityType::ROBOT) {
        RobotMeta* blocker = dynamic_cast<RobotMeta*>(collider);
        double blocker_free_time = timetable.get_entity_max_time(blocker);

        if (col_info.time > blocker_free_time) {
            // Blocker is stationary/idle. Move it!
            if (relocation_attempts < max_relocation_attempts &&
                (blocker->name != last_relocated_robot || (check_time - last_relocation_time > 5.0))) {

                bool success = relocate_blocking_robot(blocker, timetable, params, entities, traj);
                relocation_attempts++; // Count attempt regardless of outcome

                if (success) {
                    last_relocated_robot = blocker->name;
                    last_relocation_time = check_time;
                    // Retry this time step (decrement so next loop increment checks same time)
                    check_time -= step;
                }
            }
        }
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
// (with optional conflict tracking)
void schedule_path_segment(const EdgePath &edge_path, EntityMeta *obj_meta,
                           RobotMeta *robot, TimeTable &timetable,
                           const Params &params,
                           const std::unordered_map<std::string, EntityMeta *> &entities,
                           std::vector<ConflictRecord> *conflicts = nullptr,
                           int task_index = -1) {
  // 1. Get current available time from the timetable
  double current_avail_time = timetable.get_entity_max_time(robot);

  // 2. Convert EdgePath to Trajectory
  TrajectoryPtr traj =
      ReloPushPath2TrajPtr(edge_path, robot, obj_meta, current_avail_time);

  // FIX: Ensure entity and relative timestamps are set for collision checking!
  traj->entity = robot;
  traj->CalcualteTimeStamps(robot);

  // 3. Find safe start time
  std::string blocker_name = "";
  double safe_start_time =
      find_safe_start_time(traj.get(), current_avail_time, timetable, params, entities, &blocker_name);

  // Track temporal dependency if delay was significant
  if (conflicts && safe_start_time > current_avail_time + 1.0) {
    ConflictRecord cr;
    cr.type = ConflictType::TEMPORAL_DELAY_VICTIM;
    cr.task_index = task_index;
    cr.robot_name = robot->name;
    cr.blocking_entity = blocker_name;
    cr.blocking_task_index = -1;
    cr.time = current_avail_time;
    cr.detail = "Segment delayed by " + std::to_string(safe_start_time - current_avail_time) + "s";
    if (!blocker_name.empty()) cr.detail += " (blocked by " + blocker_name + ")";
    conflicts->push_back(cr);
  }

  // 4. Update timestamps and add to timetable
  traj->start_time = safe_start_time;
  traj->CalcualteTimeStamps(robot);
  timetable.add_trajectory(*traj);
}

// Process task execution with conflict tracking
bool process_task_execution(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    std::vector<ConflictRecord> *conflicts = nullptr,
    int task_index = -1) {
  // 1. Plan Transit to Task Start
  double robot_avail_time = timetable.get_entity_max_time(robot);
  if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time,
                            timetable, entities, params)) {
    std::cerr << "Aborting task due to transit failure." << std::endl;
    // Track as deadlock conflict
    if (conflicts) {
      ConflictRecord cr;
      cr.type = ConflictType::DEADLOCK_VICTIM;
      cr.task_index = task_index;
      cr.robot_name = robot->name;
      cr.blocking_entity = "";
      cr.blocking_task_index = -1;
      cr.time = robot_avail_time;
      cr.detail = "Transit planning failed completely";
      conflicts->push_back(cr);
    }
    return false;
  }

  // 2. ObsRelo (if exists)
  if (task.vertexChain.size() > 2) {
    for (size_t obs_ind = 1; obs_ind < task.vertexChain.size() - 1; obs_ind++) {
      std::cout << "Obstacle Relocation" << std::endl;
      std::string obs_name = task.vertexChain[obs_ind].name;
      auto obs_meta = entities.at(obs_name);

      size_t push_path_idx = 0;
      size_t post_path_idx = 1;

      if (task.obsReloPaths->size() > post_path_idx) {
        schedule_path_segment(task.obsReloPaths->at(push_path_idx), obs_meta,
                              robot, timetable, params, entities, conflicts, task_index);
        schedule_path_segment(task.obsReloPaths->at(post_path_idx), obs_meta,
                              robot, timetable, params, entities, conflicts, task_index);
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

    path_ptr->entity = robot;
    path_ptr->CalcualteTimeStamps(robot);

    Pose segment_goal = path_ptr->waypoints.back();

    std::cout << "  [Segment " << segment_idx << "] Checking schedule..."
              << std::endl;
    std::string blocker_name = "";
    double safe_start_time = find_safe_start_time(
        path_ptr.get(), segment_ready_time, timetable, params, entities, &blocker_name);
    if (safe_start_time < 0) {
      std::cerr << " [Error] Segment " << segment_idx
                << " failed (permanent blockage or empty path)" << std::endl;
      if (DEBUG_VIS) {
        Pose start_pose = timetable.get_pose(robot, segment_ready_time);
        visualize_current_state(timetable, entities, params, segment_ready_time,
                                start_pose, segment_goal);
      }
      // Track as deadlock
      if (conflicts) {
        ConflictRecord cr;
        cr.type = ConflictType::DEADLOCK_VICTIM;
        cr.task_index = task_index;
        cr.robot_name = robot->name;
        cr.blocking_entity = blocker_name;
        cr.blocking_task_index = -1;
        cr.time = segment_ready_time;
        cr.detail = "Segment " + std::to_string(segment_idx) + " permanently blocked";
        if (!blocker_name.empty()) cr.detail += " by " + blocker_name;
        conflicts->push_back(cr);
      }
      return false;
    }

    // Track temporal delay
    if (conflicts && safe_start_time > segment_ready_time + 1.0) {
      ConflictRecord cr;
      cr.type = ConflictType::TEMPORAL_DELAY_VICTIM;
      cr.task_index = task_index;
      cr.robot_name = robot->name;
      cr.blocking_entity = blocker_name;
      cr.blocking_task_index = -1;
      cr.time = segment_ready_time;
      cr.detail = "Edge segment " + std::to_string(segment_idx) + " delayed by " +
                  std::to_string(safe_start_time - segment_ready_time) + "s";
      if (!blocker_name.empty()) cr.detail += " (blocked by " + blocker_name + ")";
      conflicts->push_back(cr);
    }

    path_ptr->start_time = safe_start_time;
    path_ptr->is_transfer = path_ptr->is_transfer;
    path_ptr->transferred_object =
        path_ptr->is_transfer ? path_ptr->transferred_object : nullptr;
    timetable.add_trajectory(*path_ptr);

    if (path_ptr->is_transfer) {
      append_retraction(robot, *path_ptr, timetable);
    }
  }
  return true;
}


// ==========================================
// 4. ALLOCATION RUNNER (for LNS)
// ==========================================

// Pre-parks idle robots to safe locations before the task loop.
// Robots whose first assigned task comes late in the sequence sit idle at their
// initial positions and can block transit paths for other robots. This function
// moves them to safe workspace corners preemptively — mimicking how greedy
// allocation naturally assigns idle robots first so they move out of the way.
void pre_park_idle_robots(
    const std::vector<std::string> &robot_assignment,
    const std::vector<RobotMeta *> &robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    bool verbose) {

  int num_robots = static_cast<int>(robots.size());

  // Skip pre-parking in greedy mode (all assignments empty)
  bool has_forced_assignment = false;
  for (const auto &a : robot_assignment) {
    if (!a.empty()) { has_forced_assignment = true; break; }
  }
  if (!has_forced_assignment) return;

  // Find each robot's first assigned task index
  std::unordered_map<std::string, int> first_task_idx;
  for (const auto *r : robots) {
    first_task_idx[r->name] = -1; // -1 = not assigned to any task
  }
  for (size_t i = 0; i < robot_assignment.size(); i++) {
    const auto &rname = robot_assignment[i];
    if (!rname.empty() && first_task_idx.count(rname) &&
        first_task_idx[rname] < 0) {
      first_task_idx[rname] = static_cast<int>(i);
    }
  }

  // Safe parking corners (spread across workspace)
  double margin = 0.6;
  std::vector<Pose> safe_spots = {
      {params.min_x + margin, params.min_y + margin, 0.0},           // bottom-left
      {params.max_x - margin, params.min_y + margin, M_PI},          // bottom-right
      {params.max_x - margin, params.max_y - margin, M_PI},          // top-right
      {params.min_x + margin, params.max_y - margin, 0.0},           // top-left
  };

  int spot_idx = 0;
  for (auto *robot : robots) {
    int first_task = first_task_idx[robot->name];

    // If robot's first task is late (after the first num_robots tasks),
    // or robot has no assigned task at all, pre-park it
    if (first_task < 0 || first_task >= num_robots) {
      // Pick a safe spot that's different from the robot's current position
      Pose target = safe_spots[spot_idx % safe_spots.size()];
      spot_idx++;

      double ready_time = timetable.get_entity_max_time(robot);
      Pose current = timetable.get_pose(robot, ready_time);

      // Skip if robot is already near the target
      double dist = std::hypot(current.x - target.x, current.y - target.y);
      if (dist < 0.5) continue;

      if (verbose) {
        std::cout << "  [Pre-park] Moving idle " << robot->name
                  << " from (" << current.x << ", " << current.y
                  << ") to safe corner (" << target.x << ", " << target.y
                  << ")" << std::endl;
      }

      robot->initial_pose = current;
      PHAStar planner(robot, target, &timetable, &entities, params, false, "",
                      ready_time);
      planner.max_search_iterations = 3000;
      auto res = planner.Planning_with_res(ready_time);

      if (res.status == PlanningStatus::SUCCESS && !res.waypoints.empty()) {
        Trajectory park_traj;
        park_traj.entity = robot;
        park_traj.start_time = ready_time;
        park_traj.waypoints = res.waypoints;
        for (auto &wp : park_traj.waypoints)
          wp.time -= ready_time;
        timetable.add_trajectory(park_traj);

        if (verbose) {
          std::cout << "  [Pre-park] " << robot->name << " parked successfully."
                    << std::endl;
        }
      } else {
        if (verbose) {
          std::cout << "  [Pre-park] " << robot->name
                    << " parking failed (will relocate on demand)." << std::endl;
        }
      }
    }
  }
}

// Runs the full task allocation with a given robot assignment.
// robot_assignment[task_i] = robot name for the preferred robot.
// If "", the greedy (earliest-free) heuristic is used.
// Returns an AllocationSolution with conflict records.
AllocationSolution run_allocation(
    const std::vector<FinalAllocation> &loadedSequence,
    const std::vector<std::string> &robot_assignment,
    const Params &params_template,
    bool verbose = true) {

  // --- Fresh environment for each allocation run ---
  Params params = params_template;
  auto entities = initialize_entities(loadedSequence);
  TimeTable timetable(0.5);
  timetable.add_initial(entities);

  // Build robot pointer list (fresh copies)
  std::vector<RobotMeta *> robots;
  for (const auto &[name, ent] : entities) {
    if (ent->type == EntityType::ROBOT)
      robots.push_back(dynamic_cast<RobotMeta *>(ent));
  }

  // Build fresh tasks
  std::vector<Task> tasks;
  for (const auto &fa : loadedSequence) {
    tasks.emplace_back(fa, entities);
  }

  // Pre-park idle robots to prevent blocking early tasks
  pre_park_idle_robots(robot_assignment, robots, timetable, entities, params,
                       verbose);

  AllocationSolution solution;
  solution.robot_assignment.resize(tasks.size(), "");
  int task_counter = 0;

  for (size_t ti = 0; ti < tasks.size(); ti++) {
    task_counter++;
    auto &task = tasks[ti];

    if (verbose) {
      std::cout << "\n=== Processing Task " << task_counter << " ("
                << task.targetObject->name << ") ===" << std::endl;
    }

    // Build candidate list
    auto candidates = get_sorted_candidate_robots(robots, timetable);

    // If we have a preferred robot assignment, prioritize it
    if (ti < robot_assignment.size() && !robot_assignment[ti].empty()) {
      std::string preferred_name = robot_assignment[ti];
      auto it = std::find_if(candidates.begin(), candidates.end(),
          [&](const auto &p) { return p.first->name == preferred_name; });
      if (it != candidates.end()) {
        std::rotate(candidates.begin(), it, it + 1);
      }
    }

    TaskAllocationResult result;
    result.task_index = static_cast<int>(ti);
    result.success = false;
    result.completion_time = 0.0;
    
    // Track mapping from robot name to its last successful task index
    static std::unordered_map<std::string, int> robot_last_task_idx;
    if (ti == 0) robot_last_task_idx.clear(); // Clear on first task of allocation run

    for (auto &[cand_robot, free_time] : candidates) {
      task.assignedRobot = cand_robot;
      if (verbose) {
        std::cout << "[Assign] Attempting " << cand_robot->name
                  << " (Free at t=" << std::fixed << std::setprecision(2)
                  << free_time << "s)" << std::endl;
      }

      std::vector<ConflictRecord> task_conflicts;

      if (process_task_execution(cand_robot, task, timetable, entities, params,
                                 &task_conflicts, static_cast<int>(ti))) {
        result.success = true;
        result.robot_name = cand_robot->name;
        result.completion_time = timetable.get_entity_max_time(cand_robot);
        result.conflicts = task_conflicts;

        // Record which robot was used (by name for determinism)
        solution.robot_assignment[ti] = cand_robot->name;
        robot_last_task_idx[cand_robot->name] = static_cast<int>(ti);

        // RETROACTIVE LABELING: Check for TEMPORAL conflicts and blame the blocker
        for (const auto& c : task_conflicts) {
            if (c.type == ConflictType::TEMPORAL_DELAY_VICTIM && !c.blocking_entity.empty()) {
                if (robot_last_task_idx.count(c.blocking_entity)) {
                    int last_idx = robot_last_task_idx[c.blocking_entity];
                    if (last_idx >= 0 && last_idx < solution.results.size()) {
                        ConflictRecord retro_cr;
                        retro_cr.type = ConflictType::TEMPORAL_DELAY_CAUSE;
                        retro_cr.task_index = last_idx;
                        retro_cr.robot_name = c.blocking_entity;
                        retro_cr.blocking_task_index = static_cast<int>(ti); // Caused delay here
                        retro_cr.time = solution.results[last_idx].completion_time;
                        retro_cr.detail = "Caused delay for Task " + std::to_string(ti) +
                                          " (" + c.detail + ")";
                        
                        // Avoid duplicates if possible? simple vector push is fine for now
                        solution.results[last_idx].conflicts.push_back(retro_cr);
                    }
                }
            }
        }

        if (verbose) {
          std::cout << "[Assign] SUCCESS with " << cand_robot->name << std::endl;
        }
        break;
      } else {
        // Record the failure attempt
        ConflictRecord cr;
        cr.type = ConflictType::PHYSICAL_BLOCK_VICTIM; // Default
        
        // Enhance: Check if it was a DEADLOCK from transit failure
        bool is_deadlock = false;
        for(const auto& c : task_conflicts) {
            if (c.type == ConflictType::DEADLOCK_VICTIM) {
                is_deadlock = true;
                cr.type = ConflictType::DEADLOCK_VICTIM;
                cr.detail = c.detail;
                break;
            }
        }
        
        if (!is_deadlock) {
             cr.detail = "Robot " + cand_robot->name + " failed execution";
        }

        cr.task_index = static_cast<int>(ti);
        cr.robot_name = cand_robot->name;
        cr.blocking_entity = "";
        cr.blocking_task_index = -1;
        cr.time = free_time;
        
        task_conflicts.push_back(cr);
        result.conflicts = task_conflicts;

        // RETROACTIVE LABELING:
        // If this robot is DEADLOCKED/TRAPPED, blame the previous task
        if (is_deadlock && robot_last_task_idx.count(cand_robot->name)) {
            int last_idx = robot_last_task_idx[cand_robot->name];
            if (last_idx >= 0 && last_idx < solution.results.size()) {
                 ConflictRecord retro_cr;
                 retro_cr.type = ConflictType::DEADLOCK_CAUSE;
                 retro_cr.task_index = last_idx;
                 retro_cr.robot_name = cand_robot->name;
                 retro_cr.blocking_task_index = static_cast<int>(ti); // Caused failure here
                 retro_cr.time = solution.results[last_idx].completion_time;
                 retro_cr.detail = "Ended in trap state, causing failure in Task " + std::to_string(ti);
                 
                 solution.results[last_idx].conflicts.push_back(retro_cr);
            }
        }

        if (verbose) {
          std::cout << "[Assign] FAILED with " << cand_robot->name
                    << " (Transit/Exec Failed) - Trying next..." << std::endl;
        }
      }
    }

    if (!result.success) {
      if (verbose) {
        std::cerr << "[Critical] Task " << task_counter
                  << " failed with ALL available robots." << std::endl;
      }
      // EARLY EXIT: If a task fails completely, the solution is invalid/worse. 
      // Stop wasting time on this iteration.
      solution.results.push_back(result);
      solution.failed_tasks++;
      // We still return the partial solution so LNS can count it as a failure, 
      // but we shouldn't compute the rest.
      break; 
    }

    solution.results.push_back(result);
  }

  solution.compute_stats();


  // Cleanup fresh entities
  for (auto &pair : entities)
    delete pair.second;

  return solution;
}


// ==========================================
// 5. LARGE NEIGHBORHOOD SEARCH
// ==========================================

void print_solution_summary(const AllocationSolution &sol, const std::string &label) {
  std::cout << "\n--- " << label << " ---" << std::endl;
  std::cout << "  Makespan: " << std::fixed << std::setprecision(2) << sol.makespan << "s" << std::endl;
  std::cout << "  Failed tasks: " << sol.failed_tasks << " / " << sol.results.size() << std::endl;
  std::cout << "  Total conflicts: " << sol.total_conflicts << std::endl;
  std::cout << "  Assignment: [";
  for (size_t i = 0; i < sol.robot_assignment.size(); i++) {
    if (i > 0) std::cout << ", ";
    std::cout << sol.robot_assignment[i];
  }
  std::cout << "]" << std::endl;

  // Print per-task details
  for (const auto &r : sol.results) {
    std::string status = r.success ? "OK" : "FAIL";
    std::cout << "  Task " << r.task_index << ": " << status
              << " by " << r.robot_name
              << " (t=" << std::fixed << std::setprecision(2) << r.completion_time << "s)"
              << " conflicts=" << r.conflicts.size() << std::endl;
    for (const auto &c : r.conflicts) {
      std::string type_str;
      switch (c.type) {
        case ConflictType::PHYSICAL_BLOCK_VICTIM: type_str = "BLOCKED"; break;
        case ConflictType::PHYSICAL_BLOCK_CAUSE: type_str = "BLOCKING"; break;
        case ConflictType::TEMPORAL_DELAY_VICTIM: type_str = "DELAYED"; break;
        case ConflictType::TEMPORAL_DELAY_CAUSE: type_str = "CAUSED_DELAY"; break;
        case ConflictType::DEADLOCK_VICTIM: type_str = "DEADLOCK_VICTIM"; break;
        case ConflictType::DEADLOCK_CAUSE: type_str = "DEADLOCK_CAUSE"; break;
      }
      std::cout << "    *** [" << type_str << "] " << c.detail << std::endl;
    }
  }
}

// Compare two solutions: returns true if 'candidate' is better than 'current'
bool is_better_solution(const AllocationSolution &candidate, const AllocationSolution &current) {
  // Priority 1: Fewer failed tasks
  if (candidate.failed_tasks != current.failed_tasks)
    return candidate.failed_tasks < current.failed_tasks;
  // Priority 2: Lower makespan
  if (std::abs(candidate.makespan - current.makespan) > 0.5)
    return candidate.makespan < current.makespan;
  // Priority 3: Fewer conflicts
  return candidate.total_conflicts < current.total_conflicts;
}

AllocationSolution run_lns(
    const std::vector<FinalAllocation> &loadedSequence,
    const std::vector<std::string> &robot_names,
    const Params &params,
    const AllocationSolution &initial_solution,
    int max_iterations = 50) {

  int num_robots = static_cast<int>(robot_names.size());

  std::mt19937 rng(42); // Fixed seed for reproducibility
  int num_tasks = static_cast<int>(initial_solution.robot_assignment.size());

  AllocationSolution best = initial_solution;
  AllocationSolution current = initial_solution;

  std::cout << "\n========================================" << std::endl;
  std::cout << " STARTING LARGE NEIGHBORHOOD SEARCH" << std::endl;
  std::cout << "========================================" << std::endl;
  print_solution_summary(best, "Initial Solution (Greedy)");

  for (int iter = 0; iter < max_iterations; iter++) {
    std::cout << "\n--- LNS Iteration " << (iter + 1) << "/" << max_iterations
              << " ---" << std::endl;

    // === DESTROY: Select which tasks to re-assign ===
    std::set<int> destroyed_tasks;
    int destroy_strategy = iter % 6; // Cycle through 6 strategies

    if (destroy_strategy == 0) {
      // Strategy 1: Random removal (2-4 tasks)
      std::uniform_int_distribution<int> k_dist(2, std::min(4, num_tasks));
      int k = k_dist(rng);
      std::vector<int> indices(num_tasks);
      std::iota(indices.begin(), indices.end(), 0);
      std::shuffle(indices.begin(), indices.end(), rng);
      for (int i = 0; i < k; i++) {
        destroyed_tasks.insert(indices[i]);
      }
      std::cout << "[LNS] Destroy strategy: RANDOM (" << k << " tasks)" << std::endl;

    } else if (destroy_strategy == 1) {
      // Strategy 2: Conflict-guided (tasks with most conflicts + neighbors)
      int worst_task = -1;
      int worst_conflicts = -1;
      for (const auto &r : current.results) {
        int nc = static_cast<int>(r.conflicts.size());
        if (!r.success) nc += 10;
        if (nc > worst_conflicts) {
          worst_conflicts = nc;
          worst_task = r.task_index;
        }
      }
      if (worst_task >= 0) {
        destroyed_tasks.insert(worst_task);
        if (worst_task > 0) destroyed_tasks.insert(worst_task - 1);
        if (worst_task < num_tasks - 1) destroyed_tasks.insert(worst_task + 1);
      }
      std::cout << "[LNS] Destroy strategy: CONFLICT-GUIDED" << std::endl;

    } else if (destroy_strategy == 2) {
      // Strategy 3: Robot-targeted (unassign all tasks from one robot)
      std::uniform_int_distribution<int> robot_dist(0, num_robots - 1);
      int target_idx = robot_dist(rng);
      std::string target_robot_name = robot_names[target_idx];
      for (int t = 0; t < num_tasks; t++) {
        if (current.robot_assignment[t] == target_robot_name) {
          destroyed_tasks.insert(t);
        }
      }
      std::cout << "[LNS] Destroy strategy: ROBOT-TARGETED (" << target_robot_name << ")" << std::endl;

    } else if (destroy_strategy == 3) {
      // Strategy 4: BLOCKING Conflict Targeting
      // Destroy tasks that faced PHYSICAL BLOCKING (VICTIM) or CAUSED it
      for (const auto &r : current.results) {
        for (const auto &c : r.conflicts) {
          if (c.type == ConflictType::PHYSICAL_BLOCK_VICTIM || c.type == ConflictType::PHYSICAL_BLOCK_CAUSE) {
            destroyed_tasks.insert(r.task_index);
            if (c.blocking_task_index >= 0) destroyed_tasks.insert(c.blocking_task_index);
          }
        }
      }
      if (destroyed_tasks.empty() && !current.results.empty()) {
          // Fallback if no specific blocking conflicts
          destroyed_tasks.insert(rng() % num_tasks);
      }
      std::cout << "[LNS] Destroy strategy: BLOCKING-GUIDED" << std::endl;

    } else if (destroy_strategy == 4) {
      // Strategy 5: TEMPORAL Conflict Targeting
      for (const auto &r : current.results) {
        for (const auto &c : r.conflicts) {
          if (c.type == ConflictType::TEMPORAL_DELAY_VICTIM || c.type == ConflictType::TEMPORAL_DELAY_CAUSE) {
            destroyed_tasks.insert(r.task_index);
            if (c.blocking_task_index >= 0) destroyed_tasks.insert(c.blocking_task_index);
          }
        }
      }
      if (destroyed_tasks.empty() && !current.results.empty()) {
          destroyed_tasks.insert(rng() % num_tasks);
      }
      std::cout << "[LNS] Destroy strategy: TEMPORAL-GUIDED" << std::endl;

    } else {
      // Strategy 6: DEADLOCK / Failure Targeting
      for (const auto &r : current.results) {
        if (!r.success) destroyed_tasks.insert(r.task_index);
        for (const auto &c : r.conflicts) {
          if (c.type == ConflictType::DEADLOCK_VICTIM || c.type == ConflictType::DEADLOCK_CAUSE) {
            destroyed_tasks.insert(r.task_index);
            if (c.blocking_task_index >= 0) destroyed_tasks.insert(c.blocking_task_index);
          }
        }
      }
      if (destroyed_tasks.empty() && !current.results.empty()) {
          destroyed_tasks.insert(rng() % num_tasks);
      }
      std::cout << "[LNS] Destroy strategy: DEADLOCK-GUIDED" << std::endl;
    }

    if (destroyed_tasks.empty()) {
      std::cout << "[LNS] No tasks destroyed, skipping iteration." << std::endl;
      continue;
    }

    std::cout << "[LNS] Destroyed tasks: {";
    for (auto it = destroyed_tasks.begin(); it != destroyed_tasks.end(); ++it) {
      if (it != destroyed_tasks.begin()) std::cout << ", ";
      std::cout << *it;
    }
    std::cout << "}" << std::endl;

    // === REPAIR: Build new assignment ===
    // Keep non-destroyed tasks at their current assignment
    std::vector<std::string> new_assignment = current.robot_assignment;

    // Constraint: Load Balancing (max tasks per robot = ceil(avg) + 2)
    // +2 allows some flexibility while preventing extreme imbalance (e.g. 7 vs 0)
    int max_tasks_per_robot = (num_tasks / num_robots) + 2;

    // Count current assignments for non-destroyed tasks
    std::unordered_map<std::string, int> task_counts;
    for (int t = 0; t < num_tasks; t++) {
        if (destroyed_tasks.find(t) == destroyed_tasks.end()) {
             task_counts[new_assignment[t]]++;
        }
    }

    std::vector<int> destroyed_vec(destroyed_tasks.begin(), destroyed_tasks.end());
    // Shuffle destroyed tasks to avoid bias in assignment order
    std::shuffle(destroyed_vec.begin(), destroyed_vec.end(), rng);

    for (int t : destroyed_vec) {
      // Find robots that aren't overloaded
      std::vector<int> valid_robot_indices;
      for(int r=0; r<num_robots; ++r) {
          if (task_counts[robot_names[r]] < max_tasks_per_robot) {
              valid_robot_indices.push_back(r);
          }
      }

      // Fallback: if all valid are overloaded (shouldn't happen with sufficient buffer), use all
      if (valid_robot_indices.empty()) {
          for(int r=0; r<num_robots; ++r) valid_robot_indices.push_back(r);
      }

      std::uniform_int_distribution<> rob_dist(0, valid_robot_indices.size() - 1);
      int new_idx = valid_robot_indices[rob_dist(rng)];
      std::string new_robot_name = robot_names[new_idx];

      // Try to ensure change if possible (unless only 1 valid robot)
      if (new_robot_name == current.robot_assignment[t] && valid_robot_indices.size() > 1) {
          // If we picked the same one, try again (simple retry)
          new_idx = valid_robot_indices[rob_dist(rng)];
          new_robot_name = robot_names[new_idx];
      }

      new_assignment[t] = new_robot_name;
      task_counts[new_robot_name]++;
    }

    std::cout << "[LNS] New assignment: [";
    for (size_t i = 0; i < new_assignment.size(); i++) {
      if (i > 0) std::cout << ", ";
      if (destroyed_tasks.count(static_cast<int>(i))) {
        std::cout << "\033[1;33m" << new_assignment[i] << "\033[0m"; // Highlight changed
      } else {
        std::cout << new_assignment[i];
      }
    }
    std::cout << "]" << std::endl;

    // === EVALUATE: Run full allocation with new assignment ===
    AllocationSolution candidate = run_allocation(
        loadedSequence, new_assignment, params, false /*verbose*/);

    std::cout << "[LNS] Candidate: makespan=" << std::fixed << std::setprecision(2)
              << candidate.makespan << "s, failed=" << candidate.failed_tasks
              << ", conflicts=" << candidate.total_conflicts << std::endl;

    // === ACCEPT/REJECT ===
    if (is_better_solution(candidate, current)) {
      std::cout << "[LNS] ACCEPTED (improvement)" << std::endl;
      current = candidate;
      if (is_better_solution(candidate, best)) {
        best = candidate;
        std::cout << "[LNS] *** NEW BEST SOLUTION ***" << std::endl;
      }
    } else {
      std::cout << "[LNS] Rejected (no improvement)" << std::endl;
    }

    std::cout << "[LNS] Current best: makespan=" << std::fixed << std::setprecision(2)
              << best.makespan << "s, failed=" << best.failed_tasks << std::endl;
  }

  std::cout << "\n========================================" << std::endl;
  std::cout << " LNS COMPLETE" << std::endl;
  std::cout << "========================================" << std::endl;
  print_solution_summary(best, "Best Solution (LNS)");

  return best;
}


// ==========================================
// 6. MAIN ENTRY POINT
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

  // Identify Robots and build sorted name list for deterministic ordering
  std::vector<std::string> robot_names;
  for (const auto &[name, ent] : entities) {
    if (ent->type == EntityType::ROBOT)
      robot_names.push_back(name);
  }
  std::sort(robot_names.begin(), robot_names.end());
  int num_robots = static_cast<int>(robot_names.size());
  int num_tasks = static_cast<int>(loadedSequence.size());

  std::cout << "[System] " << num_robots << " robots, " << num_tasks << " tasks." << std::endl;

  // --- 3. Run Greedy Allocation (Initial Solution) ---
  std::cout << "\n========================================" << std::endl;
  std::cout << " PHASE 1: GREEDY ALLOCATION" << std::endl;
  std::cout << "========================================" << std::endl;

  // Use empty strings for all tasks (let greedy decide)
  std::vector<std::string> greedy_assignment(num_tasks, "");
  AllocationSolution greedy_solution = run_allocation(
      loadedSequence, greedy_assignment, params, true);

  print_solution_summary(greedy_solution, "Greedy Solution");



  // --- 4. Run LNS Improvement ---
  std::cout << "\n========================================" << std::endl;
  std::cout << " PHASE 2: LNS IMPROVEMENT" << std::endl;
  std::cout << "========================================" << std::endl;

  AllocationSolution best_solution = run_lns(
      loadedSequence, robot_names, params, greedy_solution,
      /*max_iterations=*/50);

  // --- 5. Print Improvement Summary ---
  std::cout << "\n========================================" << std::endl;
  std::cout << " IMPROVEMENT SUMMARY" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "  Greedy makespan:  " << std::fixed << std::setprecision(2) << greedy_solution.makespan << "s" << std::endl;
  std::cout << "  LNS best makespan: " << best_solution.makespan << "s" << std::endl;
  double improvement = greedy_solution.makespan - best_solution.makespan;
  double pct = (greedy_solution.makespan > 0) ? (improvement / greedy_solution.makespan * 100.0) : 0.0;
  std::cout << "  Improvement: " << improvement << "s (" << std::setprecision(1) << pct << "%)" << std::endl;
  std::cout << "  Greedy failed: " << greedy_solution.failed_tasks << ", LNS failed: " << best_solution.failed_tasks << std::endl;
  std::cout << "  Greedy conflicts: " << greedy_solution.total_conflicts << ", LNS conflicts: " << best_solution.total_conflicts << std::endl;

  // --- 6. Re-run best solution for visualization ---
  std::cout << "\n[System] Re-running best solution for visualization..." << std::endl;

  // Re-initialize for final visualization run
  for (auto &pair : entities)
    delete pair.second;
  entities = initialize_entities(loadedSequence);
  TimeTable final_timetable(0.5);
  final_timetable.add_initial(entities);

  std::vector<RobotMeta *> viz_robots;
  for (const auto &[name, ent] : entities) {
    if (ent->type == EntityType::ROBOT)
      viz_robots.push_back(dynamic_cast<RobotMeta *>(ent));
  }

  std::vector<Task> viz_tasks;
  for (const auto &fa : loadedSequence) {
    viz_tasks.emplace_back(fa, entities);
  }

  // Re-run with best assignment
  for (size_t ti = 0; ti < viz_tasks.size(); ti++) {
    auto &task = viz_tasks[ti];
    auto candidates = get_sorted_candidate_robots(viz_robots, final_timetable);

    if (ti < best_solution.robot_assignment.size() &&
        !best_solution.robot_assignment[ti].empty()) {
      std::string preferred_name = best_solution.robot_assignment[ti];
      auto it = std::find_if(candidates.begin(), candidates.end(),
          [&](const auto &p) { return p.first->name == preferred_name; });
      if (it != candidates.end()) {
        std::rotate(candidates.begin(), it, it + 1);
      }
    }

    for (auto &[cand_robot, free_time] : candidates) {
      task.assignedRobot = cand_robot;
      if (process_task_execution(cand_robot, task, final_timetable, entities, params)) {
        break;
      }
    }
  }

  //show_results(argc, argv, final_timetable, entities, params);

  for (auto &pair : entities)
    delete pair.second;
  return 0;
}
