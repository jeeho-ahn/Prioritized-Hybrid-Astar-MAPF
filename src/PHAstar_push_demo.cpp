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
#include <config.h>
#include <iomanip>
#include <iostream>

// for finding parking
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>


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
        
        // OPTIMIZATION: Set strict iteration limit for parking search.
        // If a spot is hard to reach, it's likely bad. Fail fast and try next.
        planner.max_search_iterations = 2000; 
        
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
  bool relocated = false;
  do {
    if (path_res.status == PlanningStatus::BLOCKED_BY_ROBOT) {
        std::cout << "  [Transit] Path blocked by robot " << path_res.colliding_entity << ". Attempting relocation..." << std::endl;
        
        Trajectory ghost_traj;
        ghost_traj.waypoints = path_res.waypoints;
        ghost_traj.entity = robot;
        
        if (ghost_traj.waypoints.empty()) {
            std::cerr << " [Error] Blocked path has no waypoints! Cannot relocate." << std::endl;
            path_res.status = PlanningStatus::NO_PATH_FOUND;
            break;
        }
        
        double initial_t = ghost_traj.waypoints.front().time;
        for(auto& wp : ghost_traj.waypoints) wp.time -= initial_t;
        
        CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);
        
        if (!col_info.is_valid && !col_info.entity_name.empty()) {
            if (entities.count(col_info.entity_name)) {
                EntityMeta* collider = entities.at(col_info.entity_name);
                if (collider && collider->type == EntityType::ROBOT) {
                    RobotMeta* blocker = dynamic_cast<RobotMeta*>(collider);
                    relocated = relocate_blocking_robot(blocker, timetable, params, entities, &ghost_traj);
                    if (relocated) {
                        std::cout << "  [Transit] Relocation successful. Retrying plan..." << std::endl;
                        PHAStar retry_planner(robot, target_pose, &timetable, &entities, params, false, "", start_time);
                        path_res = retry_planner.Planning_with_res(start_time);
                    } else {
                        std::cerr << "  [Transit] Relocation FAILED. Discarding blocked path." << std::endl;
                        path_res.waypoints.clear();
                        path_res.status = PlanningStatus::NO_PATH_FOUND;
                        break;
                    }
                } else {
                    path_res.status = PlanningStatus::NO_PATH_FOUND;
                    break;
                }
            } else {
                std::cerr << "  [Transit] Blocked by unknown entity/boundary (" << col_info.entity_name << "). Cannot relocate." << std::endl;
                path_res.waypoints.clear();
                path_res.status = PlanningStatus::NO_PATH_FOUND;
                break;
            }
        }
    } else {
        break;
    }
  } while (relocated && path_res.status == PlanningStatus::BLOCKED_BY_ROBOT);

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

  // Check collision for final push segment
  Trajectory push_traj;
  push_traj.entity = robot;
  push_traj.start_time = start_time + (path_res.waypoints.size() > 1 ? path_res.waypoints[path_res.waypoints.size() - 2].time : 0);
  push_traj.waypoints = {path_res.waypoints[path_res.waypoints.size() - 2], final_push_wpt};
  // Make times relative for check
  double push_start_rel = push_traj.waypoints[0].time;
  for (auto &wp : push_traj.waypoints) wp.time -= push_start_rel;

  CollisionInfo push_col = check_collision_trajectory_detailed(push_traj, push_traj.start_time, timetable, params);
  if (!push_col.is_valid) {
    std::cerr << " [Error] Final push collides with " << push_col.entity_name << " at t=" << push_col.time << std::endl;
    return false;
  }

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
                            const std::unordered_map<std::string, EntityMeta *> &entities) {
  double check_time = earliest_start;
  double step = 0.5;
  int max_retries = 200; // ~100 seconds wait limit
  
  std::string last_relocated_robot = "";
  double last_relocation_time = -100.0;

  for (int i = 0; i < max_retries; ++i) {
    CollisionInfo col_info = check_collision_trajectory_detailed(*traj, check_time, timetable, params, false);
    
    if (col_info.is_valid) {
        if (i > 0) std::cout << "  [Delay] Delayed " << (i * step) << "s for safety." << std::endl;
        return check_time;
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
            if (blocker->name != last_relocated_robot || (check_time - last_relocation_time > 5.0)) {
                
                if (relocate_blocking_robot(blocker, timetable, params, entities, traj)) {
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
                       TimeTable &timetable, const Params &params) {
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

  // Check collision for retraction
  CollisionInfo retract_col = check_collision_trajectory_detailed(retract_traj, retract_traj.start_time, timetable, params);
  if (!retract_col.is_valid) {
    std::cerr << " [Error] Retraction collides with " << retract_col.entity_name << " at t=" << retract_col.time << std::endl;
    // Optionally shorten or skip retraction
    return;
  }

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
                           const Params &params,
                           const std::unordered_map<std::string, EntityMeta *> &entities) {
  // 1. Get current available time from the timetable
  double current_avail_time = timetable.get_entity_max_time(robot);

  // 2. Convert EdgePath to Trajectory
  // ReloPushPath2TrajPtr is defined in Task.h
  TrajectoryPtr traj =
      ReloPushPath2TrajPtr(edge_path, robot, obj_meta, current_avail_time);

  // FIX: Ensure entity and relative timestamps are set for collision checking!
  traj->entity = robot;
  traj->CalcualteTimeStamps(robot);

  // 3. Find safe start time
  double safe_start_time =
      find_safe_start_time(traj.get(), current_avail_time, timetable, params, entities);

  // 4. Update timestamps and add to timetable
  traj->start_time = safe_start_time;

  // CalcualteTimeStamps is defined in Entities.h
  traj->CalcualteTimeStamps(robot);

  timetable.add_trajectory(*traj);
}

bool process_task_execution(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params) {
  // 1. Plan Transit to Task Start
  double robot_avail_time = timetable.get_entity_max_time(robot);
  if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time,
                            timetable, entities, params)) {
    std::cerr << "Aborting task due to transit failure." << std::endl;
    return false;
  }

  // 2. ObsRelo (if exists)
  if (task.vertexChain.size() > 2) {
    // Iterate through obstacles
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
                              robot, timetable, params, entities);

        // Step B: Schedule the "Post-Obs/Return" path
        schedule_path_segment(task.obsReloPaths->at(post_path_idx), obs_meta,
                              robot, timetable, params, entities);
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

    // B. Find Valid Start Time (Collision Delay)
    std::cout << "  [Segment " << segment_idx << "] Checking schedule..."
              << std::endl;
    double safe_start_time = find_safe_start_time(
        path_ptr.get(), segment_ready_time, timetable, params, entities);
    if (safe_start_time < 0) { // Or if waypoints.empty() after any re-plan
      std::cerr << " [Error] Segment " << segment_idx
                << " failed (permanent blockage or empty path)" << std::endl;
      if (DEBUG_VIS) {
        Pose start_pose = timetable.get_pose(
            robot, segment_ready_time); // Current at ready time
        visualize_current_state(timetable, entities, params, segment_ready_time,
                                start_pose, segment_goal);
      }
      return false; // Break logic was here, now return false
    }

    // C. Register Trajectory
    path_ptr->start_time = safe_start_time;
    path_ptr->is_transfer = path_ptr->is_transfer; // Explicit for clarity
    path_ptr->transferred_object =
        path_ptr->is_transfer ? path_ptr->transferred_object : nullptr;
    timetable.add_trajectory(*path_ptr);

    // D. Handle Retraction (if this was a push)
    if (path_ptr->is_transfer) {
      append_retraction(robot, *path_ptr, timetable, params);
    }
  }
  return true;
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

    // Generate candidate list sorted by availability
    auto candidates = get_sorted_candidate_robots(all_robots, timetable);

    // If a robot was pre-assigned, prioritize it
    if (task.assignedRobot) {
        auto it = std::find_if(candidates.begin(), candidates.end(),
            [&](const auto& p) { return p.first == task.assignedRobot; });
        if (it != candidates.end()) {
            std::rotate(candidates.begin(), it, it + 1);
        }
    }

    bool task_success = false;
    for (auto& [cand_robot, free_time] : candidates) {
        task.assignedRobot = cand_robot; // Tentative assignment
        std::cout << "[Assign] Attempting " << cand_robot->name 
                  << " (Free at t=" << std::fixed << std::setprecision(2) << free_time << "s)" << std::endl;

        if (process_task_execution(cand_robot, task, timetable, entities, params)) {
             task_success = true;
             std::cout << "[Assign] SUCCESS with " << cand_robot->name << std::endl;
             break;
        } else {
             std::cout << "[Assign] FAILED with " << cand_robot->name << " (Transit Blocked) - Trying next..." << std::endl;
        }
    }

    if (!task_success) {
        std::cerr << "[Critical] Task " << task_counter << " failed with ALL available robots." << std::endl;
    }
  }

  // --- 5. Visualization & Cleanup ---
  show_results(argc, argv, timetable, entities, params);

  for (auto &pair : entities)
    delete pair.second;
  return 0;
}
