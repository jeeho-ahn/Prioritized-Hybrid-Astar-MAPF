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
#include <queue>
#include <unordered_set>
#include <memory>


const bool DEBUG_VIS = false;
const bool show_plan_result = true;
constexpr double TIMETABLE_MARGIN = 0.5;

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

  // 2. Motion Primitive Sampling (Minimize Displacement)
  // Directions: Forward (+1), Backward (-1)
  // Steers: 0 (Straight), +Max (Left), -Max (Right)
  // Distances: Small discrete steps
  
  std::vector<int> prim_dirs = {1, -1};
  std::vector<double> prim_steers = {0.0, maxc * wb, -maxc * wb}; // maxc = 1/Rmin, max_steer ~ atan(wb * maxc) -> actually we use curvature control in ReedsShepp, but here let's approximate
  // Actually max_steer is atan(wheel_base * max_curvature). 
  // Let's rely on geometric update directly using Curvature (k = tan(delta)/L).
  std::vector<double> prim_ks = {0.0, maxc, -maxc};
  
  std::vector<double> prim_dists = {0.5, 1.0, 1.5, 2.0, 3.0}; 

  for (double dist : prim_dists) {
      for (int dir : prim_dirs) {
          for (double k : prim_ks) {
                double d = dir * dist;
                double x_new, y_new, yaw_new;
                
                if (std::abs(k) < 1e-5) {
                    // Straight
                    x_new = current_pose.x + d * std::cos(current_pose.yaw);
                    y_new = current_pose.y + d * std::sin(current_pose.yaw);
                    yaw_new = current_pose.yaw;
                } else {
                    // Arc
                    // R = 1/k. d = R * beta -> beta = d * k
                    double R = 1.0 / k;
                    double beta = d / R; // d and R have same sign logic? ReedsShepp uses d > 0 usually?
                    // Let's stick to standard formula:
                    // theta' = theta + beta
                    // x' = x + R(sin(theta + beta) - sin(theta))
                    // y' = y - R(cos(theta + beta) - cos(theta)) -- Standard math usually: y' = y + R(cos(theta) - cos(theta+beta))? 
                    // Let's use the one from PHAstar::generate_node
                    // x += R * (std::sin(yaw + beta) - std::sin(yaw));
                    // y += R * (std::cos(yaw) - std::cos(yaw + beta));
                    // yaw += beta;
                    
                    x_new = current_pose.x + R * (std::sin(current_pose.yaw + beta) - std::sin(current_pose.yaw));
                    y_new = current_pose.y + R * (std::cos(current_pose.yaw) - std::cos(current_pose.yaw + beta));
                    yaw_new = current_pose.yaw + beta;
                }
                
                // Wrap Yaw
                while(yaw_new > M_PI) yaw_new -= 2*M_PI;
                while(yaw_new < -M_PI) yaw_new += 2*M_PI;
                
                // Create Candidate
                Pose p{x_new, y_new, yaw_new};
                
                // Bounds Check immediately to prune useful candidates
                if (p.x >= params.min_x + 0.2 && p.x <= params.max_x - 0.2 &&
                    p.y >= params.min_y + 0.2 && p.y <= params.max_y - 0.2) {
                     candidates.push_back({p, dist}); // Cost is just the move distance (cheaper than driving to corner)
                }
          }
      }
  }

  // 3. Workspace corners with multiple orientations (out-of-the-way locations)
  double margin = 0.6; // Safe margin from exact bounds
  std::vector<double> corner_yaws = {0.0, M_PI / 2, M_PI, -M_PI / 2};
  
  // Dense Boundary Sampling
  double boundary_step = 1.0; 
  for (double x = params.min_x + margin; x <= params.max_x - margin; x += boundary_step) {
       for (double yaw : corner_yaws) {
            candidates.push_back({{x, params.min_y + margin, yaw}, 0.0});
            candidates.push_back({{x, params.max_y - margin, yaw}, 0.0});
       }
  }
  for (double y = params.min_y + margin; y <= params.max_y - margin; y += boundary_step) {
       for (double yaw : corner_yaws) {
            candidates.push_back({{params.min_x + margin, y, yaw}, 0.0});
            candidates.push_back({{params.max_x - margin, y, yaw}, 0.0});
       }
  }

  // Calculate costs after generating all
  for(auto& cand : candidates) {
       if (cand.estimated_rs_length == 0.0 && (cand.pose.x != robot->initial_pose.x)) { // Don't recalc initial
           cand.estimated_rs_length = compute_rs_length(cand.pose);
       }
  }

  
  // Random samples removed for determinism
  // Fixed candidates only (initial + corners)

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
        // if (ent->type == EntityType::ROBOT) continue; // FIX: Do NOT ignore robots! Pushed object must avoid them too.
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
                             const Trajectory* blocked_traj_hint = nullptr,
                             bool* moved_out = nullptr) {
    
    double ready_time = timetable.get_entity_max_time(blocker);
    // Add small buffer
    ready_time += 0.1; 
    
    Pose start_pose = timetable.get_pose(blocker, ready_time);
    blocker->initial_pose = start_pose; // Sync

    std::cout << "  [Relocate] Searching for clearing motion for " << blocker->name 
              << " from (" << start_pose.x << ", " << start_pose.y << ")..." << std::endl;

    // BFS / Dijkstra Structures
    struct SearchNode {
        double x, y, yaw;
        double cost;
        SearchNode* parent;
        double steer;
        int dir; // 1 or -1
        
        SearchNode(double _x, double _y, double _yaw, double _c, SearchNode* _p, double _s, int _d)
            : x(_x), y(_y), yaw(_yaw), cost(_c), parent(_p), steer(_s), dir(_d) {}
    };
    
    auto cmp = [](const SearchNode* a, const SearchNode* b) { return a->cost > b->cost; };
    std::priority_queue<SearchNode*, std::vector<SearchNode*>, decltype(cmp)> open_set(cmp);
    std::vector<std::unique_ptr<SearchNode>> all_nodes;
    
    SearchNode* start_node = new SearchNode(start_pose.x, start_pose.y, start_pose.yaw, 0.0, nullptr, 0.0, 0);
    all_nodes.emplace_back(start_node);
    open_set.push(start_node);
    
    // Visited set (discretized)
    std::unordered_set<std::string> visited;
    auto get_key = [&](double x, double y, double yaw) {
        int ix = static_cast<int>(x / 0.05);
        int iy = static_cast<int>(y / 0.05);
        int iyaw = static_cast<int>(yaw / 0.1);
        return std::to_string(ix) + "_" + std::to_string(iy) + "_" + std::to_string(iyaw);
    };
    visited.insert(get_key(start_pose.x, start_pose.y, start_pose.yaw));

    // Primitives
    double step_size = 0.25;
    std::vector<int> dirs = {1, -1};
    double max_curv = 1.0 / blocker->min_turning_radius;
    // Straight, Left, Right
    std::vector<double> curvatures = {0.0, max_curv, -max_curv}; 
    
    int max_iter = 2000;
    int iter = 0;
    int future_wait_rejects = 0;
    
    while (!open_set.empty()) {
        SearchNode* current = open_set.top();
        open_set.pop();
        iter++;
        
        if (iter > max_iter) {
            std::cout << "  [Relocate] Max iterations reached." << std::endl;
            break;
        }

        // Check if Current is Safe Parking Spot
        // 1. Check against blocked_traj_hint (The main reason we are moving)
        bool conflict_hint = false;
        Corners curr_corners = get_corners(current->x, current->y, current->yaw, 
                                           blocker->size.front_length, blocker->size.rear_length, blocker->size.width);
        
        // Bounds Check
        if (!is_in_bounds(curr_corners, params.min_x, params.max_x, params.min_y, params.max_y)) {
            continue; // Invalid node, don't expand
        }
                                           
        if (blocked_traj_hint && !blocked_traj_hint->waypoints.empty()) {
          // Check intersection against continuous hinted trajectory in time (dense interpolation).
          double hint_duration = blocked_traj_hint->waypoints.back().time;
          double dt_hint = 0.1;
          for (double t_hint = 0.0; t_hint <= hint_duration + 1e-6; t_hint += dt_hint) {
            auto pose_tuple = interpolate_timed_path(blocked_traj_hint->waypoints, t_hint);
            Pose wp_pose{std::get<0>(pose_tuple), std::get<1>(pose_tuple), std::get<2>(pose_tuple)};

            Corners wp_corners = get_corners(wp_pose.x, wp_pose.y, wp_pose.yaw,
                             blocker->size.front_length, blocker->size.rear_length, blocker->size.width);
            if (rectangles_intersect(curr_corners, wp_corners)) {
              conflict_hint = true;
              break;
            }

            // If hinted trajectory is a transfer, also check the pushed object's footprint.
            if (blocked_traj_hint->is_transfer && blocked_traj_hint->transferred_object) {
              EntityMeta* tr_obj = blocked_traj_hint->transferred_object;
              Pose obj_pose = TimeTable::compute_object_pose(wp_pose, blocker->size, tr_obj->size);
              Corners obj_corners = get_corners(obj_pose.x, obj_pose.y, obj_pose.yaw,
                                tr_obj->size.front_length,
                                tr_obj->size.rear_length,
                                tr_obj->size.width);
              if (rectangles_intersect(curr_corners, obj_corners)) {
                conflict_hint = true;
                break;
              }
            }
          }
        }
        
        // 2. Check Other Static/Dynamic Obstacles at ready_time and while waiting
        bool conflict_env = false;
        if (!conflict_hint) {
             // Validate against timetable at ready_time
             // We reuse check_pose_collision logic or similar
             // But we don't have easy access to check_pose_collision here (it's in PHAStar class).
             // However, we have `timetable`.
             auto poses = timetable.get_poses(ready_time);
             CollisionGeometry my_geom = setup_collision_geometry({current->x, current->y, current->yaw}, blocker->size, params.inflation);
             
             for(auto& [ent, p] : poses) {
                 if (ent == blocker) continue;
                 // Don't check against the robot who owns blocked_traj_hint (we already checked hint)
                 if (blocked_traj_hint && ent == blocked_traj_hint->entity) continue; 
                 
                 // Use standard collision check
                 if (check_entity_collision(my_geom, {current->x, current->y, current->yaw}, ent, p, params).has_collision) {
                     conflict_env = true;
                     break;
                 }
             }

               // Critical: also validate future waiting at this parking pose until timetable horizon.
               // This prevents selecting a pose that is free now but later blocks an existing path.
               if (!conflict_env) {
                 double speed = std::max(1e-6, blocker->speed_transit);
                 double parking_arrival_time = ready_time + (current->cost / speed);
                 double horizon = timetable.get_max_time();
                 double dt_future = 0.1;

                 for (double t_check = parking_arrival_time; t_check <= horizon + 1e-6; t_check += dt_future) {
                   auto future_poses = timetable.get_poses(t_check);
                   for (auto& [ent, p] : future_poses) {
                     if (ent == blocker) continue;
                     // Do NOT skip blocked_traj_hint entity here.
                     // We must ensure parked pose does not block previously-registered paths later.
                     if (check_entity_collision(my_geom, {current->x, current->y, current->yaw}, ent, p, params).has_collision) {
                       conflict_env = true;
                       future_wait_rejects++;
                       if (future_wait_rejects % 50 == 0) {
                         std::cout << "  [Relocate] Future-wait reject x" << future_wait_rejects
                               << " at (" << current->x << ", " << current->y << ")"
                               << " due to " << ent->name << " @t=" << t_check << std::endl;
                       }
                       break;
                     }
                   }
                   if (conflict_env) break;
                 }
               }
        }

        if (!conflict_hint && !conflict_env) {
            // Found a valid spot!
            // Reconstruct Path
            std::vector<Waypoint> path;
            SearchNode* node = current;
            while(node->parent) {
                Waypoint wp;
                wp.x = node->x;
                wp.y = node->y;
                wp.yaw = node->yaw;
                wp.steering_angle = std::atan(node->steer * blocker->wheel_base); // k = tan(delta)/L -> tan(delta) = k*L
                wp.linear_velocity = node->dir * blocker->speed_transit;
                path.push_back(wp);
                node = node->parent;
            }
            // Add start
            Waypoint start_wp(start_pose);
            start_wp.time = 0; 
            path.push_back(start_wp);
            std::reverse(path.begin(), path.end());
            
            // Assign time
            double t = ready_time;
            for(size_t i=0; i<path.size(); ++i) {
                if(i > 0) {
                     double d = std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
                     t += d / blocker->speed_transit;
                }
                path[i].time = t;
            }
            
            // Detect no-op relocation (already at a valid clearing pose)
            double moved_dist = std::hypot(current->x - start_pose.x, current->y - start_pose.y);
            bool moved = (moved_dist > 0.05 || current->cost > 1e-6);
            if (moved_out) {
              *moved_out = moved;
            }

            if (!moved) {
              std::cout << "  [Relocate] SKIP: " << blocker->name
                    << " already clear at current pose. (cost=" << current->cost << ")" << std::endl;
              return true;
            }

            // Register
            Trajectory relo_traj;
            relo_traj.entity = blocker;
            relo_traj.start_time = ready_time;
            relo_traj.waypoints = path;
             for (auto &wp : relo_traj.waypoints)
                wp.time -= ready_time;
            timetable.add_trajectory(relo_traj);
            
            std::cout << "  [Relocate] SUCCESS: Clearing found at (" << current->x << ", " << current->y << ") with cost " << current->cost << std::endl;
            return true;
        }
        
        // Expand
        for (int dir : dirs) {
            for (double k : curvatures) {
                 double d = dir * step_size;
                 double x_new, y_new, yaw_new;
                 
                  if (std::abs(k) < 1e-5) {
                    x_new = current->x + d * std::cos(current->yaw);
                    y_new = current->y + d * std::sin(current->yaw);
                    yaw_new = current->yaw;
                } else {
                    double R = 1.0 / k;
                    double beta = d / R; // d = R*beta
                    x_new = current->x + R * (std::sin(current->yaw + beta) - std::sin(current->yaw));
                    y_new = current->y + R * (std::cos(current->yaw) - std::cos(current->yaw + beta));
                    yaw_new = current->yaw + beta;
                }
                 while(yaw_new > M_PI) yaw_new -= 2*M_PI;
                 while(yaw_new < -M_PI) yaw_new += 2*M_PI;
                 
                 std::string key = get_key(x_new, y_new, yaw_new);
                 if (visited.count(key)) continue;
                 
                 // Penalty for switching direction or steer?
                 double new_cost = current->cost + step_size;
                 
                 visited.insert(key);
                 SearchNode* next_node = new SearchNode(x_new, y_new, yaw_new, new_cost, current, k, dir);
                 all_nodes.emplace_back(next_node);
                 open_set.push(next_node);
            }
        }
    }

    std::cerr << "  [Relocate] FAILED: Could not find clearing motion within iteration limit."
          << " future_wait_rejects=" << future_wait_rejects << std::endl;
    if (moved_out) {
        *moved_out = false;
    }
    return false;
}

  struct PostGoalCollision {
    bool has_collision = false;
    EntityMeta* collider = nullptr;
    std::string collider_name;
    double collision_time = 0.0;
  };

  PostGoalCollision detect_stationary_post_goal_collision(
      RobotMeta* robot,
      const Pose& stationary_pose,
      double start_time,
      TimeTable& timetable,
      const Params& params) {
    PostGoalCollision out;
    double horizon = timetable.get_max_time();
    if (horizon <= start_time + 1e-6) {
      return out;
    }

    CollisionGeometry robot_geom = setup_collision_geometry(stationary_pose, robot->size, 1.0);
    double dt = 0.1;

    for (double t = start_time; t <= horizon + 1e-6; t += dt) {
      auto others = timetable.get_poses(t);
      for (const auto& [ent, pose] : others) {
        if (!ent || ent == robot) continue;
        auto collision = check_entity_collision(robot_geom, stationary_pose, ent, pose, params);
        if (collision.has_collision) {
          out.has_collision = true;
          out.collider = ent;
          out.collider_name = ent->name;
          out.collision_time = t;
          return out;
        }
      }
    }

    return out;
  }

  Trajectory build_future_hint_trajectory(
      EntityMeta* moving_entity,
      double start_time,
      double end_time,
      TimeTable& timetable) {
    Trajectory hint;
    hint.entity = moving_entity;
    hint.start_time = start_time;
    hint.is_transfer = false;

    if (!moving_entity || end_time <= start_time + 1e-6) {
      return hint;
    }

    double dt = 0.1;
    for (double t = start_time; t <= end_time + 1e-6; t += dt) {
      Pose p = timetable.get_pose(moving_entity, t);
      Waypoint wp;
      wp.x = p.x;
      wp.y = p.y;
      wp.yaw = p.yaw;
      wp.time = t - start_time;
      hint.waypoints.push_back(wp);
    }

    if (hint.waypoints.size() == 1) {
      Waypoint wp = hint.waypoints.front();
      wp.time = 0.1;
      hint.waypoints.push_back(wp);
    }

    return hint;
  }

  bool resolve_post_goal_conflict_after_add(
      RobotMeta* robot,
      double traj_end_time,
      TimeTable& timetable,
      const Params& params,
      const std::unordered_map<std::string, EntityMeta*>& entities,
      bool has_following_segment) {
    Pose stationary_pose = timetable.get_pose(robot, traj_end_time);
    PostGoalCollision post_col = detect_stationary_post_goal_collision(
        robot, stationary_pose, traj_end_time, timetable, params);

    if (!post_col.has_collision) {
      return true;
    }

    std::cout << "  [PostGoal] " << robot->name
              << " would collide after path end at t=" << post_col.collision_time
              << " with " << post_col.collider_name << std::endl;

    if (has_following_segment) {
      std::cout << "  [PostGoal] Deferring conflict handling because next segment exists." << std::endl;
      return true;
    }

    std::cout << "  [PostGoal] No following segment. Relocating " << robot->name
              << " to safe parking." << std::endl;

    Trajectory hint = build_future_hint_trajectory(
        post_col.collider, post_col.collision_time, timetable.get_max_time(), timetable);
    bool moved = false;
    const Trajectory* hint_ptr = hint.waypoints.empty() ? nullptr : &hint;

    if (!relocate_blocking_robot(robot, timetable, params, entities, hint_ptr, &moved)) {
      std::cerr << "  [PostGoal] Failed to relocate " << robot->name
                << " after post-goal conflict." << std::endl;
      return false;
    }

    double new_end_time = timetable.get_entity_max_time(robot, 0.0);
    Pose new_stationary_pose = timetable.get_pose(robot, new_end_time);
    PostGoalCollision recheck = detect_stationary_post_goal_collision(
        robot, new_stationary_pose, new_end_time, timetable, params);
    if (recheck.has_collision) {
      std::cerr << "  [PostGoal] Conflict still remains after relocation: "
                << recheck.collider_name << " at t=" << recheck.collision_time << std::endl;
      return false;
    }

    return true;
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

  double transit_end_time = transit_traj.start_time + transit_traj.waypoints.back().time;
  if (!resolve_post_goal_conflict_after_add(robot, transit_end_time, timetable, params, entities, true)) {
    std::cerr << "  [Transit] Post-goal conflict unresolved." << std::endl;
    return false;
  }

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

// ==========================================
// 3. MAIN TASK PIPELINE (Refactored for ALNS)
// ==========================================

// Global Timeline for Coin Attribution
// Map: Robot -> Vector of <Interval, Task*>
std::map<RobotMeta*, std::vector<std::pair<std::pair<double, double>, Task*>>> solution_timeline;

void clear_timeline() {
    solution_timeline.clear();
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

void register_task_schedule(RobotMeta* robot, Task* task, double start_t, double end_t) {
    solution_timeline[robot].push_back({{start_t, end_t}, task});
}

Task* get_task_at_time(RobotMeta* robot, double time) {
    if (solution_timeline.find(robot) == solution_timeline.end()) return nullptr;
    for (const auto& entry : solution_timeline[robot]) {
        if (time >= entry.first.first && time <= entry.first.second) {
            return entry.second;
        }
    }
    return nullptr;
}

void visualize_segment_failure_instance(
    TimeTable& timetable,
    const std::unordered_map<std::string, EntityMeta *>& entities,
    const Params& params,
    const Trajectory& traj,
    double attempted_start_time,
    const CollisionInfo& last_motion_collision,
    const std::string& fail_tag) {
  double query_t = attempted_start_time;
  if (!last_motion_collision.is_valid && last_motion_collision.time > 0.0) {
    query_t = last_motion_collision.time;
  }

  Pose start_pose = timetable.get_pose(traj.entity, attempted_start_time);
  Pose goal_pose = start_pose;
  if (!traj.waypoints.empty()) {
    const Waypoint& goal_wp = traj.waypoints.back();
    goal_pose = {goal_wp.x, goal_wp.y, goal_wp.yaw};
  }

  std::cout << "  [DebugViz] Segment failure " << fail_tag
            << " at t=" << query_t << " (start=" << attempted_start_time << ")" << std::endl;
  visualize_current_state(timetable, entities, params, query_t, start_pose, goal_pose);
}

// Helper function to handle the scheduling of a single path segment
// Helper function to handle the scheduling of a single path segment
bool schedule_path_segment(const EdgePath &edge_path, EntityMeta *obj_meta,
                           RobotMeta *robot, TimeTable &timetable,
                           const Params &params,
                           const std::unordered_map<std::string, EntityMeta *> &entities,
                           Task* current_task,
                           bool has_following_segment = false) { // Added current_task for coin attribution
                           
  // 1. Get current available time from the timetable
  double current_avail_time = timetable.get_entity_max_time(robot);

  // 2. Convert EdgePath to Trajectory
  TrajectoryPtr traj = ReloPushPath2TrajPtr(edge_path, robot, obj_meta, current_avail_time);

  // FIX: Ensure entity and relative timestamps are set for collision checking!
  traj->entity = robot;
  traj->CalcualteTimeStamps(robot);

  int force_clear_retry_count = 0;

schedule_retry:

  // 3. Find safe start time
  // We need to intercept find_safe_start_time to record coins, but function signature is fixed.
  // Instead, let's copy the logic of find_safe_start_time here or wrap it. 
  // For cleaner code, I will duplicate the relevant parts of find_safe_start_time logic 
  // OR modify find_safe_start_time to accept a callback/Task pointer. 
  // Let's modify find_safe_start_time to be unaware of Task, but return info we can use? 
  // No, `find_safe_start_time` is defined above in this file. Let's modify it to accept Task* optional.
  
  // -- Wait, I can't easily modify the function signature in previous block without another tool call.
  // actually I can just use the existing one, but we need to track *why* it delayed.
  // The existing `find_safe_start_time` prints to stdout. 
  // Let's redefine `find_safe_start_time_instrumented` locally or inline it.

  // Inline "Find Safe Start Time" with Coin Logic
  double check_time = current_avail_time;
  double step = 0.5;
  double wait_horizon = std::max(100.0, (timetable.get_max_time() - current_avail_time) + 120.0);
  int max_retries = std::max(200, static_cast<int>(std::ceil(wait_horizon / step))); 

  std::string last_relocated_robot_name = "";
  double last_relocation_time = -100.0;
  std::unordered_map<std::string, double> no_progress_blockers;
  int post_goal_reject_count = 0;
  std::string last_post_goal_entity = "";
  double last_post_goal_time = -1.0;
  int relocation_attempts = 0;
  int relocation_failures = 0;
  CollisionInfo last_motion_collision{true, "", "", 0.0};
  
  // Track wait duration for this segment
  double wait_duration = 0.0;

  bool success = false;
  for (int i = 0; i < max_retries; ++i) {
    CollisionInfo col_info = check_collision_trajectory_detailed(*traj, check_time, timetable, params, false);
    if (!col_info.is_valid) {
      last_motion_collision = col_info;
    }
    
    if (col_info.is_valid) {
      // Deadlock guard: if next segment exists, avoid entering a goal-wait state
      // that will collide with already-registered (older) trajectories.
      if (has_following_segment && !traj->waypoints.empty()) {
        double end_time = check_time + traj->waypoints.back().time;
        Pose end_pose = {traj->waypoints.back().x, traj->waypoints.back().y, traj->waypoints.back().yaw};
        PostGoalCollision post_col = detect_stationary_post_goal_collision(
          robot, end_pose, end_time, timetable, params);
        if (post_col.has_collision) {
          post_goal_reject_count++;
          last_post_goal_entity = post_col.collider_name;
          last_post_goal_time = post_col.collision_time;
          if (post_goal_reject_count % 20 == 0) {
            std::cout << "  [PostGoal-Delay] Task " << current_task->id
                      << " " << robot->name << " candidate delayed " << post_goal_reject_count
                      << " times due to post-goal conflict with " << post_col.collider_name
                      << " at t=" << post_col.collision_time << std::endl;
          }
          check_time += step;
          wait_duration += step;
          continue;
        }
      }

        // Success
        success = true;
        break;
    }

    // Handle Collision & Attributes Coins
    EntityMeta* collider = nullptr;
    if (entities.count(col_info.entity_name)) {
        collider = entities.at(col_info.entity_name);
    }

    if (collider && collider->type == EntityType::ROBOT) {
        RobotMeta* blocker = dynamic_cast<RobotMeta*>(collider);
        
        // Attibute Blocker Coin
        Task* blocking_task = get_task_at_time(blocker, col_info.time);
        if (blocking_task && blocking_task != current_task) {
            blocking_task->blocker_coins += 1.0; // Basic weight
            // std::cout << "    [Coin] Task " << blocking_task->id << " gets Blocker coin from Task " << current_task->id << std::endl;
        }

        double blocker_free_time = timetable.get_entity_max_time(blocker);
        if (col_info.time > blocker_free_time) {
            // Blocker is stationary. Relocate.
           bool can_retry_no_progress = (!no_progress_blockers.count(blocker->name) ||
             (check_time - no_progress_blockers[blocker->name] > 20.0));
           if (can_retry_no_progress &&
             (blocker->name != last_relocated_robot_name || (check_time - last_relocation_time > 5.0))) {
            relocation_attempts++;
            bool moved = false;
            if (relocate_blocking_robot(blocker, timetable, params, entities, traj.get(), &moved)) {
                    last_relocated_robot_name = blocker->name;
                    last_relocation_time = check_time;
              if (moved) {
                check_time -= step; // Retry same time only when blocker actually moved
              } else {
                no_progress_blockers[blocker->name] = check_time;
              }
                } else {
                    // Relocation Failed. Abort.
                  relocation_failures++;
                    std::cerr << "    [Abort] Unmovable blocker " << blocker->name << " on path." << std::endl;
                    success = false;
                    break;
                }
            }
        }
    }
    
    check_time += step;
    wait_duration += step;
  }
  
  if (wait_duration > 0) {
      current_task->waiter_coins += wait_duration;
  }
  
  double safe_start_time = success ? check_time : -1.0;

  if (safe_start_time < 0) {
       std::cerr << " [Error] Segment failed."
                 << " task=" << current_task->id
                 << " robot=" << robot->name
                 << " start_ready=" << current_avail_time
                 << " retries=" << max_retries
                 << " waited=" << wait_duration
                 << " post_goal_rejects=" << post_goal_reject_count;
       if (!last_post_goal_entity.empty()) {
         std::cerr << " last_post_goal_blocker=" << last_post_goal_entity
                   << "@t=" << last_post_goal_time;
       }
       if (!last_motion_collision.is_valid) {
         std::cerr << " last_motion_collision=(" << last_motion_collision.reason
                   << ", entity=" << last_motion_collision.entity_name
                   << ", t=" << last_motion_collision.time << ")";
       }
       std::cerr << " relocate_attempts=" << relocation_attempts
                 << " relocate_failures=" << relocation_failures
                 << std::endl;

       // Force-clear fallback: if blocked by a robot, relocate blocker and retry once.
       if (force_clear_retry_count == 0 && !last_motion_collision.is_valid &&
           entities.count(last_motion_collision.entity_name)) {
         EntityMeta* maybe_blocker = entities.at(last_motion_collision.entity_name);
         if (maybe_blocker && maybe_blocker->type == EntityType::ROBOT) {
           RobotMeta* blocker = dynamic_cast<RobotMeta*>(maybe_blocker);
           bool moved = false;
           if (relocate_blocking_robot(blocker, timetable, params, entities, traj.get(), &moved)) {
             force_clear_retry_count++;
             std::cout << "  [ForceClear] schedule_path_segment retry after relocating blocker "
                       << blocker->name << " moved=" << (moved ? "Y" : "N") << std::endl;
             goto schedule_retry;
           }
         }
       }

         visualize_segment_failure_instance(
           timetable, entities, params, *traj, check_time, last_motion_collision,
           "schedule_path_segment task=" + std::to_string(current_task->id));

       return false; 
  }

  // 4. Update timestamps and add to timetable
  traj->start_time = safe_start_time;
  traj->CalcualteTimeStamps(robot);
  timetable.add_trajectory(*traj);

  double segment_end_time = traj->start_time + traj->waypoints.back().time;
  if (!resolve_post_goal_conflict_after_add(robot, segment_end_time, timetable, params, entities, has_following_segment)) {
      std::cerr << " [Error] Post-goal conflict unresolved after segment." << std::endl;
      return false;
  }

  return true;
}

bool process_task_execution_instrumented(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params) {
    
  double start_record_time = timetable.get_entity_max_time(robot);
  
  // 1. Plan Transit to Task Start
  double robot_avail_time = timetable.get_entity_max_time(robot);
  
  // Use existing transit planner but monitor result?
  // `plan_initial_transit` internally handles relocation. 
  // It's hard to inject coin logic there without modifying it.
  // For now, let's rely on standard transit. If it fails or waits heavily...
  // Actually, plan_initial_transit uses A*. The "wait" is implicit in the path cost/travel time?
  // PHAstar output `path_res.waypoints` has times. 
  // Extra time taken vs ideal time could be "Waiter" coin.
  
  double ideal_transit_time = std::hypot(task.TaskStartPoseRobot.x - robot->initial_pose.x, 
                                         task.TaskStartPoseRobot.y - robot->initial_pose.y) / robot->speed_transit;
                                         
  if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time,
                            timetable, entities, params)) {
    // If transit fails completely...
    std::cerr << "Aborting task due to transit failure." << std::endl;
    return false;
  }
  
  // Calculate Transit Wait
  double actual_transit_end = timetable.get_entity_max_time(robot);
  double actual_transit_duration = actual_transit_end - robot_avail_time;
  if (actual_transit_duration > ideal_transit_time + 5.0) { // Tolerance
      task.waiter_coins += (actual_transit_duration - ideal_transit_time);
  }

  // 2. ObsRelo (if exists)
  if (task.vertexChain.size() > 2) {
    for (size_t obs_ind = 1; obs_ind < task.vertexChain.size() - 1; obs_ind++) {
      std::string obs_name = task.vertexChain[obs_ind].name;
      auto obs_meta = entities.at(obs_name);
      size_t push_path_idx = 0;
      size_t post_path_idx = 1;

      if (task.obsReloPaths->size() > post_path_idx) {
      if (task.obsReloPaths->size() > post_path_idx) {
        bool has_more_obs_after_this = (obs_ind + 1 < task.vertexChain.size() - 1);
        bool has_edge_paths_after_obs = !task.EdgePaths.empty();
        bool push_has_following = true; // post path follows
        bool post_has_following = has_more_obs_after_this || has_edge_paths_after_obs;

        if (!schedule_path_segment(task.obsReloPaths->at(push_path_idx), obs_meta,
                              robot, timetable, params, entities, &task, push_has_following)) return false;
        if (!schedule_path_segment(task.obsReloPaths->at(post_path_idx), obs_meta,
                              robot, timetable, params, entities, &task, post_has_following)) return false;
      }
      }
    }
  }

  // 3. Execute Edge Paths
  for (size_t path_i = 0; path_i < task.EdgePaths.size(); ++path_i) {
    auto &path_ptr = task.EdgePaths[path_i];
    bool has_following_segment = (path_i + 1 < task.EdgePaths.size());
    double segment_ready_time = timetable.get_entity_max_time(robot);
    path_ptr->entity = robot;
    path_ptr->CalcualteTimeStamps(robot);

    // Reuse the instrumented logic? No, I implemented schedule_path_segment above to be instrumented.
    // Wait, the original code had inline find_safe_start_time call.
    // I should check validity.
    
    // We already defined schedule_path_segment above to replace the inline code.
    // But `Task::EdgePaths` contains pointers that need to be updated.
    // `schedule_path_segment` takes EdgePath (const ref) and converts to new Trajectory.
    // The original code updated `path_ptr` directly. 
    // Let's manually do it here to ensure `path_ptr` in Task is updated (for visualization/record).
    
    TrajectoryPtr traj = path_ptr; // Shared ptr
    // traj->entity/timestamps already set

    int force_clear_retry_count = 0;

  edge_retry:
    
    // Inline Safe Start Instrumented
    double check_time = segment_ready_time;
    double step = 0.5;
    double wait_horizon = std::max(100.0, (timetable.get_max_time() - segment_ready_time) + 120.0);
    int max_retries = std::max(200, static_cast<int>(std::ceil(wait_horizon / step)));
    std::string last_relocated_robot = "";
    double last_relocation_time = -100.0;
    std::unordered_map<std::string, double> no_progress_blockers;
    int post_goal_reject_count = 0;
    std::string last_post_goal_entity = "";
    double last_post_goal_time = -1.0;
    int relocation_attempts = 0;
    int relocation_failures = 0;
    CollisionInfo last_motion_collision{true, "", "", 0.0};
    double wait_val = 0.0;
    bool seg_success = false;
    
    for (int i = 0; i < max_retries; ++i) {
        CollisionInfo col_info = check_collision_trajectory_detailed(*traj, check_time, timetable, params, false);
        if (!col_info.is_valid) {
          last_motion_collision = col_info;
        }
     
        if (col_info.is_valid) {
        // Deadlock guard: if another segment follows, reject start times that
        // cause post-goal waiting collisions with older timetable trajectories.
        if (has_following_segment && !traj->waypoints.empty()) {
          double end_time = check_time + traj->waypoints.back().time;
          Pose end_pose = {traj->waypoints.back().x, traj->waypoints.back().y, traj->waypoints.back().yaw};
          PostGoalCollision post_col = detect_stationary_post_goal_collision(
            robot, end_pose, end_time, timetable, params);
          if (post_col.has_collision) {
            post_goal_reject_count++;
            last_post_goal_entity = post_col.collider_name;
            last_post_goal_time = post_col.collision_time;
            if (post_goal_reject_count % 20 == 0) {
              std::cout << "  [PostGoal-Delay] Task " << task.id
                        << " EdgeIdx=" << path_i
                        << " " << robot->name << " delayed " << post_goal_reject_count
                        << " times due to " << post_col.collider_name
                        << "@t=" << post_col.collision_time << std::endl;
            }
            check_time += step;
            wait_val += step;
            continue;
          }
        }

            seg_success = true;
            break;
        }
        
        // Coins
        if (entities.count(col_info.entity_name)) {
             EntityMeta* ent = entities.at(col_info.entity_name);
             if (ent->type == EntityType::ROBOT) {
                 RobotMeta* blocker = dynamic_cast<RobotMeta*>(ent);
                 Task* blk_task = get_task_at_time(blocker, col_info.time);
                 if (blk_task && blk_task != &task) {
                     blk_task->blocker_coins += 1.0;
                 }
                 
                 // Relocate logic
                 if (col_info.time > timetable.get_entity_max_time(blocker)) {
                    bool can_retry_no_progress = (!no_progress_blockers.count(blocker->name) ||
                      (check_time - no_progress_blockers[blocker->name] > 20.0));
                    if (can_retry_no_progress &&
                      (blocker->name != last_relocated_robot || (check_time - last_relocation_time > 5.0))) {
                    relocation_attempts++;
                    bool moved = false;
                    if (relocate_blocking_robot(blocker, timetable, params, entities, traj.get(), &moved)) {
                            last_relocated_robot = blocker->name;
                            last_relocation_time = check_time;
                      if (moved) {
                        check_time -= step;
                      } else {
                        no_progress_blockers[blocker->name] = check_time;
                      }
                        } else {
                              relocation_failures++;
                             std::cerr << "    [Abort] Unmovable blocker " << blocker->name << " on path (EdgePath)." << std::endl;
                             seg_success = false;
                             break;
                        }
                    }
                 }
             }
        }
        
        check_time += step;
        wait_val += step;
    }
    
    if (!seg_success) {
        std::cerr << "[Error] Segment failed."
                  << " task=" << task.id
                  << " edge_idx=" << path_i
                  << " robot=" << robot->name
                  << " start_ready=" << segment_ready_time
                  << " retries=" << max_retries
                  << " waited=" << wait_val
                  << " post_goal_rejects=" << post_goal_reject_count;
        if (!last_post_goal_entity.empty()) {
          std::cerr << " last_post_goal_blocker=" << last_post_goal_entity
                    << "@t=" << last_post_goal_time;
        }
        if (!last_motion_collision.is_valid) {
          std::cerr << " last_motion_collision=(" << last_motion_collision.reason
                    << ", entity=" << last_motion_collision.entity_name
                    << ", t=" << last_motion_collision.time << ")";
        }
        std::cerr << " relocate_attempts=" << relocation_attempts
                  << " relocate_failures=" << relocation_failures
                  << std::endl;

        // Force-clear fallback: if blocked by a robot, relocate blocker and retry once.
        if (force_clear_retry_count == 0 && !last_motion_collision.is_valid &&
            entities.count(last_motion_collision.entity_name)) {
          EntityMeta* maybe_blocker = entities.at(last_motion_collision.entity_name);
          if (maybe_blocker && maybe_blocker->type == EntityType::ROBOT) {
            RobotMeta* blocker = dynamic_cast<RobotMeta*>(maybe_blocker);
            bool moved = false;
            if (relocate_blocking_robot(blocker, timetable, params, entities, traj.get(), &moved)) {
              force_clear_retry_count++;
              std::cout << "  [ForceClear] edge_loop task=" << task.id
                        << " edge_idx=" << path_i
                        << " retry after relocating blocker " << blocker->name
                        << " moved=" << (moved ? "Y" : "N") << std::endl;
              goto edge_retry;
            }
          }
        }

        visualize_segment_failure_instance(
          timetable, entities, params, *traj, check_time, last_motion_collision,
          "edge_loop task=" + std::to_string(task.id) + " edge_idx=" + std::to_string(path_i));

        return false;
    }
    
    if (wait_val > 0) task.waiter_coins += wait_val;
    
    traj->start_time = check_time;
    traj->CalcualteTimeStamps(robot);
    timetable.add_trajectory(*traj);
    
    if (traj->is_transfer) {
      append_retraction(robot, *traj, timetable);
    }

    double check_from_time = timetable.get_entity_max_time(robot, 0.0);
    if (!resolve_post_goal_conflict_after_add(robot, check_from_time, timetable, params, entities, has_following_segment)) {
      std::cerr << " [Error] Post-goal conflict unresolved in edge execution." << std::endl;
      return false;
    }
  }

  // Final post-task stationary check: no following segment remains.
  double final_check_time = timetable.get_entity_max_time(robot, 0.0);
  if (!resolve_post_goal_conflict_after_add(robot, final_check_time, timetable, params, entities, false)) {
    std::cerr << " [Error] Final post-goal conflict unresolved for task " << task.id << std::endl;
    return false;
  }
  
  double end_record_time = timetable.get_entity_max_time(robot);
  register_task_schedule(robot, &task, start_record_time, end_record_time);

  return true;
}

void solve_allocation(std::vector<Task>& tasks, std::vector<RobotMeta*>& robots, 
                      TimeTable& timetable, const Params& params,
                      const std::unordered_map<std::string, EntityMeta *>& entities,
                      const std::map<int, std::string>& tabu_list = {},
                      const std::map<int, std::string>& avoid_same_robot_list = {}) {
    
    // Reset Robots
    for(auto* r : robots) {
        r->initial_pose = timetable.get_pose(r, 0.0); // Reset to t=0 pose? 
        // Actually TimeTable is passed in. If LNS, we might rebuild TimeTable from scratch.
        // The user says: "The robots start from the same initial poses as they did in the greedy allocation."
        // So we should assume `timetable` is FRESH (empty except initials) when calling this.
    }
    
    clear_timeline();

    // Atomic task attempt: if planning fails, rollback any partial side-effects.
    auto try_process_task_atomic = [&](RobotMeta* candidate, Task& task) {
      size_t label_count_before = timetable.get_trajectory_labels().size();
      double makespan_before = timetable.get_max_time();

      TimeTable timetable_backup = timetable;
      double blocker_backup = task.blocker_coins;
      double waiter_backup = task.waiter_coins;
      double deadlock_backup = task.deadlock_coins;

      std::map<RobotMeta*, Pose> robot_pose_backup;
      for (auto* r : robots) {
        robot_pose_backup[r] = r->initial_pose;
      }

      bool ok = process_task_execution_instrumented(candidate, task, timetable, entities, params);
      if (!ok) {
        size_t label_count_after = timetable.get_trajectory_labels().size();
        if (label_count_after > label_count_before) {
          std::cout << "    [Rollback] Task " << task.id
                    << " with " << candidate->name
                    << " discarded tentative paths ["
                    << (label_count_before + 1) << ".." << label_count_after << "]"
                    << " tentative_makespan=" << timetable.get_max_time()
                    << " final_makespan=" << makespan_before
                    << std::endl;
        }
        timetable = timetable_backup;
        task.blocker_coins = blocker_backup;
        task.waiter_coins = waiter_backup;
        task.deadlock_coins = deadlock_backup;
        for (const auto& [r, pose] : robot_pose_backup) {
          r->initial_pose = pose;
        }
      } else {
        size_t label_count_after = timetable.get_trajectory_labels().size();
        if (label_count_after > label_count_before) {
          std::cout << "    [Commit] Task " << task.id
                    << " with " << candidate->name
                    << " committed paths ["
                    << (label_count_before + 1) << ".." << label_count_after << "]"
                    << " makespan=" << timetable.get_max_time()
                    << std::endl;
        }
      }
      return ok;
    };
    
    int task_idx = 0;
    for(auto& task : tasks) {
        task_idx++;
        // If already assigned (kept from previous generation), just plan it.
        // If not assigned, try to assign.
        
        if (task.assignedRobot) {
             // Pre-assigned (kept)
             // We still need to re-plan the PATH because other robots might have changed schedules.
             // "From here, we need to do the necessary path planning from scratch"
           if (!try_process_task_atomic(task.assignedRobot, task)) {
                 // Feasibility failed?
                 std::cerr << " [Warn] Task " << task.id << " failed with pre-assigned " << task.assignedRobot->name << ". Re-allocating..." << std::endl;
                 task.assignedRobot = nullptr; // Fall through to re-allocation
             }
        }
        
        if (!task.assignedRobot) {
             // Greedy Allocation
             auto candidates = get_sorted_candidate_robots(robots, timetable);
             
           // Build avoid list (soft):
           // 1) Iteration-level tabu (robot before destroy)
           // 2) Initial greedy owner (persistent across LNS iterations)
           std::unordered_set<std::string> avoided_robot_names;
           if (tabu_list.count(task.id)) {
             avoided_robot_names.insert(tabu_list.at(task.id));
           }
           if (avoid_same_robot_list.count(task.id)) {
             avoided_robot_names.insert(avoid_same_robot_list.at(task.id));
           }

             bool assigned = false;

           // Pass 1: Try non-avoided robots first.
           for(auto& [cand, t] : candidates) {
             if (avoided_robot_names.count(cand->name)) continue;
             if (try_process_task_atomic(cand, task)) {
               task.assignedRobot = cand;
               assigned = true;
               std::cout << "    [Alloc] Task " << task.id << " assigned to " << cand->name << " (non-avoided)" << std::endl;
               break;
             }
           }

           // Pass 2: Feasibility fallback (allow avoided robots).
           if (!assigned) {
             for(auto& [cand, t] : candidates) {
               if (try_process_task_atomic(cand, task)) {
                 task.assignedRobot = cand;
                 assigned = true;
                 std::cout << "    [Alloc] Task " << task.id << " assigned to " << cand->name;
                 if (avoided_robot_names.count(cand->name)) {
                   std::cout << " (fallback on avoided robot)";
                 }
                 std::cout << std::endl;
                 break;
               }
             }
           }

             if (!assigned) {
                 std::cout << " [Fail] Task " << task.id << " could not be assigned (Deadlock)." << std::endl;
                 task.deadlock_coins += 10.0;
                 // Assign deadlock to previous tasks?
                 // "Give deadlock coin to the prior task by another robot" -- hard to track "cause".
                 // For now, just mark this task.
             }
        }
    }
}

void destroy_tasks(std::vector<Task>& tasks, int num_to_destroy) {
    std::vector<std::pair<double, int>> weights;
    for(int i=0; i<tasks.size(); ++i) {
        double w = tasks[i].blocker_coins * 1.0 + 
                   tasks[i].waiter_coins * 1.0 + 
                   tasks[i].deadlock_coins * 5.0 + 
                   1.0; // Base weight
        weights.push_back({w, i});
    }
    
    // Random selection based on weights
    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::cout << "  [Destroy] Selecting " << num_to_destroy << " tasks to remove..." << std::endl;
    for(int k=0; k<num_to_destroy; ++k) {
        if (weights.empty()) break;
        
        std::discrete_distribution<> d(weights.size(), 0.0, 1.0, 
            [&](double i) { return weights[static_cast<size_t>(i)].first; }); 
            // Standard discrete_distribution takes iterators or init list.
            // Let's just use simple roulette wheel or sort.
        
        // Simple roulette
        double total_w = 0;
        for(auto& p : weights) total_w += p.first;
        std::uniform_real_distribution<> dist(0, total_w);
        double r = dist(gen);
        
        double acc = 0;
        int selected_idx = -1;
        int vec_idx = -1;
        for(size_t i=0; i<weights.size(); ++i) {
            acc += weights[i].first;
            if (acc >= r) {
                selected_idx = weights[i].second;
                vec_idx = i;
                break;
            }
        }
        if (selected_idx != -1) {
            tasks[selected_idx].assignedRobot = nullptr;
            std::cout << "    -> Removed Task " << tasks[selected_idx].id << " (Coins: B=" 
                      << tasks[selected_idx].blocker_coins << ", W=" << tasks[selected_idx].waiter_coins << ")" << std::endl;
            
            // Remove from weights
            weights.erase(weights.begin() + vec_idx);
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
  
  // Identify Robots
  std::vector<RobotMeta *> all_robots;
  std::map<RobotMeta*, Pose> original_initial_poses; // Backup for reset
  for (const auto &[name, ent] : entities) {
    if (ent->type == EntityType::ROBOT) {
      RobotMeta* r = dynamic_cast<RobotMeta *>(ent);
      all_robots.push_back(r);
      original_initial_poses[r] = r->initial_pose;
    }
  }

  // --- 3. Initialize Tasks ---
  std::vector<Task> tasks;
  int oid = 0;
  for (const auto &fa : loadedSequence) {
    tasks.emplace_back(fa, entities);
    tasks.back().id = oid++;
  }
  std::cout << "[System] Initialized " << tasks.size() << " tasks." << std::endl;

  // Helper to reset robots
  auto reset_robots = [&]() {
      std::cout << "  [Reset] Restoring initial poses:" << std::endl;
      for(auto& [r, pose] : original_initial_poses) {
          r->initial_pose = pose;
          std::cout << "    - " << r->name << ": (" << pose.x << ", " << pose.y << ")" << std::endl;
      }
  };
  
  // Helper to verify collisions
  auto verify_collisions = [&](const TimeTable& tt, double duration) {
       std::cout << "  [Verify] Checking for collisions in final timeline (0 to " << duration << "s)..." << std::endl;
       double dt = 0.2;
       int collision_count = 0;
       for(double t=0; t<=duration; t+=dt) {
           auto poses = tt.get_poses(t);
           std::vector<std::string> names;
           for(auto& [ent, p] : poses) names.push_back(ent->name);
           
           for(size_t i=0; i<names.size(); ++i) {
               for(size_t j=i+1; j<names.size(); ++j) {
                   auto ent1 = entities.at(names[i]);
                   auto ent2 = entities.at(names[j]);
                   Pose p1 = poses[ent1];
                   Pose p2 = poses[ent2];
                   if(ent1->type == EntityType::ROBOT && ent2->type == EntityType::ROBOT) {
                       // Robot-Robot
                       CollisionGeometry g1 = setup_collision_geometry(p1, ent1->size, 0.0); // No inflation for check
                       auto col = check_entity_collision(g1, p1, ent2, p2, params);
                       if(col.has_collision) {
                           std::cout << "    [CRITICAL] Collision at t=" << t << "s between " << ent1->name << " and " << ent2->name << std::endl;
                           collision_count++;
                       }
                   }
                   // Can add Object checks too
               }
           }
           if (collision_count > 5) break; // Limit output
       }
           if(collision_count == 0) std::cout << "  [Verify] No collisions detected." << std::endl;
           return collision_count > 0;
  };

  // --- 4. Initial Greedy Allocation ---
  std::cout << "\n\n=== Initial Greedy Allocation ===" << std::endl;
  reset_robots(); // Ensure clean state
  TimeTable initial_timetable(0.5);
  initial_timetable.add_initial(entities);
  
  solve_allocation(tasks, all_robots, initial_timetable, params, entities);
  
  double initial_makespan = initial_timetable.get_max_time();
  std::cout << "[Result] Initial Solution Makespan: " << initial_makespan << "s" << std::endl;
  bool initial_has_collision = verify_collisions(initial_timetable, initial_makespan);
  if (initial_has_collision) {
      initial_timetable.print_grouped_entries_by_trajectory();
  }

    // Keep initial greedy owner per task, to avoid reusing the same robot in repair if possible.
    std::map<int, std::string> initial_owner_by_task;
    for (const auto& task : tasks) {
      if (task.assignedRobot) {
        initial_owner_by_task[task.id] = task.assignedRobot->name;
      }
    }
  
  // Backup best solution
  double best_makespan = initial_makespan;
  TimeTable best_timetable = initial_timetable; // Store best found so far
  
  std::cout << "[System] Visualizing Initial Solution..." << std::endl;
  if(show_plan_result)
    show_results(argc, argv, initial_timetable, entities, params);

  // --- 5. ALNS Loop ---
  int max_iterations = 5;
  int destroy_count = 3;

  struct IterationLog {
      int iter;
      double makespan;
      std::vector<int> destroyed_tasks;
      std::string heuristics_info;
      double improvement;
      std::vector<std::string> allocation_changes;
  };
  std::vector<IterationLog> history;
  
  // Record initial
  history.push_back({0, initial_makespan, {}, "Initial Greedy", 0.0, {}});

  for (int iter = 1; iter <= max_iterations; ++iter) {
      std::cout << "\n\n=== ALNS Iteration " << iter << " ===" << std::endl;

      // Snapshot allocation before destroy/repair
      std::map<int, std::string> alloc_before;
      for (const auto& t : tasks) {
        alloc_before[t.id] = t.assignedRobot ? t.assignedRobot->name : "UNASSIGNED";
      }
      
      // 1. Destroy
      std::vector<int> currently_destroyed;
      std::map<int, std::string> tabu_list; // Map TaskID -> RobotName

      std::stringstream heuristics_ss;
      
      std::vector<std::pair<double, int>> weights;
      for(int i=0; i<tasks.size(); ++i) {
          double w = tasks[i].blocker_coins * 1.0 + 
                     tasks[i].waiter_coins * 1.0 + 
                     tasks[i].deadlock_coins * 5.0 + 
                     1.0; 
          weights.push_back({w, i});
      }
      
      // Deterministic destroy: top-k highest weight tasks
      std::sort(weights.rbegin(), weights.rend());
      std::cout << "  [Destroy] Selecting top " << destroy_count << " tasks by weight..." << std::endl;
      
      for(int k=0; k<destroy_count && k<static_cast<int>(weights.size()); ++k) {
          int selected_idx = weights[k].second;
          
          // Record Tabu info BEFORE destroying assignment
          if (tasks[selected_idx].assignedRobot) {
              tabu_list[tasks[selected_idx].id] = tasks[selected_idx].assignedRobot->name;
          }

          tasks[selected_idx].assignedRobot = nullptr;
          currently_destroyed.push_back(tasks[selected_idx].id);
          
          // Determine dominant heuristic
          std::string reason = "TopWeight";
          double b = tasks[selected_idx].blocker_coins;
          double w = tasks[selected_idx].waiter_coins;
          double d = tasks[selected_idx].deadlock_coins;
          if (d > 0) reason = "Deadlock(" + std::to_string(static_cast<int>(d)) + ")";
          else if (b > w && b > 1) reason = "Blocker(" + std::to_string(static_cast<int>(b)) + ")";
          else if (w > b && w > 1) reason = "Waiter(" + std::to_string(static_cast<int>(w)) + ")";
          
          heuristics_ss << "T" << tasks[selected_idx].id << ":" << reason << " ";
          
          std::cout << "    -> Removed Task " << tasks[selected_idx].id << " [" << reason << ", w=" << weights[k].first << "]" << std::endl;
      }
      
      // 2. Clear coins
      for(auto& t : tasks) {
          t.blocker_coins = 0;
          t.waiter_coins = 0;
          t.deadlock_coins = 0;
      }
      
      // 3. Repair (Solve)
      reset_robots(); 
      TimeTable current_timetable(0.5);
      current_timetable.set_capture_registration_snapshots(true);
      current_timetable.add_initial(entities);
      
      solve_allocation(tasks, all_robots, current_timetable, params, entities, tabu_list, initial_owner_by_task);
      
      double current_makespan = current_timetable.get_max_time();

        // Snapshot allocation after repair and build per-task allocation changes
        std::map<int, std::string> alloc_after;
        for (const auto& t : tasks) {
          alloc_after[t.id] = t.assignedRobot ? t.assignedRobot->name : "UNASSIGNED";
        }
        std::vector<std::string> alloc_changes;
        alloc_changes.reserve(tasks.size());
        for (const auto& t : tasks) {
          std::string obj_name = (t.targetObject ? t.targetObject->name : "unknown_obj");
          std::string before = alloc_before.count(t.id) ? alloc_before.at(t.id) : "UNASSIGNED";
          std::string after = alloc_after.count(t.id) ? alloc_after.at(t.id) : "UNASSIGNED";
          alloc_changes.push_back("T" + std::to_string(t.id) + "(" + obj_name + "): " + before + " -> " + after);
        }
      
      // Log result
      IterationLog log_entry;
      log_entry.iter = iter;
      log_entry.makespan = current_makespan;
      log_entry.destroyed_tasks = currently_destroyed;
      log_entry.heuristics_info = heuristics_ss.str();
      log_entry.improvement = initial_makespan - current_makespan; // Relative to initial
        log_entry.allocation_changes = std::move(alloc_changes);
      
      history.push_back(log_entry);

      std::cout << "[Result] Iteration " << iter << " Makespan: " << current_makespan << "s";
      if (current_makespan < best_makespan) { 
          std::cout << " (NEW BEST) ";
          best_makespan = current_makespan;
          best_timetable = current_timetable; // Update best found
      }
      std::cout << std::endl;
      
        bool has_collision = verify_collisions(current_timetable, current_makespan);
        if (has_collision) {
          std::cout << "  [Debug] Collision found. Printing timetable groups by trajectory..." << std::endl;
          current_timetable.print_grouped_entries_by_trajectory();

          const auto& snaps = current_timetable.get_registration_snapshots();
          std::cout << "  [Debug] Opening advanced replay visualizer for " << snaps.size() << " registered trajectories..." << std::endl;
          show_trajectory_registration_replay(current_timetable, entities, params);
        }
      
      // 4. Visualize (Skipped per request/comment)
      if(show_plan_result)
      {
        std::cout << "[System] Visualizing Iteration " << iter << "..." << std::endl;
        show_results(argc, argv, current_timetable, entities, params);
      }
  }

  // --- Final Summary ---
  std::cout << "\n\n=========================================" << std::endl;
  std::cout << "           ALNS RESULTS SUMMARY          " << std::endl;
  std::cout << "=========================================" << std::endl;
  std::cout << "Iter | Makespan (s) | Improvement | Destroyed Tasks (Constraint)" << std::endl;
  std::cout << "----------------------------------------------------------------" << std::endl;
  for(const auto& log : history) {
      std::cout << std::setw(4) << log.iter << " | " 
                << std::setw(12) << std::fixed << std::setprecision(2) << log.makespan << " | "
                << std::setw(11) << (log.iter==0 ? "-" : (log.improvement >= 0 ? "+" + std::to_string(log.improvement) : std::to_string(log.improvement))) << " | ";
      
      if (log.iter == 0) {
          std::cout << "Initial Solution";
      } else {
          std::cout << log.heuristics_info;
      }
      std::cout << std::endl;

        if (log.iter > 0) {
          std::cout << "      Allocation (Task(Object): initial -> new):" << std::endl;
          for (const auto& line : log.allocation_changes) {
            std::cout << "        - " << line << std::endl;
          }
        }
  }
  std::cout << "=========================================" << std::endl;
  std::cout << "Best Makespan Found: " << best_makespan << "s" << std::endl;

  // --- 6. Final Visualization (Side-by-Side Comparison) ---
  std::cout << "\n[System] Visualizing Comparison (Initial vs Best " << best_makespan << "s)..." << std::endl;
  show_comparison(argc, argv, initial_timetable, best_timetable, entities, params);

  // Final Cleanup
  for (auto &pair : entities)
    delete pair.second;
  return 0;
}
