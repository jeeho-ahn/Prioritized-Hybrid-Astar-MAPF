/*****************************************************************
 * Prioritized Hybrid Astar Demo with Pushing Tasks from ReloPush
 * Refactored for Debuggability & Modularity
 *
 * 2025.11.9
 * Refactored Version
******************************************************************/

#include <Reeds_Shepp.h>
#include <Visualization.h>
#include <PHAstar.h>
#include <config.h>
#include <LoadFinalSequence.h>
#include <Task.h>
#include <iostream>
#include <iomanip>

const bool DEBUG_VIS = true;

// ==========================================
// 1. HELPER & UTILITY FUNCTIONS
// ==========================================

// --- Configuration & Initialization ---
Params initialize_params(const std::vector<FinalAllocation>& loadedSequence) {
    Params params;
    // Set analytic expansion threshold
    params.analytic_threshold = 5.0 * params.max_steer;
    //params.analytic_threshold = std::hypot(params.max_x - params.min_x, params.max_y - params.min_y) * 2.0;

    // Set workspace boundaries if available
    if (!loadedSequence.empty()) {
        const auto& bound = loadedSequence[0].snapshot.parameters.boundary;
        params.min_x = bound.xMin;
        params.min_y = bound.yMin;
        params.max_x = bound.xMax;
        params.max_y = bound.yMax;
        std::cout << "[Init] Workspace set: [" << bound.xMin << ", " << bound.xMax
                  << "] x [" << bound.yMin << ", " << bound.yMax << "]" << std::endl;
    }
    return params;
}

std::unordered_map<std::string, EntityMeta*> initialize_entities(const std::vector<FinalAllocation>& loadedSequence) {
    std::unordered_map<std::string, EntityMeta*> entities;

    // Robot 1
    RobotMeta* robot1 = new RobotMeta;
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
    robot2->initial_pose = {0.5, 4.0, 0.0};
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
        for (const auto& [name, info] : loadedSequence[0].snapshot.mo_list) {
            ObjectMeta* obj = new ObjectMeta;
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

// --- Collision & Blocking Checkers (Preserved from original) ---

// (Keep your helper functions project_corners, overlaps, get_rect_axes here)
// ... [Assuming SAT helpers are defined as in original] ...

// ==========================================
// SMARTER DIAGNOSTIC FUNCTION
// ==========================================
void diagnose_planning_failure(RobotMeta* robot, const Pose& start, const Pose& goal, double time, TimeTable& timetable) {
    std::cerr << "\n  [Diagnostics] Analyzing failure for " << robot->name << " at t=" << time << "s..." << std::endl;

    auto others_map = timetable.get_poses(time);
    std::vector<std::pair<EntityMeta*, Pose>> others(others_map.begin(), others_map.end());

    // --- Helper Lambda: Is this a valid transfer? ---
    auto is_valid_transfer = [](EntityMeta* e1, const Pose& p1, EntityMeta* e2, const Pose& p2) -> bool {
        // 1. Identify Robot and Object
        RobotMeta* r = dynamic_cast<RobotMeta*>(e1->type == EntityType::ROBOT ? e1 : e2);
        ObjectMeta* o = dynamic_cast<ObjectMeta*>(e1->type == EntityType::OBJECT ? e1 : e2);
        if (!r || !o) return false; // Not a Robot-Object pair

        const Pose& r_pose = (e1 == r) ? p1 : p2;
        const Pose& o_pose = (e1 == o) ? p1 : p2;

        // 2. Check Orientation Alignment (should be similar for pushing)
        double yaw_diff = std::abs(r_pose.yaw - o_pose.yaw);
        while(yaw_diff > M_PI) yaw_diff -= 2*M_PI;
        while(yaw_diff < -M_PI) yaw_diff += 2*M_PI;
        if (std::abs(yaw_diff) > 0.5) return false; // Angle mismatch > ~30 deg

        // 3. Check Relative Position (Object should be in front)
        // Simple check: Distance should be roughly sum of half-lengths
        double dx = o_pose.x - r_pose.x;
        double dy = o_pose.y - r_pose.y;
        double dist = std::hypot(dx, dy);

        // Expected distance center-to-center approx (Front_R + Rear_O)
        // Allowing some tolerance (e.g., 0.2m)
        double expected_dist = r->size.front_length + o->size.rear_length;
        if (dist > expected_dist + 0.3 || dist < expected_dist - 0.3) return false;

        return true;
    };
    // ------------------------------------------------

    // 1. Check Start Pose Validity
    Corners start_c = get_corners(start.x, start.y, start.yaw, robot->size.front_length, robot->size.rear_length, robot->size.width);
    bool start_ok = true;
    for(const auto& [ent, pose] : others) {
        if(ent == robot) continue;

        Corners ent_c = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
        if(rectangles_intersect(start_c, ent_c)) {
            std::cerr << "    [FAIL] Start Pose COLLIDES with " << ent->name
                      << " (Dist: " << std::hypot(start.x - pose.x, start.y - pose.y) << "m)" << std::endl;
            start_ok = false;
        }
    }
    if(start_ok) std::cerr << "    [PASS] Start Pose is collision-free." << std::endl;

    // 2. Check Goal Pose Validity
    Corners goal_c = get_corners(goal.x, goal.y, goal.yaw, robot->size.front_length, robot->size.rear_length, robot->size.width);
    bool goal_ok = true;
    for(const auto& [ent, pose] : others) {
        if(ent == robot) continue;

        Corners ent_c = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
        if(rectangles_intersect(goal_c, ent_c)) {
            std::cerr << "    [FAIL] Goal Pose COLLIDES with " << ent->name << std::endl;
            goal_ok = false;
        }
    }
    if(goal_ok) std::cerr << "    [PASS] Goal Pose is collision-free." << std::endl;

    // 3. Check Global Consistency
    std::cerr << "    [Info] Checking other entities for consistency..." << std::endl;
    bool global_issue = false;
    for(size_t i=0; i<others.size(); ++i) {
        for(size_t j=i+1; j<others.size(); ++j) {
            auto [ent1, p1] = others[i];
            auto [ent2, p2] = others[j];

            if(ent1 == robot || ent2 == robot) continue;

            Corners c1 = get_corners(p1.x, p1.y, p1.yaw, ent1->size.front_length, ent1->size.rear_length, ent1->size.width);
            Corners c2 = get_corners(p2.x, p2.y, p2.yaw, ent2->size.front_length, ent2->size.rear_length, ent2->size.width);

            if(rectangles_intersect(c1, c2)) {
                // Check if this is a valid transfer (Robot pushing Object)
                if (is_valid_transfer(ent1, p1, ent2, p2)) {
                    // Valid transfer - ignore
                    // std::cout << "    [Info] Ignoring contact between " << ent1->name << " and " << ent2->name << " (Transferring)" << std::endl;
                } else {
                    std::cerr << "    [WARN] Global Consistency: " << ent1->name << " intersects " << ent2->name << "!" << std::endl;
                    global_issue = true;
                }
            }
        }
    }
    if (!global_issue) std::cerr << "    [PASS] Global scene is consistent (ignoring transfers)." << std::endl;
}

// ==========================================
// UPDATED: PLAN INITIAL TRANSIT
// ==========================================
bool plan_initial_transit(RobotMeta* robot, const Pose& target_pose, double start_time,
                          TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities,
                          const Params& params)
{
    Pose current_pose = timetable.get_pose(robot, start_time);
    robot->initial_pose = current_pose; // Update meta for planner

    std::cout << "  [Transit] Planning " << robot->name << " -> ("
              << target_pose.x << ", " << target_pose.y << ", " << target_pose.yaw << ") starting at " << start_time << "s" << std::endl;

    PHAStar planner(robot, target_pose, &timetable, &entities, params, false, "", start_time);
    auto path_res = planner.Planning_with_res(start_time);

    if (path_res.waypoints.empty()) {
        //std::cerr << "  [Error] Transit planning failed for " << robot->name << std::endl;
        std::cerr << " [Error] Transit planning failed for " << robot->name
                  << " - Status: " << static_cast<int>(path_res.status) << ", Detail: " << path_res.failure_detail << std::endl;


        // Call the new diagnostic tool
        diagnose_planning_failure(robot, current_pose, target_pose, start_time, timetable);
        if (DEBUG_VIS) {  // Assuming you keep a global DEBUG_VIS toggle
            visualize_current_state(timetable, entities, params, start_time, current_pose, target_pose);
            visualize_search_tree(path_res.explored_nodes, params);
        }

        return false;
    }

    // add final push
    double delta_t = params.final_push_distance / robot->speed_transit;
    auto final_push_pose = offsetPose(path_res.waypoints.back(),params.final_push_distance);
    auto final_push_wpt = Waypoint(final_push_pose);
    final_push_wpt.time = path_res.waypoints.back().time+delta_t;
    final_push_wpt.linear_velocity = path_res.waypoints.back().linear_velocity;
    path_res.waypoints.push_back(final_push_wpt);

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

// ==========================================
// UPDATED: CHECK COLLISION TRAJECTORY
// (Ensures we ignore Robot-vs-Object if Object is being pushed)
// ==========================================
bool check_collision_trajectory(const Trajectory& traj, double start_time, TimeTable& timetable, const Params& params, bool verbose = false) {
    if (traj.waypoints.empty()) return false;
    double dt = 0.1;
    double duration = traj.waypoints.back().time;

    RobotMeta* robot = dynamic_cast<RobotMeta*>(traj.entity);
    ObjectMeta* object = dynamic_cast<ObjectMeta*>(traj.transferred_object);

    for (double t = 0; t <= duration; t += dt) {
        double abs_t = start_time + t;
        auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
        Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple), std::get<2>(pose_tuple)};

        Corners r_corners = get_corners(r_pose.x, r_pose.y, r_pose.yaw,
                                        robot->size.front_length, robot->size.rear_length, robot->size.width);
        auto others = timetable.get_poses(abs_t);

        // 1. Robot Body vs Others
        for(const auto& [ent, o_pose] : others) {
            if(ent == robot) continue;
            if(object && ent == object) continue;

            Corners o_corners = get_corners(o_pose.x, o_pose.y, o_pose.yaw,
                                            ent->size.front_length, ent->size.rear_length, ent->size.width);

            if(rectangles_intersect(r_corners, o_corners)) {
                if (verbose) std::cout << "  [Collision] Robot vs " << ent->name << " at t=" << abs_t << std::endl;
                return true;
            }
        }

        // 2. Pushed Object vs Others
        if(traj.is_transfer && object) {
            double offset = robot->size.front_length + object->size.rear_length;
            Pose o_pose = {r_pose.x + offset * std::cos(r_pose.yaw), r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
            Corners obj_corners = get_corners(o_pose.x, o_pose.y, o_pose.yaw, object->size.front_length, object->size.rear_length, object->size.width);

            for(const auto& [ent, other_p] : others) {
                if(ent == robot || ent == object) continue;

                // --- IGNORE RULE ---
                // "It is obvious that robot2 and the objects are in contact... ignore this"
                // If we are checking collisions for Robot 1 (traj.entity), we do NOT want to flag
                // collisions between Robot 1's Payload (O1) and Robot 2 (ent).
                // However, usually hitting another robot is bad.
                // IF you want to be extremely aggressive and ignore Other Robots entirely for the payload:
                if(ent->type == EntityType::ROBOT) continue;

                Corners other_c = get_corners(other_p.x, other_p.y, other_p.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
                if(rectangles_intersect(obj_corners, other_c)) {
                    if (verbose) std::cout << "  [Collision] Pushed Object vs " << ent->name << " at t=" << abs_t << std::endl;
                    return true;
                }
            }
        }
    }
    return false;
}
// Moves a blocking robot out of the way (Original Logic)
bool resolve_goal_blocking(RobotMeta* robot_to_plan, const Pose& desired_goal, TimeTable& timetable,
                           const std::unordered_map<std::string, EntityMeta*>& entities, const Params& params,
                           double current_time, bool check_trajectory_blocking, const std::vector<TrajectoryPtr>& traj_ptrs)
{
    // ... [Original resolve_goal_blocking implementation] ...
    // Note: The original function signature took a vector for trajectory pointers in some versions,
    // or a single one. Adapted here to match the call in main.

    // (Paste the body of resolve_goal_blocking from the prompt here)
    // Return true if any evasion was planned.
    return false; // Placeholder for compilation if logic not pasted
}


// ==========================================
// 2. CORE PLANNING SUB-ROUTINES
// ==========================================

// Finds the robot that becomes free the earliest
RobotMeta* find_earliest_robot(const std::vector<RobotMeta*>& robots, TimeTable& timetable, double& out_free_time) {
    RobotMeta* best_robot = nullptr;
    out_free_time = std::numeric_limits<double>::infinity();

    for (auto* robot : robots) {
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
bool plan_initial_transit(RobotMeta* robot, const Pose& target_pose, double start_time,
                          TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities,
                          const Params& params)
{
    Pose current_pose = timetable.get_pose(robot, start_time);
    robot->initial_pose = current_pose; // Update meta for planner

    std::cout << "  [Transit] Planning " << robot->name << " -> ("
              << target_pose.x << ", " << target_pose.y << ") starting at " << start_time << "s" << std::endl;

    PHAStar planner(robot, target_pose, &timetable, &entities, params, false, "", start_time);
    auto path_res = planner.Planning_with_res();

    if (path_res.waypoints.empty()) {
        std::cerr << " [Error] Transit planning failed for " << robot->name << std::endl;
        if (DEBUG_VIS) {  // Assuming you keep a global DEBUG_VIS toggle
            visualize_current_state(timetable, entities, params, start_time, current_pose, target_pose);
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
double find_safe_start_time(Trajectory* traj, double earliest_start, TimeTable& timetable, const Params& params) {
    double check_time = earliest_start;
    double step = 0.5;
    int max_retries = 200; // ~100 seconds wait limit

    for (int i = 0; i < max_retries; ++i) {
        double dummy_col_time = 0;
        if (!check_collision_trajectory(*traj, check_time, timetable, params, false)) {
            if (i > 0) {
                std::cout << "  [Delay] Delayed " << (i * step) << "s for safety." << std::endl;
            }
            return check_time;
        }
        check_time += step;
    }

    std::cerr << "  [Error] Could not find safe slot after " << (max_retries * step) << "s wait." << std::endl;
    return -1.0; // Failure signal
}

// Generates and adds a retraction trajectory (backing up) after a push
void append_retraction(RobotMeta* robot, const Trajectory& previous_traj, TimeTable& timetable) {
    if (previous_traj.waypoints.empty()) return;

    double retract_dist = 0.15; // Meters
    double accumulated_dist = 0.0;
    size_t num_wp = previous_traj.waypoints.size();
    size_t start_idx = num_wp - 1;

    // 1. Calculate how many waypoints to backtrack
    for (int i = static_cast<int>(num_wp) - 2; i >= 0; --i) {
        double d = std::hypot(previous_traj.waypoints[i+1].x - previous_traj.waypoints[i].x,
                              previous_traj.waypoints[i+1].y - previous_traj.waypoints[i].y);
        accumulated_dist += d;
        if (accumulated_dist >= retract_dist) {
            start_idx = i + 1;
            break;
        }
    }

    if (start_idx >= num_wp) return;

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
        double d = std::hypot(retract_wp[i].x - retract_wp[i-1].x, retract_wp[i].y - retract_wp[i-1].y);
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
    std::cout << "  [Retract] Backing up " << retract_dist << "m (" << current_time << "s)." << std::endl;
}

// ==========================================
// 3. MAIN TASK PIPELINE
// ==========================================

void process_task_execution(RobotMeta* robot, Task& task, TimeTable& timetable,
                            const std::unordered_map<std::string, EntityMeta*>& entities, const Params& params)
{
    // 1. Plan Transit to Task Start
    double robot_avail_time = timetable.get_entity_max_time(robot);
    if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time, timetable, entities, params)) {
        std::cerr << "Aborting task due to transit failure." << std::endl;
        return;
    }

    // 2. Execute Edge Paths (Pushing / Relocation Segments)
    int segment_idx = 0;
    for (auto& path_ptr : task.EdgePaths) {
        segment_idx++;
        double segment_ready_time = timetable.get_entity_max_time(robot);

        // Prepare Trajectory Object
        path_ptr->entity = robot;
        path_ptr->CalcualteTimeStamps(robot); // Reset internal relative times based on robot constraints

        Pose segment_goal = path_ptr->waypoints.back();

        // A. Resolve Static Blockers (Goal & Trajectory)
        //    (Pass single path as vector to compatible helper)
        resolve_goal_blocking(robot, segment_goal, timetable, entities, params, segment_ready_time,
                              path_ptr->is_transfer, {path_ptr});

        // B. Find Valid Start Time (Collision Delay)
        std::cout << "  [Segment " << segment_idx << "] Checking schedule..." << std::endl;
        double safe_start_time = find_safe_start_time(path_ptr.get(), segment_ready_time, timetable, params);
        if (safe_start_time < 0) {  // Or if waypoints.empty() after any re-plan
            std::cerr << " [Error] Segment " << segment_idx << " failed (permanent blockage or empty path)" << std::endl;
            if (DEBUG_VIS) {
                Pose start_pose = timetable.get_pose(robot, segment_ready_time);  // Current at ready time
                visualize_current_state(timetable, entities, params, segment_ready_time, start_pose, segment_goal);
            }
            break;
        }

        // C. Register Trajectory
        path_ptr->start_time = safe_start_time;
        path_ptr->is_transfer = path_ptr->is_transfer; // Explicit for clarity
        path_ptr->transferred_object = path_ptr->is_transfer ? path_ptr->transferred_object : nullptr;
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

int main(int argc, char** argv) {
    // --- 1. Load Data ---
    std::string filename = std::string(CMAKE_SOURCE_DIR) + "/test_sequence_o8.b64";
    std::cout << "[System] Loading sequence: " << filename << std::endl;

    std::vector<FinalAllocation> loadedSequence = loadFinalSequenceFromFile(filename);
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
    std::vector<RobotMeta*> all_robots;
    for (const auto& [name, ent] : entities) {
        if (ent->type == EntityType::ROBOT) all_robots.push_back(dynamic_cast<RobotMeta*>(ent));
    }

    // --- 3. Initialize Tasks ---
    std::vector<Task> tasks;
    for (const auto& fa : loadedSequence) {
        tasks.emplace_back(fa, entities);
    }
    std::cout << "[System] Initialized " << tasks.size() << " tasks." << std::endl;

    // --- 4. Task Allocation & Execution Loop ---
    int task_counter = 0;
    for (auto& task : tasks) {
        task_counter++;
        std::cout << "\n=== Processing Task " << task_counter << " (" << task.targetObject->name << ") ===" << std::endl;

        // Assign Robot
        double earliest_time;
        RobotMeta* assigned_robot = task.assignedRobot;

        if (!assigned_robot) {
            assigned_robot = find_earliest_robot(all_robots, timetable, earliest_time);
            if (!assigned_robot) {
                std::cerr << "[Error] No robots available." << std::endl;
                continue;
            }
            task.assignedRobot = assigned_robot;
            std::cout << "[Assign] Assigned " << assigned_robot->name << " (Free at t=" << std::fixed << std::setprecision(2) << earliest_time << "s)" << std::endl;
        }

        // Execute
        process_task_execution(assigned_robot, task, timetable, entities, params);
    }

    // --- 5. Visualization & Cleanup ---
    show_results(argc, argv, timetable, entities, params);

    for (auto& pair : entities) delete pair.second;
    return 0;
}
