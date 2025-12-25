/*****************************************************************
 * Prioritized Hybrid Astar Demo with Pushing Tasks from ReloPush
 *
 * 2025.11.9
 * Jeeho Ahn, jeeho@umich.edu
******************************************************************/

#include <Reeds_Shepp.h>
#include <Visualization.h>
#include <PHAstar.h>
#include <config.h> // to parse CMake project directory
#include <LoadFinalSequence.h>
#include <Task.h>

#define USE_GEM // testing differet versions


const bool print_path = false;

Params initialize_params() {
    Params params;
    params.analytic_threshold = 5.0 * params.max_steer;  // Adjust if needed
    return params;
}

std::unordered_map<std::string, EntityMeta*> initialize_entities(const std::vector<FinalAllocation>& loadedSequence) {
    std::unordered_map<std::string, EntityMeta*> entities;

    // Initialize robots manually
    RobotMeta* robot1 = new RobotMeta;
    robot1->name = "robot1";
    robot1->type = EntityType::ROBOT;
    robot1->initial_pose = {0.5, 0.45, 0.0};
    robot1->size.front_length = 0.3;
    robot1->size.rear_length = 0.2;
    robot1->size.width = 0.3;
    robot1->min_turning_radius = 1.0;
    robot1->wheel_base = 0.4;
    robot1->speed_transit = 0.2;
    robot1->speed_transfer = 0.15;
    entities["robot1"] = robot1;

    RobotMeta* robot2 = new RobotMeta;
    robot2->name = "robot2";
    robot2->type = EntityType::ROBOT;
    robot2->initial_pose = {0.5, 4.0, 0.0};
    robot2->size.front_length = 0.3;
    robot2->size.rear_length = 0.2;
    robot2->size.width = 0.3;
    robot2->min_turning_radius = 1.0;
    robot2->wheel_base = 0.4;
    robot2->speed_transit = 0.2;
    robot2->speed_transfer = 0.15;
    entities["robot2"] = robot2;

    // Parse objects from loadedSequence
    if (!loadedSequence.empty()) {
        for (const auto& [name, info] : loadedSequence[0].snapshot.mo_list) {
            ObjectMeta* obj = new ObjectMeta;
            obj->name = name;
            obj->type = EntityType::OBJECT;
            obj->initial_pose = {info.x, info.y, info.nominalOrientation};
            obj->size.front_length = 0.075;
            obj->size.rear_length = 0.075;
            obj->size.width = 0.15;
            entities[name] = obj;
        }
    }

    return entities;
}


std::pair<double, double> project_corners(const Corners& corners, const Point& axis) {
    double min_d = std::numeric_limits<double>::infinity();
    double max_d = -std::numeric_limits<double>::infinity();
    for (const auto& c : corners) {
        double dot = c.x * axis.x + c.y * axis.y;
        min_d = std::min(min_d, dot);
        max_d = std::max(max_d, dot);
    }
    return {min_d, max_d};
}

// Helper to check overlap (SAT)
bool overlaps(std::pair<double, double> p1, std::pair<double, double> p2) {
    return p1.second >= p2.first && p2.second >= p1.first;
}

// Helper to get axes (SAT)
std::vector<Point> get_rect_axes(const Corners& corners) {
    std::vector<Point> axes;
    for (size_t i = 0; i < 4; ++i) {
        Point p1 = corners[i];
        Point p2 = corners[(i + 1) % 4];
        double ex = p2.x - p1.x;
        double ey = p2.y - p1.y;
        double len = std::hypot(ex, ey);
        axes.push_back({-ey / len, ex / len}); // Normal
    }
    return axes;
}


bool check_collision_trajectory(const Trajectory& traj, double start_time, TimeTable& timetable, const Params& params, bool verbose = true) {
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

        for(const auto& [ent, o_pose] : others) {
            if(ent == robot) continue;
            if(object && ent == object) continue;

            Corners o_corners = get_corners(o_pose.x, o_pose.y, o_pose.yaw,
                                            ent->size.front_length, ent->size.rear_length, ent->size.width);

            if(rectangles_intersect(r_corners, o_corners)) {
                if (verbose) {
                    std::cout << "  [COLLISION] Time=" << abs_t << "s. "
                              << "Robot '" << robot->name << "' at (" << r_pose.x << "," << r_pose.y << ") "
                              << "hit '" << ent->name << "' (" << (ent->type==EntityType::ROBOT?"Robot":"Object") << ") "
                              << "at (" << o_pose.x << "," << o_pose.y << ")" << std::endl;
                }
                return true;
            }
        }

        if(traj.is_transfer && object) {
            double offset = robot->size.front_length + object->size.rear_length;
            Pose o_pose;
            o_pose.x = r_pose.x + offset * std::cos(r_pose.yaw);
            o_pose.y = r_pose.y + offset * std::sin(r_pose.yaw);
            o_pose.yaw = r_pose.yaw;

            Corners obj_corners = get_corners(o_pose.x, o_pose.y, o_pose.yaw,
                                              object->size.front_length, object->size.rear_length, object->size.width);

            for(const auto& [ent, other_p] : others) {
                if(ent == robot) continue;
                if(ent == object) continue;

                Corners other_c = get_corners(other_p.x, other_p.y, other_p.yaw,
                                              ent->size.front_length, ent->size.rear_length, ent->size.width);
                if(rectangles_intersect(obj_corners, other_c)) {
                    if (verbose) {
                        std::cout << "  [COLLISION] Time=" << abs_t << "s. "
                                  << "Pushed Object '" << object->name << "' hit '" << ent->name << "' "
                                  << "at (" << other_p.x << "," << other_p.y << ")" << std::endl;
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

// --- Helper to move a blocking robot out of the way ---
// Moves the blocker to a nearby safe spot (heuristic: shift X or Y by 2.0m)
bool resolve_goal_blocking(
    RobotMeta* robot_to_plan,                                   // the robot we want to plan for
    const Pose& desired_goal,                                   // its goal pose
    TimeTable& timetable,
    const std::unordered_map<std::string, EntityMeta*>& entities,
    const Params& params,
    double current_time,                                        // current time of this robot
    bool check_trajectory_blocking = false,                     // NEW: optional - also check along preplanned trajectory?
    const TrajectoryPtr traj_ptr = nullptr)          // NEW: pass EdgePaths if checking trajectory blocks
{
    constexpr double BLOCK_DIST_THRESHOLD   = 0.90;   // [m] if anyone is closer than this → blocking
    constexpr double BLOCK_YAW_THRESHOLD    = M_PI_2; // 90° yaw tolerance
    constexpr double STATIONARY_THRESHOLD   = 0.20;   // moved <20 cm in 4 seconds → considered waiting
    constexpr double LOOKAHEAD_TIME         = 4.0;    // seconds
    constexpr double TRAJ_SAMPLE_STEP       = 0.5;    // [m] sample every 50cm along trajectory for blocks
    constexpr double TRAJ_BLOCK_DIST        = 0.60;   // [m] closer than this to traj point → blocking along path

    bool cleared_any = false;

    // ---- PART 1: Check goal blocking (always on) ----
    for (const auto& [name, ent] : entities) {
        if (ent->type != EntityType::ROBOT || ent == robot_to_plan) continue;
        RobotMeta* blocker = dynamic_cast<RobotMeta*>(ent);

        Pose now_pose = timetable.get_pose(blocker, current_time);
        Pose future_pose = timetable.get_pose(blocker, current_time + LOOKAHEAD_TIME);

        double dist_to_goal = std::hypot(now_pose.x - desired_goal.x, now_pose.y - desired_goal.y);
        double yaw_diff = std::abs(pi_2_pi(now_pose.yaw - desired_goal.yaw));
        double moved = std::hypot(future_pose.x - now_pose.x, future_pose.y - now_pose.y);

        if (dist_to_goal < BLOCK_DIST_THRESHOLD &&
            yaw_diff < BLOCK_YAW_THRESHOLD &&
            moved < STATIONARY_THRESHOLD)
        {
            std::cout << "[GOAL BLOCK] " << blocker->name << " is parked at the goal of "
                      << robot_to_plan->name << " (dist=" << dist_to_goal << "m). Forcing it to move aside.\n";

            // Generate evasive goal (left first, then right)
            std::vector<double> directions = { now_pose.yaw + M_PI_2, now_pose.yaw - M_PI_2 };
            bool cleared = false;

            for (double dir : directions) {
                Pose evade_goal = now_pose;
                evade_goal.x += 1.5 * std::cos(dir);
                evade_goal.y += 1.5 * std::sin(dir);

                PHAStar evader(blocker, evade_goal, &timetable, &entities, params, false, "", current_time);
                auto evade_path = evader.Planning_with_res();

                if (!evade_path.waypoints.empty()) {
                    for (auto& wp : evade_path.waypoints) wp.time -= current_time; // relative

                    Trajectory evade_traj;
                    evade_traj.entity = blocker;
                    evade_traj.start_time = current_time;
                    evade_traj.waypoints = evade_path.waypoints;
                    evade_traj.is_transfer = false;

                    timetable.add_trajectory(evade_traj);
                    std::cout << "  → " << blocker->name << " moved aside from goal.\n";
                    cleared = true;
                    cleared_any = true;
                    break;
                }
            }

            if (!cleared) {
                std::cout << "  → WARN: No evasive path for " << blocker->name << " from goal block.\n";
            }
        }
    }

    // ---- PART 2: Check trajectory blocking (optional) ----
    if (check_trajectory_blocking && traj_ptr!=nullptr) {
        std::cout << "[TRAJ BLOCK CHECK] Scanning preplanned paths for blockers...\n";

       //     if (!traj_ptr || traj_ptr->waypoints.empty())

            const auto& waypoints = traj_ptr->waypoints;
            double traj_duration = waypoints.back().time;

            // Sample points along the trajectory (relative time)
            for (double rel_t = 0.0; rel_t <= traj_duration; rel_t += TRAJ_SAMPLE_STEP) {
                Pose traj_point = TimeTable::interpolate_waypoints(waypoints, rel_t);

                // Check all other robots against this point
                for (const auto& [name, ent] : entities) {
                    if (ent->type != EntityType::ROBOT || ent == robot_to_plan) continue;
                    RobotMeta* blocker = dynamic_cast<RobotMeta*>(ent);

                    Pose now_pose = timetable.get_pose(blocker, current_time);
                    Pose future_pose = timetable.get_pose(blocker, current_time + LOOKAHEAD_TIME);

                    double dist_to_traj = std::hypot(now_pose.x - traj_point.x, now_pose.y - traj_point.y);
                    double moved = std::hypot(future_pose.x - now_pose.x, future_pose.y - now_pose.y);

                    if (dist_to_traj < TRAJ_BLOCK_DIST && moved < STATIONARY_THRESHOLD) {
                        std::cout << "[TRAJ BLOCK] " << blocker->name << " is parked along path of "
                                  << robot_to_plan->name << " (dist=" << dist_to_traj << "m at rel_t=" << rel_t << "). Forcing aside.\n";

                        // Evasive move: perpendicular to trajectory direction at that point
                        double traj_yaw = traj_point.yaw;
                        std::vector<double> directions = { traj_yaw + M_PI_2, traj_yaw - M_PI_2 };
                        bool cleared = false;

                        for (double dir : directions) {
                            Pose evade_goal = now_pose;
                            evade_goal.x += 1.5 * std::cos(dir);
                            evade_goal.y += 1.5 * std::sin(dir);

                            PHAStar evader(blocker, evade_goal, &timetable, &entities, params, false, "", current_time);
                            auto evade_path = evader.Planning_with_res();

                            if (!evade_path.waypoints.empty()) {
                                for (auto& wp : evade_path.waypoints) wp.time -= current_time;

                                Trajectory evade_traj;
                                evade_traj.entity = blocker;
                                evade_traj.start_time = current_time;
                                evade_traj.waypoints = evade_path.waypoints;
                                evade_traj.is_transfer = false;

                                timetable.add_trajectory(evade_traj);
                                std::cout << "  → " << blocker->name << " moved aside from trajectory.\n";
                                cleared = true;
                                cleared_any = true;
                                break; // one evasive per blocker per point
                            }
                        }

                        if (!cleared) {
                            std::cout << "  → WARN: No evasive path for " << blocker->name << " from traj block.\n";
                        }
                        // Only check next traj point if cleared (avoid redundant checks)
                        break;
                    }
                }

        }
    }

    return cleared_any;  // true if we resolved at least one block
}

int main(int argc, char** argv) {
    //////// test loading sequence from file ////////
    std::string filename = std::string(CMAKE_SOURCE_DIR) + "/test_sequence_o8.b64";

    std::cout << filename << std::endl;

    std::vector<FinalAllocation> loadedSequence = loadFinalSequenceFromFile(filename);

    if (loadedSequence.empty()) {
        std::cerr << "Failed to load finalSequence from " << filename << std::endl;
        return 1;
    }


    std::cout << "Loaded " << loadedSequence.size() << " allocations from " << filename << std::endl;
    ////////

    for (size_t i = 0; i < loadedSequence.size(); ++i) {
        const auto& fa = loadedSequence[i];
        std::cout << "Allocation " << i << ": Object=" << fa.object.name
                  << ", StartPoseObj=(x=" << fa.startPose.x << ", y=" << fa.startPose.y << ", yaw=" << fa.startPose.yaw << ")"
                  << ", GoalPoseObj=(x=" << fa.goalPose.x << ", y=" << fa.goalPose.y << ", yaw=" << fa.goalPose.yaw << ")"
                  << std::endl;
        // Also print if there are pre-relocations or multiple paths (for context)
        std::cout << "  Has pre-relocation? " << (fa.paths.empty() ? "No" : (fa.paths[0].preRelo.used ? "Yes" : "No")) << std::endl;
        std::cout << "  Number of edge paths: " << fa.paths.size() << std::endl;
    }

    Params params = initialize_params();

    if (!loadedSequence.empty()) {
        const auto& bound = loadedSequence[0].snapshot.parameters.boundary;
        std::cout << "Workspace boundary: xMin=" << bound.xMin << ", yMin=" << bound.yMin << ", xMax=" << bound.xMax << ", yMax=" << bound.yMax << std::endl;
        params.min_x = bound.xMin;
        params.min_y = bound.yMin;
        params.max_x = bound.xMax;
        params.max_y = bound.yMax;
    }

    auto entities = initialize_entities(loadedSequence);

    // Init tasks from planned sequence
    std::vector<Task> tasks;
    for (const auto& fa : loadedSequence) {
        tasks.emplace_back(fa, entities);
    }

    // Allocate first m tasks to m robots (m=2)
    // todo: find better allocation
    std::vector<std::string> robots = {"robot1", "robot2"};
    size_t m = robots.size();
    for (size_t i = 0; i < std::min(m, tasks.size()); ++i) {
        tasks[i].assignedRobot = dynamic_cast<RobotMeta*>(entities.at(robots[i]));
        std::cout << "Assigned task for object " << tasks[i].targetObject->name << " to " << robots[i] << std::endl;
    }


    TimeTable timetable(0.5);
    timetable.add_initial(entities);

    // Robots
    std::vector<RobotMeta *> all_robots;
    for (const auto &[name, ent] : entities)
    {
        if (ent->type == EntityType::ROBOT)
        {
            all_robots.push_back(dynamic_cast<RobotMeta *>(ent));
        }
    }

    int task_count = 0; //for debug only
    //std::vector<std::tuple<std::string, Pose, bool, std::string, double>> plans;
    for (auto &task : tasks)
    {   task_count++;
        // Handle unassigned tasks: assign to earliest available robot
        if (task.assignedRobot == nullptr)
        {
            RobotMeta *best_robot = nullptr;
            double earliest_free_time = std::numeric_limits<double>::infinity();

            for (auto *candidate : all_robots)
            {
                // Get the time when this robot finishes all current trajectories
                double candidate_free_time = timetable.get_entity_max_time(candidate);

                // Assign to the one that becomes free earliest
                if (candidate_free_time < earliest_free_time)
                {
                    earliest_free_time = candidate_free_time;
                    best_robot = candidate;
                }
            }

            if (best_robot != nullptr)
            {
                task.assignedRobot = best_robot;
                std::cout << "Assigned unassigned task to robot " << best_robot->name
                          << " (available at t=" << earliest_free_time << "s)" << std::endl;
            }
            else
            {
                std::cerr << "ERROR: No available robot found for task!" << std::endl;
                continue; // Skip this task
            }
        }

        RobotMeta *r = task.assignedRobot;
        if (!r)
            continue; // Safety check

        // Plan the first transit: from current pose to task start pose
        double start_time = timetable.get_entity_max_time(r); // FIXED: Use actual finish time, not 0
        Pose current_pose_at_start = timetable.get_pose(r, start_time);
        // update r pose
        r->initial_pose = current_pose_at_start;
        Pose task_start_pose = task.TaskStartPoseRobot;

        std::cout << "Planning transit for " << r->name << " from ("
                  << current_pose_at_start.x << ", " << current_pose_at_start.y << ") to task start ("
                  << task_start_pose.x << ", " << task_start_pose.y << ")" << std::endl;

        PHAStar planner(r, task_start_pose, &timetable, &entities, params, false, "", start_time);
        auto path_res = planner.Planning_with_res();

        if (path_res.waypoints.empty())
        {
            std::cerr << "WARNING: Transit planning failed for " << r->name << ". Skipping task." << std::endl;
            continue;
        }

        // Fix time offsets: make waypoints relative to trajectory start
        for (auto &wp : path_res.waypoints)
        {
            wp.time -= start_time;
        }

        Trajectory transit_traj;
        transit_traj.entity = r;
        transit_traj.start_time = start_time;
        transit_traj.waypoints = path_res.waypoints;
        transit_traj.is_transfer = false;
        transit_traj.transferred_object = nullptr;

        timetable.add_trajectory(transit_traj);

        // Now process EdgePaths (pushing segments)
        for (auto &it : task.EdgePaths)
        {
            // Get when the robot finishes its previous move (transit or prev segment)
            double transit_end_time = timetable.get_entity_max_time(r);

            auto traj_to_reg = it;
            traj_to_reg->entity = r;
            traj_to_reg->CalcualteTimeStamps(r); // Ensures internal time starts at 0

            // FIXED: Use actual goal pose for blocking check (not waypoints.back().time)
            Pose segment_goal = traj_to_reg->waypoints.back(); // Last waypoint as goal
            double segment_start_time = transit_end_time;

            // NEW: Check for blocking robots (goal + trajectory if transfer)
            bool is_transfer = traj_to_reg->is_transfer;
            resolve_goal_blocking(r, segment_goal, timetable, entities, params, segment_start_time,
                                  is_transfer, {traj_to_reg}); // Pass single traj as vector

            // --- Collision delay logic (using your check_collision_trajectory) ---
            double potential_start_time = transit_end_time;
            double wait_step = 0.5;
            int max_retries = 200; // ~100s max wait
            int retry = 0;

            std::cout << "Checking collision for " << r->name << " starting at " << potential_start_time << "..." << std::endl;

            // FIXED: Add first_collision_time param if your function requires it (assuming it does from earlier doc)
            double first_collision_time = 0.0;
            while (check_collision_trajectory(*traj_to_reg, potential_start_time, timetable, params, first_collision_time))
            {
                potential_start_time += wait_step;
                retry++;
                if (retry > max_retries)
                {
                    std::cerr << "ERROR: Could not find free slot for " << r->name
                              << " after waiting " << (max_retries * wait_step) << "s."
                              << " There may be a static obstacle blocking the path." << std::endl;
                    break;
                }
            }

            if (retry > 0 && retry <= max_retries)
            {
                std::cout << "Delayed " << r->name << " by " << (retry * wait_step)
                          << "s to avoid collision. New Start: " << potential_start_time << std::endl;
            }

            traj_to_reg->start_time = potential_start_time;
            traj_to_reg->is_transfer = it->is_transfer;
            traj_to_reg->transferred_object = traj_to_reg->is_transfer ? it->transferred_object : nullptr;
            timetable.add_trajectory(*traj_to_reg);

            // retract from the delivered object
            if (traj_to_reg->is_transfer && !traj_to_reg->waypoints.empty()) {
                // After pushing, add retraction by reversing the last ~0.5m portion of the trajectory
                double retract_dist = 0.15;  // [m] backward distance to retract
                size_t num_waypoints = traj_to_reg->waypoints.size();

                // Find the starting index for the last portion (backtrack ~retract_dist)
                double accumulated_dist = 0.0;
                size_t retract_start_idx = num_waypoints - 1;
                for (int i = static_cast<int>(num_waypoints) - 2; i >= 0; --i) {
                    double dx = traj_to_reg->waypoints[i+1].x - traj_to_reg->waypoints[i].x;
                    double dy = traj_to_reg->waypoints[i+1].y - traj_to_reg->waypoints[i].y;
                    double seg_dist = std::hypot(dx, dy);
                    accumulated_dist += seg_dist;
                    if (accumulated_dist >= retract_dist) {
                        retract_start_idx = i + 1;  // Start reversing from here
                        break;
                    }
                }

                if (retract_start_idx >= num_waypoints) {
                    std::cout << "  → WARNING: Trajectory too short for retraction. Skipping." << std::endl;
                } else {
                    // Extract the last portion and reverse it (positions reversed, yaw unchanged from original)
                    std::vector<Waypoint> retract_waypoints;
                    for (int i = static_cast<int>(num_waypoints) - 1; i >= static_cast<int>(retract_start_idx); --i) {
                        Waypoint wp = traj_to_reg->waypoints[i];
                        // Keep original yaw (face forward while backing up)
                        wp.yaw = traj_to_reg->waypoints[i].yaw;
                        // Reverse motion: negate velocity and steering (for reverse curve)
                        wp.linear_velocity = -std::abs(wp.linear_velocity);
                        wp.steering_angle = -wp.steering_angle;
                        retract_waypoints.push_back(wp);
                    }

                    // Recompute times for retraction (start at 0, use transit speed)
                    double end_time = timetable.get_entity_max_time(r);
                    RobotMeta* robot = r;  // For CalcualteTimeStamps
                    retract_waypoints[0].time = 0.0;
                    double current_retract_time = 0.0;
                    double transit_speed = robot->speed_transit;  // Use faster transit speed for retraction

                    for (size_t i = 1; i < retract_waypoints.size(); ++i) {
                        double dx = retract_waypoints[i].x - retract_waypoints[i-1].x;
                        double dy = retract_waypoints[i].y - retract_waypoints[i-1].y;
                        double distance = std::hypot(dx, dy);
                        double delta_t = distance / transit_speed;
                        current_retract_time += delta_t;
                        retract_waypoints[i].time = current_retract_time;
                    }

                    Trajectory retract_traj;
                    retract_traj.entity = r;
                    retract_traj.start_time = end_time;
                    retract_traj.waypoints = retract_waypoints;
                    retract_traj.is_transfer = false;
                    retract_traj.transferred_object = nullptr;

                    timetable.add_trajectory(retract_traj);

                    std::cout << "Added retraction for " << r->name << ": reversed " << (num_waypoints - retract_start_idx)
                              << " waypoints over ~" << retract_dist << "m (yaw unchanged). Duration: " << current_retract_time << "s" << std::endl;

                    // Update for next segments (now starts from retracted position)
                    end_time += current_retract_time;
                }
            }

        }
    }
    show_results(argc, argv, timetable, entities, params);


    // Clean up memory (optional, since program ends)
    for (auto& pair : entities) {
        delete pair.second;
    }

    return 0;
}
