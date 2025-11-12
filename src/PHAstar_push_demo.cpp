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

const bool print_path = false;

Params initialize_params() {
    Params params;
    params.analytic_threshold = 5.0 * params.max_steer;  // Adjust if needed
    return params;
}

enum DependType { TRANSIT, TRANSFER };

class Task {
public:
    std::vector<Trajectory> transitTrajectoryRobot;
    std::vector<Trajectory> transferTrajectoryRobot;
    Pose transitGoal;
    Pose StartPoseObj;
    Pose GoalPoseObj;
    std::pair<Task*, DependType> transferDepend;
    std::pair<Task*, DependType> transitDepend;
    RobotMeta* assignedRobot = nullptr;
    ObjectMeta* targetObject = nullptr;

    Task(const FinalAllocation& fa, const std::unordered_map<std::string, EntityMeta*>& entities) {
        StartPoseObj = {fa.startPose.x, fa.startPose.y, fa.startPose.yaw};
        GoalPoseObj = {fa.goalPose.x, fa.goalPose.y, fa.goalPose.yaw};
        targetObject = dynamic_cast<ObjectMeta*>(entities.at(fa.object.name));
    }

    Pose calcRobotPoseFromObj(const Pose& pose_in) {
        if (!assignedRobot || !targetObject) {
            std::cerr << "Assigned robot or target object not set." << std::endl;
            return {0.0, 0.0, 0.0};
        }
        double offset = assignedRobot->size.front_length + targetObject->size.rear_length + 0.1;
        Pose robot_pose;
        robot_pose.x = pose_in.x - offset * std::cos(pose_in.yaw);
        robot_pose.y = pose_in.y - offset * std::sin(pose_in.yaw);
        robot_pose.yaw = pose_in.yaw;
        return robot_pose;
    }

    Pose calcStartPoseRobot() {
        if (!assignedRobot || !targetObject) {
            std::cerr << "Assigned robot or target object not set." << std::endl;
            return {0.0, 0.0, 0.0};
        }
        Pose attached = calcRobotPoseFromObj(StartPoseObj);
        double extra_offset = 0.05;
        attached.x -= extra_offset * std::cos(attached.yaw);
        attached.y -= extra_offset * std::sin(attached.yaw);
        return attached;
    }
};

std::unordered_map<std::string, EntityMeta*> initialize_entities(const std::vector<FinalAllocation>& loadedSequence) {
    std::unordered_map<std::string, EntityMeta*> entities;

    // Initialize robots manually
    RobotMeta* robot1 = new RobotMeta;
    robot1->name = "robot1";
    robot1->type = EntityType::ROBOT;
    robot1->initial_pose = {1.0, 1.0, 0.0};
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
    robot2->initial_pose = {1.0, 4.0, 0.0};
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

int main(int argc, char** argv) {
    //////// test loading sequence from file ////////
    std::string filename = std::string(CMAKE_SOURCE_DIR) + "/test_sequence2.b64";

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

    std::vector<Task> tasks;
    for (const auto& fa : loadedSequence) {
        tasks.emplace_back(fa, entities);
    }

    // Allocate first m tasks to m robots (m=2)
    std::vector<std::string> robots = {"robot1", "robot2"};
    size_t m = robots.size();
    for (size_t i = 0; i < std::min(m, tasks.size()); ++i) {
        tasks[i].assignedRobot = dynamic_cast<RobotMeta*>(entities.at(robots[i]));
        std::cout << "Assigned task for object " << tasks[i].targetObject->name << " to " << robots[i] << std::endl;
    }

    // Construct plans based on tasks
    std::vector<std::tuple<std::string, Pose, bool, std::string, double>> plans;
    for (auto& task : tasks) {
        if (!task.assignedRobot) continue;

        std::string r_name = task.assignedRobot->name;
        std::string obj_name = task.targetObject->name;

        // Transit to start pose
        Pose start_robot = task.calcStartPoseRobot();
        plans.emplace_back(r_name, start_robot, false, "", 0.0);
        std::cout << "Planning transit for " << r_name << " to start pose for " << obj_name << std::endl;

        // Transfer to goal
        Pose goal_robot = task.calcRobotPoseFromObj(task.GoalPoseObj);
        plans.emplace_back(r_name, goal_robot, true, obj_name, 0.0);
        std::cout << "Planning transfer for " << r_name << " with " << obj_name << " to goal" << std::endl;

        // Set goal pose for object
        task.targetObject->goal_pose = task.GoalPoseObj;
    }

    for (auto& task : tasks) {
        if (task.assignedRobot) {
            Pose start_robot = task.calcStartPoseRobot();
            Pose attached_start = task.calcRobotPoseFromObj(task.StartPoseObj);
            Pose goal_robot = task.calcRobotPoseFromObj(task.GoalPoseObj);
            std::cout << "Task for " << task.targetObject->name << " with " << task.assignedRobot->name << ":"
                      << "\n  StartPoseObj=(x=" << task.StartPoseObj.x << ", y=" << task.StartPoseObj.y << ", yaw=" << task.StartPoseObj.yaw << ")"
                      << "\n  CalcStartPoseRobot (transit goal, 0.05m behind attached)=(x=" << start_robot.x << ", y=" << start_robot.y << ", yaw=" << start_robot.yaw << ")"
                      << "\n  Attached start pose for pushing=(x=" << attached_start.x << ", y=" << attached_start.y << ", yaw=" << attached_start.yaw << ")"
                      << "\n  GoalPoseObj=(x=" << task.GoalPoseObj.x << ", y=" << task.GoalPoseObj.y << ", yaw=" << task.GoalPoseObj.yaw << ")"
                      << "\n  Calc goal robot pose=(x=" << goal_robot.x << ", y=" << goal_robot.y << ", yaw=" << goal_robot.yaw << ")" << std::endl;
        }
    }

    TimeTable timetable(0.5);
    timetable.add_initial(entities);

    auto all_trajectories = perform_planning(entities, plans, timetable, params, print_path);

    print_timetable_poses(entities, all_trajectories, timetable);

    show_results(argc, argv, timetable, entities, all_trajectories, params);

    // Clean up memory (optional, since program ends)
    for (auto& pair : entities) {
        delete pair.second;
    }

    return 0;
}
