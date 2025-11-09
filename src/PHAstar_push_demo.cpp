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

std::unordered_map<std::string, EntityMeta*> initialize_entities() {
    static RobotMeta robot1;
    robot1.name = "robot1";
    robot1.type = EntityType::ROBOT;
    robot1.initial_pose = {1.0, 1.0, 0.0};
    robot1.size = {0.4, 0.2, 0.3};
    robot1.min_turning_radius = 1.0;  // Example
    robot1.wheel_base = 0.4;
    robot1.speed_transit = 0.2;
    robot1.speed_transfer = 0.15;

    static RobotMeta robot2;
    robot2.name = "robot2";
    robot2.type = EntityType::ROBOT;
    robot2.initial_pose = {1.0, 2.0, 0.0};
    robot2.size = {0.4, 0.2, 0.3};
    robot2.min_turning_radius = 1.0;
    robot2.wheel_base = 0.4;
    robot2.speed_transit = 0.2;
    robot2.speed_transfer = 0.15;

    static ObjectMeta obj1;
    obj1.name = "obj1";
    obj1.type = EntityType::OBJECT;
    obj1.initial_pose = {1.2, 1.0, 0};
    obj1.size = {0.075, 0.075, 0.15};
    obj1.goal_pose = {4.6, 4.0, M_PI/2};

    static ObjectMeta obj2;
    obj2.name = "obj2";
    obj2.type = EntityType::OBJECT;
    obj2.initial_pose = {3.5, 2.4, 0.0};
    obj2.size = {0.075, 0.075, 0.15};
    obj2.goal_pose = {3.6, 3.0, M_PI / 2};

    return {
        {"robot1", &robot1}, {"robot2", &robot2}, {"obj1", &obj1}, {"obj2", &obj2}
    };
}

std::vector<std::tuple<std::string, Pose, bool, std::string, double>> initialize_plans() {
    return {
            {"robot1", {3.0, 2.8, M_PI / 2}, true, "obj1", 0.0},
            {"robot1", {3.0, 2.4, M_PI / 2}, false, "obj1", 19.0},
            {"robot2", {2, 3.3, 0}, false, "", 0.0},
            {"robot2", {2.5, 3.3, 0.0}, false, "obj1", 22},
            {"robot2", {3.5, 3.3, 0.0}, true, "obj1", 25},
            };
}


int main(int argc, char** argv) {
    //////// test loading sequence from file ////////
    std::string filename = std::string(CMAKE_SOURCE_DIR) + "/test_sequence.b64";

    std::cout << filename << std::endl;

    std::vector<FinalAllocation> loadedSequence = loadFinalSequenceFromFile(filename);

    if (loadedSequence.empty()) {
        std::cerr << "Failed to load finalSequence from " << filename << std::endl;
        return 1;
    }

    std::cout << "Loaded " << loadedSequence.size() << " allocations from " << filename << std::endl;
    ////////


    Params params = initialize_params();

    auto entities = initialize_entities();

    auto plans = initialize_plans();

    TimeTable timetable(0.5);
    timetable.add_initial(entities);

    auto all_trajectories = perform_planning(entities, plans, timetable, params);

    print_timetable_poses(entities, all_trajectories, timetable);

    show_results(argc, argv, timetable, entities, all_trajectories, params);

    return 0;
}
