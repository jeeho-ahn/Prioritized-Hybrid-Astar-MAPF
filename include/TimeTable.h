#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <iostream>
#include <iomanip>
#include <map>
#include <unordered_map>

#include <Entities.h>
#include <Utils.h>  // Added for mod2pi

class TimeTable {
public:
    double time_increment = 0.5;

    TimeTable(double inc = 0.5) : time_increment(inc) {}

    void add_initial(const std::unordered_map<std::string, EntityMeta*>& entities) {
        for (const auto& [name, ent] : entities) {
            per_entity_table[ent][0.0] = ent->initial_pose;
        }
    }

    void add_trajectory(const Trajectory& traj) {
        EntityMeta* ent = traj.entity;
        if (traj.waypoints.empty()) return;
        double min_relative = traj.waypoints.front().time;
        double max_relative = traj.waypoints.back().time;
        double offset = traj.start_time;
        double absolute_min_t = offset + min_relative;
        double absolute_max_t = offset + max_relative;
        double initial_obj_yaw = 0.0;
        double robot_start_yaw = traj.waypoints.front().yaw;
        if (traj.is_transfer && traj.transferred_object) {
            initial_obj_yaw = get_pose(traj.transferred_object, absolute_min_t).yaw;
        }
        for (double absolute_t = absolute_min_t; absolute_t <= absolute_max_t + time_increment; absolute_t += time_increment) {
            double relative_t = absolute_t - offset;
            if (relative_t >= min_relative && relative_t <= max_relative) {
                Pose p = interpolate_waypoints(traj.waypoints, relative_t);
                per_entity_table[ent][absolute_t] = p;
                if (traj.is_transfer && traj.transferred_object) {
                    Pose obj_p = compute_object_pose(p, ent->size, traj.transferred_object->size);
                    double delta_yaw = mod2pi(p.yaw - robot_start_yaw);
                    obj_p.yaw = mod2pi(initial_obj_yaw + delta_yaw);
                    per_entity_table[traj.transferred_object][absolute_t] = obj_p;
                }
            }
        }
        // Explicitly add the first and last waypoints
        double relative_min = min_relative;
        Pose p_min = interpolate_waypoints(traj.waypoints, relative_min);
        per_entity_table[ent][absolute_min_t] = p_min;
        if (traj.is_transfer && traj.transferred_object) {
            Pose obj_p_min = compute_object_pose(p_min, ent->size, traj.transferred_object->size);
            double delta_yaw = mod2pi(p_min.yaw - robot_start_yaw);
            obj_p_min.yaw = mod2pi(initial_obj_yaw + delta_yaw);
            per_entity_table[traj.transferred_object][absolute_min_t] = obj_p_min;
        }
        double relative_max = max_relative;
        Pose p_max = interpolate_waypoints(traj.waypoints, relative_max);
        per_entity_table[ent][absolute_max_t] = p_max;
        if (traj.is_transfer && traj.transferred_object) {
            Pose obj_p_max = compute_object_pose(p_max, ent->size, traj.transferred_object->size);
            double delta_yaw = mod2pi(p_max.yaw - robot_start_yaw);
            obj_p_max.yaw = mod2pi(initial_obj_yaw + delta_yaw);
            per_entity_table[traj.transferred_object][absolute_max_t] = obj_p_max;
        }
    }

    Pose get_pose(EntityMeta* ent, double t) const {
        auto it = per_entity_table.find(ent);
        if (it == per_entity_table.end() || it->second.empty()) {
            return ent->initial_pose;
        }
        const auto& m = it->second;
        auto it_upper = m.upper_bound(t);
        if (it_upper == m.begin()) {
            return m.begin()->second;
        }
        if (it_upper == m.end()) {
            auto it_last = m.end();
            --it_last;
            return it_last->second;
        }
        auto it_lower = it_upper;
        --it_lower;
        if (it_lower->first == t) {
            return it_lower->second;
        }
        return interpolate_pose(it_lower->second, it_lower->first, it_upper->second, it_upper->first, t);
    }

    std::unordered_map<EntityMeta*, Pose> get_poses(double t) const {
        std::unordered_map<EntityMeta*, Pose> poses;
        for (const auto& [ent, m] : per_entity_table) {
            poses[ent] = get_pose(ent, t);
        }
        return poses;
    }

    double get_max_time() const {
        double max_t = 0.0;
        for (const auto& [ent, m] : per_entity_table) {
            if (!m.empty()) {
                max_t = std::max(max_t, m.rbegin()->first);
            }
        }
        return max_t;
    }

    double get_entity_max_time(EntityMeta* ent, double margin=0.5) const {
        auto it = per_entity_table.find(ent);
        if (it == per_entity_table.end() || it->second.empty()) {
            return 0.0;
        }
        return it->second.rbegin()->first + margin;
    }

    static Pose compute_object_pose(const Pose& robot_pose, const OccuRect& robot_size, const OccuRect& obj_size) {
        Pose obj_pose;
        double offset = robot_size.front_length + obj_size.rear_length;
        obj_pose.x = robot_pose.x + offset * std::cos(robot_pose.yaw);
        obj_pose.y = robot_pose.y + offset * std::sin(robot_pose.yaw);
        obj_pose.yaw = robot_pose.yaw;
        return obj_pose;
    }

    static Pose interpolate_waypoints(const std::vector<Waypoint>& waypoints, double t) {
        if (waypoints.empty()) {
            return {};
        }
        if (t <= waypoints.front().time) return waypoints.front();
        if (t >= waypoints.back().time) return waypoints.back();
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            if (waypoints[i].time <= t && t <= waypoints[i + 1].time) {
                return interpolate_pose(waypoints[i], waypoints[i].time, waypoints[i + 1], waypoints[i + 1].time, t);
            }
        }
        return waypoints.back();
    }

private:
    std::unordered_map<EntityMeta*, std::map<double, Pose>> per_entity_table;

    static Pose interpolate_pose(const Pose& p1, double t1, const Pose& p2, double t2, double t) {
        if (t1 == t2) return p1;
        double frac = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + frac * (p2.x - p1.x);
        p.y = p1.y + frac * (p2.y - p1.y);
        //double dyaw = mod2pi(p2.yaw - p1.yaw); // mod2pi causes funny spinning
        //p.yaw = mod2pi(p1.yaw + frac * dyaw);
        double dyaw = pi_2_pi(p2.yaw - p1.yaw);
        double yaw = p1.yaw + frac * dyaw;
        p.yaw = pi_2_pi(yaw);
        p.yaw = mod2pi(p.yaw);
        return p;
    }


};

void print_timetable_poses(const std::unordered_map<std::string, EntityMeta*>& entities, const std::vector<Trajectory>& all_trajectories, const TimeTable& timetable)
{
    std::cout << "TimeTable poses for robot2:" << std::endl;
    EntityMeta* robot2_ent = entities.at("robot2");
    double max_tt = 0.0;
    for (const auto& traj : all_trajectories) {
        if (!traj.waypoints.empty()) {
            max_tt = std::max(max_tt, traj.waypoints.back().time);
        }
    }
    for (double t = 0.0; t <= max_tt + 1e-6; t += 0.1) {  // Step of 0.1 for fine-grained view
        Pose p = timetable.get_pose(robot2_ent, t);
        std::cout << "t=" << std::fixed << std::setprecision(4) << t
                  << ", x=" << p.x << ", y=" << p.y << ", yaw=" << p.yaw << std::endl;
    }

}

#endif // TIMETABLE_H
