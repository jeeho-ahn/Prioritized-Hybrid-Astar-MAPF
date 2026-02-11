#ifndef LNS_TYPES_H
#define LNS_TYPES_H

#include <string>
#include <vector>

// ==========================================
// Conflict Types & Tracking for LNS
// ==========================================

enum class ConflictType {
    PHYSICAL_BLOCK_VICTIM, // This task was blocked by another
    PHYSICAL_BLOCK_CAUSE,  // This task blocked another (retroactive label)
    TEMPORAL_DELAY_VICTIM, // This task waited for another
    TEMPORAL_DELAY_CAUSE,  // This task caused a delay (retroactive label)
    DEADLOCK_VICTIM,       // This task failed completely due to deadlock
    DEADLOCK_CAUSE         // This task caused a deadlock/trap (retroactive label)
};

struct ConflictRecord {
    ConflictType type;
    int task_index;              // The task that experienced the conflict
    std::string robot_name;      // The robot executing the task
    std::string blocking_entity; // Entity that caused the conflict
    int blocking_task_index;     // Task index of the blocking entity (-1 if idle/unknown)
    double time;                 // Time at which conflict was detected
    std::string detail;          // Human-readable description
};

// Snapshot of one task's allocation result
struct TaskAllocationResult {
    int task_index;
    std::string robot_name;
    bool success;
    double completion_time;      // Time at which the robot finishes this task
    std::vector<ConflictRecord> conflicts; // Conflicts encountered during execution
};

// Complete allocation solution
struct AllocationSolution {
    std::vector<TaskAllocationResult> results;
    std::vector<std::string> robot_assignment;  // robot_assignment[task_i] = robot name ("" = greedy)
    double makespan;                    // Max completion time across all robots
    int total_conflicts;
    int failed_tasks;

    void compute_stats() {
        makespan = 0.0;
        total_conflicts = 0;
        failed_tasks = 0;
        for (const auto& r : results) {
            if (r.success) {
                makespan = std::max(makespan, r.completion_time);
            } else {
                failed_tasks++;
            }
            total_conflicts += static_cast<int>(r.conflicts.size());
        }
    }
};

#endif // LNS_TYPES_H
