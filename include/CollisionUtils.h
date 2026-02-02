#ifndef COLLISIONUTILS_H
#define COLLISIONUTILS_H

#include <Entities.h>
#include <Params.h>
#include <Utils.h>
#include <unordered_map>
#include <string>

// ==========================================
// COLLISION GEOMETRY STRUCTURE
// ==========================================

/**
 * @brief Precomputed collision geometry for efficient collision checking
 */
struct CollisionGeometry {
    Corners corners;           // Inflated corner positions
    double diagonal_radius;    // Diagonal radius for fast circle-based checks
};

// ==========================================
// COLLISION RESULT STRUCTURES
// ==========================================

/**
 * @brief Result of collision check against a single entity
 */
struct EntityCollisionResult {
    bool has_collision;
    EntityMeta* colliding_entity;
};

/**
 * @brief Result of collision check against multiple entities
 */
struct MultiEntityCollisionResult {
    bool has_collision;
    EntityMeta* colliding_entity;
    std::string collision_type;  // "Robot", "Object", "Boundary", etc.
};

// ==========================================
// UTILITY FUNCTIONS
// ==========================================

/**
 * @brief Compute collision geometry (corners and diagonal radius) for an entity
 * 
 * @param pose Entity pose (x, y, yaw)
 * @param size Entity size dimensions
 * @param inflation_factor Inflation multiplier for safety margins
 * @return CollisionGeometry containing corners and diagonal radius
 */
inline CollisionGeometry setup_collision_geometry(
    const Pose& pose,
    const OccuRect& size,
    double inflation_factor = 1.0
) {
    CollisionGeometry geom;
    
    double front_inf = size.front_length * inflation_factor;
    double rear_inf = size.rear_length * inflation_factor;
    double width_inf = size.width * inflation_factor;
    
    geom.corners = get_corners(pose.x, pose.y, pose.yaw, front_inf, rear_inf, width_inf);
    geom.diagonal_radius = std::sqrt((front_inf + rear_inf) * (front_inf + rear_inf) + 
                                      width_inf * width_inf) / 2.0;
    
    return geom;
}

/**
 * @brief Check if corners are within world bounds
 * 
 * @param corners Corner positions to check
 * @param min_x Minimum x boundary
 * @param max_x Maximum x boundary
 * @param min_y Minimum y boundary
 * @param max_y Maximum y boundary
 * @return true if within bounds, false otherwise
 */
inline bool check_bounds_collision(
    const Corners& corners,
    double min_x, double max_x,
    double min_y, double max_y
) {
    // Returns true if ANY corner is out of bounds (collision with boundary)
    for (const auto& c : corners) {
        if (!(min_x <= c.x && c.x <= max_x && min_y <= c.y && c.y <= max_y)) {
            return true;  // Out of bounds = collision
        }
    }
    return false;  // All corners within bounds = no collision
}

/**
 * @brief Check collision between subject entity and another entity
 * 
 * Performs two-stage collision detection:
 * 1. Fast circle-based distance check using diagonal radii
 * 2. Precise rectangle intersection check if circles overlap
 * 
 * @param subject_geom Collision geometry of the subject entity
 * @param subject_pose Pose of the subject entity
 * @param other_entity Pointer to the other entity
 * @param other_pose Pose of the other entity
 * @param params System parameters (inflation, safety_margin, etc.)
 * @return EntityCollisionResult indicating collision status
 */
inline EntityCollisionResult check_entity_collision(
    const CollisionGeometry& subject_geom,
    const Pose& subject_pose,
    EntityMeta* other_entity,
    const Pose& other_pose,
    const Params& params
) {
    EntityCollisionResult result{false, nullptr};
    
    // Fast circle-based check
    double dist = std::hypot(subject_pose.x - other_pose.x, 
                            subject_pose.y - other_pose.y);
    
    double other_diag = std::sqrt(
        std::pow(other_entity->size.front_length + other_entity->size.rear_length, 2) + 
        std::pow(other_entity->size.width, 2)
    ) / 2.0 * params.inflation;
    
    if (dist <= subject_geom.diagonal_radius + other_diag + params.safety_margin) {
        // Precise rectangle intersection check
        Corners other_corners = get_corners(
            other_pose.x, other_pose.y, other_pose.yaw,
            other_entity->size.front_length * params.inflation,
            other_entity->size.rear_length * params.inflation,
            other_entity->size.width * params.inflation
        );
        
        if (rectangles_intersect(subject_geom.corners, other_corners)) {
            result.has_collision = true;
            result.colliding_entity = other_entity;
        }
    }
    
    return result;
}

/**
 * @brief Check collision against multiple entities with advanced filtering
 * 
 * Supports checking both a robot body and an optional transferred object
 * against a set of entities at a specific time.
 * 
 * @param robot_geom Collision geometry of the robot
 * @param robot_pose Pose of the robot
 * @param object_geom Collision geometry of transferred object (nullptr if not transferring)
 * @param object_pose Pose of transferred object (nullptr if not transferring)
 * @param entities_at_time Map of entities and their poses at the check time
 * @param params System parameters
 * @param robot_entity Pointer to the robot entity (to skip self-collision)
 * @param transferred_entity Pointer to transferred object (to skip)
 * @param ignored_entity Optional pointer to an entity to ignore
 * @return MultiEntityCollisionResult with collision details
 */
inline MultiEntityCollisionResult check_multiple_entities_collision(
    const CollisionGeometry& robot_geom,
    const Pose& robot_pose,
    const CollisionGeometry* object_geom,
    const Pose* object_pose,
    const std::unordered_map<EntityMeta*, Pose>& entities_at_time,
    const Params& params,
    EntityMeta* robot_entity,
    EntityMeta* transferred_entity = nullptr,
    EntityMeta* ignored_entity = nullptr
) {
    MultiEntityCollisionResult result{false, nullptr, ""};
    EntityMeta* first_robot_collider = nullptr;
    
    for (const auto& [ent, pose] : entities_at_time) {
        // Skip self, transferred object, and ignored entity
        if (ent == robot_entity || ent == transferred_entity || ent == ignored_entity) {
            continue;
        }
        
        // Check robot body collision
        auto robot_collision = check_entity_collision(robot_geom, robot_pose, ent, pose, params);
        if (robot_collision.has_collision) {
             if (ent->type != EntityType::ROBOT) {
                 // HARD COLLISION (Static/Wall/Object) - Return IMMEDIATELY
                 result.has_collision = true;
                 result.colliding_entity = ent;
                 result.collision_type = "Object"; // Or Static/Wall
                 return result;
             } else {
                 // SOFT COLLISION (Robot) - Store and continue checking for Hard Collisions
                 if (!first_robot_collider) first_robot_collider = ent;
             }
        }
        
        // Check transferred object collision (if applicable)
        if (object_geom && object_pose && transferred_entity) {
            auto object_collision = check_entity_collision(*object_geom, *object_pose, ent, pose, params);
            if (object_collision.has_collision) {
                 // Object collision is usually considered Hard if it hits another object/wall.
                 // If it hits a robot, strictly speaking it's also a "Soft" collision we could potentially clear?
                 // For now, let's assume pushing object vs robot is also Resolvable (Soft).
                 if (ent->type != EntityType::ROBOT) {
                     result.has_collision = true;
                     result.colliding_entity = ent;
                     result.collision_type = "Object";
                     return result;
                 } else {
                     if (!first_robot_collider) first_robot_collider = ent;
                 }
            }
        }
    }
    
    // If we finished loop and found a robot collider but no hard collider
    if (first_robot_collider) {
        result.has_collision = true;
        result.colliding_entity = first_robot_collider;
        result.collision_type = "Robot";
        return result;
    }
    
    return result;
}

/**
 * @brief Simplified collision check for a single entity (robot only, no object)
 * 
 * @param robot_geom Collision geometry of the robot
 * @param robot_pose Pose of the robot
 * @param entities_at_time Map of entities and their poses at the check time
 * @param params System parameters
 * @param robot_entity Pointer to the robot entity (to skip self-collision)
 * @param ignored_entity Optional pointer to an entity to ignore
 * @return true if collision detected, false otherwise
 */
inline bool check_simple_collision(
    const CollisionGeometry& robot_geom,
    const Pose& robot_pose,
    const std::unordered_map<EntityMeta*, Pose>& entities_at_time,
    const Params& params,
    EntityMeta* robot_entity,
    EntityMeta* ignored_entity = nullptr
) {
    auto result = check_multiple_entities_collision(
        robot_geom, robot_pose,
        nullptr, nullptr,  // No transferred object
        entities_at_time,
        params,
        robot_entity,
        nullptr,
        ignored_entity
    );
    
    return result.has_collision;
}

#endif // COLLISIONUTILS_H
