#ifndef RELOPUSH_H
#define RELOPUSH_H


#include <vector>
#include <memory>
#include <unordered_map>
#include <variant>     // for std::variant

enum class VertexType
{
    OBJECT_VERTEX,
    GOAL_VERTEX
};

enum class ConnectionMode
{
    NONE,          // no valid mode
    NORMAL_MODE,   // "normalMode"
    PRE_RELOCATION // "preRelocation"
};

enum StateValidity
{
    valid,
    collision,
    out_of_boundary,
    no_approach
};

namespace ReloPush
{
struct State
{
    double x;
    double y;
    double yaw;
    float time;
    float vel;
    bool is_pushing;
};

typedef std::vector<State> StatePath;
typedef std::shared_ptr<StatePath> StatePathPtr;
}
struct ObjectInfo
{
    std::string name;
    double x;
    double y;
    double nominalOrientation;
    int numberOfSides;
    double enclosingRadius;
};

class ObjectMap : public std::unordered_map<std::string, ObjectInfo>
{
public:

    // Default constructor
    ObjectMap() = default;

    // Constructor from std::unordered_map
    ObjectMap(const std::unordered_map<std::string, ObjectInfo>& other)
        : std::unordered_map<std::string, ObjectInfo>(other.begin(), other.end()) {}
};

struct VertexData
{
    VertexType type;
    std::string name;
    int orientationIndex;
    double nominalOrientation;
    double x;
    double y;
    int numberOfSides;
    double radius;
};

struct PreRelocationInfo
{
    bool used;
    double xRelocated_robot;
    double yRelocated_robot;
    double yawReloacted_robot;
    double xRelocated_object;
    double yRelocated_object;
    double yawRelocated_object;

    double extraCost;
    int relocatingIndex;
    StateValidity reason;
};

using EdgePathTypes = std::variant<std::monostate, ReloPush::StatePathPtr>; // Simplified, ignoring reloDubinsPath

struct EdgePath
{
    bool is_pushing;
    EdgePathTypes path;
};

typedef std::shared_ptr<EdgePath> EdgePathPtr;

struct EdgeData
{
    double weight;
    VertexData srcVertexData;
    VertexData sinkVertexData;
    ConnectionMode mode;
    PreRelocationInfo preRelo;
    std::vector<EdgePathPtr> paths;
};

struct WorkspaceBoundary
{
    double xMin;
    double yMin;
    double xMax;
    double yMax;
};

struct TurningRadiusPair
{
    float push;
    float non_push;
};

struct PlanningParameters
{
    WorkspaceBoundary boundary;
    TurningRadiusPair turning_rad_pair;
    float  map_resolution;
    float car_width;
    float obs_rad;
    float LF_push;
    float LF_nonpush;
    float LB;
    double PrePush_dist;
};

struct PlanningContext
{
    PlanningParameters parameters;
    ObjectMap mo_list;
    std::unordered_map<std::string, ObjectInfo> delivered_list;
    int sample_N;
    std::int64_t timeout_ms;
    bool print_res;
    bool use_prelo_optimization;
    size_t num_of_obj;
    bool no_init_guess;
    std::vector<ReloPush::State> sampledPositions;
};

struct FinalAllocation
{
    ObjectInfo object;
    ObjectInfo goal;
    double cost;
    int row;
    int col;
    std::vector<VertexData> vertexChain;
    ReloPush::State startPose;
    ReloPush::State goalPose;
    PlanningContext snapshot;
    std::vector<EdgeData> paths;
    std::vector<ReloPush::StatePathPtr> edgeTransitPaths;
    ReloPush::StatePathPtr firstApproachPath;
    std::shared_ptr<std::vector<EdgePath>> obsReloPaths;
    std::unordered_map<std::string, ReloPush::State> obsReloUpdate;
};

#endif // RELOPUSH_H
