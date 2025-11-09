#ifndef LOADFINALSEQUENCE_H
#define LOADFINALSEQUENCE_H

#include <string>
#include <cstddef>     // for std::size_t
#include <cstdint>     // for std::int64_t, std::uint64_t
#include <limits>      // for numeric_limits
#include <stdexcept>   // for runtime_error
#include <iostream>
#include <sstream>
#include <fstream>

#include <base64.h>
#include <ReloPush.h>

// Project headers in ReloPush
/*
#include <GraphData.hpp>
#include <ObjectInfo.hpp>
#include <State.h>
#include <TaskAllocation.hpp>
#include <PlanningContext.hpp>
#include <PathPlanningTools.h>
#include "PlanResult.hpp"
#include <DubinsTools.h>
*/

// for debugging deserialization
void dump_remaining(std::istream &is)
{
    std::streampos current_pos = is.tellg();
    is.seekg(0, is.end);
    std::streampos end_pos = is.tellg();
    is.seekg(current_pos); // Reset to original position

    std::streamsize remaining_size = end_pos - current_pos;
    std::cout << "Remaining bytes: " << remaining_size << std::endl;

    /* prints remaining binary
    if (remaining_size > 0)
    {
        std::string buffer(remaining_size, '\0');
        is.read(&buffer[0], remaining_size);

        std::cout << "Hex dump of remaining data: ";
        for (unsigned char c : buffer)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
        }
        std::cout << std::dec << std::endl; // Reset to decimal

        // Reset position again for continued deserialization
        is.seekg(current_pos);
    }
    else
    {
        std::cout << "No remaining data." << std::endl;
    }
        */
}

namespace Serialization
{
    // Declarations for deserialize
    void deserialize(std::istream &is, int &value);
    void deserialize(std::istream &is, double &value);
    void deserialize(std::istream &is, float &value);
    void deserialize(std::istream &is, bool &value);
    void deserialize(std::istream &is, std::size_t &value);
    void deserialize(std::istream &is, std::int64_t &value);
    void deserialize(std::istream &is, std::string &value);

    void deserialize(std::istream &is, VertexType &value);
    void deserialize(std::istream &is, ConnectionMode &value);
    void deserialize(std::istream &is, StateValidity &value);

    void deserialize(std::istream &is, ReloPush::State &s);
    void deserialize(std::istream &is, ObjectInfo &oi);
    void deserialize(std::istream &is, VertexData &vd);
    void deserialize(std::istream &is, PreRelocationInfo &pri);
    void deserialize(std::istream &is, EdgePath &ep);
    void deserialize(std::istream &is, EdgeData &ed);
    void deserialize(std::istream &is, WorkspaceBoundary &wb);
    void deserialize(std::istream &is, TurningRadiusPair &trp);
    void deserialize(std::istream &is, PlanningParameters &pp);
    void deserialize(std::istream &is, PlanningContext &pc);
    void deserialize(std::istream &is, FinalAllocation &fa);

    void deserialize(std::istream &is, ObjectMap &value);

    // Template declarations for deserialize
    template <typename T>
    void deserialize(std::istream &is, std::vector<T> &value);

    template <typename T>
    void deserialize(std::istream &is, std::shared_ptr<T> &value);

    template <typename V>
    void deserialize(std::istream &is, std::unordered_map<std::string, V> &value);

    // Deserialization implementations
    inline void deserialize(std::istream &is, int &value)
    {
        is.read(reinterpret_cast<char *>(&value), sizeof(value));
    }

    inline void deserialize(std::istream &is, double &value)
    {
        is.read(reinterpret_cast<char *>(&value), sizeof(value));
    }

    inline void deserialize(std::istream &is, float &value)
    {
        is.read(reinterpret_cast<char *>(&value), sizeof(value));
    }

    inline void deserialize(std::istream &is, bool &value)
    {
        char v;
        is.read(&v, 1);
        value = (v != 0);
    }

    inline void deserialize(std::istream &is, std::size_t &value)
    {
        std::uint64_t v;
        is.read(reinterpret_cast<char *>(&v), sizeof(v));
        if (v > std::numeric_limits<std::size_t>::max())
        {
            throw std::runtime_error("Deserialized size_t too large");
        }
        value = static_cast<std::size_t>(v);
    }

    inline void deserialize(std::istream &is, std::int64_t &value)
    {
        is.read(reinterpret_cast<char *>(&value), sizeof(value));
    }

    inline void deserialize(std::istream &is, std::string &value)
    {
        std::size_t len;
        deserialize(is, len);
        value.resize(len);
        is.read(&value[0], static_cast<std::streamsize>(len));
    }

    inline void deserialize(std::istream &is, VertexType &value)
    {
        int v;
        deserialize(is, v);
        value = static_cast<VertexType>(v);
    }

    inline void deserialize(std::istream &is, ConnectionMode &value)
    {
        int v;
        deserialize(is, v);
        value = static_cast<ConnectionMode>(v);
    }

    inline void deserialize(std::istream &is, StateValidity &value)
    {
        int v;
        deserialize(is, v);
        value = static_cast<StateValidity>(v);
    }

    inline void deserialize(std::istream &is, ReloPush::State &s)
    {
        deserialize(is, s.x);
        deserialize(is, s.y);
        deserialize(is, s.yaw);
        deserialize(is, s.time);
        deserialize(is, s.vel);
        deserialize(is, s.is_pushing);
    }

    inline void deserialize(std::istream &is, ObjectInfo &oi)
    {
        deserialize(is, oi.name);
        deserialize(is, oi.x);
        deserialize(is, oi.y);
        deserialize(is, oi.nominalOrientation);
        deserialize(is, oi.numberOfSides);
        deserialize(is, oi.enclosingRadius);
    }

    inline void deserialize(std::istream &is, VertexData &vd)
    {
        deserialize(is, vd.type);
        deserialize(is, vd.name);
        deserialize(is, vd.orientationIndex);
        deserialize(is, vd.nominalOrientation);
        deserialize(is, vd.x);
        deserialize(is, vd.y);
        deserialize(is, vd.numberOfSides);
        deserialize(is, vd.radius);
    }

    inline void deserialize(std::istream &is, PreRelocationInfo &pri)
    {
        deserialize(is, pri.used);
        deserialize(is, pri.xRelocated_robot);
        deserialize(is, pri.yRelocated_robot);
        deserialize(is, pri.yawReloacted_robot);
        deserialize(is, pri.xRelocated_object);
        deserialize(is, pri.yRelocated_object);
        deserialize(is, pri.yawRelocated_object);
        deserialize(is, pri.extraCost);
        deserialize(is, pri.relocatingIndex);
        deserialize(is, pri.reason);
    }

    inline void deserialize(std::istream &is, EdgePath &ep)
    {
        deserialize(is, ep.is_pushing);
        ReloPush::StatePath sp;
        deserialize(is, sp);
        ep.path = std::make_shared<ReloPush::StatePath>(sp);
    }

    inline void deserialize(std::istream &is, EdgeData &ed)
    {
        deserialize(is, ed.weight);
        deserialize(is, ed.srcVertexData);
        deserialize(is, ed.sinkVertexData);
        deserialize(is, ed.mode);
        deserialize(is, ed.preRelo);
        deserialize(is, ed.paths);
    }

    inline void deserialize(std::istream &is, WorkspaceBoundary &wb)
    {
        deserialize(is, wb.xMin);
        deserialize(is, wb.yMin);
        deserialize(is, wb.xMax);
        deserialize(is, wb.yMax);
    }

    inline void deserialize(std::istream &is, TurningRadiusPair &trp)
    {
        deserialize(is, trp.push);
        deserialize(is, trp.non_push);
    }

    inline void deserialize(std::istream &is, PlanningParameters &pp)
    {
        deserialize(is, pp.boundary);
        deserialize(is, pp.turning_rad_pair);
        deserialize(is, pp.map_resolution);
        deserialize(is, pp.car_width);
        deserialize(is, pp.obs_rad);
        deserialize(is, pp.LF_push);
        deserialize(is, pp.LF_nonpush);
        deserialize(is, pp.LB);
        deserialize(is, pp.PrePush_dist);
    }

    inline void deserialize(std::istream &is, PlanningContext &pc)
    {
        deserialize(is, pc.parameters);
        deserialize(is, pc.mo_list);
        deserialize(is, pc.delivered_list);
        deserialize(is, pc.sample_N);
        deserialize(is, pc.timeout_ms);
        deserialize(is, pc.print_res);
        deserialize(is, pc.use_prelo_optimization);
        deserialize(is, pc.num_of_obj);
        deserialize(is, pc.no_init_guess);
        deserialize(is, pc.sampledPositions);
    }

    inline void deserialize(std::istream &is, FinalAllocation &fa)
    {
        deserialize(is, fa.object);
        deserialize(is, fa.goal);
        deserialize(is, fa.cost);
        deserialize(is, fa.row);
        deserialize(is, fa.col);
        deserialize(is, fa.vertexChain);
        deserialize(is, fa.startPose);
        deserialize(is, fa.goalPose);
        deserialize(is, fa.snapshot);
        deserialize(is, fa.paths);
        deserialize(is, fa.edgeTransitPaths);
        deserialize(is, fa.firstApproachPath);
        deserialize(is, fa.obsReloPaths);
        deserialize(is, fa.obsReloUpdate);
    }

    // For ObjectMap
    inline void deserialize(std::istream &is, ObjectMap &value)
    {
        std::unordered_map<std::string, ObjectInfo> temp;
        deserialize(is, temp);
        value = ObjectMap(temp);
    }

    // Template implementations for deserialize
    template <typename T>
    void deserialize(std::istream &is, std::vector<T> &value)
    {
        std::size_t len;
        deserialize(is, len);
        value.resize(len);
        for (auto &item : value)
        {
            deserialize(is, item);
        }
    }

    template <typename T>
    void deserialize(std::istream &is, std::shared_ptr<T> &value)
    {
        bool is_null;
        deserialize(is, is_null);
        if (is_null)
        {
            value = nullptr;
        }
        else
        {
            value = std::make_shared<T>();
            deserialize(is, *value);
        }
    }

    template <typename V>
    void deserialize(std::istream &is, std::unordered_map<std::string, V> &value)
    {
        std::size_t len;
        deserialize(is, len);
        value.clear();
        for (std::size_t i = 0; i < len; ++i)
        {
            std::string key;
            V val;
            deserialize(is, key);
            deserialize(is, val);
            value[key] = val;
        }
    }

} // namespace Serialization

inline std::vector<FinalAllocation> deserializeFinalSequence(const std::string &data)
{
    std::istringstream iss(data, std::ios::binary);
    std::size_t size;
    Serialization::deserialize(iss, size);
    std::vector<FinalAllocation> fs(size);
    for (auto &fa : fs)
    {
        Serialization::deserialize(iss, fa);
    }
    return fs;
}

std::vector<FinalAllocation> loadFinalSequenceFromFile(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile) {
        std::cerr << "Error: Could not open file " << filename << " for reading." << std::endl;
        return {};
    }
    std::string base64Data((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
    inFile.close();
    std::string binaryData = base64_decode(base64Data);
    return deserializeFinalSequence(binaryData);
}


#endif
