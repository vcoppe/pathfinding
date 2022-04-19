#ifndef SAFE_INTERVAL_PATH_PLANNING_HPP
#define SAFE_INTERVAL_PATH_PLANNING_HPP

#include <set>
#include <unordered_map>
#include <unordered_set>

#include <boost/functional/hash.hpp>

#include "graph.hpp"
#include "reservation_table.hpp"
#include "reverse_resumable_a_star.hpp"
#include "structs.hpp"

class SafeIntervalPathPlanning
{
private:
    struct State
    {
        int vertex, intervalId;
        Interval interval;
        double g, h;

        bool operator==(const State &other) const
        {
            return vertex == other.vertex && intervalId == other.intervalId;
        }

        bool operator<(const State &other) const
        {
            return g + h < other.g + other.h;
        }
    };

    struct StateHasher
    {
        std::size_t operator()(const State &state) const
        {
            std::size_t seed = 0;
            boost::hash_combine(seed, state.vertex);
            boost::hash_combine(seed, state.intervalId);
            return seed;
        }
    };

    Graph graph;
    const std::vector<Mobile> mobiles;
    std::shared_ptr<ReservationTable> reservationTable;
    std::shared_ptr<ReverseResumableAStar> reverseResumableAStar;
    std::unordered_map<int, Path> paths;
    std::multiset<State> queue;
    std::unordered_map<State, double, StateHasher> distance;
    std::unordered_set<State, StateHasher> visited;
    std::unordered_map<State, State, StateHasher> parent;
    std::vector<State> successors;

    Path findPath(int mobile, int from, int to, double start);
    void getSuccessors(int mobile, const State &state);
    Path getPath(int mobile, const State &state);
    void reservePath(int mobile, const Path &path);
public:
    SafeIntervalPathPlanning(const Graph &graph, const std::vector<Mobile> &mobiles,
        std::shared_ptr<ReservationTable> reservationTable, std::shared_ptr<ReverseResumableAStar> reverseResumableAStar);
    SafeIntervalPathPlanning(const Graph &graph, const std::vector<Mobile> &mobiles);
    ~SafeIntervalPathPlanning();

    void addZoneCapacityConstraint(const std::vector<int> &weights, int capacity, const Polygon &polygon);
    Path plan(int mobile, int from, int to, double start);
};

#endif // SAFE_INTERVAL_PATH_PLANNING_HPP