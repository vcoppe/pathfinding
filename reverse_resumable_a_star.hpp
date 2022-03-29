#ifndef REVERSE_RESUMABLE_A_STAR_HPP
#define REVERSE_RESUMABLE_A_STAR_HPP

#include <set>
#include <unordered_map>

#include "graph.hpp"

class ReverseResumableAStar
{
private:
    struct State
    {
        int vertex;
        double g, h;

        bool operator==(const State &other) const
        {
            return vertex == other.vertex;
        }

        bool operator<(const State &other) const
        {
            return g + h < other.g + other.h;
        }
    };

    const Graph graph;
    const std::vector<Mobile> mobiles;
    int mobile, from;
    std::unordered_map<int, double> distance;
    std::set<State> queue;
    std::set<int> closed;
public:
    ReverseResumableAStar(const Graph &graph, const std::vector<Mobile> &mobiles);
    ~ReverseResumableAStar();
    
    void init(int mobile, int from, int to);
    double getHeuristic(int vertex);
};

#endif // REVERSE_RESUMABLE_A_STAR_HPP