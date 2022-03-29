#ifndef ZONE_CAPACITY_CONSTRAINT_HPP
#define ZONE_CAPACITY_CONSTRAINT_HPP

#include <boost/icl/interval_map.hpp>

using namespace boost::icl;

class ZoneCapacityConstraint
{
private:
    int *weight, maxSum;
    interval_map<time_t, unsigned int> map;
public:
    ZoneCapacityConstraint(/* args */);
    ~ZoneCapacityConstraint();

    void reserve(unsigned int agentId, time_t start, time_t end);
    bool isAvailable(unsigned int agentId, time_t start, time_t end);
};

#endif // ZONE_CAPACITY_CONSTRAINT_HPP