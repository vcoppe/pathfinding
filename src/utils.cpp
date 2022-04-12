#include "utils.hpp"

void mergeIntervals(std::vector<Interval> &intervals)
{
    for (int i = intervals.size() - 2; i >= 0; i--)
    {
        while (i + 1 < intervals.size() && intervals[i].end >= intervals[i+1].start)
        {
            intervals[i].end = std::max(intervals[i].end, intervals[i+1].end);
            intervals.erase(intervals.begin() + i + 1);
        }
    }
}

Polygon getPolygon(const std::vector<Point> &points)
{
    Polygon polygon;
    for (const auto &point : points)
    {
        bg::append(polygon.outer(), point);
    }
    bg::append(polygon.outer(), points[0]);
    return polygon;
}