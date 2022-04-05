#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>

#include "structs.hpp"

void mergeIntervals(std::vector<Interval> &intervals);

// give vertices in CCW order
Polygon getPolygon(const std::vector<Point> &points);

#endif // UTILS_HPP