#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/utils.hpp"

#include "algorithm_test.hpp"

using namespace testing;

TEST_F(AlgorithmTest, ZoneCapacityConstraint) {
    std::vector<int> weights(mobiles.size(), 1);
    planner->addZoneCapacityConstraint(
        weights, 1, 
        getPolygon({
            Point(GRID_SIZE-2, 0),
            Point(1, 0),
            Point(1, GRID_SIZE-1),
            Point(GRID_SIZE-2, GRID_SIZE-1)
        })
    );

    auto start1 = getVertexIndex(0, 0), goal1 = getVertexIndex(GRID_SIZE-1, 0);
    auto start2 = getVertexIndex(0, 1), goal2 = getVertexIndex(GRID_SIZE-1, 1);
    auto path1 = planner->plan(0, start1, goal1, 0);
    auto path2 = planner->plan(1, start2, goal2, 0);

    EXPECT_THAT(path1.timedPositions, SizeIs(GRID_SIZE));
    EXPECT_THAT(path1.timedPositions[0], FieldsAre(start1, 0));
    EXPECT_THAT(path1.timedPositions[path1.timedPositions.size()-1], FieldsAre(goal1, DoubleEq(GRID_SIZE-1)));

    EXPECT_THAT(path2.timedPositions, SizeIs(Gt(GRID_SIZE)));
    EXPECT_THAT(path2.timedPositions[0], FieldsAre(start2, 0));
    EXPECT_THAT(path2.timedPositions[path2.timedPositions.size()-1], FieldsAre(goal2, Gt(GRID_SIZE-1)));
}