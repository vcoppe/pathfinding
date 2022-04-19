#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../src/utils.hpp"

using namespace testing;

TEST(UtilsTest, MergeNoIntervals) {
    std::vector<Interval> intervals;

    mergeIntervals(intervals);

    EXPECT_THAT(intervals, IsEmpty());
}

TEST(UtilsTest, MergeSingleInterval) {
    std::vector<Interval> intervals{{0, 1}};

    mergeIntervals(intervals);

    EXPECT_THAT(intervals, ElementsAre(FieldsAre(0, 1)));
}

TEST(UtilsTest, MergeNonOverlappingIntervals) {
    std::vector<Interval> intervals{{0, 1}, {2, 3}};

    mergeIntervals(intervals);

    EXPECT_THAT(intervals, ElementsAre(FieldsAre(0, 1), FieldsAre(2, 3)));
}

TEST(UtilsTest, MergeOverlappingIntervals) {
    std::vector<Interval> intervals{{0, 2}, {1, 3}};

    mergeIntervals(intervals);

    EXPECT_THAT(intervals, ElementsAre(FieldsAre(0, 3)));
}

TEST(UtilsTest, MergeOverlappingIntervalsMultipleOverlaps) {
    std::vector<Interval> intervals{{0, 1}, {2, 6}, {3, 4}, {5, 7}};

    mergeIntervals(intervals);

    EXPECT_THAT(intervals, ElementsAre(FieldsAre(0, 1), FieldsAre(2, 7)));
}