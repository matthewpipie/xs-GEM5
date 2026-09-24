#include <gtest/gtest.h>

#include "cpu/pred/btb/ftq.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

namespace test
{

namespace
{

constexpr ThreadID TID = 0;

FetchTarget
makeTarget(Addr start_pc, ThreadID tid = TID)
{
    FetchTarget target;
    target.tid = tid;
    target.startPC = start_pc;
    target.predEndPC = start_pc + 32;
    return target;
}

void
fill(FetchTargetQueue &ftq, unsigned n, Addr base_pc = 0x80000000)
{
    for (unsigned i = 0; i < n; ++i) {
        auto target = makeTarget(base_pc + i * 0x40);
        ftq.insert(target);
    }
}

} // anonymous namespace

TEST(FetchTargetQueueTest, PrefetchPointerStartsAfterFetchPointer)
{
    FetchTargetQueue ftq(1, 8);
    EXPECT_EQ(ftq.prefetchId(TID), ftq.fetchId(TID) + 1);
    EXPECT_FALSE(ftq.hasPrefetchTarget(TID));

    fill(ftq, 1);
    // Only the fetching target exists, nothing to prefetch
    EXPECT_TRUE(ftq.hasTarget(ftq.fetchId(TID), TID));
    EXPECT_FALSE(ftq.hasPrefetchTarget(TID));

    fill(ftq, 1, 0x90000000);
    ASSERT_TRUE(ftq.hasPrefetchTarget(TID));
    EXPECT_EQ(ftq.prefetchTarget(TID).startPC, 0x90000000);
    EXPECT_EQ(ftq.prefetchDistance(TID), 1u);
}

TEST(FetchTargetQueueTest, AdvancePrefetchWalksTowardsTail)
{
    FetchTargetQueue ftq(1, 8);
    fill(ftq, 4);

    ftq.advancePrefetch(TID);
    ftq.advancePrefetch(TID);
    EXPECT_EQ(ftq.prefetchDistance(TID), 3u);
    ASSERT_TRUE(ftq.hasPrefetchTarget(TID));
    EXPECT_EQ(ftq.prefetchTarget(TID).startPC, 0x80000000 + 3 * 0x40);

    ftq.advancePrefetch(TID);
    EXPECT_FALSE(ftq.hasPrefetchTarget(TID));
    EXPECT_EQ(ftq.prefetchId(TID), ftq.backId(TID) + 1);
}

TEST(FetchTargetQueueTest, FinishTargetDragsPrefetchPointer)
{
    FetchTargetQueue ftq(1, 8);
    fill(ftq, 4);

    // Prefetch pointer right after fetch pointer is pushed along
    ftq.finishTarget(TID);
    EXPECT_EQ(ftq.prefetchId(TID), ftq.fetchId(TID) + 1);

    // Prefetch pointer already ahead is not moved
    ftq.advancePrefetch(TID);
    const auto pf_id = ftq.prefetchId(TID);
    ftq.finishTarget(TID);
    EXPECT_EQ(ftq.prefetchId(TID), pf_id);
    EXPECT_EQ(ftq.prefetchDistance(TID), 1u);
}

TEST(FetchTargetQueueTest, CommitDoesNotMovePrefetchPointer)
{
    FetchTargetQueue ftq(1, 8);
    fill(ftq, 4);
    ftq.finishTarget(TID);
    ftq.advancePrefetch(TID);
    const auto pf_id = ftq.prefetchId(TID);

    ftq.commitTarget(TID);
    EXPECT_EQ(ftq.prefetchId(TID), pf_id);
}

TEST(FetchTargetQueueTest, SquashAfterResetsPrefetchPointer)
{
    FetchTargetQueue ftq(1, 8);
    fill(ftq, 6);
    const auto front = ftq.frontId(TID);

    // Prefetch pointer ahead of the squash point
    for (int i = 0; i < 4; ++i) {
        ftq.advancePrefetch(TID);
    }
    ftq.squashAfter(front + 1, TID);
    EXPECT_EQ(ftq.fetchId(TID), front + 2);
    EXPECT_EQ(ftq.prefetchId(TID), front + 3);
    EXPECT_FALSE(ftq.hasPrefetchTarget(TID));

    // New targets after the squash become prefetchable again
    fill(ftq, 2, 0xa0000000);
    ASSERT_TRUE(ftq.hasPrefetchTarget(TID));
    EXPECT_EQ(ftq.prefetchTarget(TID).startPC, 0xa0000000 + 0x40);
}

TEST(FetchTargetQueueTest, SquashBehindPrefetchPointerKeepsInvariant)
{
    FetchTargetQueue ftq(1, 8);
    fill(ftq, 6);
    const auto front = ftq.frontId(TID);
    ftq.finishTarget(TID);
    ftq.finishTarget(TID);
    ftq.finishTarget(TID);

    // Squash at an already fetched target: fetch and prefetch restart there
    ftq.squashAfter(front, TID);
    EXPECT_EQ(ftq.fetchId(TID), front + 1);
    EXPECT_EQ(ftq.prefetchId(TID), ftq.fetchId(TID) + 1);
}

TEST(FetchTargetQueueTest, ClearResetsPrefetchPointer)
{
    FetchTargetQueue ftq(1, 8);
    fill(ftq, 5);
    ftq.advancePrefetch(TID);
    ftq.advancePrefetch(TID);

    ftq.clear(TID);
    EXPECT_TRUE(ftq.empty(TID));
    EXPECT_EQ(ftq.prefetchId(TID), ftq.fetchId(TID) + 1);
    EXPECT_FALSE(ftq.hasPrefetchTarget(TID));

    fill(ftq, 2);
    ASSERT_TRUE(ftq.hasPrefetchTarget(TID));
    EXPECT_EQ(ftq.prefetchTarget(TID).startPC, 0x80000000 + 0x40);
}

TEST(FetchTargetQueueTest, PrefetchPointersArePerThread)
{
    FetchTargetQueue ftq(2, 8);
    for (int i = 0; i < 3; ++i) {
        auto t0 = makeTarget(0x80000000 + i * 0x40, 0);
        auto t1 = makeTarget(0x90000000 + i * 0x40, 1);
        ftq.insert(t0);
        ftq.insert(t1);
    }

    ftq.advancePrefetch(0);
    EXPECT_EQ(ftq.prefetchDistance(0), 2u);
    EXPECT_EQ(ftq.prefetchDistance(1), 1u);

    ftq.clear(0);
    EXPECT_FALSE(ftq.hasPrefetchTarget(0));
    ASSERT_TRUE(ftq.hasPrefetchTarget(1));
    EXPECT_EQ(ftq.prefetchTarget(1).startPC, 0x90000040);
}

} // namespace test

} // namespace btb_pred

} // namespace branch_prediction

} // namespace gem5
