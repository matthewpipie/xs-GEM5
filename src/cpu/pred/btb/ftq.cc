#include <algorithm>

#include "ftq.hh"

namespace gem5
{

namespace branch_prediction
{

namespace btb_pred
{

int
FetchTargetQueue::getTargetTid()
{
    for (int i = roundRobinPtr; i < numThreads + roundRobinPtr; ++i) {
        ThreadID tid = i % numThreads;
        if (!queue[tid].cap.empty() && hasTarget(fetchId(tid), tid)) {
            roundRobinPtr = (tid + 1) % numThreads;
            FetchTarget& target = get(queue[tid].fetchptr, tid);
            return target.tid;
        }
    }
    return -1;
}

void
FetchTargetQueue::insert(FetchTarget& target)
{
    ThreadID tid = target.tid;
    assert(queue[tid].cap.size() < ftqSize[tid]);
    queue[tid].cap.push_back(target);
}

void
FetchTargetQueue::finishTarget(ThreadID tid)
{
    queue[tid].fetchptr++;
    queue[tid].pfptr = std::max(queue[tid].pfptr, queue[tid].fetchptr + 1);
}

void
FetchTargetQueue::advancePrefetch(ThreadID tid)
{
    assert(hasPrefetchTarget(tid));
    queue[tid].pfptr++;
}

void
FetchTargetQueue::commitTarget(ThreadID tid)
{
    queue[tid].cap.pop_front();
    queue[tid].baseTargetId++;
}

void
FetchTargetQueue::squashAfter(FetchTargetId squashId, ThreadID tid)
{
    while (!empty(tid) && backId(tid) > squashId) {
        queue[tid].cap.pop_back();
    }
    queue[tid].fetchptr = squashId + 1;
    // Every surviving target up to squashId is at or before fetchptr, so
    // prefetching restarts right after the new fetching target.
    queue[tid].pfptr = queue[tid].fetchptr + 1;
}

void
FetchTargetQueue::clear(ThreadID tid)
{
    const FetchTargetId nextTargetId = std::max(
        queue[tid].fetchptr,
        queue[tid].baseTargetId +
            static_cast<FetchTargetId>(queue[tid].cap.size()));

    queue[tid].cap.clear();
    queue[tid].baseTargetId = nextTargetId;
    queue[tid].fetchptr = nextTargetId;
    queue[tid].pfptr = nextTargetId + 1;
}


}
}
}
