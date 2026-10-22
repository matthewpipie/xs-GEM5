#include "cpu/o3/fdip.hh"

#include <algorithm>
#include <vector>

#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/pred/btb/decoupled_bpred.hh"
#include "debug/FDIP.hh"
#include "mem/cache/prefetch/fdip.hh"
#include "mem/cache/shadow_l1i.hh"
#include "params/BaseO3CPU.hh"
#include "sim/drain.hh"

namespace gem5
{

namespace o3
{

FDIPGenerator::FDIPGenerator(
        CPU *_cpu, const BaseO3CPUParams &params,
        branch_prediction::btb_pred::DecoupledBPUWithBTB *_bpu,
        unsigned blk_size, unsigned fetch_buffer_size,
        statistics::Group *stats_parent)
    : cpu(_cpu), bpu(_bpu), prefetcher(params.fdipPrefetcher),
      blkSize(blk_size), fetchBufferSize(fetch_buffer_size),
      distance(params.fdipDistance), issueWidth(params.fdipIssueWidth),
      maxInflight(params.fdipMaxInflight),
      filterSize(params.fdipFilterSize), stats(stats_parent)
{
    fatal_if(!prefetcher,
             "fdipEnable is set but fdipPrefetcher is not connected to an "
             "L1I FDIPPrefetcher");
    fatal_if(!bpu, "FDIP requires the decoupled BTB predictor");
    fatal_if(distance == 0 || issueWidth == 0 || maxInflight == 0,
             "fdipDistance, fdipIssueWidth and fdipMaxInflight must be > 0");
    stats.distanceAtIssue.init(1, distance, 1);
    // The shadow L1I counts every FDIP fill, by the fill id set below
    prefetcher->setShadowFillCallback([this](uint64_t id) {
        if (auto *shadow = cpu->getShadowL1I()) {
            shadow->prefetchFilled(id);
        }
    });
}

void
FDIPGenerator::tick(const std::list<ThreadID> &active_threads)
{
    if (active_threads.empty() ||
        cpu->drainState() == DrainState::Draining) {
        return;
    }

    // Round robin across active threads, one line per thread per pass
    std::vector<ThreadID> order(active_threads.begin(), active_threads.end());
    std::rotate(order.begin(), order.begin() + rrThread % order.size(),
                order.end());
    rrThread = (rrThread + 1) % order.size();

    unsigned budget = issueWidth;
    bool progress = true;
    while (budget > 0 && progress) {
        progress = false;
        for (ThreadID tid : order) {
            if (budget == 0) {
                break;
            }
            if (inflightTranslations >= maxInflight) {
                stats.inflightLimitCycles++;
                return;
            }
            if (generateOne(tid)) {
                --budget;
                progress = true;
            }
        }
    }
}

bool
FDIPGenerator::generateOne(ThreadID tid)
{
    auto &st = threads[tid];

    // Targets whose lines are all filtered are skipped in the same cycle;
    // the loop is bounded by distance targets.
    while (bpu->ftqHasPrefetchTarget(tid) &&
           bpu->ftqPrefetchDistance(tid) <= distance) {
        const auto id = bpu->ftqPrefetchId(tid);
        const auto &target = bpu->ftqPrefetchTarget(tid);

        // FTQ ids are reused after a squash, so also match the contents
        const Addr end = std::min<Addr>(target.predEndPC,
                                        target.startPC + fetchBufferSize);
        const Addr end_addr = end > target.startPC ? end : target.startPC + 1;
        if (st.targetId != id || st.startPC != target.startPC ||
            st.endAddr != end_addr) {
            st.targetId = id;
            st.startPC = target.startPC;
            st.nextLine = lineAlign(target.startPC);
            st.endAddr = end_addr;
            stats.targetsScanned++;
        }

        if (st.nextLine >= st.endAddr) {
            bpu->ftqAdvancePrefetch(tid);
            st.targetId = 0;
            continue;
        }

        const Addr line = st.nextLine;
        st.nextLine += blkSize;
        stats.linesGenerated++;

        const bool last_line = st.nextLine >= st.endAddr;
        bool started = false;
        if (!isFiltered(tid, line)) {
            stats.distanceAtIssue.sample(bpu->ftqPrefetchDistance(tid));
            startTranslation(tid, line, target.startPC, target.uid);
            started = true;
        }

        if (last_line) {
            bpu->ftqAdvancePrefetch(tid);
            st.targetId = 0;
        }
        if (started) {
            return true;
        }
    }
    return false;
}

bool
FDIPGenerator::isFiltered(ThreadID tid, Addr line)
{
    // Lines the demand fetch of the fetching target already requests
    if (bpu->ftqHasFetching(tid)) {
        const auto &fetching = bpu->ftqFetchingTarget(tid);
        if (line >= lineAlign(fetching.startPC) &&
            line < fetching.startPC + fetchBufferSize) {
            stats.filteredDemand++;
            return true;
        }
    }

    if (filterSize == 0) {
        return false;
    }

    auto &recent = threads[tid].recentLines;
    if (std::find(recent.begin(), recent.end(), line) != recent.end()) {
        stats.filteredRecent++;
        return true;
    }
    recent.push_back(line);
    if (recent.size() > filterSize) {
        recent.pop_front();
    }
    return false;
}

void
FDIPGenerator::startTranslation(ThreadID tid, Addr line, Addr pc,
                                uint64_t target_uid)
{
    RequestPtr req = std::make_shared<Request>(
        line, blkSize, Request::INST_FETCH | Request::PREFETCH,
        cpu->instRequestorId(), pc, cpu->thread[tid]->contextId());
    req->taskId(cpu->taskId());

    DPRINTF(FDIP, "[tid:%i] Translate line %#lx (target pc %#lx), "
            "distance %lu\n", tid, line, pc, bpu->ftqPrefetchDistance(tid));

    stats.translationsStarted++;
    // Count before translating, the translation may finish synchronously
    ++inflightTranslations;
    auto *trans = new Translation(this, tid, threads[tid].squashGen,
                                  target_uid);
    cpu->mmu->translateTiming(req, cpu->thread[tid]->getTC(), trans,
                              BaseMMU::Execute);
}

void
FDIPGenerator::Translation::finish(const Fault &fault, const RequestPtr &req,
                                   gem5::ThreadContext *tc,
                                   BaseMMU::Mode mode)
{
    assert(mode == BaseMMU::Execute);
    gen->finishTranslation(fault, req, tid, squashGen, targetUid);
    delete this;
}

void
FDIPGenerator::finishTranslation(const Fault &fault, const RequestPtr &req,
                                 ThreadID tid, uint64_t squash_gen,
                                 uint64_t target_uid)
{
    assert(inflightTranslations > 0);
    --inflightTranslations;

    if (squash_gen != threads[tid].squashGen || cpu->switchedOut()) {
        stats.translationsSquashed++;
        return;
    }

    // Faulting prefetches are dropped silently, never raised
    if (fault != NoFault) {
        stats.translationFaults++;
        DPRINTF(FDIP, "[tid:%i] Drop line %#lx, translation fault\n", tid,
                req->getVaddr());
        return;
    }

    auto *shadow = cpu->getShadowL1I();
    if (shadow) {
        req->setShadowFillId(ShadowL1I::PrefetchFillBit | ++shadowFillCount);
    }
    if (prefetcher->enqueue(req)) {
        stats.sentToPrefetcher++;
        if (shadow) {
            shadow->prefetchQueued(tid, req->getShadowFillId(), target_uid,
                                   lineAlign(req->getPaddr()));
        }
    } else {
        stats.rejectedByPrefetcher++;
    }
}

void
FDIPGenerator::squash(ThreadID tid)
{
    auto &st = threads[tid];
    ++st.squashGen;
    st.targetId = 0;
    // The filter holds virtual lines, a redirect may change address space
    st.recentLines.clear();
    prefetcher->squash(cpu->thread[tid]->contextId());
    stats.squashes++;
}

FDIPGenerator::FDIPStats::FDIPStats(statistics::Group *parent)
    : statistics::Group(parent, "fdip"),
      ADD_STAT(targetsScanned, statistics::units::Count::get(),
               "Number of FTQ targets split into lines"),
      ADD_STAT(linesGenerated, statistics::units::Count::get(),
               "Number of candidate prefetch lines generated"),
      ADD_STAT(filteredRecent, statistics::units::Count::get(),
               "Number of lines dropped by the recently requested filter"),
      ADD_STAT(filteredDemand, statistics::units::Count::get(),
               "Number of lines dropped as covered by the demand fetch"),
      ADD_STAT(translationsStarted, statistics::units::Count::get(),
               "Number of prefetch ITLB translations started"),
      ADD_STAT(translationFaults, statistics::units::Count::get(),
               "Number of prefetch translations dropped on fault"),
      ADD_STAT(translationsSquashed, statistics::units::Count::get(),
               "Number of prefetch translations dropped after a squash"),
      ADD_STAT(sentToPrefetcher, statistics::units::Count::get(),
               "Number of lines queued by the L1I FDIP prefetcher"),
      ADD_STAT(rejectedByPrefetcher, statistics::units::Count::get(),
               "Number of lines the L1I FDIP prefetcher filtered"),
      ADD_STAT(squashes, statistics::units::Count::get(),
               "Number of frontend squashes seen by FDIP"),
      ADD_STAT(inflightLimitCycles, statistics::units::Cycle::get(),
               "Cycles FDIP stopped at the in-flight translation limit"),
      ADD_STAT(distanceAtIssue, statistics::units::Count::get(),
               "FTQ targets between fetch and prefetch pointer at issue")
{
}

} // namespace o3
} // namespace gem5
