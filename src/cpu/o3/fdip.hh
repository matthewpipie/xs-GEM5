/**
 * @file
 * Fetch-directed instruction prefetch (FDIP), fetch side.
 *
 * The decoupled BPU runs ahead of fetch and fills the fetch target queue
 * (FTQ). FDIPGenerator walks the FTQ targets after the one currently being
 * fetched, splits each target into cache lines, translates them through the
 * ITLB and hands the physical lines to the L1 I-cache FDIP prefetcher, which
 * issues them through the cache's hardware-prefetch path.
 */

#ifndef __CPU_O3_FDIP_HH__
#define __CPU_O3_FDIP_HH__

#include <deque>
#include <list>

#include "arch/generic/mmu.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/o3/limits.hh"
#include "cpu/pred/btb/common.hh"
#include "mem/request.hh"

namespace gem5
{

struct BaseO3CPUParams;

namespace prefetch
{
class FDIPPrefetcher;
} // namespace prefetch

namespace branch_prediction
{
namespace btb_pred
{
class DecoupledBPUWithBTB;
} // namespace btb_pred
} // namespace branch_prediction

namespace o3
{

class CPU;

class FDIPGenerator
{
  public:
    FDIPGenerator(CPU *_cpu, const BaseO3CPUParams &params,
                  branch_prediction::btb_pred::DecoupledBPUWithBTB *_bpu,
                  unsigned blk_size, unsigned fetch_buffer_size,
                  statistics::Group *stats_parent);

    /** Generate up to issueWidth line requests; called once per cycle. */
    void tick(const std::list<ThreadID> &active_threads);

    /** Drop pending prefetch work of a thread after a frontend redirect. */
    void squash(ThreadID tid);

    /** True when no ITLB translation of a prefetch is outstanding. */
    bool isDrained() const { return inflightTranslations == 0; }

  private:
    class Translation : public BaseMMU::Translation
    {
      public:
        Translation(FDIPGenerator *_gen, ThreadID _tid, uint64_t _squash_gen,
                    uint64_t _target_uid)
            : gen(_gen), tid(_tid), squashGen(_squash_gen),
              targetUid(_target_uid)
        {}

        void markDelayed() override {}

        void finish(const Fault &fault, const RequestPtr &req,
                    gem5::ThreadContext *tc, BaseMMU::Mode mode) override;

      private:
        FDIPGenerator *gen;
        ThreadID tid;
        uint64_t squashGen;
        /** FetchTarget::uid of the target the line belongs to */
        uint64_t targetUid;
    };

    struct ThreadState
    {
        /** Bumped on squash to drop translations of the old path */
        uint64_t squashGen = 0;
        /** FTQ id of the target being split into lines, 0 if none */
        branch_prediction::btb_pred::FetchTargetId targetId = 0;
        /** Start PC of that target */
        Addr startPC = 0;
        /** Next line vaddr to request within that target */
        Addr nextLine = 0;
        /** End vaddr (exclusive) of that target's fetched bytes */
        Addr endAddr = 0;
        /** Recently requested line vaddrs, oldest first */
        std::deque<Addr> recentLines;
    };

    /**
     * Try to generate one line request for a thread.
     * @return true if a translation was started
     */
    bool generateOne(ThreadID tid);

    bool isFiltered(ThreadID tid, Addr line);

    void startTranslation(ThreadID tid, Addr line, Addr pc,
                          uint64_t target_uid);

    void finishTranslation(const Fault &fault, const RequestPtr &req,
                           ThreadID tid, uint64_t squash_gen,
                           uint64_t target_uid);

    Addr lineAlign(Addr addr) const { return addr & ~Addr(blkSize - 1); }

    CPU *cpu;
    branch_prediction::btb_pred::DecoupledBPUWithBTB *bpu;
    prefetch::FDIPPrefetcher *prefetcher;

    const unsigned blkSize;
    /** Bytes a demand fetch reads from a target's start PC */
    const unsigned fetchBufferSize;

    /** Max FTQ targets the prefetch pointer runs ahead of fetch */
    const unsigned distance;
    /** Max line requests generated per cycle */
    const unsigned issueWidth;
    /** Max outstanding prefetch translations */
    const unsigned maxInflight;
    /** Entries of the per-thread recently requested line filter */
    const unsigned filterSize;

    unsigned inflightTranslations = 0;
    /** Prefetches given a shadow L1I fill id so far */
    uint64_t shadowFillCount = 0;
    ThreadID rrThread = 0;
    ThreadState threads[MaxThreads];

    struct FDIPStats : public statistics::Group
    {
        FDIPStats(statistics::Group *parent);

        statistics::Scalar targetsScanned;
        statistics::Scalar linesGenerated;
        statistics::Scalar filteredRecent;
        statistics::Scalar filteredDemand;
        statistics::Scalar translationsStarted;
        statistics::Scalar translationFaults;
        statistics::Scalar translationsSquashed;
        statistics::Scalar sentToPrefetcher;
        statistics::Scalar rejectedByPrefetcher;
        statistics::Scalar squashes;
        statistics::Scalar inflightLimitCycles;
        statistics::Distribution distanceAtIssue;
    } stats;
};

} // namespace o3
} // namespace gem5

#endif // __CPU_O3_FDIP_HH__
