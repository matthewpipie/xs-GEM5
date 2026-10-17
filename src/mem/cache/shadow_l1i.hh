/*
 * Committed-only shadow L1 instruction cache -- off-path pollution study.
 *
 * A functional, tag-only, set-associative LRU model of the L1I that is updated
 * ONLY by committed (correct-path) instruction fetches. It never observes a
 * wrong-path (off-path) fetch or any prefetch, so its miss stream is what the
 * L1I would see on an idealized machine that only fetched committed lines.
 *
 * Comparing the shadow's misses against the real L1I's demand misses
 * isolates the net effect of all non-committed L1I traffic:
 *   - real misses > shadow misses  => that traffic net-pollutes the L1I
 *   - real misses < shadow misses  => that traffic acts as useful prefetch
 * Non-committed traffic includes wrong-path demand fetches, unused lines of
 * the fetch window and, with FDIP enabled, FTQ-driven prefetches on both the
 * correct and the wrong path. The shadow itself does not depend on FDIP.
 *
 * Miss dissection. Fetch marks each real L1I demand miss on the first op
 * built with bytes in the missed line, so a committed op carries the real L1I
 * outcome of its lines. Each committed line access is classified against the
 * shadow:
 *   A  onPathPollutionMisses  real miss, shadow hit: the committed-only cache
 *                             would have kept the line
 *   B  onPathInherentMisses   real miss, shadow miss: the committed path
 *                             misses anyway (cold, capacity, conflict)
 *   D  onPathPrefetchHits     real hit, shadow miss: non-committed traffic
 *                             brought the line in first
 * A and B are split into primary misses and misses that merged into an
 * in-flight MSHR (e.g. a late FDIP prefetch). A real miss on a fetched line
 * that no op used, in a fill whose ops commit, is counted in
 * unusedLineMisses (U). Every other real L1I demand miss is off-path or
 * squashed-and-refetched (C), derived as
 *   C = icache.demandMisses - A - B - U
 * and by construction shadowMisses == B + D.
 *
 * The shadow is indexed and tagged by the physical fetch address, like the
 * real (physically indexed) L1I, so both have the same set mapping.
 *
 * This is measurement only: it models hit/miss accounting exclusively and does
 * not touch the real cache, MSHRs, coherence, or timing. It is only built
 * when enabled (l1i_shadow_enable), so it has zero effect on IPC.
 */

#ifndef __MEM_CACHE_SHADOW_L1I_HH__
#define __MEM_CACHE_SHADOW_L1I_HH__

#include <array>
#include <deque>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/inst_seq.hh"

namespace gem5
{

/**
 * Per-L1I functional shadow tag store fed by the committed instruction stream.
 * Sized (sets x ways x block) identically to the L1I it shadows.
 *
 * Owned by the O3 CPU (not the icache): it is sized from scalar geometry
 * params and carries no handle to any cache SimObject. Its stats nest under
 * whatever statistics::Group parent is passed (the CPU), landing at
 * system.cpu.shadowL1I.*.
 */
class ShadowL1I : public statistics::Group
{
  public:
    /** One I-cache line an op was fetched from */
    struct FetchLine
    {
        /** Line-aligned physical address, MaxAddr if unknown */
        Addr paddr = MaxAddr;
        /**
         * The real L1I demand-missed on this line in this op's fill. Set on
         * the first op with bytes in the line only, so each real miss is
         * carried by at most one op.
         */
        bool miss = false;
        /** That miss merged into an in-flight MSHR */
        bool mshrMerge = false;
    };

    /** An op's bytes span at most two lines */
    using FetchLines = std::array<FetchLine, 2>;

    ShadowL1I(statistics::Group *parent, unsigned num_threads,
              unsigned num_sets, unsigned assoc, unsigned blk_size);

    /**
     * Observe one committed op. Each of its lines that starts a new run of
     * committed ops in a line for its thread probes the shadow tags: a
     * resident block is an LRU hit, otherwise it is a miss and the block is
     * installed (evicting the set's LRU way). The real L1I outcome carried by
     * the op is then classified against the shadow.
     *
     * @param tid       committing thread
     * @param seq       sequence number of the op
     * @param lines     the op's fetch lines and real L1I outcomes
     * @param num_insts committed instructions this op adds to the MPKI
     *                  denominator (same rule as committedInsts)
     */
    void commitInst(ThreadID tid, InstSeqNum seq, const FetchLines &lines,
                    unsigned num_insts);

    /**
     * A fill retired with a real L1I demand miss on a line that no op used.
     * It counts as an unused-line miss once an op of the fill commits; if
     * none does, the miss stays off-path.
     *
     * @param first_seq sequence number of the first op built from the fill
     * @param last_seq  sequence number of the last op built from the fill
     */
    void unusedLineMiss(ThreadID tid, InstSeqNum first_seq,
                        InstSeqNum last_seq, bool mshr_merge);

  private:
    /** Index of the primary / MSHR-merge split of the miss vectors */
    enum MissKind { Primary, MshrMerge, NumMissKinds };

    /** A retired fill's unused-line miss, waiting for the fill to commit */
    struct PendingUnusedMiss
    {
        InstSeqNum firstSeq;
        InstSeqNum lastSeq;
        bool mshrMerge;
    };

    /** Block-align an address to the shadow's line size. */
    Addr blockAddr(Addr addr) const { return addr & ~blkMask; }

    /** Modulo set index of a block-aligned address. */
    unsigned setIndex(Addr blk_addr) const
    {
        return (blk_addr >> blkShift) % numSets;
    }

    /** Probe the shadow tags with LRU update, install on a miss. */
    bool probe(Addr blk_addr);

    /** Classify one committed line access. */
    void accessLine(ThreadID tid, const FetchLine &line);

    /** Count pending unused-line misses whose fill committed op @p seq. */
    void creditUnusedLineMisses(ThreadID tid, InstSeqNum seq);

    static MissKind
    missKind(bool mshr_merge)
    {
        return mshr_merge ? MshrMerge : Primary;
    }

    const unsigned numSets;
    const unsigned assoc;
    const Addr blkMask;
    const unsigned blkShift;

    /**
     * One MRU-ordered vector of resident block addresses per set. front() is
     * most-recently-used, back() is the LRU eviction victim. Size bounded by
     * assoc.
     */
    std::vector<std::vector<Addr>> sets;

    /**
     * Per-thread block address most recently probed, to dedup intra-line
     * commits. MaxAddr when nothing was probed yet.
     */
    std::vector<Addr> lastBlockAddr;

    /** Per-thread sequence number of the last committed op, 0 if none */
    std::vector<InstSeqNum> lastCommitSeq;

    /** Per-thread unused-line misses of retired fills, oldest first */
    std::vector<std::deque<PendingUnusedMiss>> pendingUnused;

    /** Committed instructions observed (MPKI denominator). */
    statistics::Scalar shadowInsts;
    /** Distinct committed fetch-block accesses probed against the shadow. */
    statistics::Scalar shadowAccesses;
    /** Committed-path accesses that hit in the shadow. */
    statistics::Scalar shadowHits;
    /** Committed-path accesses that missed in the shadow. */
    statistics::Scalar shadowMisses;
    /** Committed micro-ops skipped because no fetch paddr was recorded. */
    statistics::Scalar shadowNoPAddr;
    /** shadowMisses / shadowAccesses. */
    statistics::Formula shadowMissRate;
    /** Misses per kilo committed instruction. */
    statistics::Formula shadowMpki;

    /** A: committed line access, real miss, shadow hit */
    statistics::Vector onPathPollutionMisses;
    /** B: committed line access, real miss, shadow miss */
    statistics::Vector onPathInherentMisses;
    /** D: committed line access, real hit, shadow miss */
    statistics::Scalar onPathPrefetchHits;
    /** U: real miss on a line no op used, in a committed fill */
    statistics::Vector unusedLineMisses;
};

} // namespace gem5

#endif // __MEM_CACHE_SHADOW_L1I_HH__
