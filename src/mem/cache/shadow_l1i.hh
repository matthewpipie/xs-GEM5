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
 * Fill attribution. Every fetch line request and FDIP prefetch carries a fill
 * id; the L1I stores the id of the request that filled each line, and returns
 * it on a fetch hit, or the id of the request that started the fill on a
 * merge into an in-flight MSHR. Whether a fetch was on the committed path is
 * only known later, so the id is resolved when its fill retires and commit
 * reaches it. A committed op that uses the line commits after the filling
 * fetch is resolved, since that fetch is older. D and the merges in B are
 * split by what brought the line in:
 *   offPath     a fetch with no committed op using the line (C)
 *   unusedLine  a fetch whose fill committed but whose line no op used (U)
 *   fdip        an FDIP prefetch
 *   onPath      an earlier committed access (the shadow evicted the line
 *               sooner than the real L1I)
 *   unknown     no fill id (expected 0)
 * fillsUsed / fillsUsedLate count, once per filled line, the off-path,
 * unused-line and FDIP fills that a committed access later hit, or merged
 * into while in flight.
 *
 * FDIP prefetch path. A prefetch is for one line of one FTQ target (by
 * FetchTarget::uid). It is on the correct path if an op of that target
 * commits with bytes in that line, and on the wrong path if fetch skips the
 * target (it was squashed before being fetched) or commit passes the
 * target's last op without such an op. A prefetch queued after fetch had
 * reached its target, or whose target is still pending at the end of the
 * window, stays unresolved. fdipFills / fdipFillsUsed / fdipFillsUsedLate /
 * fdipHits / fdipMerges split FDIP fills, their committed use, and the fdip
 * parts of prefetchHitFillers and inherentMergeFillers by path; the
 * unresolved rest is the difference to fdipFillsTotal, fillsUsed::fdip,
 * fillsUsedLate::fdip, prefetchHitFillers::fdip and
 * inherentMergeFillers::fdip.
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
#include <map>
#include <unordered_map>
#include <unordered_set>
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
        /**
         * Fill id of the request that brought the line in for this op: its
         * own request on a primary miss, else the one the L1I reported
         */
        uint64_t filler = 0;
    };

    /** An op's bytes span at most two lines */
    using FetchLines = std::array<FetchLine, 2>;

    /** One line of a fill retired from the fetch buffer */
    struct RetiredLine
    {
        /** Fill id of the line's request, 0 if none */
        uint64_t id = 0;
        /** The real L1I demand-missed on the line */
        bool miss = false;
        /** That miss merged into an in-flight MSHR */
        bool mshrMerge = false;
        /** A built op had bytes in the line */
        bool used = false;
    };

    /** Fill ids with this bit set are FDIP prefetches */
    static constexpr uint64_t PrefetchFillBit = uint64_t(1) << 63;

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
     * @param target_uid FetchTarget::uid of the target the op was fetched
     *                  for, 0 if unknown
     * @param lines     the op's fetch lines and real L1I outcomes
     * @param num_insts committed instructions this op adds to the MPKI
     *                  denominator (same rule as committedInsts)
     */
    void commitInst(ThreadID tid, InstSeqNum seq, uint64_t target_uid,
                    const FetchLines &lines, unsigned num_insts);

    /**
     * A fill with at least one missed line will supply no more ops. Its
     * lines that filled the L1I are recorded for attribution. Once an op of
     * the fill commits, the fill is on the committed path and a missed line
     * no op used counts as an unused-line miss; if none commits, the fill
     * stays off-path.
     *
     * @param first_seq sequence number of the first op built from the fill
     * @param last_seq  sequence number of the last op built from the fill
     */
    void retireFill(ThreadID tid, InstSeqNum first_seq, InstSeqNum last_seq,
                    const std::array<RetiredLine, 2> &lines);

    /** FDIP queued prefetch @p id for line @p line_addr of a target. */
    void prefetchQueued(ThreadID tid, uint64_t id, uint64_t target_uid,
                        Addr line_addr);

    /** FDIP prefetch @p id filled an L1I line. */
    void prefetchFilled(uint64_t id);

    /** Fetch started fetching target @p target_uid. */
    void targetFetched(ThreadID tid, uint64_t target_uid);

    /**
     * The fill of target @p target_uid retired; its ops are in
     * [first_seq, last_seq], or first_seq is 0 if there are none.
     */
    void targetRetired(ThreadID tid, uint64_t target_uid,
                       InstSeqNum first_seq, InstSeqNum last_seq);

  private:
    /** Index of the primary / MSHR-merge split of the miss vectors */
    enum MissKind { Primary, MshrMerge, NumMissKinds };

    /** What brought a line into the L1I; the first three are fills */
    enum Source { OffPath, UnusedLine, Fdip, OnPath, Unknown, NumSources };
    static constexpr unsigned NumFillSources = OnPath;

    /** A retired fill, waiting for commit to reach it */
    struct PendingFill
    {
        InstSeqNum firstSeq;
        InstSeqNum lastSeq;
        std::array<RetiredLine, 2> lines;
    };

    /** Path of an FDIP prefetch, by the fate of its target */
    enum PfPath : uint8_t
    {
        CorrectPath, WrongPath, NumPfPaths, Unresolved = NumPfPaths
    };
    enum PfUse : uint8_t { NotUsed, UsedTimely, UsedLate };

    /** An FDIP prefetch queued by fetch */
    struct PrefetchRecord
    {
        uint64_t targetUid = 0;
        Addr lineAddr = 0;
        PfPath path = Unresolved;
        PfUse use = NotUsed;
        bool filled = false;
        /** fdipHits / fdipMerges events waiting for the path */
        uint32_t pendingHits = 0;
        uint32_t pendingMerges = 0;
    };

    /** A target with unresolved prefetches */
    struct PfTarget
    {
        std::vector<uint64_t> prefetches;
        bool fetched = false;
    };

    /** A line a retired fill brought into the L1I */
    struct LineRecord
    {
        /** A built op had bytes in the line */
        bool used = false;
        /** An op of the fill committed */
        bool committedFill = false;
        /** The op carrying the line's miss committed */
        bool committedUse = false;
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

    /** Resolve pending fills that committed op @p seq is in or past. */
    void resolveFills(ThreadID tid, InstSeqNum seq);

    /** A fill had a committed op. */
    void commitFill(const PendingFill &fill);

    /** What brought in the line filled by request @p id. */
    Source source(uint64_t id) const;

    /** A committed access used a line that request @p id filled. */
    void noteUse(ThreadID tid, uint64_t id, bool late);

    /** Decide the path of FDIP prefetch @p rec and count what waited. */
    void resolvePrefetch(PrefetchRecord &rec, PfPath path);

    /** Resolve the unresolved prefetches of a target, then forget it. */
    void resolveTarget(ThreadID tid,
                       std::map<uint64_t, PfTarget>::iterator it,
                       PfPath path);

    /** Prefetches of an op's target for the op's lines are correct-path. */
    void markCorrectPath(ThreadID tid, uint64_t target_uid,
                         const FetchLines &lines);

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

    /** Per-thread retired fills not yet resolved, oldest first */
    std::vector<std::deque<PendingFill>> pendingFills;

    /** Lines filled by retired fills, by fill id */
    std::unordered_map<uint64_t, LineRecord> lineRecords;

    /** Fills already counted in fillsUsed / fillsUsedLate */
    std::unordered_set<uint64_t> usedFills;

    /** Per-thread fill id last passed to noteUse, to skip repeats */
    std::vector<uint64_t> lastNoted;

    /** FDIP prefetches queued, by fill id */
    std::unordered_map<uint64_t, PrefetchRecord> prefetches;

    /** Per-thread targets with unresolved prefetches, by uid */
    std::vector<std::map<uint64_t, PfTarget>> pfTargets;

    /** Per-thread retired fetched targets with unresolved prefetches,
     *  as (last op seq, uid), oldest first */
    std::vector<std::deque<std::pair<InstSeqNum, uint64_t>>> pfRetired;

    /** Per-thread uid of the target fetch most recently started */
    std::vector<uint64_t> lastFetchedUid;

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
    /** D by what brought the line in */
    statistics::Vector prefetchHitFillers;
    /** Merges in B by what started the fill */
    statistics::Vector inherentMergeFillers;
    /** Fills whose first committed use was a hit */
    statistics::Vector fillsUsed;
    /** Fills whose first committed use merged into the in-flight fill */
    statistics::Vector fillsUsedLate;
    /** FDIP prefetch fills, any path */
    statistics::Scalar fdipFillsTotal;
    /** FDIP prefetch fills by path */
    statistics::Vector fdipFills;
    /** fillsUsed::fdip by path */
    statistics::Vector fdipFillsUsed;
    /** fillsUsedLate::fdip by path */
    statistics::Vector fdipFillsUsedLate;
    /** prefetchHitFillers::fdip by path */
    statistics::Vector fdipHits;
    /** inherentMergeFillers::fdip by path */
    statistics::Vector fdipMerges;
};

} // namespace gem5

#endif // __MEM_CACHE_SHADOW_L1I_HH__
