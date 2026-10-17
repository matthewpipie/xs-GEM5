/*
 * Committed-only shadow L1 instruction cache -- off-path pollution study.
 * See shadow_l1i.hh for the design overview.
 */

#include "mem/cache/shadow_l1i.hh"

#include <algorithm>

#include "base/intmath.hh"
#include "base/logging.hh"

namespace gem5
{

ShadowL1I::ShadowL1I(statistics::Group *parent, unsigned num_threads,
                     unsigned num_sets, unsigned assoc_, unsigned blk_size)
    : statistics::Group(parent, "shadowL1I"),
      numSets(num_sets),
      assoc(assoc_),
      blkMask(blk_size - 1),
      blkShift(floorLog2(blk_size)),
      sets(num_sets),
      lastBlockAddr(num_threads, MaxAddr),
      lastCommitSeq(num_threads, 0),
      pendingUnused(num_threads),
      ADD_STAT(shadowInsts, statistics::units::Count::get(),
               "Committed instructions observed by the shadow L1I"),
      ADD_STAT(shadowAccesses, statistics::units::Count::get(),
               "Committed fetch-block accesses probed against the shadow L1I"),
      ADD_STAT(shadowHits, statistics::units::Count::get(),
               "Committed-path accesses that hit in the shadow L1I"),
      ADD_STAT(shadowMisses, statistics::units::Count::get(),
               "Committed-path accesses that missed in the shadow L1I"),
      ADD_STAT(shadowNoPAddr, statistics::units::Count::get(),
               "Committed micro-ops skipped for lack of a fetch paddr"),
      ADD_STAT(shadowMissRate, statistics::units::Ratio::get(),
               "Shadow L1I miss rate (misses / accesses)"),
      ADD_STAT(shadowMpki, statistics::units::Rate<
                   statistics::units::Count, statistics::units::Count>::get(),
               "Shadow L1I misses per kilo committed instruction"),
      ADD_STAT(onPathPollutionMisses, statistics::units::Count::get(),
               "Real L1I demand misses on committed lines that hit in the "
               "shadow L1I (pollution)"),
      ADD_STAT(onPathInherentMisses, statistics::units::Count::get(),
               "Real L1I demand misses on committed lines that also miss in "
               "the shadow L1I"),
      ADD_STAT(onPathPrefetchHits, statistics::units::Count::get(),
               "Real L1I hits on committed lines that miss in the shadow "
               "L1I"),
      ADD_STAT(unusedLineMisses, statistics::units::Count::get(),
               "Real L1I demand misses on fetched lines no op used, in fills "
               "with committed ops")
{
    fatal_if(num_sets == 0 || assoc == 0 || !isPowerOf2(blk_size),
             "Shadow L1I needs sets > 0, assoc > 0 and a power-of-2 block "
             "size (sets %u, assoc %u, block %u)", num_sets, assoc, blk_size);

    shadowMissRate = shadowMisses / shadowAccesses;
    shadowMpki = (shadowMisses / shadowInsts) * 1000;

    for (auto *vec : {&onPathPollutionMisses, &onPathInherentMisses,
                      &unusedLineMisses}) {
        vec->init(NumMissKinds)
            .subname(Primary, "primary")
            .subname(MshrMerge, "mshrMerge")
            .flags(statistics::total);
    }
}

void
ShadowL1I::commitInst(ThreadID tid, InstSeqNum seq, const FetchLines &lines,
                      unsigned num_insts)
{
    shadowInsts += num_insts;
    creditUnusedLineMisses(tid, seq);

    if (lines[0].paddr == MaxAddr) {
        shadowNoPAddr++;
        return;
    }
    for (const auto &line : lines) {
        if (line.paddr != MaxAddr) {
            accessLine(tid, line);
        }
    }
}

void
ShadowL1I::unusedLineMiss(ThreadID tid, InstSeqNum first_seq,
                          InstSeqNum last_seq, bool mshr_merge)
{
    // When the fill retires, its ops are the youngest the thread has built,
    // so any committed op at or after first_seq belongs to this fill.
    if (lastCommitSeq[tid] >= first_seq) {
        unusedLineMisses[missKind(mshr_merge)]++;
        return;
    }
    pendingUnused[tid].push_back({first_seq, last_seq, mshr_merge});
}

void
ShadowL1I::creditUnusedLineMisses(ThreadID tid, InstSeqNum seq)
{
    lastCommitSeq[tid] = seq;
    // Ops commit in sequence order, so the first commit at or after a fill's
    // first op decides it: inside the fill means the fill was on the
    // committed path, past it means every op of the fill was squashed.
    auto &pending = pendingUnused[tid];
    while (!pending.empty() && pending.front().firstSeq <= seq) {
        if (seq <= pending.front().lastSeq) {
            unusedLineMisses[missKind(pending.front().mshrMerge)]++;
        }
        pending.pop_front();
    }
}

void
ShadowL1I::accessLine(ThreadID tid, const FetchLine &line)
{
    const Addr blk_addr = blockAddr(line.paddr);
    // An I-cache line is accessed once per fetched block, not once per
    // instruction; collapse the run of commits that stay within one line.
    // Within a run the line is the shadow's MRU block, i.e. a hit.
    bool shadow_hit = true;
    if (blk_addr != lastBlockAddr[tid]) {
        lastBlockAddr[tid] = blk_addr;
        shadow_hit = probe(blk_addr);
        if (!shadow_hit && !line.miss) {
            onPathPrefetchHits++;
        }
    }

    if (line.miss) {
        auto &bucket =
            shadow_hit ? onPathPollutionMisses : onPathInherentMisses;
        bucket[missKind(line.mshrMerge)]++;
    }
}

bool
ShadowL1I::probe(Addr blk_addr)
{
    shadowAccesses++;

    std::vector<Addr> &set = sets[setIndex(blk_addr)];
    auto it = std::find(set.begin(), set.end(), blk_addr);
    if (it != set.end()) {
        // Hit: promote to MRU.
        shadowHits++;
        set.erase(it);
        set.insert(set.begin(), blk_addr);
        return true;
    }

    // Miss: install at MRU, evict the LRU way if the set is now over capacity.
    shadowMisses++;
    set.insert(set.begin(), blk_addr);
    if (set.size() > assoc) {
        set.pop_back();
    }
    return false;
}

} // namespace gem5
