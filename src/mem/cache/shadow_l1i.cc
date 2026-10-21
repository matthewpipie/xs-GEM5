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
      pendingFills(num_threads),
      lastNoted(num_threads, 0),
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
               "with committed ops"),
      ADD_STAT(prefetchHitFillers, statistics::units::Count::get(),
               "onPathPrefetchHits by what brought the line into the L1I"),
      ADD_STAT(inherentMergeFillers, statistics::units::Count::get(),
               "onPathInherentMisses merged into an in-flight MSHR, by what "
               "started the fill"),
      ADD_STAT(fillsUsed, statistics::units::Count::get(),
               "Lines filled off the committed path whose first committed "
               "use was a hit"),
      ADD_STAT(fillsUsedLate, statistics::units::Count::get(),
               "Lines filled off the committed path whose first committed "
               "use merged into the fill in flight")
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
    for (auto *vec : {&prefetchHitFillers, &inherentMergeFillers}) {
        vec->init(NumSources)
            .subname(OffPath, "offPath")
            .subname(UnusedLine, "unusedLine")
            .subname(Fdip, "fdip")
            .subname(OnPath, "onPath")
            .subname(Unknown, "unknown")
            .flags(statistics::total);
    }
    for (auto *vec : {&fillsUsed, &fillsUsedLate}) {
        vec->init(NumFillSources)
            .subname(OffPath, "offPath")
            .subname(UnusedLine, "unusedLine")
            .subname(Fdip, "fdip")
            .flags(statistics::total);
    }
}

void
ShadowL1I::commitInst(ThreadID tid, InstSeqNum seq, const FetchLines &lines,
                      unsigned num_insts)
{
    shadowInsts += num_insts;
    resolveFills(tid, seq);

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
ShadowL1I::retireFill(ThreadID tid, InstSeqNum first_seq,
                      InstSeqNum last_seq,
                      const std::array<RetiredLine, 2> &lines)
{
    for (const auto &line : lines) {
        // A primary miss is the request that filled the line
        if (line.id != 0 && line.miss && !line.mshrMerge) {
            lineRecords[line.id].used = line.used;
        }
    }
    const PendingFill fill{first_seq, last_seq, lines};
    // When the fill retires, its ops are the youngest the thread has built,
    // so any committed op at or after first_seq belongs to this fill.
    if (lastCommitSeq[tid] >= first_seq) {
        commitFill(fill);
    } else {
        pendingFills[tid].push_back(fill);
    }
}

void
ShadowL1I::resolveFills(ThreadID tid, InstSeqNum seq)
{
    lastCommitSeq[tid] = seq;
    // Ops commit in sequence order, so the first commit at or after a fill's
    // first op decides it: inside the fill means the fill was on the
    // committed path, past it means every op of the fill was squashed.
    auto &pending = pendingFills[tid];
    while (!pending.empty() && pending.front().firstSeq <= seq) {
        if (seq <= pending.front().lastSeq) {
            commitFill(pending.front());
        }
        pending.pop_front();
    }
}

void
ShadowL1I::commitFill(const PendingFill &fill)
{
    for (const auto &line : fill.lines) {
        if (line.miss && !line.used) {
            unusedLineMisses[missKind(line.mshrMerge)]++;
        }
        auto it = lineRecords.find(line.id);
        if (it != lineRecords.end()) {
            it->second.committedFill = true;
        }
    }
}

ShadowL1I::Source
ShadowL1I::source(uint64_t id) const
{
    if (id == 0) {
        return Unknown;
    }
    if (id & PrefetchFillBit) {
        return Fdip;
    }
    auto it = lineRecords.find(id);
    // No record: the fetch was squashed before its data reached fetch
    if (it == lineRecords.end() || !it->second.committedFill) {
        return OffPath;
    }
    if (!it->second.used) {
        return UnusedLine;
    }
    // Used only by ops that were squashed, e.g. after a mispredict inside
    // the fill
    return it->second.committedUse ? OnPath : OffPath;
}

void
ShadowL1I::noteUse(ThreadID tid, uint64_t id, bool late)
{
    if (id == lastNoted[tid]) {
        return;
    }
    lastNoted[tid] = id;
    const Source src = source(id);
    if (src < NumFillSources && usedFills.insert(id).second) {
        (late ? fillsUsedLate : fillsUsed)[src]++;
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
            prefetchHitFillers[source(line.filler)]++;
        }
    }

    if (line.miss) {
        auto &bucket =
            shadow_hit ? onPathPollutionMisses : onPathInherentMisses;
        bucket[missKind(line.mshrMerge)]++;
        if (line.mshrMerge && !shadow_hit) {
            inherentMergeFillers[source(line.filler)]++;
        }
        if (!line.mshrMerge) {
            // This op's own request filled the line
            auto it = lineRecords.find(line.filler);
            if (it != lineRecords.end()) {
                it->second.committedUse = true;
            }
            return;
        }
    }
    noteUse(tid, line.filler, line.miss);
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
