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
      pfTargets(num_threads),
      pfRetired(num_threads),
      lastFetchedUid(num_threads, 0),
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
               "use merged into the fill in flight"),
      ADD_STAT(fdipFillsTotal, statistics::units::Count::get(),
               "FDIP prefetch fills of the L1I, any path"),
      ADD_STAT(fdipFills, statistics::units::Count::get(),
               "FDIP prefetch fills by the path of their FTQ target"),
      ADD_STAT(fdipFillsUsed, statistics::units::Count::get(),
               "fillsUsed::fdip by the path of the prefetch's FTQ target"),
      ADD_STAT(fdipFillsUsedLate, statistics::units::Count::get(),
               "fillsUsedLate::fdip by the path of the prefetch's FTQ "
               "target"),
      ADD_STAT(fdipHits, statistics::units::Count::get(),
               "prefetchHitFillers::fdip by the path of the prefetch's FTQ "
               "target"),
      ADD_STAT(fdipMerges, statistics::units::Count::get(),
               "inherentMergeFillers::fdip by the path of the prefetch's "
               "FTQ target")
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
    for (auto *vec : {&fdipFills, &fdipFillsUsed, &fdipFillsUsedLate,
                      &fdipHits, &fdipMerges}) {
        vec->init(NumPfPaths)
            .subname(CorrectPath, "correctPath")
            .subname(WrongPath, "wrongPath")
            .flags(statistics::total);
    }
}

void
ShadowL1I::commitInst(ThreadID tid, InstSeqNum seq, uint64_t target_uid,
                      const FetchLines &lines, unsigned num_insts)
{
    shadowInsts += num_insts;
    resolveFills(tid, seq);

    if (lines[0].paddr == MaxAddr) {
        shadowNoPAddr++;
    } else {
        markCorrectPath(tid, target_uid, lines);
        for (const auto &line : lines) {
            if (line.paddr != MaxAddr) {
                accessLine(tid, line);
            }
        }
    }

    // Targets whose last op is this one or older have no op left to commit
    auto &retired = pfRetired[tid];
    while (!retired.empty() && retired.front().first <= seq) {
        auto it = pfTargets[tid].find(retired.front().second);
        if (it != pfTargets[tid].end()) {
            resolveTarget(tid, it, WrongPath);
        }
        retired.pop_front();
    }
}

void
ShadowL1I::prefetchQueued(ThreadID tid, uint64_t id, uint64_t target_uid,
                          Addr line_addr)
{
    auto &rec = prefetches[id];
    rec.targetUid = target_uid;
    rec.lineAddr = line_addr;
    // Once fetch has reached the target, its ops may already have
    // committed, so the path of a prefetch queued that late stays
    // unresolved
    if (target_uid != 0 && target_uid > lastFetchedUid[tid]) {
        pfTargets[tid][target_uid].prefetches.push_back(id);
    }
}

void
ShadowL1I::prefetchFilled(uint64_t id)
{
    auto it = prefetches.find(id);
    if (it != prefetches.end()) {
        if (it->second.filled) {
            return;
        }
        it->second.filled = true;
        if (it->second.path != Unresolved) {
            fdipFills[it->second.path]++;
        }
    }
    fdipFillsTotal++;
}

void
ShadowL1I::targetFetched(ThreadID tid, uint64_t target_uid)
{
    lastFetchedUid[tid] = std::max(lastFetchedUid[tid], target_uid);
    // Fetch takes targets in uid order, so an older target fetch never
    // started was squashed
    auto &targets = pfTargets[tid];
    for (auto it = targets.begin();
         it != targets.end() && it->first < target_uid;) {
        auto next = std::next(it);
        if (!it->second.fetched) {
            resolveTarget(tid, it, WrongPath);
        }
        it = next;
    }
    auto it = targets.find(target_uid);
    if (it != targets.end()) {
        it->second.fetched = true;
    }
}

void
ShadowL1I::targetRetired(ThreadID tid, uint64_t target_uid,
                         InstSeqNum first_seq, InstSeqNum last_seq)
{
    auto it = pfTargets[tid].find(target_uid);
    if (it == pfTargets[tid].end()) {
        return;
    }
    // No ops, or all of them already committed or squashed
    if (first_seq == 0 || lastCommitSeq[tid] >= last_seq) {
        resolveTarget(tid, it, WrongPath);
    } else {
        pfRetired[tid].emplace_back(last_seq, target_uid);
    }
}

void
ShadowL1I::markCorrectPath(ThreadID tid, uint64_t target_uid,
                           const FetchLines &lines)
{
    auto &targets = pfTargets[tid];
    auto it = targets.find(target_uid);
    if (target_uid == 0 || it == targets.end()) {
        return;
    }
    auto &ids = it->second.prefetches;
    for (const auto &line : lines) {
        if (line.paddr == MaxAddr) {
            continue;
        }
        const Addr blk_addr = blockAddr(line.paddr);
        for (auto i = ids.begin(); i != ids.end();) {
            auto &rec = prefetches[*i];
            if (rec.lineAddr == blk_addr) {
                resolvePrefetch(rec, CorrectPath);
                i = ids.erase(i);
            } else {
                ++i;
            }
        }
    }
    if (ids.empty()) {
        targets.erase(it);
    }
}

void
ShadowL1I::resolveTarget(ThreadID tid,
                         std::map<uint64_t, PfTarget>::iterator it,
                         PfPath path)
{
    for (uint64_t id : it->second.prefetches) {
        resolvePrefetch(prefetches[id], path);
    }
    pfTargets[tid].erase(it);
}

void
ShadowL1I::resolvePrefetch(PrefetchRecord &rec, PfPath path)
{
    if (rec.path != Unresolved) {
        return;
    }
    rec.path = path;
    if (rec.filled) {
        fdipFills[path]++;
    }
    if (rec.use == UsedTimely) {
        fdipFillsUsed[path]++;
    } else if (rec.use == UsedLate) {
        fdipFillsUsedLate[path]++;
    }
    fdipHits[path] += rec.pendingHits;
    fdipMerges[path] += rec.pendingMerges;
    rec.pendingHits = rec.pendingMerges = 0;
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
        auto it = src == Fdip ? prefetches.find(id) : prefetches.end();
        if (it != prefetches.end()) {
            auto &rec = it->second;
            rec.use = late ? UsedLate : UsedTimely;
            if (rec.path != Unresolved) {
                (late ? fdipFillsUsedLate : fdipFillsUsed)[rec.path]++;
            }
        }
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
            const Source src = source(line.filler);
            prefetchHitFillers[src]++;
            auto it = src == Fdip ? prefetches.find(line.filler)
                                  : prefetches.end();
            if (it != prefetches.end()) {
                if (it->second.path != Unresolved) {
                    fdipHits[it->second.path]++;
                } else {
                    it->second.pendingHits++;
                }
            }
        }
    }

    if (line.miss) {
        auto &bucket =
            shadow_hit ? onPathPollutionMisses : onPathInherentMisses;
        bucket[missKind(line.mshrMerge)]++;
        if (line.mshrMerge && !shadow_hit) {
            const Source src = source(line.filler);
            inherentMergeFillers[src]++;
            auto it = src == Fdip ? prefetches.find(line.filler)
                                  : prefetches.end();
            if (it != prefetches.end()) {
                if (it->second.path != Unresolved) {
                    fdipMerges[it->second.path]++;
                } else {
                    it->second.pendingMerges++;
                }
            }
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
