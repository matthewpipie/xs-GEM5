#include "mem/cache/prefetch/fdip.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/FDIP.hh"
#include "mem/cache/base.hh"
#include "mem/cache/shadow_l1i.hh"
#include "mem/packet.hh"
#include "params/FDIPPrefetcher.hh"
#include "sim/system.hh"

namespace gem5
{

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

FDIPPrefetcher::FDIPPrefetcher(const FDIPPrefetcherParams &p)
    : Queued(p), fdipStats(this)
{
}

void
FDIPPrefetcher::setParentInfo(System *sys, ProbeManager *pm,
                              CacheAccessor *_cache, unsigned blk_size)
{
    Queued::setParentInfo(sys, pm, _cache, blk_size);
    parentCache = dynamic_cast<BaseCache *>(_cache);
    fatal_if(!parentCache, "%s: FDIP prefetcher must be attached to a cache",
             name());
}

bool
FDIPPrefetcher::enqueue(const RequestPtr &req)
{
    assert(req->hasPaddr());
    fdipStats.received++;

    const Addr blk_paddr = blockAddress(req->getPaddr());

    if (!system->isMemAddr(blk_paddr)) {
        fdipStats.filteredNotMem++;
        return false;
    }

    // Build the prefetch info from a transient packet of the fetch request
    Packet trigger(req, MemCmd::ReadReq);
    PrefetchInfo pfi(&trigger, blk_paddr, true,
                     Request::XsMetadata(PrefetchSourceType::FDIP));

    if (cacheSnoop && (inCache(blk_paddr, pfi.isSecure()) ||
                       inMissQueue(blk_paddr, pfi.isSecure()))) {
        fdipStats.filteredInCache++;
        statsQueued.pfInCache++;
        DPRINTF(FDIP, "Drop pa %#lx (va %#lx), already in cache/MSHR\n",
                blk_paddr, req->getVaddr());
        return false;
    }

    const int32_t priority = 0;
    if (queueFilter && alreadyInQueue(pfq, pfi, priority)) {
        fdipStats.filteredInQueue++;
        DPRINTF(FDIP, "Drop pa %#lx (va %#lx), already queued\n", blk_paddr,
                req->getVaddr());
        return false;
    }

    statsQueued.pfIdentified++;
    DeferredPacket dpp(this, pfi, 0, priority);
    // Not initialized by the DeferredPacket constructor
    dpp.pfahead = false;
    dpp.pfahead_host = 0;
    const Tick pf_time = curTick() + clockPeriod() * latency;
    dpp.createPkt(blk_paddr, blkSize, requestorId, tagPrefetch, pf_time,
                  PrefetchSourceType::FDIP, 0);
    // Keep the request instruction-side for lower levels
    dpp.pkt->req->setFlags(Request::INST_FETCH);
    // Lets the shadow L1I attribute committed accesses to this prefetch
    dpp.pkt->req->setShadowFillId(req->getShadowFillId());
    addToQueue(pfq, dpp);
    fdipStats.queued++;

    DPRINTF(FDIP, "Queued pa %#lx (va %#lx, pc %#lx), pfq size %lu\n",
            blk_paddr, req->getVaddr(), req->hasPC() ? req->getPC() : 0,
            pfq.size());

    wakeParentCache();
    return true;
}

void
FDIPPrefetcher::notifyFill(const PacketPtr &pkt)
{
    const uint64_t id = pkt->req->getShadowFillId();
    if (shadowFillCallback && (id & ShadowL1I::PrefetchFillBit)) {
        shadowFillCallback(id);
    }
}

void
FDIPPrefetcher::squash(ContextID context_id)
{
    for (auto it = pfq.begin(); it != pfq.end();) {
        if (it->pfInfo.hasContextId() &&
            it->pfInfo.contextId() == context_id) {
            DPRINTF(FDIP, "Squash queued pa %#lx\n", it->pfInfo.getAddr());
            delete it->pkt;
            it = pfq.erase(it);
            fdipStats.squashed++;
        } else {
            ++it;
        }
    }
}

void
FDIPPrefetcher::wakeParentCache()
{
    // The cache only polls its prefetcher on its own activity, which may be
    // idle while fetch waits on a miss, so schedule the issue explicitly.
    const Tick next_pf_time = nextPrefetchReadyTime();
    if (next_pf_time != MaxTick) {
        parentCache->schedMemSideSendEvent(
            std::max(next_pf_time, parentCache->clockEdge()));
    }
}

FDIPPrefetcher::FDIPStats::FDIPStats(statistics::Group *parent)
    : statistics::Group(parent, "fdip"),
      ADD_STAT(received, statistics::units::Count::get(),
               "Number of line requests received from fetch"),
      ADD_STAT(queued, statistics::units::Count::get(),
               "Number of line requests inserted into the prefetch queue"),
      ADD_STAT(filteredNotMem, statistics::units::Count::get(),
               "Number of line requests dropped for non-memory addresses"),
      ADD_STAT(filteredInCache, statistics::units::Count::get(),
               "Number of line requests dropped as already in cache/MSHR"),
      ADD_STAT(filteredInQueue, statistics::units::Count::get(),
               "Number of line requests dropped as already queued"),
      ADD_STAT(squashed, statistics::units::Count::get(),
               "Number of queued prefetches dropped by frontend squash")
{
}

} // namespace prefetch
} // namespace gem5
