/**
 * @file
 * Fetch-directed instruction prefetcher (FDIP), cache side.
 *
 * This prefetcher does not train on cache accesses. The O3 fetch stage walks
 * the fetch target queue (FTQ) ahead of the fetch pointer, translates the
 * cache lines of future fetch targets, and pushes the physical line
 * addresses here. The owning L1 I-cache then issues them through its regular
 * hardware-prefetch path, which keeps demand priority, the demand MSHR
 * reserve, tag-port arbitration and prefetch usefulness accounting.
 */

#ifndef __MEM_CACHE_PREFETCH_FDIP_HH__
#define __MEM_CACHE_PREFETCH_FDIP_HH__

#include <functional>

#include "base/statistics.hh"
#include "base/types.hh"
#include "mem/cache/prefetch/queued.hh"
#include "mem/request.hh"

namespace gem5
{

class BaseCache;
struct FDIPPrefetcherParams;

GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
namespace prefetch
{

class FDIPPrefetcher : public Queued
{
  public:
    FDIPPrefetcher(const FDIPPrefetcherParams &p);

    void setParentInfo(System *sys, ProbeManager *pm, CacheAccessor *_cache,
                       unsigned blk_size) override;

    // No training from cache accesses, requests are pushed by fetch
    void calculatePrefetch(const PrefetchInfo &pfi,
                           std::vector<AddrPriority> &addresses) override {}
    void notify(const PacketPtr &pkt, const PrefetchInfo &pfi) override {}

    /** Reports the shadow L1I fill id of each fill of an FDIP prefetch */
    void notifyFill(const PacketPtr &pkt) override;

    /** Called with the shadow L1I fill id of every FDIP prefetch fill */
    void
    setShadowFillCallback(std::function<void(uint64_t)> callback)
    {
        shadowFillCallback = std::move(callback);
    }

    /**
     * Queue a prefetch for the cache line of a translated fetch request.
     * @param req translated request carrying paddr, vaddr, pc and context
     * @return true if a prefetch was queued, false if it was filtered
     */
    bool enqueue(const RequestPtr &req);

    /**
     * Drop queued, not yet issued prefetches of a context, e.g. on a
     * frontend redirect.
     */
    void squash(ContextID context_id);

  private:
    BaseCache *parentCache = nullptr;

    std::function<void(uint64_t)> shadowFillCallback;

    void wakeParentCache();

    struct FDIPStats : public statistics::Group
    {
        FDIPStats(statistics::Group *parent);

        statistics::Scalar received;
        statistics::Scalar queued;
        statistics::Scalar filteredNotMem;
        statistics::Scalar filteredInCache;
        statistics::Scalar filteredInQueue;
        statistics::Scalar squashed;
    } fdipStats;
};

} // namespace prefetch
} // namespace gem5

#endif // __MEM_CACHE_PREFETCH_FDIP_HH__
