# kmhv3_fdip.py: kmhv3.py plus fetch-directed instruction prefetch (FDIP).
#
# This file is a copy of configs/example/kmhv3.py. kmhv3.py itself is left
# unchanged because it is the RTL-matched reference configuration. Every
# difference from kmhv3.py is listed here and marked with an "FDIP:" comment
# in the code, so `diff kmhv3.py kmhv3_fdip.py` shows only these blocks:
#
#   1. FDIP-only command line options are parsed from sys.argv and removed
#      before xiangshan_system_init(), so the shared option parser is not
#      modified. See add_fdip_options() for the list and defaults.
#   2. When caches are enabled, setKmhV3Params() attaches an FDIPPrefetcher
#      to the L1 I-cache, enables FDIP in the O3 fetch stage and passes the
#      FDIP knobs to the CPU. --no-fdip skips this block, which gives an
#      FDIP-off run from the same config file.
#   3. EXPLORATION ONLY, NOT RTL-JUSTIFIED: --l1i-mshrs N overrides the L1I
#      MSHR count. It defaults to unset, which keeps the RTL-matched value.
#      It exists to measure how much FDIP is limited by MSHRs; results with
#      it set do not model the RTL. It applies with or without --no-fdip.
#
# Deliberately NOT changed (must stay RTL-matched until the RTL says
# otherwise): L1I MSHR count (4, unless --l1i-mshrs is given),
# demand_mshr_reserve (1), tag_load_read_ports, FTQ size, and every other
# non-FDIP structure size. With mshrs=4 and demand_mshr_reserve=1 the cache
# issues a prefetch only while fewer than 2 MSHRs are allocated
# (MSHRQueue::canPrefetch()).
#
# When kmhv3.py changes, re-copy it and re-apply the FDIP-marked blocks.
#
# Example sweep through the SPEC06 suite runner:
#   run_suite.py <selector> --config configs/example/kmhv3_fdip.py \
#       --extra-args --fdip-distance 16
import argparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *

addToPath('../')

from ruby import Ruby
from common.LSQBankConflict import set_lsq_bank_conflict_cache_params

from common.FSConfig import *
from common.SysPaths import *
from common.Benchmarks import *
from common import Simulation
from common.Caches import *
from common.xiangshan import *

# FDIP: parse FDIP-only options before the shared parser sees sys.argv
def add_fdip_options():
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--no-fdip", action="store_true", default=False,
                        help="Disable FDIP (same config, FDIP-off baseline)")
    parser.add_argument("--fdip-distance", type=int, default=8,
                        help="Max FTQ targets the prefetch pointer runs "
                             "ahead of the fetching target")
    parser.add_argument("--fdip-issue-width", type=int, default=2,
                        help="Max FDIP line requests generated per cycle")
    parser.add_argument("--fdip-max-inflight", type=int, default=8,
                        help="Max FDIP ITLB translations in flight")
    parser.add_argument("--fdip-filter-size", type=int, default=16,
                        help="Entries in the recently requested line filter")
    parser.add_argument("--fdip-queue-size", type=int, default=16,
                        help="L1I FDIP prefetch queue entries")
    parser.add_argument("--l1i-mshrs", type=int, default=None,
                        help="EXPLORATION ONLY: override the L1I MSHR count "
                             "(default: keep the RTL-matched value)")
    fdip_args, rest = parser.parse_known_args(sys.argv[1:])
    sys.argv = sys.argv[:1] + rest
    return fdip_args

def setFdipParams(fdip_args, cpu):
    cpu.icache.prefetcher = FDIPPrefetcher(
        queue_size=fdip_args.fdip_queue_size)
    cpu.fdipEnable = True
    cpu.fdipPrefetcher = cpu.icache.prefetcher
    cpu.fdipDistance = fdip_args.fdip_distance
    cpu.fdipIssueWidth = fdip_args.fdip_issue_width
    cpu.fdipMaxInflight = fdip_args.fdip_max_inflight
    cpu.fdipFilterSize = fdip_args.fdip_filter_size
# FDIP: end

def setKmhV3Params(args, system, fdip_args):  # FDIP: extra fdip_args
    for cpu in system.cpu:

        # fetch (idealfetch not care)
        cpu.mmu.itb.size = 96
        cpu.fetchWidth = 32
        cpu.iewToFetchDelay = 2 # for resolved update, should train branch after squash
        cpu.commitToFetchDelay = 2
        cpu.fetchQueueSize = 64

        # decode
        cpu.decodeWidth = 8
        cpu.enable_loadFusion = False
        cpu.enableConstantFolding = False

        # rename
        cpu.renameWidth = 8
        cpu.numPhysIntRegs = 224
        cpu.numPhysFloatRegs = 256
        cpu.enable_storeSet_train = False

        # dispatch
        cpu.enableDispatchStage = False
        cpu.numDQEntries = [8, 8, 8]
        cpu.dispWidth = [8, 8, 8]

        # scheduler
        cpu.scheduler = KMHV3Scheduler()
        cpu.scheduler.disableAllRegArb()
        cpu.scheduler.enableMainRdpOpt = False
        cpu.scheduler.intRegfileBanks = 1
        # intiq0
        cpu.scheduler.IQs[0].oports[0].rp = [IntRD(0, 0), IntRD(1, 0)]
        cpu.scheduler.IQs[0].oports[1].rp = [IntRD(0, 1), IntRD(1, 1)]

        # intiq1
        cpu.scheduler.IQs[1].oports[0].rp = [IntRD(2, 0), IntRD(3, 0)]
        cpu.scheduler.IQs[1].oports[1].rp = [IntRD(2, 1), IntRD(3, 1)]

        # intiq2
        cpu.scheduler.IQs[2].oports[0].rp = [IntRD(4, 0), IntRD(5, 0)]
        cpu.scheduler.IQs[2].oports[1].rp = [IntRD(4, 1), IntRD(5, 1)]

        # rob
        cpu.commitWidth = 8
        cpu.squashWidth = 8
        cpu.phyregReleaseWidth = 8
        cpu.RobCompressPolicy = 'none'
        cpu.numROBEntries = 352
        cpu.CROB_instPerGroup = 2 # 1 if not using ROB compression

        # lsu
        cpu.StoreWbStage = 4
        cpu.EnableLdMissReplay = True
        cpu.EnablePipeNukeCheck = True
        cpu.BankConflictCheck = True
        cpu.sbufferBankWriteAccurately = False

        # lsq
        cpu.LQEntries = 72
        cpu.SQEntries = 56
        cpu.RARQEntries = 72
        cpu.RAWQEntries = 32
        cpu.LoadCompletionWidth = 8
        cpu.StoreCompletionWidth = 4
        cpu.RARDequeuePerCycle = 4
        cpu.RAWDequeuePerCycle = 4
        cpu.SbufferEntries = 16
        cpu.SbufferEvictThreshold = 7
        cpu.store_prefetch_train = False

        # branch predictor
        if args.bp_type == 'DecoupledBPUWithBTB':
            cpu.branchPred.ftq_size = 64
            cpu.branchPred.fsq_size = 64

            if args.btb_tage_upper_bound:
                cpu.branchPred.tage = BTBTAGEUpperBound(
                    usePathHashHistory=True)

            cpu.branchPred.mbtb.resolvedUpdate = True
            cpu.branchPred.tage.resolvedUpdate = True
            cpu.branchPred.ittage.resolvedUpdate = True

            cpu.branchPred.ubtb.enabled = True
            cpu.branchPred.abtb.enabled = True
            cpu.branchPred.microtage.enabled = True
            cpu.branchPred.mbtb.enabled = True
            cpu.branchPred.tage.enabled = True
            cpu.branchPred.ittage.enabled = True
            cpu.branchPred.mgsc.enabled = True
            cpu.branchPred.ras.enabled = True

            if getattr(args, 'standalone_sc', False):
                cpu.branchPred.microtage.enabled = False
                cpu.branchPred.tage.enabled = False

                cpu.branchPred.mgsc.forceUseSC = True
                cpu.branchPred.mgsc.allowMissingTageInfo = True

        # l1 cache per core
        if args.caches:
            cpu.icache.size = '64kB'
            cpu.dcache.size = '64kB'
            cpu.dcache.tag_load_read_ports = 3
            cpu.dcache.mshrs = 16
            cpu.dcache.do_fast_writeline = False
            cpu.dcache.simulate_dcache_refill = True
            cpu.dcache.prefetch_can_offload = False
            set_lsq_bank_conflict_cache_params(cpu, system)
            # Committed-only shadow L1I, off unless --shadow-l1i
            set_shadow_l1i_params(args, system, cpu)

            # FDIP: attach the L1I FDIP prefetcher and enable FDIP in fetch
            if not fdip_args.no_fdip:
                setFdipParams(fdip_args, cpu)
            # FDIP: exploration-only L1I MSHR override, not RTL-justified
            if fdip_args.l1i_mshrs is not None:
                cpu.icache.mshrs = fdip_args.l1i_mshrs

    # l2 caches
    if args.l2cache:
        for i in range(args.num_cpus):
            if args.classic_l2:
                system.l2_caches[i].slice_num = 4
                system.l2_caches[i].wpu = NULL
                system.l2_caches[i].do_fast_writeline = False
                system.l2_caches[i].prefetch_can_offload = False
                # Configure XSDRRIP replacement policy (DRRIP mode)
                # L2: 2MB, 8-way, 64B line → 4096 sets
                system.l2_caches[i].replacement_policy = XSDRRIPRP(mode=2, num_sets=4096)
            else:
                l2_wrapper = system.l2_wrappers[i]
                l2_wrapper.data_sram_banks = 1
                l2_wrapper.dir_sram_banks = 1
                l2_wrapper.pipe_dir_write_stage = 3
                l2_wrapper.dir_read_bypass = False
                for j in range(args.l2_slices):
                    l2_wrapper.slices[j].inner_cache.wpu = NULL
                    l2_wrapper.slices[j].inner_cache.do_fast_writeline = False
                    l2_wrapper.slices[j].inner_cache.prefetch_can_offload = False
                    # Configure XSDRRIP replacement policy (DRRIP mode)
                    # Each slice: 2MB/4 = 512KB, 8-way, 64B line → 1024 sets
                    l2_wrapper.slices[j].inner_cache.replacement_policy = XSDRRIPRP(mode=2, num_sets=1024)
            system.tol2bus_list[i].forward_latency = 3  # 3->0
            system.tol2bus_list[i].response_latency = 3  # 3->0
            system.tol2bus_list[i].hint_wakeup_ahead_cycles = 1  # 1->0

            # Enable dual-port for DCache → L2 communication
            # ReqLayer[0]: ICache+DCache+ITB+DTB → L2, allow 2 requests per cycle
            # RespLayer[1]: L2 → DCache, allow 2 responses per cycle
            # system.tol2bus_list[i].layer_bandwidth_configs = [
            #     LayerBandwidthConfig(direction="req", port_index=0, max_per_cycle=2),
            #     LayerBandwidthConfig(direction="resp", port_index=1, max_per_cycle=2),
            # ]

    # l3 cache
    if args.l3cache:
        system.l3.mshrs = 64
        system.l3.do_fast_writeline = False
        system.l3.prefetch_can_offload = False
        system.l3.num_slices = 4

if __name__ == '__m5_main__':
    FutureClass = None

    fdip_args = add_fdip_options()  # FDIP: must run before the shared parser
    args = xiangshan_system_init()

    assert not args.external_memory_system

    # Set default bp_type based on ideal_kmhv3 flag
    # If user didn't specify bp_type, set default based on ideal_kmhv3
    args.bp_type = 'DecoupledBPUWithBTB'
    args.l2_size = '1MB'
    args.kmh_align = True   # align prefetcher in RTL, spec06 decrease 1 score

    # Match the memories with the CPUs, based on the options for the test system
    TestMemClass = Simulation.setMemClass(args)

    test_sys = build_xiangshan_system(args)
    if args.raw_cpt and args.generic_rv_cpt and os.path.basename(args.generic_rv_cpt) == "linux.bin":
        configure_xiangshan_linux_workload(test_sys, args)
    # Set ideal parameters here with the highest priority, over command-line arguments
    setKmhV3Params(args, test_sys, fdip_args)  # FDIP: pass fdip_args

    root = Root(full_system=True, system=test_sys)

    Simulation.run_vanilla(args, root, test_sys, FutureClass)
