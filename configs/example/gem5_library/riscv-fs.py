# Copyright (c) 2021 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This example runs a simple linux boot. It uses the 'riscv-disk-img' resource.
It is built with the sources in `src/riscv-fs` in [gem5 resources](
https://gem5.googlesource.com/public/gem5-resources).

Characteristics
---------------

* Runs exclusively on the RISC-V ISA with the classic caches
* Assumes that the kernel is compiled into the bootloader
* Automatically generates the DTB file
* Will boot but requires a user to login using `m5term` (username: `root`,
  password: `root`)
"""

import argparse
import os
import m5
import m5.util

m5.util.addToPath("../../")

from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.cachehierarchies.classic.\
    private_l1_private_l2_cache_hierarchy import (
        PrivateL1PrivateL2CacheHierarchy,
    )
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.utils.requires import requires
from gem5.resources.resource import Resource, CustomResource
from gem5.simulate.simulator import Simulator
from gem5.simulate.exit_event import ExitEvent
from common import SysPaths

# Run a check to ensure the right version of gem5 is being used.
requires(isa_required=ISA.RISCV)

default_disk = 'riscv-disk-img'
default_kernel = 'riscv-bootloader-vmlinux-5.10'

cpu_types = {
    "atomic" : CPUTypes.ATOMIC,
    "timing" : CPUTypes.TIMING,
    "minor"  : CPUTypes.MINOR,
    "o3"     : CPUTypes.O3,
    "kvm"    : CPUTypes.KVM,
}

def addOptions(parser):
    parser.add_argument("--restore-from", type=str, default=None,
                        help="Restore from checkpoint")
    parser.add_argument("--kernel", type=str, default=None,
                        help="Linux kernel")
    parser.add_argument("--disk", type=str, default=None,
                        help="Disks to instantiate")
    parser.add_argument("--cpu-type", type=str, choices=list(cpu_types.keys()),
                        default="timing",
                        help="CPU simulation mode. Default: %(default)s")
    return parser

def instantiate(options):
    """Instantiate Simulator object."""
    cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
        l1d_size="32KiB", l1i_size="32KiB", l2_size="512KiB")
    # Setup the system memory.
    memory = SingleChannelDDR3_1600()

    # Setup a single core Processor.
    processor = SimpleProcessor(
        cpu_type=cpu_types[options.cpu_type],
        isa=ISA.RISCV,
        num_cores=1)

    # Setup the board.
    board = RiscvBoard(
        clk_freq="1GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy)

    if options.kernel is None or len(options.kernel) == 0:
        kernel = Resource(default_kernel)
    elif os.path.isabs(options.kernel):
        kernel = CustomResource(local_path=options.kernel)
    else:
        kernel = CustomResource(local_path=SysPaths.binary(options.kernel))

    if options.disk is None or len(options.disk) == 0:
        disk = Resource(default_disk)
    elif os.path.isabs(options.disk):
        disk = CustomResource(local_path=options.disk)
    else:
        disk = CustomResource(local_path=SysPaths.disk(options.disk))

    # Set the Full System workload.
    board.set_kernel_disk_workload(kernel=kernel, disk_image=disk)

    # Set checkpoint path
    if options.restore_from:
        if not os.path.isabs(options.restore_from):
            cpt = options.restore_from
        else:
            cpt = os.path.abspath(options.restore_from)
            if not os.path.isdir(cpt):
                cpt = os.path.join(m5.options.outdir, options.restore_from)
        if not os.path.isdir(cpt):
            raise IOError("Can't find checkpoint directory")
        simulator = Simulator(board=board, checkpoint_path=cpt)
    else:
        simulator = Simulator(board=board)

    return simulator

def main():
    parser = argparse.ArgumentParser(
        description="Generic RISC-V Full System configuration")
    addOptions(parser)
    options = parser.parse_args()
    simulator = instantiate(options)
    print("Beginning simulation!")
    simulator.run()
    exit_cause = simulator.get_last_exit_event_cause()
    print('last exit event: %s' % exit_cause)
    exit_enum = ExitEvent.translate_exit_status(exit_cause)
    if exit_enum is ExitEvent.CHECKPOINT:
        simulator.save_checkpoint(m5.options.outdir)
        print('Saved checkpoint')


if __name__ == "__m5_main__":
    main()
