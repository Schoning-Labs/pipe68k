#!/usr/bin/env python3
# Copyright 2026 Franz Schoning
# Project: https://github.com/Schoning-Labs/pipe68k
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Logic levels counted Gowin's way, from the synthesized netlist (thin wrapper around timing_est.py --levels).

usage: tools/levels.py NETLIST.vg [--min N] [--mcp SUBSTR]
Gowin's timing report counts the launching register as 1, each LUT, MUX2_LUTx and INV (it becomes a LUT1) as 1, and a carry
chain by its length (1 cell 1, 2-6 cells 2, 7-17 cells 3, 18-27 cells 4); this matches 3196 of 3200 reported paths.
Levels alone do not predict timing (8-level paths ran from 13.3 to 20.1 ns in the same build); use tools/timing_est.py.

Background for a beginner. A signal in a synchronous circuit travels from one flip-flop (register), through a cloud of
combinational logic, to the next flip-flop, and must arrive within one clock period. On this FPGA the logic cloud is
built from LUTs (look-up tables, small memories that implement any function of up to 4 to 6 inputs, one "level" each),
sometimes joined by wide multiplexers, and from carry chains (the fast adder cells, ALU in Gowin's netlist). The number of
LUTs a signal passes through on its way is the number of logic levels. Gowin's guide SUG113 (section 4.3) tells the user
to cut any path of more than 8 levels at 100 MHz, which is why this script lists the paths above 8 (its default).

The levels are only half the story, because the wires between the LUTs usually take about 85 percent of a path's delay
and their length depends on placement. Two paths of 8 levels ran from 13.3 to 20.1 ns in the same build. The count is kept
as a quick screen and because tools/build.sh prints its last line after every synthesis. The counting itself is
done in tools/timing_est.py (function max_levels), and this file only translates its own command line (--min N) into
the --levels option of that tool.

Options: --min N lists paths above N levels (default 8). --mcp SUBSTR names a module (for example i_fpe/) whose
registers are timed as multicycle paths: only paths that start outside it are counted for endpoints inside it, because a
path that starts inside it gets several clocks and so a larger allowance.
Output: one line per group of endpoints with the level count and the endpoint name, and a last line "LEVELS: ...".
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # find timing_est.py next to this file, whatever the current folder is
import timing_est

if __name__ == '__main__':
    args, n = sys.argv[1:], '8'                     # the command line without the script name; the default threshold is 8 levels
    if '--min' in args:                             # the user gave their own threshold
        k = args.index('--min'); n = args[k + 1]; del args[k:k + 2]   # take the value out of the list so timing_est does not see --min
    sys.argv = [sys.argv[0]] + args + ['--levels', n]   # timing_est.main() reads sys.argv itself: hand it the translated command line
    timing_est.main()                               # run the level count there
