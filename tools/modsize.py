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
"""Logic size per unit from a hierarchical Gowin netlist (impl/gwsynthesis/*.vg).

Usage: modsize.py NETLIST [--depth N] [--min LUTS]
Counts primitives per module (LUT1-6 and MUX2* as LUT, ALU, DFF*, BSRAM, DSP), adds child instances, and prints
the instance tree to the given depth with totals and the share of all LUT+ALU.

What it is for. After synthesis, Gowin writes the design as a netlist in Verilog (the .vg file) that contains no
behavioral code any more, only instances of the chip's own cells. When the netlist is hierarchical (tools/build.sh,
HIER=1, which is the default) the module structure of the source survives, so every module of the RTL is still a module
in the netlist, holding its own cells and instances of its child modules. This script walks that structure and prints how
many cells each unit of the CPU (fetch, decode, execute, the FPU, the caches, ...) contains, so that one can see where
the chip area goes, which unit is big enough that a path through it is likely to be slow (the timing estimate uses the
size of a module as one input, see tools/timing_est.py), and what a change did to a unit. The same numbers feed the
region box plans that were tried and dropped (docs/learn/HISTORY.md, section 15).

Cell kinds counted, in the vocabulary of the Gowin GW5A architecture:
  LUT   a look-up table with up to 6 inputs (LUT1 to LUT6), plus the MUX2 cells that combine two LUTs into a wider
        function. This is the unit in which "logic size" is quoted everywhere in this project.
  ALU   a cell of a carry chain (an adder bit), counted next to the LUTs for the "logic" share.
  FF    a flip-flop or latch (DFF..., DL...), the registers.
  RAM   block RAM (BSRAM) and ROM primitives, and the 16-entry distributed RAM (RAM16).
  DSP   multiplier and multiply-add blocks.
Output columns: unit path, LUT, ALU, FF, RAM, DSP, and the share of all LUT + ALU in the design in percent.
Options: --depth N prints child units down to N levels below the top (default 3); --min LUTS hides units below that
many LUT + ALU cells (default 500).
"""
import re, sys, argparse, collections

def kind(cell):
    """Class of a netlist cell type: 'lut', 'alu', 'ff', 'ram', 'dsp', or None when it is not a counted primitive."""
    if re.match(r'LUT\d', cell) or cell.startswith('MUX2'): return 'lut'   # LUT1..LUT6 and the MUX2_LUT5..8 wide muxes
    if cell == 'ALU' or cell.startswith('ALU'): return 'alu'               # carry chain cell
    if re.match(r'DFF|DL', cell): return 'ff'                              # flip-flops and latches
    if re.match(r'(SDPB|DPB|SP|pROM|SDPX9B|DPX9B|SPX9|pROMX9|RAM16)', cell): return 'ram'   # block RAM and ROM variants, LUT RAM
    if re.match(r'(MULT|PADD|MULTALU|MULTADDALU)', cell): return 'dsp'     # multipliers and pre-adders of the DSP blocks
    return None                                                            # anything else is a child module or an unimportant cell

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('vg'); ap.add_argument('--depth', type=int, default=3)
    ap.add_argument('--min', type=int, default=500); a = ap.parse_args()   # command line: the netlist, how deep to print, smallest unit to print
    txt = open(a.vg, errors='ignore').read()                               # the whole netlist as one string (tens of MB for the full chip)
    own, kids = {}, {}                                                     # per module: its own primitive counts, and its child instances
    for m in re.finditer(r'^module\s+(\S+?)\s*\((.*?)^endmodule', txt, re.S | re.M):   # each "module name (...) ... endmodule" block
        name, body = m.group(1), m.group(2)
        c = collections.Counter(); k = []                                  # counts of this module's own primitives; its non-primitive instances
        for im in re.finditer(r'^[ \t]*(\\?\S+)[ \t]+(\\?\S+)[ \t]*\(', body, re.M):   # an instance line is "celltype instname (": two words and a bracket
            cell, inst = im.group(1), im.group(2)
            if cell in ('module', 'input', 'output', 'inout', 'wire', 'assign', 'reg', 'defparam'): continue   # declarations look similar, skip them
            t = kind(cell)
            if t: c[t] += 1                                                # a primitive: count it
            else: k.append((cell, inst))                                   # not a primitive: probably an instance of another module
        own[name], kids[name] = c, k
    memo = {}                                                              # totals per module, so each module is summed once
    def tot(mod):
        """Own primitives of a module plus those of all modules below it."""
        if mod in memo: return memo[mod]
        c = collections.Counter(own.get(mod, {}))
        for cell, _ in kids.get(mod, []):
            if cell in own: c += tot(cell)                                 # only instances of modules defined in this file are descended into
        memo[mod] = c; return c
    tops = set(own) - {c for v in kids.values() for c, _ in v}             # a top module is one that no module instantiates
    top = max(tops, key=lambda m: sum(tot(m).values()))                    # if there are several, the biggest one is the design
    T = tot(top); all_l = T['lut'] + T['alu']                              # whole-design LUT + ALU count, the 100 percent of the last column
    print(f"{'unit':60s} {'LUT':>7s} {'ALU':>6s} {'FF':>7s} {'RAM':>4s} {'DSP':>4s} {'logic%':>6s}")
    def walk(mod, path, d):
        """Print this unit, then its children (biggest first) when they are big enough and not deeper than --depth."""
        c = tot(mod); l = c['lut'] + c['alu']
        print(f"{path[:60]:60s} {c['lut']:7d} {c['alu']:6d} {c['ff']:7d} {c['ram']:4d} {c['dsp']:4d} {100*l/all_l:6.1f}")
        if d >= a.depth: return
        ch = [(cell, inst) for cell, inst in kids.get(mod, []) if cell in own]   # children that are modules of the netlist
        for cell, inst in sorted(ch, key=lambda x: -(tot(x[0])['lut'] + tot(x[0])['alu'])):   # largest first
            if tot(cell)['lut'] + tot(cell)['alu'] >= a.min:               # skip tiny units to keep the table short
                walk(cell, path + '/' + inst.lstrip('\\'), d + 1)           # path shows instance names, as the netlist escapes them with a backslash
    walk(top, top, 0)

if __name__ == '__main__':
    main()
