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
"""Writes rtl/core/p68_pdrom.vh: the fill-predecode tables of p68_pdrom.v, from the functions in rtl/core/p68_pdf.vh.

usage: tools/gen_pdrom.py      (needs iverilog and vvp: from the folder OSS_CAD_SUITE named in settings.ini or the
                                environment, else from the PATH)

Why it exists. The I-cache stores a 12-bit predecode next to every 16-bit word (instruction length, sequence flag and
the pairing classes, as if the word were an opcode). As logic that was about 1,100 LUTs in fetch, so the core reads it
from two levels of block ROM instead (p68_pdrom.v). The ROM contents must be exactly what the Verilog functions in
p68_pdf.vh compute, and the simplest way to be sure of that is to let a Verilog simulator evaluate those very functions
for every possible word. So this script writes a tiny test bench that includes the same header files the core uses,
runs it with Icarus Verilog once without and once with the FPU, and turns the two lists of 65,536 values into tables.

The tables:
T1 (1024 x 7): bits 15-6 of a word -> class (rows with the same 64 predecodes for both FPU settings share a class).
T2 (8192 x 12): {class, bits 5-0} -> predecode; one value per FPU setting, chosen by `P68_FPU. Every entry is assigned.

A "row" is the 64 words that share bits 15-6. Most rows are identical (e.g. every row of an unused opcode area), so
there are only about 75 classes, and 7 bits of class keep T1 small. The script stops if there are ever more than 128.

The script writes the license and explanation header of p68_pdrom.vh itself, then the tables. After running it, run the
lockstep bench to check the new tables.
"""
import os, subprocess, sys, tempfile
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # repository root (this file is in tools/)
core = os.path.join(root, 'rtl', 'core')                            # where the Verilog headers and the output live
def _setting(name):                                                 # NAME from the environment, else from settings.ini (NAME=value lines)
    if os.environ.get(name): return os.environ[name]
    try:
        for ln in open(os.path.join(root, 'settings.ini'), encoding='utf-8'):
            ln = ln.strip()
            if ln.startswith(name + '='): return ln.split('=', 1)[1].strip().strip("'\"")
    except OSError: pass
    return ''
_oss = _setting('OSS_CAD_SUITE')
VL = (_oss + '/bin') if _oss else ''                                # folder with iverilog and vvp; empty: both come from the PATH
# The test bench: it includes the same function headers, in the same order, as p68_fetch.v does, and writes
# pdall(w) for every word w as three hex digits per line.
TB = r"""`include "p68_uop.vh"
module tb;
`include "p68_dec.vh"
`include "p68_sysf.vh"
`include "p68_md.vh"
`include "p68_mdf.vh"
`include "p68_flowf.vh"
`include "p68_bff.vh"
`include "p68_p7f.vh"
`include "p68_pdf.vh"
integer i, f; reg [15:0] w;
initial begin
  f = $fopen("pd.txt", "w");
  for (i = 0; i < 65536; i = i + 1) begin w = i; $fwrite(f, "%03x\n", pdall(w)); end
  $fclose(f); $finish;
end
endmodule
"""
HEAD = r"""// Copyright 2026 Franz Schoning
// Project: https://github.com/Schoning-Labs/pipe68k
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ==== p68_pdrom.vh: GENERATED contents of the fill-predecode tables ==================================================
//
// This file is generated, so it is not commented line by line and must not be edited by hand. It is the initial block
// that fills the two ROM tables of p68_pdrom.v (module p68_pdr2, where it is included inside the module body):
//   t1[r] (1024 entries, 7 bits): the row class of all opcode words whose bits 15-6 equal r.
//   t2[c * 64 + l] (8192 entries, 12 bits): the predecode of a word of row class c whose bits 5-0 equal l. Each entry
//   holds two values, one for a build with the FPU and one without, and `P68_FPU picks one when the file is read.
// The predecode bits are described in p68_pdf.vh (function pdall).
//
// To regenerate it, run tools/gen_pdrom.py (it needs Icarus Verilog from the oss-cad-suite). The script evaluates the
// predecode functions of p68_pdf.vh over all 65,536 words, once with and once without the FPU, groups identical rows
// into classes and writes this file, including this header. Do that after every change to p68_pdf.vh or to the decode
// helpers it calls, then run the lockstep bench.
"""
tabs = []                                                            # tabs[0]: predecodes without the FPU, tabs[1]: with
with tempfile.TemporaryDirectory() as d:                             # scratch folder, removed afterwards
    open(os.path.join(d, 'tb.v'), 'w').write(TB)
    for fpu in (0, 1):
        # compile with P68_FPU set, the core headers on the include path, then run and read the 65,536 values back
        subprocess.check_call([(VL + '/' if VL else '') + 'iverilog', '-g2012', '-DP68_FPU=%d' % fpu, '-I' + core, '-o', 'tb', 'tb.v'], cwd=d)
        subprocess.check_call([(VL + '/' if VL else '') + 'vvp', 'tb'], cwd=d, stdout=subprocess.DEVNULL)
        tabs.append([int(l, 16) for l in open(os.path.join(d, 'pd.txt'))])
# Level 1: give each row of 64 words a class number; rows whose 64 values match (for both FPU settings) share it.
cls, t1 = {}, []
for r in range(1024):
    k = tuple(tabs[0][r * 64:(r + 1) * 64]) + tuple(tabs[1][r * 64:(r + 1) * 64])   # the row's values, both settings
    t1.append(cls.setdefault(k, len(cls)))                         # known row: its class; new row: the next number
if len(cls) > 128: sys.exit('%d classes: T1 needs more than 7 bits' % len(cls))
# Level 2: for each class, the 64 predecodes (as a pair: without FPU, with FPU). Unused classes stay 0.
t2 = [(0, 0)] * 8192
for r in range(1024):
    for c in range(64): t2[t1[r] * 64 + c] = (tabs[0][r * 64 + c], tabs[1][r * 64 + c])
# Write the Verilog initial block; each T2 entry chooses its value by `P68_FPU when the core is compiled.
out = HEAD.splitlines() + ['// Generated by tools/gen_pdrom.py from p68_pdf.vh (%d classes). Do not edit.' % len(cls), 'initial begin']
out += ["  t1[%d] = 7'd%d;" % (r, v) for r, v in enumerate(t1)]
out += ["  t2[%d] = (`P68_FPU != 0) ? 12'h%03x : 12'h%03x;" % (i, b, a) for i, (a, b) in enumerate(t2)]
out.append('end')
open(os.path.join(core, 'p68_pdrom.vh'), 'w', newline='\n').write('\n'.join(out) + '\n')
print('classes', len(cls))
