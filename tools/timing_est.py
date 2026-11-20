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
"""Path delay estimate per endpoint and per clock from a GowinSynthesis netlist (t.vg), before place-and-route.

usage: tools/timing_est.py NETLIST.vg [--sdc FILE.sdc] [--period NS] [--margin M] [--model FILE] [--all] [--top-module M]
       tools/timing_est.py --fit NETLIST.vg SDC REPORT.tr [--fit ...] [--model FILE]   (calibrate on place-and-route reports)
       tools/timing_est.py NETLIST.vg --levels N [--mcp SUBSTR]                         (Gowin logic levels above N; levels.py)

Clocks come from the SDC the build uses: create_clock, create_generated_clock (-multiply_by, -divide_by, -duty_cycle,
-edges, -invert, -phase), set_clock_groups (no paths between groups), set_false_path, set_multicycle_path and
set_max_delay (-from / -to with get_clocks, get_regs, get_cells, get_pins; -through is not supported and is ignored).
Each register, RAM or DSP clock pin is traced back through buffers, inverters, LUTs and clock selectors (DCS, DQCE, BUFG)
to a clock source: a net, pin or port named in the SDC. A source the SDC does not name gets its own clock with the default
period (--period, or Gowin's 10 ns when not given); a CLKDIV the SDC does not name is derived from its input (DIV_MODE).
Every path is checked against its own launch and capture edges (the smallest positive distance, as Gowin's STA); negative
edge registers (DFFN*) and inverted clocks count; multicycle and max delay replace the requirement; paths between
asynchronous groups and false paths are skipped. Without --sdc every clock source gets --period (default 12.5) and all
clocks are related.

For every endpoint (register / RAM / DSP input) it estimates the slowest path from a register, RAM or DSP output:
  est     estimated data path delay in ns (clock-to-out + cells + nets + setup; clock skew taken as 0)
  req     the requirement of that path (launch -> capture edge, multicycle, max delay); slack = req - est
  lvl     Gowin logic levels of that path (start register 1, each LUT / MUX2_LUTx / INV 1, a carry chain 1..5 by length)
  cell, local, fanout, far: ns in cells, in local nets, in nets with fanout > 16, in nets whose ends sit in a module of
          more than 2048 cells (why a path is slow)
Net delays are not known before placement. Each connection gets a delay from its fanout, the size of the smallest module
holding both ends and the number of modules among the loads. The weights are fitted per endpoint on real place-and-route
reports (--fit): the estimate of every reported endpoint should match its real worst path, and an endpoint not in the
report must stay below the report's bound (its real slack is known to be better). Model: tools/timing_model.json.
Accuracy and the design rules that follow: docs/learn/HISTORY.md, section 15. SSRAM (RAM16SDP4) is treated as a register.
Last lines: 'CLOCK: name period P endpoints N above: n worst est E req R slack S' per capture clock, then
'TIMING: n endpoints above the requirement - M ns ...'. Exit status 0 always.

Timing closure, for a beginner. A digital circuit on an FPGA is a set of registers (flip-flops) that all take a new value
on the rising edge of a clock, and combinational logic between them. A signal leaves a register right after a clock edge
(the clock-to-output time), travels through the logic (look-up tables, called LUTs, each one a "logic level") and over
the wires between them (the net delay), and must have arrived at the next register a little before the next edge: the
register's setup time, the time its input must be stable before the edge. The time available is one clock period
(12.5 ns at 80 MHz, 15 ns at 66.7 MHz). The slack of a path is the period minus the path's total delay, so a positive slack
is a path that makes it and a negative slack is a path that fails; the sum of the failing slacks is the total negative slack
(TNS). The hold time is the opposite requirement, that a new value must not arrive so early that it is taken by the
register in the same edge; fast paths can violate it, and Gowin's router repairs those by adding delay.
Timing closure is the work of getting every path to non-negative slack at the target clock. Fmax, the highest working clock, is
1 / (period - worst slack) for the worst path of a clock.

Why an estimate is needed. On Gowin the wire delay is about 85 percent of a typical path here (cell delays are near
constants, about 0.39 ns for a LUT), and it is only known after place-and-route has placed every cell and routed every wire,
which takes 5 minutes to many hours for the full chip. gw_sh has no timing command before that (it runs "syn", "pnr" or
both, and the placement file is encrypted), and GowinSynthesis' own "Max Frequency" was 4 to 9 ns optimistic on two of
three designs. A design with hundreds of failing paths is a bad thing to learn about after hours of waiting, so this
script reads the synthesized netlist (after synthesis, before place-and-route) and estimates the delay of the worst path
into every register, block RAM and DSP input.

How it works, in outline.
  1. The netlist (.vg) is read and flattened into one list of cells with full hierarchical names (class Netlist). Every
     net has one driver and a list of loads.
  2. The build's SDC file is read (class Clocks): the clocks, the generated clocks, the clock groups, the false and
     multicycle paths. Each register's clock pin is traced back to a clock. The requirement of each path is the distance from
     its launch edge to its capture edge, changed by any exception that applies.
  3. For every net connection a delay is computed from what the netlist shows about it (the model below). The cell delays
     are constants of the chip, measured from reports.
  4. The longest path into every endpoint is found by one pass through the combinational cells in topological order
     (class Est), per group of sources with the same launching clock edge. The estimate is
     clock-to-output + cells + nets + setup, the clock skew being taken as zero.
  5. Endpoints whose estimate is above their requirement (minus the --margin) are listed in groups, with the explanation of
     where the path spends its time, so the paths to cut can be picked before any place-and-route is run.

The net delay model. The netlist does not say how far apart the placer will put two cells, but it says four things that
are known to correlate with it: the fanout of the net, the size (in cells) of the smallest module that contains both the
driver and the load, the number of different modules among the loads, and the size of the whole design (a bigger design
gives the placer less chance to pack cells close together). The delay is a weighted sum of 1, log2 of fanout, log2 of the
module size, log2 of the load modules and log2 of the design size, with the weights fitted to real reports. For a net with
one load the model gives about 0.55 ns. For fanout above 1 it is about -0.09 + 0.167 * log2(fanout) + 0.048 *
log2(module cells) + 0.164 * log2(load modules) + 0.81 * (log2(design cells) - 16), never below 0.05 ns.

Fitting (--fit). A model is fitted per endpoint: the estimate of every endpoint that a Gowin report lists must match
its real worst path, and an endpoint that the report does not list must not be estimated slower than the report's
last listed path, because its real slack is known to be at least that good. The fit is a weighted least squares problem,
repeated a few times because the worst path of an endpoint changes with the weights. With several reports the script also
shows a held out test: the model fitted on the other builds, tried on the one left out.

How far off it was measured to be. The first model was fitted on the nets of one report and estimated 24.2 ns for a design
whose real worst path was 14.0 ns, because report paths are the worst ones and timing driven placement pulls the cells
of such a path together, so its nets are shorter than the average net of the same kind. The second model, fitted per endpoint
on three builds with the design size term, estimated the worst path within 0.07 to 0.7 ns of the real one for two of the
builds (0.07 and 0.7 ns when fitted on all three, 0.2 and 0.5 ns when held out) and 2.3 ns too slow for the third (3.6 ns when it was held out).
Two later models were fitted on the full A1200 system, one build and then four builds, with a separate clock-to-output for
block RAM outputs (tools/timing_model_a1200.json and tools/timing_model_a1200b.json). A model fitted on a different design
(a 290,000 cell one) was far off, so on another project the model must be refitted on its own reports first.
The estimate is a screen and a ranking of the paths to cut. It can not see how tightly the placer packs a design, and the
same RTL ran 5 ns slower when the FPU was placed beside it. The clock of a build is only ever judged from a real
place-and-route. tools/build.sh runs this script after every synthesis and tools/postbuild.py compares it with the real
result after the build (its check 7).

Other modes. --levels N prints Gowin's logic level count instead (tools/levels.py is a thin wrapper), and --fit calibrates
the model. The JSON model files (tools/timing_model*.json) are data written by --fit.
"""
import re, sys, os, json, argparse, collections, math, fnmatch

# Pin names that are OUTPUTS of a Gowin primitive (the net on such a pin is driven by the cell). Anything else on a
# cell is an input. DO/DOA/DOB/DOUT are block RAM data outputs, Q is a flip-flop output, F is a LUT output, SUM and COUT
# belong to the carry cell (ALU), CASO* are the cascade outputs of DSP and block RAM.
OUTS = {'DO', 'DOA', 'DOB', 'DOUT', 'CASO', 'SOA', 'SOB', 'Q', 'F', 'O', 'SUM', 'COUT', 'G', 'V', 'Q0', 'Q1',
        'CASOA', 'CASOB'}
# Cell types that are pure combinational logic: a path runs through them. The LUTs, the wide multiplexers built from two
# LUTs, the carry cell ALU, the inverter, the I/O buffers, the constants, and ROM16 (a LUT used as a small ROM).
COMB = {'LUT1', 'LUT2', 'LUT3', 'LUT4', 'LUT5', 'LUT6', 'MUX2', 'MUX2_LUT5', 'MUX2_LUT6', 'MUX2_LUT7', 'MUX2_LUT8', 'ALU',
        'INV', 'IBUF', 'OBUF', 'GND', 'VCC', 'ROM16'}                  # ROM16: a LUT used as an asynchronous ROM
# The wide multiplexers (MUX2 and MUX2_LUT5 to LUT8) that combine LUT outputs inside one logic slice. Gowin's logic-level
# count treats each one as one level.
WIDE = {'MUX2', 'MUX2_LUT5', 'MUX2_LUT6', 'MUX2_LUT7', 'MUX2_LUT8'}
# Names of clock cells (clock buffers, clock enables, dividers, PLLs, oscillators). They carry a clock, not data, so the data
# path analysis skips them and only the clock tracing goes through them.
CLKSEL = ('BUFG', 'BUFS', 'DQCE', 'DHCE', 'DCS', 'DCC', 'DCE', 'CLKDIV', 'PLL', 'OSC')   # clock cells: no data timing
# Cells without a delay to count: pins, constants, global set/reset.
NOTIME = {'IBUF', 'OBUF', 'IOBUF', 'TBUF', 'GND', 'VCC', 'GSR'}
DEFAULT_CLK = 10.0             # Gowin's period for a clock the SDC does not name (Arora family, SUG940 4.1.2)
# The folder of this script, where timing_model.json (the fitted delays) lives.
HERE = os.path.dirname(os.path.abspath(__file__))


# True when a cell type is a clock cell: its name starts with one of the names above or contains it.
def is_clkcell(t):
    return any(t.startswith(k) or k in t for k in CLKSEL)


# ---- Reading the netlist. GowinSynthesis writes a Verilog netlist (.vg) in which every cell is an instance of a chip
# primitive. This section reads it with regular expressions (a full Verilog parser is not needed for that restricted
# form), flattens the module hierarchy into one list of cells, and joins the nets that cross module ports.
# ---------------------------------------------------------------- netlist
# Split a comma separated list at the commas that are not inside brackets, braces or parentheses (port lists such as
# .I({a, b}, c) contain commas of their own).
def split_top(s):
    out, d, cur = [], 0, ''  # out = the pieces, d = bracket depth, cur = the piece being built
    for c in s:
        if c in '({[': d += 1  # opening bracket: one level deeper
        elif c in ')}]': d -= 1  # closing bracket: one level up
        if c == ',' and d == 0:  # a comma at depth 0 ends a piece
            out.append(cur); cur = ''
        else:
            cur += c
    if cur.strip(): out.append(cur)  # the last piece
    return out


# Parse a netlist file into modules. Result: {module name: (declarations, instances, parameters, port names)}.
def parse(path):
    # The whole file; characters that are not UTF-8 are replaced and do not stop the run.
    txt = open(path, encoding='utf-8', errors='replace').read()
    # Remove // comments, so that a comment can not look like code.
    txt = re.sub(r'//[^\n]*', '', txt)
    mods = {}  # the modules by name
    # Each module has the form "module name (ports); body endmodule".
    for m in re.finditer(r'\bmodule\s+(\S+)\s*\((.*?)\)\s*;(.*?)\bendmodule', txt, re.S):
        name, body = m.group(1), m.group(3)
        decl, insts, prm, ports = {}, [], {}, []  # decl = width of each declared wire or port; insts = instances; prm = parameters of cells (INIT, DIV_MODE); ports = port names
        # Statements are separated by semicolons.
        for st in body.split(';'):
            st = st.strip()
            if not st: continue
            # A declaration such as "input [7:0] name" or "wire name". Group 2 and 3 are the bit range, if any.
            dm = re.match(r'(input|output|inout|wire)\s*(?:\[(\d+):(\d+)\])?\s*(\S+)$', st)
            if dm:
                k, a, b, n = dm.groups()
                decl[n] = (int(a), int(b)) if a is not None else None  # remember the bus range, or None for a single bit
                if k != 'wire': ports.append(n)  # input, output and inout declarations are ports, plain wires are not
                continue
            # A defparam line sets a parameter of one cell; only INIT (the LUT's truth table) and DIV_MODE (a clock divider's ratio) are used here.
            if st.startswith('defparam'):
                pm = re.match(r'defparam\s+(\S+)\.(\w+)\s*=\s*(.*)$', st, re.S)
                if pm and pm.group(2) in ('INIT', 'DIV_MODE'): prm[(pm.group(1), pm.group(2))] = pm.group(3).strip()
                continue
            # "assign" lines are only name aliases in these netlists; the net unions are made from port connections instead.
            if st.startswith('assign'): continue
            # An instance has the form "celltype instname (pins)".
            im = re.match(r'(\S+)\s+(\S+)\s*\((.*)\)$', st, re.S)
            if im:
                t, n, pr = im.groups()
                pins = []  # the (pin, net expression) pairs of this instance
                # Each pin looks like ".PIN(net expression)".
                for p in split_top(pr):
                    pm = re.match(r'\s*\.(\\\S+|\w+)\s*\((.*)\)\s*$', p, re.S)
                    if pm: pins.append((pm.group(1), pm.group(2).strip()))
                insts.append((t, n, pins))
        # Keep everything found about this module.
        mods[name] = (decl, insts, prm, ports)
    return mods


# Turn a net expression of a port connection into the list of single-bit net names it stands for, bit by bit (MSB first, as
# Verilog does): a concatenation {a,b}, a constant, a single bit, a range a[7:4], or a whole bus declared in decl.
def expand(expr, decl):
    expr = expr.strip()
    if not expr: return []  # nothing connected
    if expr[0] == '{':  # a concatenation: expand each piece and join the lists
        out = []
        for p in split_top(expr[1:-1]): out += expand(p, decl)
        return out
    lm = re.match(r"(\d+)'[bBhHdD]([0-9a-fA-FxXzZ_]+)$", expr)  # a sized constant such as 4'b0101: all bits are treated as the constant net GND, since constants have no delay
    if lm: return ['GND'] * int(lm.group(1))
    if expr in ('GND', 'VCC'): return [expr]  # the names GND and VCC stand for themselves
    sm = re.match(r'(.+?)\[(\d+)(?::(\d+))?\]$', expr)  # a bit or a range of a signal: name[hi:lo] or name[bit]
    if sm:
        n, a, b = sm.group(1).rstrip(), int(sm.group(2)), sm.group(3)   # escaped names: '\a.b [3]' is bit 3 of '\a.b'
        if b is None: return ['%s[%d]' % (n, a)]
        b = int(b); st = -1 if a >= b else 1  # ranges run up or down, so the step is -1 or +1
        return ['%s[%d]' % (n, i) for i in range(a, b + st, st)]
    r = decl.get(expr)  # a plain name: if it was declared as a bus, expand it to all its bits
    if r:
        a, b = r; st = -1 if a >= b else 1
        return ['%s[%d]' % (expr, i) for i in range(a, b + st, st)]
    return [expr]  # otherwise it is a single-bit net


# Remove the backslash that starts an escaped Verilog identifier (an identifier with special characters).
def clean(n):
    return n[1:] if n.startswith('\\') else n


# ---- The flattened netlist. The class reads a netlist, descends from the top module through every submodule and builds
# one list of primitive cells with full hierarchical names such as core/i_ex/.../cell. Port connections between a
# module and its child join two net names into one net (a union-find structure, find() below). Afterwards every net has
# one driver and a list of loads, and the number of cells under every hierarchy prefix is known (the module size, used by
# the delay model).
class Netlist:
    """Flattened netlist. cells[i] = (full name, type, [(pin, net)]); drv[net] = (cell, pin); loads[net] = [(cell, pin)]."""

    def __init__(self, path, top=None):
        # Parse the file and pick the top module: given by the user or the one no other module instantiates.
        mods = parse(path)
        if top is None:                                # the module no other module instantiates
            used = {t for d, ins, pr, po in mods.values() for t, n, p in ins if t in mods}  # used = every cell type that is itself a module of this netlist
            tops = [m for m in mods if m not in used]
            top = tops[-1] if tops else list(mods)[-1]  # if there are several, the last one defined is taken (Gowin writes the top module last)
        elif top not in mods: sys.exit('top module %s not in netlist' % top)
        parent = self.parent = {}  # parent is the union-find table of net names

        # Join two net names so that they are one net from now on.
        def union(x, y):
            rx, ry = self.find(x), self.find(y)
            if rx != ry: parent[rx] = ry

        # raw = the primitive cells found, self.param = parameters (INIT, DIV_MODE) per cell
        raw, self.param = [], {}
        stack = [(top, '')]  # work list: (module name, prefix for its instances' names)
        while stack:  # depth-first over the hierarchy
            mname, pre = stack.pop()
            decl, insts, prm, ports = mods[mname]  # the module's declarations, instances, parameters and ports
            nid = lambda n: n if n in ('GND', 'VCC') else pre + n  # nid gives a net name its hierarchical prefix (constants stay global)
            for (inst, k), v in prm.items(): self.param[(pre + clean(inst), k)] = v  # parameters of primitives get the full instance name
            for t, n, pins in insts:
                full = pre + clean(n)  # full hierarchical name of the instance
                if t in mods:  # an instance of another module: connect its ports and descend into it
                    cdecl = mods[t][0]
                    for pin, ex in pins:
                        for c, e in zip(expand(pin, cdecl), expand(ex, decl)):  # join each bit of the child's port with the bit of the parent's net connected to it
                            union(full + '/' + c, nid(e))
                    stack.append((t, full + '/'))  # the child's cells get a longer prefix
                else:  # a primitive: record it with its pins and the net of each pin
                    raw.append((full, t, [(pin, nid(e)) for pin, ex in pins for e in expand(ex, decl)]))
        tdecl = mods[top][0]  # the top module's own ports (clk and so on), mapped to their nets
        self.ports = {b: self.find(b) for p in mods[top][3] for b in expand(p, tdecl)}
        self.cells = [(f, t, [(p, self.find(n)) for p, n in pl]) for f, t, pl in raw]  # every cell with its pin nets replaced by the representative net of the union
        self.drv, self.loads = {}, collections.defaultdict(list)  # drv[net] = the driver cell and pin; loads[net] = the list of load cells and pins
        for i, (f, t, pl) in enumerate(self.cells):
            if t in ('GND', 'VCC'): continue  # constants are not cells that matter
            for p, n in pl:
                if n in ('GND', 'VCC'): continue  # a net tied to a constant carries no timing
                # A pin is a driver when it is a known output pin (a flip-flop only counts its Q output; the other pins are inputs), or a clock
                # output of a PLL or divider. Otherwise it is a load.
                if p in OUTS and not (t.startswith('DFF') and p != 'Q') or p.startswith('CLKOUT'): self.drv[n] = (i, p)
                else: self.loads[n].append((i, p))
        self.byname = {f: i for i, (f, t, pl) in enumerate(self.cells)}  # index from full cell name to cell number
        self.size = collections.Counter()              # cells under every hierarchy prefix
        for f, t, pl in self.cells:  # count the cells under every hierarchy prefix (the size of every module, including the top)
            if t in ('GND', 'VCC'): continue
            parts = f.split('/')
            for k in range(len(parts)): self.size['/'.join(parts[:k])] += 1  # a cell at depth d adds one to each of its d prefixes, and the top ('' prefix) counts them all

    # Union-find: the representative net of x, with path compression so that later lookups are fast.
    def find(self, x):
        parent = self.parent
        r = x
        while parent.get(r, r) != r: r = parent[r]  # walk up to the root
        while parent.get(x, x) != r: parent[x], x = r, parent[x]  # compress the path: point every net on it at the root
        return r

    # The module (hierarchy prefix) a cell lives in: its full name without the last part.
    def mod(self, i):
        f = self.cells[i][0]
        return f.rsplit('/', 1)[0] if '/' in f else ''

    # Lowest common ancestor of two module paths: the longest prefix both share, which is the smallest module that holds
    # both cells. Its size is an input of the net delay model.
    def lca(self, a, b):
        pa, pb = a.split('/') if a else [], b.split('/') if b else []
        k = 0
        while k < len(pa) and k < len(pb) and pa[k] == pb[k]: k += 1  # step down while both paths agree
        return '/'.join(pa[:k])

    def match_cells(self, pats):  # cells whose full name matches any of the shell-style patterns (* and ?) of an SDC object list
        rx = re.compile('|'.join('(?:%s)' % fnmatch.translate(p) for p in pats))  # fnmatch.translate turns a shell pattern into a regular expression; all patterns are joined with |
        return {i for i, c in enumerate(self.cells) if rx.match(c[0])}


# ---- Reading the SDC file. The SDC (Synopsys Design Constraints) is the file of timing constraints that the build gives
# to Gowin (create_clock, set_multicycle_path, ...). The estimate must obey the same rules as Gowin's analyzer, so it reads the
# file too. SDC is a Tcl dialect, so a small Tcl word splitter is needed first.
# ---------------------------------------------------------------- SDC
# Split a Tcl line into words. Each word is ('w', text) for a plain word, ('{', text) for a braced or quoted text, or ('[', text) for a
# bracketed command such as get_ports {clk}.
def tcl_split(s):
    """Tcl words: ('w', word), ('{', braced text), ('[', command text)."""
    out, i, n = [], 0, len(s)
    while i < n:
        c = s[i]
        if c.isspace(): i += 1; continue  # skip spaces
        if c in '{[':  # a brace or a bracket: find its matching closing character, counting nested ones
            o, cl, d, j = c, '}' if c == '{' else ']', 0, i  # o = opening character, cl = closing character, d = nesting depth, j = scan position
            while j < n:
                if s[j] == o: d += 1
                elif s[j] == cl:
                    d -= 1
                    if d == 0: break
                j += 1
            out.append((c, s[i + 1:j])); i = j + 1  # the word is the text between the two; continue after the closing character
        elif c == '"':  # a double quoted string is taken like a braced text
            j = s.find('"', i + 1); j = n if j < 0 else j
            out.append(('{', s[i + 1:j])); i = j + 1
        else:  # any other word runs to the next space
            j = i
            while j < n and not s[j].isspace(): j += 1
            out.append(('w', s[i:j])); i = j
    return out


# SDC options that are switches (no value follows them), unlike -period 12.5 which has one.
FLAGS = {'-setup', '-hold', '-start', '-end', '-asynchronous', '-exclusive', '-logically_exclusive', '-physically_exclusive',
         '-invert', '-add', '-combinational', '-rise', '-fall', '-datapath_only'}


# Turn an SDC object list into (kind, patterns): kind is get_ports, get_regs, get_cells, get_pins, get_nets or get_clocks, and patterns are the
# names in it. A bare braced list is taken as a list of clock names.
def sdc_obj(tok):
    """object list -> (kind, [patterns]); a braced or bare list is a list of clock names."""
    k, s = tok
    if k == '[':
        w = tcl_split(s)  # a command in brackets, for example [get_ports {clk}]
        kind, pats = w[0][1], []  # the first word is the kind; options inside the brackets (such as -hier) are skipped
        for kk, v in w[1:]:
            if kk == 'w' and v.startswith('-'): continue
            pats += v.split() if kk == '{' else [v]
        return kind, pats
    return 'get_clocks', s.split()


# Read an SDC file into a list of commands: (command name, {option: [values]}, [positional values]).
def read_sdc(path):
    """[(command, {option: [values]}, [positional])]."""
    txt, cmds = open(path, encoding='utf-8', errors='replace').read(), []
    txt = re.sub(r'\\\n', ' ', txt)  # join lines that end in a backslash
    for ln in txt.split('\n'):
        ln = ln.strip()
        if not ln or ln.startswith('//') or ln.startswith('#'): continue  # skip empty lines and comments
        w = tcl_split(ln)
        if not w or w[0][0] != 'w': continue  # the line must start with a command word
        opts, pos, k = collections.defaultdict(list), [], 1  # opts = options found, pos = positional arguments, k = word index (word 0 is the command)
        while k < len(w):
            kk, v = w[k]
            if kk == 'w' and v.startswith('-') and not re.match(r'-[\d.]', v):  # a word that starts with - is an option, unless it is a negative number
                if v in FLAGS: opts[v].append(True); k += 1  # a switch takes no value
                else: opts[v].append(w[k + 1] if k + 1 < len(w) else ('w', '')); k += 2  # any other option takes the next word as its value
            else:
                pos.append(w[k]); k += 1  # not an option: a positional argument (for example the 4 of set_multicycle_path 4)
        cmds.append((w[0][1], opts, pos))
    return cmds


# The first number in an SDC word, or NaN when the value is a bracketed command (cannot be evaluated here).
def num(tok):
    return float(tok[1].split()[0]) if tok[0] != '[' else float('nan')


# ---- Clocks. This class holds everything the SDC says about clocks: the clocks and generated clocks with period and rise and fall times, the
# groups of clocks that are unrelated, and the exceptions (false paths, multicycle paths, max delay). It also traces a clock pin of a
# register back through buffers and gates to the clock it carries, and works out the requirement of any launch/capture pair.
class Clocks:
    """Clocks, clock groups and timing exceptions of an SDC, bound to a netlist."""

    def __init__(self, nl, sdc, default_period):
        self.nl, self.dflt = nl, default_period
        self.clk = {}                 # name -> dict(T, rise, fall, declared, src)
        self.on_net = {}              # net -> clock name
        self.groups = []              # [{clock: group index}] per set_clock_groups
        self.exc = []                 # dicts: kind (false|max|mcp), n, start, D, fc, fr, tc, tr, tp
        self.warn = []  # warnings printed at the top of the report
        self._memo, self._rel = {}, {}  # caches for trace() and setup_rel()
        cmds = read_sdc(sdc) if sdc else []  # read the SDC, if there is one
        for c, o, p in cmds:
            if c == 'create_clock': self._create(o, p)  # first, the clocks that are created directly
        pend = [(o, p) for c, o, p in cmds if c == 'create_generated_clock']  # generated clocks depend on their master clock, so they are made in rounds until no more can be resolved
        while pend:
            left = [(o, p) for o, p in pend if not self._generated(o, p)]
            if len(left) == len(pend): break
            pend = left
        for o, p in pend: self.warn.append('generated clock %s: master not found, not used' % o.get('-name', [('w', '?')])[0][1])  # what is left over has no known master: warn about it
        ignored = collections.Counter()  # commands that are known but not used are counted so that the report can say so
        for c, o, p in cmds:
            if c == 'set_clock_groups':  # set_clock_groups: clocks of different groups have no timed paths between them (asynchronous clocks)
                m = {}
                for gi, g in enumerate(o.get('-group', [])):
                    for nm in self._clock_names(sdc_obj(g)[1]): m[nm] = gi
                self.groups.append(m)
            elif c in ('set_false_path', 'set_multicycle_path', 'set_max_delay'):  # false paths, multicycle paths and maximum delays
                self._exception(c, o, p)
            elif c not in ('create_clock', 'create_generated_clock', 'report_timing', 'report_high_fanout_nets',  # reporting commands and other harmless ones are not worth a warning
                           'report_route_congestion', 'report_min_pulse_width', 'set_operating_conditions'):
                ignored[c] += 1
        for c, n in sorted(ignored.items()): self.warn.append('%s ignored (x%d)' % (c, n))
        order = {'false': 0, 'max': 1, 'mcp': 2}  # the order in which exceptions are applied: a false path wins over a max delay, which wins over a multicycle path
        self.exc.sort(key=lambda x: order[x['kind']])

    # Turn an SDC object list into the nets it names (ports, pins of cells, nets), warning about names not found.
    # -- objects
    def _nets(self, obj):
        kind, pats = obj
        nl, out = self.nl, []
        for pt in pats:
            got = []
            if kind == 'get_ports':  # a port: match the names of the top module's ports
                got = [n for b, n in nl.ports.items() if fnmatch.fnmatchcase(b, pt)]
            elif kind == 'get_pins' and '/' in pt:  # a pin: split into cell path and pin name; the cell path may contain wildcards
                cp, pp = pt.rsplit('/', 1)
                cells = [nl.byname[cp]] if cp in nl.byname else sorted(nl.match_cells([cp]))
                got = [n for ci in cells for q, n in nl.cells[ci][2] if fnmatch.fnmatchcase(q, pp)]
            elif kind == 'get_nets':  # a net by name
                n = nl.find(pt)
                if n in nl.drv or n in nl.loads: got = [n]
            if not got: self.warn.append('%s {%s}: not in the netlist' % (kind, pt))  # a name that matches nothing is reported, since a mistyped SDC line would otherwise be silently ignored
            out += got
        return out

    # The clocks whose names match any of the patterns.
    def _clock_names(self, pats):
        return [c for c in self.clk if any(fnmatch.fnmatchcase(c, p) for p in pats)]

    # Register a clock: edges are taken modulo the period; if rise and fall come out equal the fall is put half a period later.
    def _add(self, name, T, rise, fall, nets, src=''):
        r, f = rise % T, fall % T
        self.clk[name] = dict(T=T, rise=r, fall=f if f != r else (r + T / 2) % T, declared=True, src=src)  # store period, rising edge time, falling edge time, whether the SDC declared it and where it came from
        for n in nets: self.on_net[n] = name  # put the clock on the nets given

    # create_clock: period from -period, waveform (rise time, fall time) from -waveform or a 50 percent clock starting at 0.
    def _create(self, o, p):
        T = num(o['-period'][0])
        wv = o['-waveform'][0][1].split() if '-waveform' in o else ['0', str(T / 2)]
        nets = self._nets(sdc_obj(p[-1])) if p else []  # the nets (ports or pins) the clock is defined on
        name = o['-name'][0][1] if '-name' in o else (sdc_obj(p[-1])[1][0] if p else 'virtual')
        self._add(name, T, float(wv[0]), float(wv[1]), nets, src='create_clock')

    # create_generated_clock: a clock derived from another one (for example a PLL output or a divided clock). Returns False when
    # its master clock is not known yet, so that the caller can try again after the others are made.
    def _generated(self, o, p):
        name = o['-name'][0][1] if '-name' in o else (sdc_obj(p[-1])[1][0] if p else '?')  # the clock's name
        m = o['-master_clock'][0][1] if '-master_clock' in o else None  # the master clock by name, if the SDC gives it
        if m is None and '-source' in o:  # otherwise find the master from the source pin or port: the clock that reaches that net
            for n in self._nets(sdc_obj(o['-source'][0])):
                m = self.on_net.get(n) or next(iter(sorted(c for c, inv in self.trace(n))), None)
                if m: break
        if m not in self.clk: return False  # the master does not exist yet
        M = self.clk[m]  # M = the master clock's data (period, rise, fall)
        if '-edges' in o:  # -edges {1 3 5}: the new clock rises at the master's edge 1 and falls at edge 3, and its period is edge 5 minus edge 1
            e = [int(x) for x in o['-edges'][0][1].split()]
            et = lambda k: (k - 1) // 2 * M['T'] + (M['rise'] if k % 2 else M['fall'])  # the time of master edge k: odd edges are rising edges, even ones falling edges, counted from 1
            rise, fall, T = et(e[0]), et(e[1]), et(e[2]) - et(e[0])
        else:
            mul = num(o['-multiply_by'][0]) if '-multiply_by' in o else 1.0  # -multiply_by and -divide_by change the frequency (a PLL multiplies, a divider divides)
            div = num(o['-divide_by'][0]) if '-divide_by' in o else 1.0
            T = M['T'] * div / mul  # new period
            duty = num(o['-duty_cycle'][0]) / 100 if '-duty_cycle' in o else 0.5  # duty cycle: the share of the period that the clock is high
            rise = M['rise'] if mul == 1 else 0.0  # without multiplication the new clock rises with its master; a multiplied one starts at 0
            if '-phase' in o: rise += num(o['-phase'][0]) / 360 * T  # -phase shifts the clock by a fraction of its period
            fall = rise + duty * T
        if '-invert' in o: rise, fall = fall, rise  # -invert swaps rising and falling edges
        self._add(name, T, rise, fall, self._nets(sdc_obj(p[-1])) if p else [], src='generated from ' + m)
        return True  # master found

    # A false path, multicycle path or max delay exception. A false path is not timed at all. A multicycle path gets
    # several clock periods instead of one (the FPU engine, which steps every n clocks, uses this). A max delay replaces the
    # requirement with a fixed time. The from and to sides select clocks, registers or pins; -through is not supported.
    def _exception(self, c, o, p):
        if '-through' in o: self.warn.append('%s -through: not supported, ignored' % c); return  # -through is not supported (the estimate has no path memory) and the line is ignored
        if '-hold' in o and '-setup' not in o: return  # a hold-only exception does not change the setup requirement the estimate looks at
        x = dict(kind={'set_false_path': 'false', 'set_multicycle_path': 'mcp', 'set_max_delay': 'max'}[c],  # x describes the exception: its kind, from/to clocks (fc, tc), from/to registers (fr, tr), to-pins (tp), number of clocks (n), counted from the start or the end clock, and the max delay D
                 fc=None, fr=None, tc=None, tr=None, tp=None, n=1, start='-start' in o, D=0.0)
        if c == 'set_multicycle_path': x['n'] = int(num(p[0])) if p else 1  # set_multicycle_path n: n clock periods
        if c == 'set_max_delay': x['D'] = num(p[0]) if p else 0.0  # set_max_delay D: D nanoseconds
        for side in ('from', 'to'):  # both ends of the path
            for t in o.get('-' + side, []) + o.get('-rise_' + side, []) + o.get('-fall_' + side, []):  # the option may also come as -rise_from or -fall_to and so on
                kind, pats = sdc_obj(t)
                if kind == 'get_clocks':  # clocks: every path launched (or captured) by one of those clocks
                    x[side[0] + 'c'] = set(self._clock_names(pats))
                elif kind in ('get_regs', 'get_cells'):  # registers or cells: every register matching the patterns
                    x[side[0] + 'r'] = self.nl.match_cells(pats)
                elif kind == 'get_pins':  # pins: remember which pins of which cells (for example only the enable pin of a register)
                    d = collections.defaultdict(list)
                    for pt in pats:
                        cp, pp = pt.rsplit('/', 1) if '/' in pt else (pt, '*')
                        for ci in self.nl.match_cells([cp]): d[ci].append(pp)
                    if side == 'from': x['fr'] = set(d)  # from-pins are treated as their cells
                    else: x['tp'] = d
                else:  # a port or other object: not timed here
                    self.warn.append('%s -%s %s: ports are not timed here, ignored' % (c, side, kind)); return
        if all(x[k] is None for k in ('fc', 'fr', 'tc', 'tr', 'tp')): return  # an exception that selected nothing is useless
        self.exc.append(x)  # remember it

    # ---- Finding the clock of a net. A register's clock pin is traced backwards through buffers, inverters, gates, clock enables, dividers
    # and PLLs until a net is reached that the SDC names. Each result is a set of (clock, inverted) pairs.
    # -- clock of a net
    # A clock that the SDC does not name still has to be timed: it gets an automatic clock with the default period (Gowin does the same).
    def auto(self, net, src='not in the SDC'):
        if net not in self.clk:
            self.clk[net] = dict(T=self.dflt, rise=0.0, fall=self.dflt / 2, declared=False, src=src)
        return net

    # The clocks on a net, as a set of (clock name, inverted). Memoized (the result of each net is stored), and a net being worked on is already
    # stored as empty so that a loop in the clock network can not run forever.
    def trace(self, net):
        """clocks on a net: frozenset of (clock name, inverted)."""
        v = self._memo.get(net)  # already known
        if v is not None: return v
        nl = self.nl  # nl is the netlist
        self._memo[net] = frozenset()                  # loop guard
        if net in self.on_net: v = frozenset([(self.on_net[net], False)])  # the SDC puts a clock on this net
        elif net in ('GND', 'VCC'): v = frozenset()  # constants carry no clock
        else:
            d = nl.drv.get(net)  # otherwise look at the cell that drives the net
            if d is None: v = frozenset([(self.auto(net), False)])  # no driver: a clock coming from outside, not named in the SDC
            else:
                ci, op = d  # ci = driver cell, op = its output pin
                f, t, pl = nl.cells[ci]
                ins = [n for p, n in pl if p not in OUTS and not p.startswith('CLKOUT') and n not in ('GND', 'VCC')]  # ins = the nets on the input pins of that cell
                if t == 'INV' or (t == 'LUT1' and nl.param.get((f, 'INIT'), '').lower().endswith("'h1")):  # an inverter, or a one-input LUT that implements an inverter (INIT = 1): the clock is inverted
                    v = frozenset((c, not i) for n in ins for c, i in self.trace(n))
                elif t.startswith('CLKDIV'):  # a clock divider: derive the new period from its divide mode (CLKDIV2 divides by 2)
                    dv = 2.0 if t == 'CLKDIV2' else float(re.sub(r'[^0-9.]', '', nl.param.get((f, 'DIV_MODE'), '2')) or 2)
                    src = [n for p, n in pl if p in ('HCLKIN', 'CLKIN', 'CLK')]  # the divider's input clock
                    out = set()
                    for c, i in (self.trace(src[0]) if src else ()):
                        nm = '%s/%s.default_gen_clk' % (f, op)  # name for the derived clock
                        if nm not in self.clk:
                            M = self.clk[c]
                            self.clk[nm] = dict(T=M['T'] * dv, rise=0.0, fall=M['T'] * dv / 2, declared=False,
                                                src='CLKDIV /%g of %s (not in the SDC)' % (dv, c))
                        out.add((nm, False))
                    v = frozenset(out) or frozenset([(self.auto(net), False)])  # if nothing could be derived, the output becomes an automatic clock
                elif t in COMB or t.startswith(('BUFG', 'BUFS', 'DQCE', 'DHCE', 'DCS', 'DCC', 'DCE')):  # buffers, gates and clock enables pass the clock through (the union of what reaches their inputs)
                    v = frozenset(x for n in ins for x in self.trace(n))
                elif 'PLL' in t:  # a PLL output that the SDC does not describe gets the default period
                    v = frozenset([(self.auto(net, 'PLL output not in the SDC (Gowin derives it; default period used)'), False)])
                else:  # anything else drives the net with something the SDC does not know
                    v = frozenset([(self.auto(net, 'driven by %s %s, not in the SDC' % (t, f)), False)])
        self._memo[net] = v  # store the result
        return v

    # The clock edges at a register's clock pin: a set of (clock, falling). Cells with two clocks (block RAM and DSP have clock A and
    # clock B) use the one that belongs to the pin asked about. A negative-edge flip-flop (DFFN) captures on the falling edge.
    def pin_clock(self, ci, pin):
        """clock edges launching / capturing at pin of a sequential cell: frozenset of (clock, falling)."""
        f, t, pl = self.nl.cells[ci]
        cks = [(p, n) for p, n in pl if p.startswith('CLK') and not p.startswith('CLKOUT') and n not in ('GND', 'VCC')]  # the clock pins of this cell
        if not cks: return frozenset()  # no clock pin, no timing
        if len({p for p, n in cks}) > 1:  # two different clock pins (a dual port RAM): choose by which port the data pin belongs to
            b = re.sub(r'\[\d+\]$', '', pin)
            want = 'CLKB' if (b.endswith('B') or b in ('DO', 'DOUT', 'OCE', 'RESET')) else 'CLKA'
            cks = [x for x in cks if x[0] == want] or cks
        s = self.trace(cks[0][1])
        if t.startswith('DFFN'): s = frozenset((c, not i) for c, i in s)  # a negative-edge register flips the edge
        return s

    # ---- Relations between clocks and requirements. A path launched by one clock edge and captured by another has as many nanoseconds
    # to travel as the distance between the two edges, and the analyzer takes the shortest positive distance.
    # -- relations
    # False when the two clocks are in different groups of one set_clock_groups (asynchronous: no path between them is timed).
    def related(self, a, b):
        for g in self.groups:
            if a in g and b in g and g[a] != g[b]: return False
        return True

    # The setup relation: the smallest positive time from a launch edge to the next capture edge, as Gowin's analyzer takes it. For two
    # clocks of the same period it is that period (or half of it between a rising and a falling edge). For clocks of different
    # periods the edges drift against one another, so all launch edges within the repeat period of the pair are tried. The result is
    # cached. Times are integers in picoseconds to keep the arithmetic exact.
    def setup_rel(self, lk, ck):
        """smallest positive time from a launch edge to the next capture edge (ns)."""
        k = (lk, ck)
        v = self._rel.get(k)
        if v is None:
            L, C = self.clk[lk[0]], self.clk[ck[0]]  # L, C = launch and capture clock
            Tl, Tc = int(round(L['T'] * 1000)), int(round(C['T'] * 1000))  # periods in picoseconds
            tl = int(round((L['fall'] if lk[1] else L['rise']) * 1000))  # time of the launching edge within its period
            tc = int(round((C['fall'] if ck[1] else C['rise']) * 1000))  # time of the capturing edge within its period
            best = None
            for j in range(min(Tc // math.gcd(Tl, Tc), 20000)):  # try every launch edge until the pattern repeats (at most 20000 of them)
                a = tl + j * Tl  # a = time of launch edge number j
                c = tc + ((a - tc) // Tc + 1) * Tc  # c = the first capture edge after it
                best = c - a if best is None or c - a < best else best  # keep the smallest distance
            v = self._rel[k] = best / 1000.0  # back to nanoseconds
        return v

    # The requirement of one path in nanoseconds: the setup relation, unless an exception changes it. None means the path is not timed
    # (a false path or asynchronous clocks). The first exception whose from and to selections all match decides. A multicycle path adds
    # (n - 1) periods of the start or end clock, as set_multicycle_path -start or -end says.
    def requirement(self, lk, srcx, ck, ci, pin):
        """requirement (ns) of a path launched by edge lk from a source matching exceptions srcx, captured by ck at
        (ci, pin); None: not timed."""
        if not self.related(lk[0], ck[0]): return None  # asynchronous clock groups: not timed
        for xi, x in enumerate(self.exc):  # the exceptions are in the order false, max delay, multicycle
            if x['fc'] is not None and lk[0] not in x['fc']: continue  # every test below skips an exception that does not apply to this path
            if x['fr'] is not None and xi not in srcx: continue
            if x['tc'] is not None and ck[0] not in x['tc']: continue
            if x['tr'] is not None and ci not in x['tr']: continue
            if x['tp'] is not None and not any(fnmatch.fnmatchcase(pin, pp) for pp in x['tp'].get(ci, ())): continue
            if x['kind'] == 'false': return None  # a false path: not timed
            if x['kind'] == 'max': return x['D']  # a max delay: that fixed time
            return self.setup_rel(lk, ck) + (x['n'] - 1) * self.clk[(lk if x['start'] else ck)[0]]['T']  # a multicycle path: the relation plus the extra periods
        return self.setup_rel(lk, ck)  # no exception: the plain relation

    # The numbers of the exceptions that name this cell as a source register (so that paths from it can be grouped for the analysis).
    def source_exc(self, ci):
        return tuple(xi for xi, x in enumerate(self.exc) if x['fr'] is not None and ci in x['fr'])


# ---- The delay model. A path delay is estimated as: clock-to-output of the launching register, plus the delay of every cell on the
# path, plus the delay of every net, plus the setup time of the capturing register. Cell delays are almost constants of the chip
# (a LUT is 0.39 ns whatever its function) and are measured from the real reports. Net delays depend on the placement, which is
# not known yet, so they are modeled from what the netlist does show about a net, described below in net_features.
# ---------------------------------------------------------------- model
# Gowin counts the logic levels of a carry chain (a run of n adder cells, ALU) as 1 for one cell, 2 for 2 to 6 cells, 3 for 7 to 17
# and so on (the fit matches 3196 of the 3200 paths of a real report). A beginner can read this as "a fast adder is worth only a few LUTs".
def chain_levels(n):
    """Gowin's Logic Level for a run of n ALU cells on one path (fit: 3196 of 3200 paths of the calibration report)."""
    return 0 if n <= 0 else 1 if n == 1 else 2 if n <= 6 else 3 + int((n - 7) / 10.33)


# The six terms of the net delay model, in the order of the weights in W0 and in timing_model.json: a constant for a net with one load, a
# constant for a net with more, the log of the fanout, the log of the size of the module holding driver and load (nets that stay inside a small
# module are shorter), the log of the number of different modules among the loads, and the log of the whole design size (a larger design
# gives the placer less room to pack the cells and so longer nets). The logarithm is used because the extra delay of a doubling of
# fanout or size is about constant.
NET_TERMS = ['fanout 1', 'fanout>1', 'log2 fanout', 'log2 module (fanout>1)', 'log2 load modules', 'log2 design (fanout>1)']
# Starting weights for the fit (ns per term); the fit replaces them. The fitted model (tools/timing_model.json) has about 0.55 ns for a net with
# one load, and 0.17 ns more per doubling of the fanout.
W0 = [0.19, -0.63, 0.256, 0.105, 0.042, 0.0]
# Design size term range as a log2 of the cell count: it is clamped to 2^15 to 2^17 because the model was calibrated on designs from 45k to 105k cells and
# must not extrapolate far.
NT_LOG2 = (15.0, 17.0)         # design size term: 32k to 128k cells (calibration: 45k to 105k)


# Features of one connection (a net seen from one load): the six numbers that the weights multiply. The feature vector is dotted with the weights to give nanoseconds.
def net_features(fo, sz, nm, nt):
    """fanout, cells in the smallest module holding driver and load, distinct modules among the loads, cells in the design."""
    m = 1.0 if fo > 1 else 0.0  # m is 1 for a net with more than one load
    d = min(max(math.log2(max(nt, 1)), NT_LOG2[0]), NT_LOG2[1]) - 16    # held to the calibrated design sizes  # design size term, centered on 2^16 cells (65,536)
    return [1.0 - m, m, math.log2(max(fo, 1)), m * math.log2(max(sz, 1)), math.log2(max(nm, 1)), m * d]  # constant for fanout 1; constant for fanout>1; log2 fanout; log2 module size (fanout>1); log2 load modules; design size term (fanout>1)


# The delay model with its fitted numbers: the cell delays, the clock-to-output, the setup times and the net weights.
class Model:
    def __init__(self, d):
        self.d = d
        self.cell = d['cell']; self.tco = d['tco']; self.tsu = d['tsu']
        self.tco_ram = d.get('tco_ram', self.tco)    # block RAM data outputs (DO*) have their own clock-to-out
        self.w = d['net']['w']
        self._c = {}

    # Delay of a cell from an input pin to an output pin. Gowin's placer may swap the inputs of a LUT, so one mean is used for all of them.
    def cell_delay(self, t, pin, out):
        c = self.cell
        if t.startswith('LUT') or t in ('INV', 'ROM16'): return c['LUT']   # P&R swaps LUT inputs: one mean for all pins; INV becomes a LUT1
        if t == 'ALU':
            if pin == 'CIN': return c.get('ALU:CIN>' + out, c['ALU:CIN>SUM'])
            return c.get('ALU:I>' + out, c['ALU:I>COUT'])
        if t in WIDE: return c.get('MUXW:' + ('S0' if pin == 'S0' else 'I'), c['MUXW:I'])
        return 0.0

    # Setup time of the capturing register for this pin type (D for data, CE for the clock enable, and so on); the bit index is dropped.
    def setup(self, pin):
        p = re.sub(r'\[\d+\]$', '', pin)
        return self.tsu.get(p, self.tsu['other'])

    # Delay of a net from its features: the weighted sum, never below 0.05 ns.
    def net_delay(self, fo, sz, nm, nt):
        k = (fo, sz, nm, nt)
        v = self._c.get(k)
        if v is None:
            v = max(0.05, sum(w * x for w, x in zip(self.w, net_features(fo, sz, nm, nt)))); self._c[k] = v
        return v


# Where the fitted model is stored (the same folder as this file).
def default_model_path():
    return os.path.join(HERE, 'timing_model.json')


# ---- The estimate. The class builds the timing graph from the netlist (every combinational cell with its input to output arcs, the sources which
# are register and RAM outputs, and the endpoints which are register, RAM and DSP inputs) and then finds, for every endpoint, the longest path
# arriving from any source, with a longest-path propagation in topological order (every cell after all cells that feed it).
# ---------------------------------------------------------------- estimate
class Est:
    def __init__(self, nl, model, clocks):
        self.nl, self.m, self.ck = nl, model, clocks
        self.isseq = [t not in COMB for f, t, pl in nl.cells]  # isseq[i] is True when cell i is not combinational: a register, a RAM, a DSP, a clock cell
        self.cmod = [nl.mod(i) for i in range(len(nl.cells))]  # the module (hierarchy prefix) of every cell
        self._lcs, self._nm = {}, {}  # caches for lca_size and nmods
        self.graph = False

    # The number of cells in the smallest module that holds both cells (hierarchy prefixes a and b); a feature of the net delay model.
    def lca_size(self, a, b):
        k = (a, b)
        v = self._lcs.get(k)
        if v is None:
            v = self.nl.size[self.nl.lca(a, b)]; self._lcs[k] = v
        return v

    # The number of different modules among the loads of a net; also a feature.
    def nmods(self, net):
        v = self._nm.get(net)
        if v is None:
            v = len({self.cmod[c] for c, p in self.nl.loads[net]}); self._nm[net] = v
        return v

    # The features (fanout, size of the common module, number of load modules, design size) of the connection from a net to one load.
    def feat(self, net, ci, pin):
        """(fanout, lca size, load modules) of the connection net -> (ci, pin); None: no delay (carry chain, no driver)."""
        nl = self.nl
        d = nl.drv.get(net)
        if d is None: return None  # the net has no driver (an input of the chip): no delay known
        dc, dp = d
        if dp == 'COUT' and pin == 'CIN': return None  # the carry chain wire (COUT to CIN of the next adder cell) is dedicated and has no routing delay
        return len(nl.loads[net]), self.lca_size(self.cmod[dc], self.cmod[ci]), self.nmods(net), nl.size['']

    # Build the timing graph. Steps: (1) number every connection (a net seen from one load) and compute its features once; (2) order the combinational cells
    # topologically, i.e. every cell after all the cells that feed it, so that one pass in that order finds the longest arrival at every net; (3) make the
    # list of sources (register, RAM and DSP outputs) and endpoints (their data, enable and reset inputs); (4) group the sources by launching clock edge.
    def build(self):
        """timing graph: topological order of combinational cells with their arcs, sources and endpoints."""
        nl = self.nl  # nl is the netlist
        self.feats, fid = [], {}                       # connection id -> (fanout, lca size, load modules) or None

        # Give each (net, load cell, pin) triple a number and compute its features once.
        def conn(n, i, p):
            k = (n, i, p)
            v = fid.get(k)
            if v is None:
                v = fid[k] = len(self.feats); self.feats.append(self.feat(n, i, p))
            return v
        comb = [i for i, (f, t, pl) in enumerate(nl.cells) if not self.isseq[i] and t not in ('GND', 'VCC', 'IBUF', 'OBUF')]  # the combinational cells that matter (not constants or pad buffers)
        # Topological sort (Kahn's algorithm). indeg[i] = how many of cell i's inputs come from other combinational cells; users[net] = the cells fed by that net.
        indeg, users = {}, collections.defaultdict(list)
        for i in comb:
            k = 0
            for p, n in nl.cells[i][2]:
                if p in OUTS or n in ('GND', 'VCC'): continue  # skip output pins and constant inputs
                d = nl.drv.get(n)
                if d and not self.isseq[d[0]] and nl.cells[d[0]][1] not in ('IBUF', 'GND', 'VCC'):  # an input fed by a combinational cell is a dependency
                    k += 1; users[n].append(i)
            indeg[i] = k
        q, order = [i for i in comb if indeg[i] == 0], []  # start with the cells that depend on nothing combinational
        while q:
            i = q.pop(); order.append(i)  # take one cell out of the work list and put it in the order
            for op, on in nl.cells[i][2]:
                if op not in OUTS: continue
                for u in users.get(on, ()):
                    indeg[u] -= 1  # one dependency of each user is now satisfied
                    if indeg[u] == 0: q.append(u)  # a user with all dependencies satisfied joins the work list
        # Cells left over with unsatisfied dependencies sit in combinational loops; they are not timed (the report says how many).
        self.loops = sum(1 for i in comb if indeg[i] > 0)
        self.order = order
        self.arcs = []                                 # per cell in order: (cell, [(out net, out pin, [(in net, pin, conn)])])
        # For every combinational cell in order, list its arcs: output net, output pin, and the inputs that can reach it. A carry cell's SUM output does not
        # depend on its own COUT arc, so that pair is left out.
        for i in order:
            f, t, pl = nl.cells[i]
            outs = []
            for op, on in pl:
                if op not in OUTS: continue
                outs.append((on, op, [(n, p, conn(n, i, p)) for p, n in pl if p not in OUTS and n not in ('GND', 'VCC')
                                      and not (t == 'ALU' and op == 'COUT' and p == 'SUM')]))
            self.arcs.append((i, outs))
        # Sources and endpoints: every register-like cell (not a clock cell, not a pad) gives a source for each output pin it drives and an endpoint for each
        # input pin that is not a clock pin.
        self.src, self.eps = [], []                    # sources: (cell, out net, pin); endpoints: (cell, pin, net, conn)
        self.ramsrc = set()                            # source nets driven by block RAM data outputs
        for i, (f, t, pl) in enumerate(nl.cells):
            if not self.isseq[i] or t in NOTIME or is_clkcell(t): continue  # only sequential cells that carry data
            for p, n in pl:
                if n in ('GND', 'VCC'): continue
                if p in OUTS:
                    if nl.drv.get(n, (None,))[0] == i:  # an output pin that really drives its net
                        self.src.append((i, n, p))
                        if p.startswith('DO'): self.ramsrc.add(n)  # block RAM data outputs (DO...) have their own, slower, clock-to-output
                elif not p.startswith('CLK'):  # every other non-clock pin is an endpoint
                    self.eps.append((i, p, n, conn(n, i, p)))
        # Group the sources by the clock edge that launches them (and by the exceptions that start at them): the longest-path pass is run once per group,
        # and each group has one launch edge so that the requirement is exact.
        lks = collections.defaultdict(list)            # passes: (launch edge, source exceptions) -> source nets
        self.nosrc = 0
        for i, n, p in self.src:
            e = self.ck.pin_clock(i, p)  # the launch edges of this source
            if not e: self.nosrc += 1  # a source with no clock is counted and not timed
            sx = self.ck.source_exc(i)  # exceptions that start at this cell
            for lk in e: lks[(lk, sx)].append(n)  # add its net to the group of each of its launch edges
        self.classes = sorted(lks.items(), key=lambda kv: str(kv[0]))  # the groups, in a fixed order
        self.epck = [self.ck.pin_clock(i, p) for i, p, n, c in self.eps]  # the capture edges of each endpoint
        self.graph = True

    # Install a delay model: the cell delay of every arc is looked up once so that the propagation loop does no lookups by name.
    def set_model(self, m):
        self.m = m
        self._cd = {}
        for i, outs in self.arcs:
            t = self.nl.cells[i][1]
            for on, op, ins in outs:
                for n, p, c in ins: self._cd[(i, p, op)] = m.cell_delay(t, p, op)

    # Net delay of every connection for the given weights: the weighted sum of its features, never below 0.05 ns (None means a dedicated wire, 0 ns).
    def delays(self, w):
        """net delay per connection id for weights w."""
        return [0.0 if f is None else max(0.05, sum(a * b for a, b in zip(w, net_features(*f)))) for f in self.feats]

    # Longest path propagation from a set of source nets. arr[net] = the latest arrival time at the net; pred[net] = the arc that gave it (so the path can be walked back).
    # Sources start at the clock-to-output delay (a different one for block RAM outputs).
    def propagate(self, srcs, nd, tco, tco_ram=None):
        """longest arrival from the given source nets; pred[net] = (in net, cell, pin, conn id)."""
        arr = {n: (tco_ram if tco_ram is not None and n in self.ramsrc else tco) for n in srcs}  # arrival at each source net
        pred = dict.fromkeys(srcs)  # sources have no predecessor
        cd = self._cd
        for i, outs in self.arcs:
            for on, op, ins in outs:
                best, bp = -1e9, None  # best arrival over all inputs of this arc, and which input gave it
                for n, p, c in ins:
                    a = arr.get(n)  # a net that no source of this group reaches has no arrival yet
                    if a is None: continue
                    a += nd[c] + cd[(i, p, op)]  # arrival at the input plus the net delay and the cell delay
                    if a > best: best, bp = a, (n, i, p, c)
                if bp is not None: arr[on] = best; pred[on] = bp  # remember the latest arrival at the output
        return arr, pred

    # Analysis: for every endpoint, the worst slack over all launch groups. slack = requirement - estimated delay, where the estimated delay is the arrival + the net into
    # the endpoint + the setup time of the endpoint's pin. A negative slack means the estimate thinks the path fails.
    def analyze(self, nd):
        """per endpoint the worst (slack, est, req, launch edge, capture edge, pass) over every launch class."""
        m, ck = self.m, self.ck
        res = [None] * len(self.eps)  # one result per endpoint: (slack, estimated delay, requirement, launch edge, capture edge, group number)
        self.passes = []  # the predecessor tables of every group, kept to walk the worst path back later
        for pi, ((lk, sx), srcs) in enumerate(self.classes):
            arr, pred = self.propagate(srcs, nd, m.tco, m.tco_ram)  # propagate from this group's sources
            self.passes.append(pred)
            for k, (i, p, n, c) in enumerate(self.eps):
                a = arr.get(n)  # the arrival at this endpoint's net, if reached
                if a is None: continue
                est = a + nd[c] + m.setup(p)  # estimated total path delay
                for cke in self.epck[k]:  # every capture edge of the endpoint
                    r = ck.requirement(lk, sx, cke, i, p)  # the requirement of this launch and capture pair
                    if r is None: continue
                    if res[k] is None or r - est < res[k][0]: res[k] = (r - est, est, r, lk, cke, pi)  # keep the pair with the smallest slack
        self.res, self.nd = res, nd
        return res

    # The worst path to endpoint k, from its start net to the endpoint, found by walking the predecessor table back.
    def path(self, k):
        """worst path of endpoint k: start net, [(net, cell, pin, conn id)] ending with the endpoint connection."""
        pred = self.passes[self.res[k][5]]  # the table of the group that gave the worst slack
        i, p, n, c = self.eps[k]
        out, net = [(n, i, p, c)], n
        while pred.get(net) is not None:  # stop at a source (no predecessor)
            bp = pred[net]; out.append(bp); net = bp[0]
        return net, out[::-1]  # the walk went backwards: reverse it

    # The cell part of a path delay: clock-to-output of the start register, setup of the endpoint, and the delay of every cell on the path.
    def path_cells(self, path, start=None):
        """cell delay of a path (tco, cells, setup) and the connections whose net delay is above the floor."""
        nl, m = self.nl, self.m
        cell = (m.tco_ram if start in self.ramsrc else m.tco) + m.setup(path[-1][2])  # start with clock-to-output and setup
        for j in range(len(path) - 1):
            n, ci, p, c = path[j]
            outp = [q for q, x in nl.cells[ci][2] if x == path[j + 1][0] and q in OUTS]  # the output pin of the cell that drives the next net on the path
            cell += m.cell_delay(nl.cells[ci][1], p, outp[0] if outp else 'F')  # add the cell's input-to-output delay
        return cell

    # Gowin's logic level count of one path: the start register counts 1, each LUT, wide multiplexer or inverter 1, and each run of carry cells by chain_levels().
    def levels(self, path):
        nl = self.nl
        lv, run = 1, 0  # lv = levels so far, run = length of the carry run being counted
        for x in path:  # walk the path from its start; each step is (net, cell, pin, connection)
            ci, p = x[1], x[2]
            t = nl.cells[ci][1]
            if t == 'ALU':  # a carry cell: its CIN pin continues the run, any other input pin starts a new one
                if p == 'CIN': run += 1
                else: lv += chain_levels(run); run = 1  # the run so far is added to the levels, a new run of length 1 starts
            else:
                lv += chain_levels(run); run = 0  # a non-carry cell ends the run
                if t.startswith('LUT') or t in WIDE or t == 'INV': lv += 1  # a LUT, wide multiplexer or inverter is one level
        return lv + chain_levels(run)  # close the last run

    # The logic level of the deepest path into every net, for the level report. A value is (levels, length of the carry run at that net). skip_src names
    # sources that are not counted (the multicycle engine's own registers, so that paths inside it do not show as long ones).
    def max_levels(self, skip_src=None):
        """Gowin's Logic Level of the deepest path into every net: {net: (levels, carry run)}."""
        nl = self.nl
        lv = {n: (1, 0) for i, n, p in self.src if not (skip_src and skip_src(i))}  # every source starts at level 1 (the launching register counts as 1)
        for i in self.order:  # the cells in topological order, so inputs are done before outputs
            f, t, pl = nl.cells[i]
            for op, on in pl:
                if op not in OUTS: continue
                best = None  # best (deepest) value over the inputs of this output
                for p, n in pl:
                    if p in OUTS or n not in lv: continue  # ignore outputs and inputs without a value
                    tot, run = lv[n]
                    if t == 'ALU' and p == 'CIN': v = (tot - chain_levels(run) + chain_levels(run + 1), run + 1)  # one more carry cell: the run's weight grows only at the break points of chain_levels
                    elif t == 'ALU': v = (tot + 1, 1)  # the first carry cell of a chain
                    elif t.startswith('LUT') or t in WIDE or t == 'INV': v = (tot + 1, 0)  # a LUT, wide multiplexer or inverter: one more level
                    else: v = (tot, 0)  # anything else adds nothing
                    if best is None or v[0] > best[0]: best = v  # keep the deepest input
                if best: lv[on] = best
        return lv


# ---- Calibration. The delay model is fitted to real place-and-route results. Gowin writes its timing report (.tr) with the worst paths: each step with its
# delay, kind (tC2Q clock to Q, tINS cell, tNET net, tSu setup), fanout and node name. The functions below read the report, join every reported path with
# the netlist of the same build, and compute the cell delays (means) and the net weights (least squares) that make the estimate agree with the real paths.
# ---------------------------------------------------------------- calibration from place-and-route timing reports
# Parse the setup paths of a Gowin timing report (.tr). Each path becomes a dict with its summary (Slack, From, To), its logic level count, its steps and its setup time.
# grp numbers the report commands, since a build asks for several lists (200 worst paths, every path above 8 levels, ...).
def parse_report(path):
    """setup paths of a Gowin .tr: dicts with ll, steps [(delay, type, fanout, loc, node)], tsu, grp (report command)."""
    paths, cur, mode, grp, cmd = [], None, None, 0, ''  # paths found, the path being read, what part of its text we are in (summary, arrival, required), the command number, the command text
    for ln in open(path, encoding='utf-8', errors='replace'):
        if ln.startswith('3.3.2 Hold Analysis Report'): break  # the hold analysis starts: only setup paths are used, so stop
        if ln.startswith('3.3.1 Setup Analysis Report'): grp = 0  # the setup section starts: command numbering restarts
        if ln.startswith('Report Command:report_timing'): grp += 1; cmd = ln.strip()  # each report_timing command starts a new list of paths
        if ln.startswith('Path Summary:'):  # a new path starts
            cur = {'steps': [], 'grp': max(grp, 1), 'cmd': cmd}; paths.append(cur); mode = 'sum'; continue
        if cur is None: continue  # nothing to attach lines to before the first path
        if mode == 'sum':  # summary lines: slack, start node, end node
            m = re.match(r'(Slack|From|To)\s*:\s*(\S+)', ln)
            if m: cur[m.group(1)] = m.group(2)
        if ln.startswith('Data Arrival Path:'): mode = 'arr'; continue  # the arrival path lists the delay steps from the launching clock to the endpoint
        if ln.startswith('Data Required Path:'): mode = 'req'; continue  # the required path (its last step is the setup time of the capturing register)
        if ln.startswith('Logic Level:'): cur['ll'] = int(ln.split(':')[1]); mode = None; continue  # the logic level count that Gowin printed; the path text ends here
        if mode in ('arr', 'req'):  # inside a step table
            f = ln.split()
            if len(f) >= 4 and re.match(r'-?\d+\.\d+$', f[0]) and f[2] in ('tCL', 'tINS', 'tNET', 'tC2Q', 'tSu'):  # a step line: delay, step type (tCL clock net, tINS cell, tNET net, tC2Q clock to Q, tSu setup), fanout, ... and node name
                rest = f[3:]
                if rest and re.match(r'^[RF]{2}$', rest[0]): rest = rest[1:]  # an optional rise/fall marker (RR, RF, ...) is skipped
                fo = int(rest[0]) if rest and rest[0].isdigit() else 0  # the fanout is the first number
                st = (float(f[1]), f[2], fo, rest[1] if len(rest) > 2 else '', rest[-1])  # step = (delay, type, fanout, location, node name)
                if mode == 'arr': cur['steps'].append(st)  # arrival steps are kept in order
                elif f[2] == 'tSu': cur['tsu'] = -st[0]  # the setup time is the negative of the last required step
    return [p for p in paths if 'll' in p and 'tsu' in p]  # only complete paths are returned


# Join every report path with the netlist. For each path: the total data delay D, the cell arcs it used with their delays, and the nets it used with their features. A path
# whose nodes are not in the netlist (a different build, or a renamed cell) is counted as missing and skipped.
def report_paths(nl, est, rep):
    """join report paths with the netlist: per path data delay D, the cell arcs and the nets with their features."""
    out, nmiss = [], 0
    for p in rep:
        st = p['steps']
        k0 = next((k for k, s in enumerate(st) if s[1] == 'tC2Q'), None)  # the step where the launching register's clock-to-Q starts the data path
        if k0 is None: continue  # a path without it is skipped
        cells, nets, ok = [], [], True  # the cells (from pin, to pin, delay) and nets (with features) of this path; ok turns False when a net can not be found
        for k in range(k0 + 1, len(st)):
            s, d = st[k], st[k - 1]  # s = this step, d = the one before it
            if s[1] == 'tINS':  # a cell step: record input pin, output pin and delay
                cells.append((d[4].rsplit('/', 1)[1], s[4].rsplit('/', 1)[1], s[0]))
            elif s[1] == 'tNET':  # a net step: find the net that leaves the driving pin
                dn, dp = d[4].rsplit('/', 1); ln, lp = s[4].rsplit('/', 1)
                a, b = nl.byname.get(dn), nl.byname.get(ln)  # the driving and the loaded cell in the netlist
                net = [n for q, n in nl.cells[a][2] if q == dp] if a is not None else []  # the net on the driving pin
                mb = re.match(r'(\w+)\[(\d+)\]$', dp)  # a bus pin such as DO[3]
                if not net and mb and a is not None:      # bus pin (block RAM DO[i]): the netlist lists the bus MSB first
                    bus = [n for q, n in nl.cells[a][2] if q == mb.group(1)]
                    if int(mb.group(2)) < len(bus): net = [bus[len(bus) - 1 - int(mb.group(2))]]
                if not net or b is None: ok = False; break  # the net was not found: give up on this path
                net = net[0]
                if dp == 'COUT' and lp == 'CIN': continue  # the dedicated carry wire has no delay to model
                fo = len(nl.loads[net]); sz = est.lca_size(est.cmod[a], est.cmod[b]); nm = est.nmods(net)  # features: fanout, size of the common module, number of load modules
                nets.append((d[4], s[4], s[0], fo, sz, nm, nl.size['']))  # record the net with its measured delay and its features
        if not ok: nmiss += 1; continue  # skipped paths are counted
        D = st[k0][0] + sum(s[0] for s in st[k0 + 1:]) + p['tsu']  # the real data path delay: clock-to-Q, all cells and nets, setup
        out.append(dict(D=D, ll=p['ll'], cells=cells, nets=nets, to=p['To'], pin=st[-1][4].rsplit('/', 1)[1], spin=st[k0][4].rsplit('/', 1)[1],
                        tco=st[k0][0], tsu=p['tsu'], slack=float(p['Slack']), grp=p['grp'], cmd=p.get('cmd', '')))
    return out, nmiss


# The key of a cell arc in the model: all LUT inputs share one delay (the placer swaps them); the wide multiplexer and carry cell have separate delays per input and output pin.
def arc_key(a, b):
    if b == 'F': return 'LUT'
    if b == 'O': return 'MUXW:S0' if a == 'S0' else 'MUXW:I'
    return ('ALU:CIN>' if a == 'CIN' else 'ALU:I>') + b


# Cell delays and register times as means over all reported paths of all designs. Arcs never seen in a report get a default.
def fit_cells(rps):
    mean = lambda x: sum(x) / len(x)  # arithmetic mean
    arcs, tsu, tco, tcr = collections.defaultdict(list), collections.defaultdict(list), [], []  # delays per arc, setup times per pin type, clock-to-Q of registers and of block RAM outputs
    for rp in rps:
        for p in rp:
            for a, b, d in p['cells']: arcs[arc_key(a, b)].append(d)  # every cell step goes in its arc's list
            tsu[re.sub(r'\[\d+\]$', '', p['pin'])].append(p['tsu'])  # the setup time, by pin type
            (tcr if p.get('spin', '').startswith('DO') else tco).append(p['tco'])  # block RAM outputs (DO) have their own clock-to-output
    cell = {k: round(mean(v), 3) for k, v in arcs.items()}  # mean per arc
    for k, v in (('LUT', 0.39), ('ALU:I>COUT', 0.5), ('ALU:CIN>COUT', 0.045), ('ALU:CIN>SUM', 0.25), ('MUXW:I', 0.135)):  # defaults for arcs that no report path used (typical values of the chip)
        cell.setdefault(k, v)
    tsud = {k: round(mean(v), 3) for k, v in tsu.items()}  # mean setup per pin type
    tsud['other'] = round(mean([x for v in tsu.values() for x in v]), 3)  # other pin types use the mean of all
    return dict(cell=cell, tco=round(mean(tco), 3), tco_ram=round(mean(tcr), 3) if tcr else round(mean(tco), 3), tsu=tsud)


# One calibration design: netlist, clocks, the reported paths, and what they say about endpoints that are not in the report. A report lists only the worst paths,
# but it also bounds all the others: an endpoint missing from the list is no slower than the last listed path (its slack is at least the smallest listed one, smin), which
# the fit uses as a one-sided constraint.
class Design:
    """one calibration design: netlist, clocks, report endpoints and the bound for endpoints not in the report."""

    def __init__(self, vg, sdc, tr, top=None):
        self.name = os.path.basename(tr)  # the file name of the report names the design
        self.nl = Netlist(vg, top)  # the netlist of this build
        self.ck = Clocks(self.nl, sdc, DEFAULT_CLK)  # its clocks from the SDC
        self.es = Est(self.nl, None, self.ck)  # an estimator without a model yet
        self.rp, self.miss = report_paths(self.nl, self.es, parse_report(tr))  # the paths of the report joined with the netlist
        self.es.build()  # the timing graph
        txt = open(tr, encoding='utf-8', errors='replace').read()  # the report text again, to read Gowin's own Fmax figures
        self.fmax = {m.group(1): float(m.group(2)) for m in  # Fmax per clock from Gowin's frequency table
                     re.finditer(r'^\s+\d+\s+(\S+)\s+[\d.]+\(MHz\)\s+([\d.]+)\(MHz\)', txt, re.M)}
        # Some builds ask for one worst-path list per pair of clocks, instead of a single list. When FIT_CLOCK has such a list for itself, that list is the bound list
        # and only the endpoints captured by FIT_CLOCK are fitted and counted. Otherwise list 1 is used.
        fc = '{%s}] -to_clock [get_clocks {%s}]' % (FIT_CLOCK, FIT_CLOCK)  # the text of the report command for that clock pair
        gc = [p['grp'] for p in self.rp if fc in p['cmd'] and 'min_logic_level' not in p['cmd']]  # the list numbers of the commands that match
        self.lst, self.clk = (gc[0], FIT_CLOCK) if gc else (1, None)  # list number and clock to use
        for p in self.rp:  # swap the numbers so that the chosen list is number 1 afterwards
            if p['grp'] == self.lst: p['grp'] = 1
            elif p['grp'] == 1: p['grp'] = self.lst
        self.real = {}                                 # (cell index, pin) -> (worst real data delay, slack, list)
        for p in self.rp:  # for every reported path, keep the worst real delay of each endpoint
            ci = self.nl.byname.get(p['to'])  # the endpoint's cell in the netlist
            if ci is None: continue
            k = (ci, p['pin'])
            if k not in self.real or p['D'] > self.real[k][0]: self.real[k] = (p['D'], p['slack'], p['grp'])  # the largest delay wins
        g1 = [p['slack'] for p in self.rp if p['grp'] == 1]  # slacks of the paths in the bound list
        self.smin = max(g1) if g1 else None            # every endpoint not in list 1 has at least this slack

    def evaluate(self, model):  # Run the estimate with a model and return the per-endpoint results.
        self.es.set_model(model)
        return self.es.analyze(self.es.delays(model.w))


# The cell part of an endpoint's worst path, and the sum of the net features along it, which are the row of the least squares system for this endpoint.
def path_terms(es, k):
    """cell part and summed net features of endpoint k's worst path."""
    start, path = es.path(k)
    cellp, F = es.path_cells(path, start), [0.0] * len(NET_TERMS)  # cell delays, and a vector with one slot per net term
    for n, ci, p, c in path:
        f = es.feats[c]
        if f is None: continue
        if es.nd[c] > 0.05:  # nets above the floor of 0.05 ns carry the features; floor nets just add 0.05 ns to the cell part (they are fixed)
            for a, v in enumerate(net_features(*f)): F[a] += v
        else: cellp += 0.05
    return cellp, F


# Fit the net weights. For every endpoint the estimate gives cell delay + weights . features. The target of a reported endpoint is its real delay; the target of an
# unreported endpoint that the estimate puts above the bound is the bound. Least squares finds the weights that best satisfy all targets. The worst path of an endpoint
# changes as the weights change, so the fit repeats up to 15 times, moving half the way each time, until the weights change by less than 0.003 ns.
def fit_designs(ds, w0, iters=15, wc=1.0):
    """net weights fitted per endpoint: reported endpoints match their real worst path; endpoints not reported that the
    estimate puts above the report's bound are pulled down to it. Iterated: the worst path per endpoint changes with w."""
    import numpy as np  # numpy is only needed for fitting, so it is imported here and the normal estimate does not need it
    cm = fit_cells([d.rp for d in ds])  # cell delays are means and need no iteration
    w = np.array(w0, float)  # start from the given weights
    for it in range(iters):
        A, b, W = [], [], []  # A = feature rows, b = targets, W = weights of the rows
        for d in ds:
            res = d.evaluate(Model(dict(cm, net=dict(w=list(w)))))  # evaluate every design with the current weights
            nrep = max(1, len(d.real))  # each design counts equally, whatever its number of endpoints
            for k, r in enumerate(res):
                if r is None or (d.clk and r[4][0] != d.clk): continue  # only endpoints captured by the fitted clock
                key = d.es.eps[k][:2]
                if key in d.real: tgt, wt = d.real[key][0], 1.0 / nrep  # reported endpoint: its real worst path delay is the target
                elif d.smin is not None and r[0] < d.smin: tgt, wt = r[2] - d.smin, wc / nrep  # unreported endpoint estimated slower than the report allows: the bound is the target
                else: continue  # otherwise no information
                cellp, F = path_terms(d.es, k)  # cell part and features of its worst path
                A.append(F); b.append(tgt - cellp); W.append(wt)  # the row, the target for the net part, and its weight
        A, b, W = np.array(A), np.array(b), np.array(W)
        R = np.eye(len(w)) * W.sum() * 1e-3  # a small ridge term that keeps the system solvable and the weights near the old ones
        wn = np.linalg.solve((A * W[:, None]).T @ A + R, (A * W[:, None]).T @ b + R @ w)  # solve the weighted normal equations
        step = float(np.max(np.abs(wn - w)))  # the largest change of any weight
        w = 0.5 * w + 0.5 * wn  # damped update: move halfway
        if step < 0.003: break  # converged
    return dict(cm, net=dict(w=[round(float(x), 4) for x in w], terms=NET_TERMS))  # the fitted model: cell delays and net weights


# Print the accuracy of a model on one design: error of the estimate for the endpoints of the worst-path list, for the reported paths themselves, and for the worst
# endpoint of each clock against Gowin's real Fmax. This is where the accuracy figures quoted in docs/learn/HISTORY.md, section 15, come from.
def report_design(d, model, out=sys.stdout):
    """accuracy on one design: per endpoint in the report, per reported path, worst endpoint vs real Fmax."""
    res = d.evaluate(model)
    es = d.es
    err = [r[1] - d.real[es.eps[k][:2]][0] for k, r in enumerate(res)  # estimate minus real for the endpoints of the worst-path list
           if r is not None and es.eps[k][:2] in d.real and d.real[es.eps[k][:2]][2] == 1]
    perr = []  # errors of the reported paths themselves, with the model's delays
    for p in d.rp:                                     # the reported path itself, with the model's delays
        e = (model.tco_ram if p.get('spin', '').startswith('DO') else model.tco) + model.setup(p['pin']) + sum(model.cell.get(arc_key(a, b), model.cell['LUT']) for a, b, x in p['cells'])
        perr.append(e + sum(model.net_delay(*n[3:]) for n in p['nets']) - p['D'])
    q = lambda v, f: sorted(v)[int(f * (len(v) - 1))] if v else float('nan')  # q is a percentile, ab absolute values, mn the mean
    ab = lambda v: [abs(x) for x in v]
    mn = lambda v: sum(v) / max(1, len(v))
    over = sum(1 for k, r in enumerate(res) if r is not None and es.eps[k][:2] not in d.real  # endpoints not in the report that the estimate puts slower than the report allows
               and d.smin is not None and r[0] < d.smin and not (d.clk and r[4][0] != d.clk))
    print('%s: %d report paths (%d not in the netlist), %d endpoints of the worst-path list; %d endpoints not in the report'
          ' estimated worse than the report allows' % (d.name, len(d.rp), d.miss, len(err), over), file=out)
    print('  worst-path list endpoints, est - real: mean %+.2f, mean |err| %.2f, 90%% |err| %.2f ns'
          % (mn(err), mn(ab(err)), q(ab(err), 0.9)), file=out)
    print('  reported paths (that path itself), est - real: mean %+.2f, mean |err| %.2f, 90%% |err| %.2f ns'
          % (mn(perr), mn(ab(perr)), q(ab(perr), 0.9)), file=out)
    per = {}  # the worst slack per capture clock
    for r in res:
        if r is not None and (r[4][0] not in per or r[0] < per[r[4][0]]): per[r[4][0]] = r[0]
    for c, s in sorted(per.items()):
        T, real = d.ck.clk[c]['T'], d.fmax.get(c)  # the clock's period and Gowin's Fmax for it
        rs = ('real %.3f MHz (worst %.2f ns): est - real %+.2f ns' % (real, 1000 / real, T - s - 1000 / real)) if real else ''  # compare the estimated worst path delay with the real one (1000 / Fmax in ns)
        print('  clock %s: worst slack %+.2f -> est %.2f ns (%.1f MHz); %s' % (c, s, T - s, 1000 / max(0.1, T - s), rs),
              file=out)


# ---- Output. Names are shortened so that the many copies of one cell are one line in the report, and each flagged endpoint is described by its worst path.
# ---------------------------------------------------------------- output
# Remove the numbering that synthesis adds to names (_s1, _2, [3]), so that a bus or a group of generated cells becomes one group.
def norm(name):
    n = name.split('/')
    leaf = re.sub(r'_s\d*$', '', n[-1]); leaf = re.sub(r'(_\d+)+$', '', leaf); leaf = re.sub(r'\[\d+\]', '', leaf)
    return '/'.join(n[:-1] + [leaf])


# The clock whose own worst-path list bounds the fit, when a report has one list per pair of clocks. It is the name of the CPU clock in the full system.
FIT_CLOCK = 'clk_cpu'           # the clock whose own worst-path list bounds the fit (per-clock-pair reports)
# Thresholds for the explanation columns: a net is "far" when the smallest module that holds both of its ends has more than 2048 cells, and counts as "fanout" above 16 loads.
LOCAL_SZ, FAN = 2048, 16       # a net is 'far' when the smallest module holding both ends has more cells; 'fanout' above FAN


# Explain an endpoint's worst path: where it starts, its logic levels, its cell delay, how much of its net delay is in local nets, high-fanout nets or far nets, and its slowest nets.
def describe(es, k):
    nl = es.nl
    start, path = es.path(k)
    d = nl.drv.get(start)  # the starting cell
    sname = nl.cells[d[0]][0] if d else '(input)'
    split, nets = [0.0, 0.0, 0.0], []  # net delay split: local, fanout, far; and the list of nets
    for n, ci, p, c in path:
        f = es.feats[c]
        if f is None: continue
        e, fo, sz = es.nd[c], f[0], f[1]
        split[1 if fo > FAN else 2 if sz > LOCAL_SZ else 0] += e  # classify the net: high fanout first, then far, otherwise local
        dn = nl.drv.get(n)
        nets.append((e, fo, sz, nl.lca(es.cmod[dn[0]] if dn else '', es.cmod[ci]) or 'top'))  # record delay, fanout, module size and the module that holds both ends
    nets.sort(reverse=True)  # slowest net first
    return sname, es.levels(path[:-1]), es.path_cells(path, start), split, nets


# Main: with --fit it calibrates the model on real builds; with --levels it prints the logic level report; otherwise it estimates the netlist and prints the report.
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])  # the first line of the module docstring is the description in --help
    ap.add_argument('netlist', nargs='?')
    ap.add_argument('--sdc', default='', help='the SDC of the build (clocks, groups, exceptions)')
    ap.add_argument('--period', type=float, default=None,
                    help="period of clocks the SDC does not name (default: 12.5 without --sdc, Gowin's 10 with it)")
    ap.add_argument('--margin', type=float, default=0.0, help='flag endpoints above requirement - margin (ns)')
    ap.add_argument('--mcp', default='', help='--levels only: SUBSTR; endpoints inside it count paths from outside only')
    ap.add_argument('--model', default=default_model_path())
    ap.add_argument('--fit', nargs=3, action='append', metavar=('NETLIST', 'SDC', 'REPORT'),
                    help="calibration design (repeat; SDC '-' for none); writes --model; prints accuracy, held out per design")
    ap.add_argument('--levels', type=int, default=0, help='instead: list endpoints deeper than N Gowin logic levels')
    ap.add_argument('--all', action='store_true', help='list every flagged endpoint, not only the worst per group')
    ap.add_argument('--top-module', default=None, help='default: the module no other module instantiates')
    a = ap.parse_args()  # command line read
    # Calibration mode: build a model from real place-and-route results. Each --fit takes the netlist, the SDC and the timing report of one build.
    if a.fit:
        ds = [Design(v, None if s == '-' else s, r, a.top_module) for v, s, r in a.fit]  # load every design (netlist, clocks, report paths)
        # With more than one design, first test honestly: fit on all designs but one and measure the accuracy on the one left out. That number says how well the model
        # will do on a build it has not seen.
        if len(ds) > 1:
            print('# held out: fitted on the other designs, tested on this one')
            for k, d in enumerate(ds):
                report_design(d, Model(fit_designs([x for j, x in enumerate(ds) if j != k], W0)))  # fit on the others (starting from the default weights), report on this one
        md = fit_designs(ds, W0)  # the final fit on all designs
        md['designs'] = [os.path.basename(r) for v, s, r in a.fit]  # remember which reports it came from
        json.dump(md, open(a.model, 'w'), indent=1, sort_keys=True)  # write the model to the JSON file
        print('# fitted on all designs -> %s: net weights %s' % (a.model, md['net']['w']))
        for d in ds: report_design(d, Model(md))  # accuracy of the final model on every design
        return
    if not a.netlist: ap.error('netlist required')  # a netlist is needed for any other mode
    nl = Netlist(a.netlist, a.top_module)  # read the netlist
    dflt = a.period if a.period else (DEFAULT_CLK if a.sdc else 12.5)  # the period of clocks the SDC does not name: Gowin's 10 ns with an SDC, 12.5 ns (80 MHz) without
    ck = Clocks(nl, a.sdc or None, dflt)  # the clocks from the SDC
    es = Est(nl, None, ck)  # the estimator, without a model yet
    es.build()  # build the timing graph
    # Level report mode (what tools/levels.py asks for): list the endpoints whose deepest path has more than --levels logic levels.
    if a.levels:
        mcp = a.mcp.split(':')[0]  # --mcp may carry a ":..." part that is ignored
        inm = (lambda i: mcp in nl.cells[i][0]) if mcp else None  # inm tells whether a source cell is inside the multicycle module
        lv1 = es.max_levels(); lv2 = es.max_levels(inm) if mcp else lv1  # levels counted with and without the sources inside that module
        rows = collections.Counter()  # group the endpoints by (levels, name)
        for i, p, n, c in es.eps:
            lv = lv2 if (inm and inm(i)) else lv1  # endpoints inside the module only count paths from outside
            if n in lv and lv[n][0] > a.levels: rows[(lv[n][0], norm(nl.cells[i][0]) + '.' + p)] += 1  # count the endpoint if it is deeper than the limit
        print('# Gowin logic levels (start register 1, LUT 1, carry chain 1..5) above %d' % a.levels)  # heading
        for (l, n), c in sorted(rows.items(), key=lambda kv: (-kv[0][0], kv[0][1])): print('%2d  %s  x%d' % (l, n, c))  # deepest first
        print('LEVELS: %d endpoints above %d, %d groups' % (sum(rows.values()), a.levels, len(rows)))  # the last line, read by tools/build.sh
        return  # done
    # Estimate mode. Load the fitted model, give the net delays, find the worst path of every endpoint.
    m = Model(json.load(open(a.model)))
    es.set_model(m)
    res = es.analyze(es.delays(m.w))
    mg = a.margin  # the margin: an endpoint is flagged when its slack is below it
    groups = collections.defaultdict(list)  # flagged endpoints grouped by (start unit, end unit and pin)
    per = {}  # per capture clock: [flagged count, worst result, endpoint count]
    risk = 0  # endpoints that are not flagged but within 1.5 ns of it
    for k, r in enumerate(res):
        if r is None: continue
        pc = per.setdefault(r[4][0], [0, r, 0])  # per clock: flagged count, worst result so far, total count
        pc[2] += 1  # one more endpoint for this clock
        if r[0] < pc[1][0]: pc[1] = r  # keep the worst result for this clock
        if r[0] < mg:  # flagged: slack below the margin
            pc[0] += 1
            i, p = es.eps[k][:2]
            sname, l, cell, split, nets = describe(es, k)  # describe its worst path
            groups[(norm(sname), norm(nl.cells[i][0]) + '.' + p)].append((r, nl.cells[i][0] + '.' + p, l, cell, split, nets))  # group it with the endpoints of the same units
        elif r[0] < mg + 1.5: risk += 1  # not flagged but close
    nflag = sum(v[0] for v in per.values())  # total flagged
    rows = sorted(groups.items(), key=lambda kv: min(x[0][0] for x in kv[1]))  # groups sorted by their worst slack
    # The report. First comment lines: command line, SDC warnings, the clocks found with period, rise and fall times, and where each came from.
    print('# timing_est.py %s  sdc %s  margin %.2f ns' % (a.netlist, a.sdc or '(none: every clock %.3f ns)' % dflt, mg))
    for w in ck.warn: print('# SDC: ' + w)
    print('# clocks: period, rise, fall (ns), source')
    for c, v in sorted(ck.clk.items()):
        print('#   %-30s %8.3f %7.3f %7.3f  %s' % (c, v['T'], v['rise'], v['fall'], v['src'] or '-'))
    if es.loops: print('# %d cells in combinational loops (not timed)' % es.loops)
    nun = sum(1 for k, r in enumerate(res) if r is None and not es.epck[k])  # endpoints that no clock reaches (no timing)
    # The summary and the column explanation.
    print('# %d of %d endpoints estimated above requirement - %.2f ns, %d groups; %d endpoints without a clock;'
          ' %d more within 1.5 ns' % (nflag, len(es.eps), mg, len(rows), nun, risk))
    print('# slack = req - est; est = estimated worst path delay; req = requirement (launch -> capture edge, exceptions);')
    print('#   lvl = Gowin logic levels of that path; cell / local / fanout / far = ns in cells, local nets, nets with fanout'
          ' > %d, nets between modules with > %d cells' % (FAN, LOCAL_SZ))
    print('# slack    est    req lvl   cell local fanout   far  launch -> capture   start -> end  xcount')
    fe = lambda e: e[0] + ('(fall)' if e[1] else '')  # shows a falling-edge launch or capture as "(fall)"
    # One line per group: the slack and delays of its worst endpoint, the split of its time into cells and into local, fanout and far nets, the clock edges and the units.
    for (s, en), l in rows:
        w = min(l, key=lambda x: x[0][0]); r = w[0]  # the worst endpoint of the group
        print('%6.2f %6.2f %6.2f %3d  %5.2f %5.2f %5.2f %5.2f  %s -> %s   %s -> %s  x%d' % (
            r[0], r[1], r[2], w[2], w[3], w[4][0], w[4][1], w[4][2], fe(r[3]), fe(r[4]), s, en, len(l)))
    # Then the slowest nets of each group's worst path. They show what to cut: a fanout above 64 needs register copies and a long net between big modules needs a register
    # at the boundary.
    print('\n# worst nets of the worst path per group: delay ns (fanout, module holding both ends: cells)')
    for (s, en), l in rows:
        w = min(l, key=lambda x: x[0][0])
        print('%6.2f  %s   %s' % (w[0][0], w[1], '; '.join('%.2f (fo %d, %s: %d)' % (e, fo, mod, sz) for e, fo, sz, mod in w[5][:3])))  # the three slowest nets with delay, fanout, module that holds both ends and its cell count
        if a.all:  # --all lists every flagged endpoint
            for x in sorted(l, key=lambda x: x[0][0])[1:]: print('        %6.2f  %s' % (x[0][0], x[1]))
    # One CLOCK line per capture clock, read by tools/build.sh and tools/postbuild.py (check 7).
    for c, (n, r, tot) in sorted(per.items()):
        print('CLOCK: %s period %.3f endpoints %d above: %d worst est %.2f req %.2f slack %+.2f (from %s)' % (
            c, ck.clk[c]['T'], tot, n, r[1], r[2], r[0], fe(r[3])))
    # The TIMING line, with the count of flagged endpoints, is the last line. tools/build.sh gets the count from it for the optional gate.
    print('TIMING: %d endpoints above the requirement - %.2f ns, %d groups, %d clocks' % (nflag, mg, len(rows), len(per)))


# Run main() only when the file is started as a script.
if __name__ == '__main__':
    main()
