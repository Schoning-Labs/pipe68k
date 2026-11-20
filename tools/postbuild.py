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
"""Post-build checklist for a Gowin place-and-route run (docs/learn/HISTORY.md, section 15).

Usage: postbuild.py BUILD_DIR [--tr FILE] [--rpt FILE] [--est FILE] [--log FILE] [--top N]
  BUILD_DIR is the Gowin project folder (the one holding gw_sh.log, impl/, timing.txt). Files default to
  BUILD_DIR/impl/pnr/<name>.tr / .rpt.txt, BUILD_DIR/timing.txt, BUILD_DIR/gw_sh.log; each can be given
  separately (e.g. kept copies after the build folder is deleted).

Checks, in SUG113 4.3 order (Gowin's own timing-closure procedure):
  1. Errors: any ERROR line (PR0004 unrouted, PA2017 over-use, ...), also "0 unrouted nets" reported as an error.
  2. Synthesis traps: EX1998 (undriven net rewired to an unrelated register), count per net name.
  3. Resources against SUG113 4.3.2: CLS >= 85 %, BSRAM or DSP > 80 % is an "excessive design".
  4. Clock resources: PRIMARY / LW full, data or reset nets put on clock lines, PR1014 (clock on general routing).
  5. Fmax per clock against its target; failing endpoint counts (TNS table).
  6. Worst paths per failing clock: net share of delay, longest net (fanout, from/to unit), logic levels,
     unit crossings, highest fanout on the path. Flags: net share > 60 % (placement / crossings),
     fanout > 64 (register copy), levels > 8 (SUG113 Arora at 100 MHz).
  7. Estimate against real: timing_est.py worst per clock against the real worst per clock.
Prints FLAG lines for everything that needs action; exit status 0 always.

What this script is for. A place-and-route run of the full chip takes from minutes to hours and writes a pile of
reports. Reading them in a haphazard order is slow and misleading, so Gowin's own timing-closure procedure (user guide
SUG113 section 4.3) fixes an order, and this script walks it and prints a FLAG line for everything that needs action.
The order matters because each step makes the next one meaningful:
  1. Errors: if the router failed or a resource is over-used, the timing numbers of the run are not real.
  2. Synthesis traps: a wrong netlist makes every later number irrelevant. EX1998 is the one met here, an undriven net
     that synthesis rewires to an unrelated register.
  3. Resources: a design that uses most of the logic slices (CLS 85 percent or more) or of the block RAM or DSP
     (above 80 percent) is called excessive by Gowin, and the placer then has little freedom. These figures are facts to
     report, not limits in this project.
  4. Clock resources: the chip has 16 global clock lines (8 PRIMARY and 8 LW in Gowin's report). A clock must sit on one;
     a clock on ordinary wiring was measured at 0.8 to 2.4 ns of skew (the same clock arriving at two registers at
     different times), which ruins both setup and hold. Run out of lines? Take resets and enables off them, never clocks.
  5. Fmax per clock: the highest frequency each clock domain reaches against its target, and how many endpoints fail.
  6. Worst paths of each failing clock, taken apart into cell time and wire time, so the cut that helps is the right one.
  7. The estimate against the real result, which tells whether tools/timing_est.py can still be trusted for ranking paths.
Do not change the place or route options of Gowin while steps 1, 2, 4 or 6 still have flags: options cannot fix a clock on
ordinary wiring or a path with too many logic levels.

Words used in the reports. Setup slack is the time left over between a signal arriving at a register and the moment the
register needs it (clock period minus path delay minus the register's setup time), so a negative slack is a failing
path. Fanout is the number of loads a net drives. A net is a wire; "net delay" is the time a signal needs to travel over
the routing between two cells, as opposed to "cell delay" inside a LUT or a register.
"""
# Everything here is standard library: argparse for the command line, glob to find the Gowin output files, re (regular
# expressions) for reading the text reports, collections for counters.
import argparse, os, re, sys, glob, collections

# Read a whole text file, or return None when it is missing, so that every check can say "no file" and go on.
def rd(p):
    try:
        return open(p, errors='ignore').read()  # errors='ignore': the Gowin reports contain odd characters that must not stop the script
    except OSError:  # file missing or unreadable
        return None

# Print one line that needs action. Every FLAG line of a run is what has to be read before the next build.
def flag(msg):
    print('  FLAG ' + msg)

# Turn a node name from a Gowin timing report into the name of the unit (hierarchy level) it sits in.
# A node looks like core/i_ex/g_fpu.i_fpu/x_s5/F, so the last part is the pin; the part before it may be the cell
# (its name ends in _s and a number), which is dropped too. depth says how many hierarchy levels count as a unit.
def unit(node, depth):
    parts = node.split('/')[:-1]          # drop the pin
    if parts and re.search(r'_s\d*$', parts[-1]):
        parts = parts[:-1]                # drop the cell
    return '/'.join(parts[:depth]) or '(top)'

# Main: find the files, then run the seven checks of the checklist in order, printing a heading for each.
def main():
    # Command line: the build folder and optional overrides for each input file (kept copies of a deleted build folder).
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--tr'); ap.add_argument('--rpt'); ap.add_argument('--est'); ap.add_argument('--log')
    ap.add_argument('--top', type=int, default=3, help='worst paths shown per failing clock')
    ap.add_argument('--depth', type=int, default=4, help='hierarchy depth that counts as a unit')
    a = ap.parse_args()
    d = a.dir
    # Find the first file that matches one of the patterns inside the build folder (for example impl/pnr/*.tr).
    def first(*pats):
        for p in pats:
            g = sorted(glob.glob(os.path.join(d, p)))
            if g:
                return g[0]
        return None
    # The place-and-route timing report (.tr), the place report with resource use (.rpt.txt), the timing estimate made
    # before place-and-route (timing.txt) and the gw_sh log. A file given on the command line wins over the default.
    tr = a.tr or first('impl/pnr/*.tr')
    rpt = a.rpt or first('impl/pnr/*.rpt.txt')
    est = a.est or first('timing.txt')
    log = a.log or first('gw_sh.log')
    # The text of each file, or None when it does not exist. T = timing report, R = place report, E = estimate, L = log.
    T, R, E, L = rd(tr) if tr else None, rd(rpt) if rpt else None, rd(est) if est else None, rd(log) if log else None

    # Check 1, errors. Gowin SUG113 section 4.3 starts with errors because a run that failed (unrouted nets, a resource
    # over-used) leaves timing numbers that mean nothing, so nothing after this can be trusted until these are gone.
    print('== 1. Errors')
    errs = []
    # Look in the log and in the place report for lines that start with ERROR.
    for txt in (L, R):
        if txt:
            errs += [l.strip() for l in txt.splitlines() if re.match(r'\s*ERROR', l)]
    # Print each distinct error once. PR0004 means nets the router could not connect, PA2017 more of a resource than the chip has.
    for e in sorted(set(errs)):
        flag(e[:200])
        # Gowin sometimes prints an error line saying "total 0 unrouted nets"; the real cause is then in the lines before it.
        if re.search(r'total 0 unrouted', e):
            flag('router reports an error with 0 unrouted nets: read the lines before it in gw_sh.log')
    if not errs:
        print('  none')
    # A log without a timing report means place-and-route stopped before the analyzer ran.
    if L and not T:
        flag('no timing report: place-and-route did not finish')

    # Check 2, synthesis traps. EX1998 is a synthesis message: when a register bit is constant, synthesis removes it and
    # has been seen to wire the net that used it to an unrelated register, which makes the hardware wrong without any
    # error. The count and the net names are shown so the RTL can be fixed.
    print('== 2. Synthesis traps')
    if L:
        # Every EX1998 line names the net it concerns.
        ex = re.findall(r"EX1998.*?Net '([^']+)'", L)
        if ex:
            flag(f'EX1998 on {len(ex)} nets (undriven net rewired to an unrelated register): ' + ', '.join(sorted(set(ex))[:12]))
        else:
            print('  EX1998: none')
    else:
        print('  no gw_sh.log')

    # Check 3, resources. SUG113 4.3.2 calls a design "excessive" when CLS (the chip's configurable logic slices) are at 85
    # percent or more, or block RAM or DSP above 80 percent: the placer then has little room, and a failing clock is more
    # likely to come from crowding than from the logic. This project treats the figures as facts to report and not as
    # limits (the chip may be used up to 100 percent); only a figure above 100 percent (shown as 101 or more) is a real problem.
    print('== 3. Resources (SUG113 4.3.2: CLS >= 85 %, BSRAM/DSP > 80 % = excessive)')
    if R:
        # Per resource: its name in the report, the percentage from which it is flagged, and whether that percentage itself is
        # flagged (ge) or only what is above it. Logic and Register are flagged only above 101 percent, which means over-used.
        for name, lim, ge in (('Logic', 101, False), ('Register', 101, False), ('CLS', 85, True), ('BSRAM', 80, False), ('DSP', 80, False)):
            # A line of the place report looks like "CLS | 12345/17000 | 72%": used, available, percent.
            m = re.search(r'^\s*' + name + r'\s*\|\s*(\d+)/(\d+)\s*\|\s*(\d+)%', R, re.M)
            if m:
                u, t, p = int(m.group(1)), int(m.group(2)), int(m.group(3))
                # CLS is flagged at its limit, the others above it.
                bad = (p >= lim) if ge else (p > lim)
                s = f'{name:9s} {u}/{t} {p}%'
                # Print through flag() when over the limit, as a plain line otherwise.
                (flag if bad else print)(('' if bad else '  ') + s + ('  above the limit' if bad else ''))
    else:
        print('  no .rpt.txt')

    # Check 4, clock resources. The chip has 16 global clock lines (UG306 2.1), shown by Gowin as 8 PRIMARY and 8 LW lines.
    # A clock should always sit on one of them, because a clock on ordinary wiring has skew of 0.8 to 2.4 ns and gave hold
    # failures. When the lines run out, resets and enables are moved off them, never clocks. This check reports lines used
    # against lines available, and which nets besides clocks use them.
    print('== 4. Clock resources')
    if R:
        # One line of the form "PRIMARY | 5/8" per kind of clock resource: used and available.
        for m in re.finditer(r'^\s*(PRIMARY|LW|GCLK_PIN|HCLK\w*)\s*\|\s*(\d+)/(\d+)', R, re.M):
            u, t = int(m.group(2)), int(m.group(3))
            (flag if u >= t else print)(('' if u >= t else '  ') + f'{m.group(1)} {u}/{t}' + ('  full' if u >= t else ''))
        # The section that lists every net that was put on a global line.
        sec = re.search(r'Global Clock Signals(.*?)====', R, re.S)
        if sec:
            # Each line of it names a net and the line type.
            for m in re.finditer(r'^\s*(\S+)\s*\|\s*(PRIMARY|LW)\s*\|', sec.group(1), re.M):
                n = m.group(1)
                # A net whose name does not look like a clock (clk, clock, pll) is a data, reset or enable net using a line: information only.
                if not re.search(r'clk|clock|pll|CLKOUT', n, re.I):
                    print(f'  data/reset net on a clock line: {n} ({m.group(2)})')
    # PR1014 is Gowin's warning that a clock was left on general routing; the fix is a global line or a CLOCK_LOC constraint.
    if L:
        for w in sorted(set(re.findall(r"PR1014.*?clock signal '([^']+)'", L))):
            flag(f"clock '{w}' on general routing (PR1014): skew and delay; give it a clock line or CLOCK_LOC")

    # Check 5, Fmax per clock. Fmax is the highest clock frequency a clock domain can run at, the inverse of its worst path delay.
    # Gowin's "Max Frequency Summary" lists target and result per clock; below it, the failing endpoint counts and the total
    # negative slack (TNS) are listed too.
    print('== 5. Fmax per clock')
    # Per clock: (total negative slack, failing endpoints).
    fails = {}
    if T:
        # The summary table sits in this part of the report (the variable is not used further).
        fm = re.search(r'Max Frequency Summary(.*?)\n\s*\n\s*\d', T, re.S)
        # One row per clock: number, name, target MHz, reached MHz, logic levels.
        for m in re.finditer(r'^\s*\d+\s+(\S+)\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)\s+(\d+)', T, re.M):
            c, tgt, got, lv = m.group(1), float(m.group(2)), float(m.group(3)), m.group(4)
            # A clock with a target under 1 MHz is a placeholder, skip it.
            if tgt < 1:
                continue
            # The clock fails when the result is below the target.
            bad = got < tgt
            (flag if bad else print)(('' if bad else '  ') + f'{c:12s} {got:8.3f} MHz, target {tgt:.3f}' + ('  FAILS' if bad else ''))
        # The failing endpoint table: clock, "setup", worst negative slack (WNS) and number of failing endpoints.
        for m in re.finditer(r'^\s*(\S+)\s+setup\s+(-[\d.]+)\s+(\d+)', T, re.M):
            fails[m.group(1)] = (float(m.group(2)), int(m.group(3)))
            print(f'  {m.group(1)}: {m.group(3)} failing endpoints, TNS {m.group(2)}')
    else:
        print('  no .tr')

    # Check 6, worst paths of the failing clocks. For each of up to --top paths per clock, this takes apart the delay into time spent in cells
    # and time spent in nets (wires), the longest net with its fanout (how many loads it drives) and the two units it joins,
    # the number of logic levels, and the number of times the path crosses from one unit to another. Those numbers say
    # what to do: a long net with a large fanout needs a register copy per unit, a path where nets are most of the delay
    # needs fewer unit crossings or registers at the boundaries, and a path with more than 8 levels needs logic cut.
    # The same loop also collects the worst real slack per clock for check 7.
    print('== 6. Worst paths per failing clock')
    # Worst real slack per clock, filled by the loop below.
    real_worst = {}
    if T:
        # seen counts paths shown per clock and remembers which start/end pairs were already printed.
        seen = collections.Counter()
        # The report is one block per path, each starting with "PathN" on a line of its own; the first piece is the heading.
        for blk in re.split(r'\n\s*Path\d+\s*\n', T)[1:]:
            # Pull out the summary fields of one path: slack, start node, end node, capture clock and launch clock.
            sm = re.search(r'Slack\s*:\s*(-?[\d.]+)', blk); fr = re.search(r'From\s*:\s*(\S+)', blk)
            to = re.search(r'To\s*:\s*(\S+)', blk); lc = re.search(r'Latch Clk\s*:\s*(\w+)', blk)
            ck = re.search(r'Launch Clk\s*:\s*(\w+)', blk)
            # A block without all of them is not a normal path (skip it).
            if not (sm and fr and to and lc):
                continue
            s, c = float(sm.group(1)), lc.group(1)
            # Only paths whose launch and capture clock are the same are looked at here; clock-crossing paths need other treatment.
            if ck and ck.group(1) != c:
                continue                  # same-clock paths only here
            # Keep the smallest slack per clock for check 7.
            real_worst[c] = min(real_worst.get(c, 99), s)
            # Show only failing paths, only the worst few per clock, and each start/end pair once.
            if s >= 0 or seen[c] >= a.top or (fr.group(1), to.group(1)) in seen:
                continue
            seen[(fr.group(1), to.group(1))] = 1
            # The arrival part of the path (before the required-time part): delays from launch to arrival at the capture register.
            arr = blk.split('Data Required Path')[0]
            # Rows of the path table: delay of this step, step type (tNET net, tINS cell, tC2Q clock-to-Q of the launching register), fanout, node name.
            rows = re.findall(r'^\s*([\d.]+)\s+([\d.]+)\s+(tNET|tINS|tC2Q|tCL|tSu\w*)\s+\S+\s+(\d+)\s+\S+\s+(\S+)', arr, re.M)
            if not rows:
                continue
            seen[c] += 1
            # Time in nets: all tNET rows except those on the clock pin itself.
            net = sum(float(r[1]) for r in rows if r[2] == 'tNET' and not r[4].endswith('/CLK') and r[3] != '')
            # Time in cells: logic cells and the launching register's clock-to-output.
            cel = sum(float(r[1]) for r in rows if r[2] in ('tINS', 'tC2Q'))
            # Number of logic cells passed.
            lv = sum(1 for r in rows if r[2] == 'tINS')
            # Fanout of every cell output on the path and its node name.
            fo = [(int(r[3]), r[4]) for r in rows if r[2] in ('tINS', 'tC2Q')]
            # The highest fanout on the path.
            mfo = max(fo) if fo else (0, '')
            # List of (net delay, driver fanout, driver unit, load unit) for every net of the path.
            nets = []
            prev = None
            for r in rows:
                if r[2] in ('tINS', 'tC2Q'):
                    prev = r
                elif r[2] == 'tNET' and prev is not None:
                    nets.append((float(r[1]), int(prev[3]), unit(prev[4], a.depth), unit(r[4], a.depth)))
            # The slowest single net.
            ln = max(nets) if nets else (0, 0, '', '')
            # The unit of every cell on the path, in order.
            units = [unit(r[4], a.depth) for r in rows if r[2] in ('tINS', 'tC2Q')]
            # A crossing is a step where the unit differs from the previous cell's unit.
            cross = sum(1 for i in range(1, len(units)) if units[i] != units[i - 1])
            # The share of the delay that is wiring, in percent. Above 60 it is the wires that make the path slow.
            share = 100 * net / (net + cel) if net + cel else 0
            # The summary of the path.
            print(f'  {c} slack {s:.3f}: {fr.group(1)} -> {to.group(1)}')
            print(f'     levels {lv}, net {net:.2f} ns ({share:.0f} %), cells {cel:.2f} ns, unit crossings {cross}, '
                  f'longest net {ln[0]:.2f} ns fanout {ln[1]} ({ln[2]} -> {ln[3]}), max fanout {mfo[0]}')
            # Flags: wires over 60 percent of the delay, fanout over 64 (Gowin never splits a high-fanout clock enable, so a
            # register copy per unit is needed), and more than 8 levels (SUG113 4.3.3).
            if share > 60: flag(f'{c}: net share {share:.0f} %: placement spread or unit crossings ({cross})')
            if mfo[0] > 64: flag(f'{c}: fanout {mfo[0]} at {mfo[1]}: register copy per unit')
            if lv > 8: flag(f'{c}: {lv} logic levels (> 8)')
    # Check 7, estimate against real. The estimate tools/timing_est.py made before place-and-route is compared with the worst
    # real slack per clock that check 6 collected. The difference is how far off the estimate was for this build; a large
    # one means the model needs a new fit on this build's reports (timing_est.py --fit) before it ranks paths well.
    print('== 7. Estimate against real (worst slack per clock, ns)')
    if E and real_worst:
        # Lines of the form "CLOCK: name period P endpoints N above: n worst est E req R slack S" in timing.txt.
        for m in re.finditer(r'^CLOCK: (\S+) period ([\d.]+) endpoints (\d+) above: (\d+) worst est ([\d.]+) req ([\d.]+) slack (-?[\d.]+)', E, re.M):
            c = m.group(1)
            # Only clocks that appear in both.
            if c in real_worst:
                # es = estimated worst slack, rs = real worst slack.
                es, rs = float(m.group(7)), real_worst[c]
                # A positive "estimate - real" means the estimate was optimistic (it thought the path was faster than it was).
                print(f'  {c:12s} estimate {es:8.2f}  real {rs:8.2f}  estimate - real {es - rs:+.2f}' +
                      ('   (estimate optimistic)' if es > rs else ''))
    else:
        # One of the two inputs is missing.
        print('  missing timing.txt or timing report')

# Run main() only when the file is started as a script, not when it is imported.
if __name__ == '__main__':
    main()
