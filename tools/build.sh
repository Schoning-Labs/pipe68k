#!/usr/bin/env bash
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
#
# tools/build.sh: one command that builds the CPU core with Gowin's tools, checks the timing estimate in between, and
# leaves every result in one folder.
#
# usage: tools/build.sh PERIOD_NS [syn]
#
# What a build is. Turning Verilog into a bitstream for the FPGA takes two big steps, both run here by Gowin's command
# line tool gw_sh (Gowin IDE V1.9.12.04, the only version this project uses). Synthesis ("run syn") turns the Verilog
# into a netlist, a list of the chip's own cells (LUTs, flip-flops, block RAMs, DSP blocks, carry chains) and the wires
# between them. Place-and-route ("run pnr") decides which physical cell of the chip each netlist cell becomes (placement)
# and which wires of the chip connect them (routing), and only then does it know how long every wire is. That is why
# Gowin can not report real timing after synthesis: the delay of a path is mostly wire delay, wire delay depends on
# where the placer put things, and gw_sh has no command that gives timing before place-and-route has finished (it has
# only "run syn", "run pnr" and "run all"). Place-and-route takes from about 5 minutes to many hours here.
#
# What this script adds. After synthesis and before place-and-route it runs tools/timing_est.py on the netlist, which
# estimates the path delays in a few minutes, and tools/levels.py, which counts logic levels. The two reports show which
# paths are going to fail the clock before hours are spent on place-and-route (the estimate is a screen and not a
# prediction, see tools/timing_est.py). Place-and-route is started only after this script has written the file pnr.go,
# and the Tcl script that gw_sh runs waits for that file (or for pnr.stop) for up to 1800 loops of 2 seconds.
#
# PERIOD_NS is the clock period of the single clock "clk" in nanoseconds (12.5 is 80 MHz, 15 is 66.7 MHz). The optional
# second argument "syn" stops after synthesis and the estimate. Output goes to build/p<PERIOD_NS><TAG>/ under the
# repository (delete it after reading, it is large; keep the netlist, constraints, reports and bitstream).
#
# Environment variables (all optional):
#   FPU=1      build with the 68882 floating point unit (the default 0 is the integer core alone; the project judges
#              every build with the FPU). The RTL is copied into the build folder with P68_FPU set to 1, so the
#              source tree is not changed.
#   FPN=n      the FPU engine takes a step every n clocks (default 2). Its internal paths then get n clocks to finish
#              (a multicycle path, see the sdc below); MCP=0 turns that off.
#   CEG=0      by default the FPU's step enable is put on a global clock line (SUG1541 2.8, CLOCK_LOC ... BUFG =
#              CE|LOGIC), because it fans out to thousands of registers; CEG=0 leaves it on ordinary wiring.
#   FGRP="R[a:b]C[c:d]"  a region box (a GROUP) for the FPU engine, rows a to b and columns c to d of the chip (SUG1541 2.5).
#              Off by default. Boxes of up to 27 x 51 cells failed routing (PR0004, unroutable nets).
#   PLACE, ROUTE, REPL   the place option (default 3), route option (default 2) and replicate-resources option (default
#              0) of Gowin place-and-route. The same code and options give the same result, so a change is judged at
#              place 2 and place 3, because neither one wins reliably (docs/learn/HISTORY.md, section 15).
#   HIER=0     flat netlist. The default is a hierarchical netlist (HIER=1), which kept modules together and routed
#              where a flat netlist of the first core failed with unroutable nets (PR0004).
#   CST=file   pin constraints (default hw/p68.cst, which only places the clock and two serial pins).
#   SRC=dir    build the old stand-alone sample layout (dir/sample/p68_core.v and p68_top.v) instead of rtl/core.
#   TAG=x      appended to the build folder name, so that several builds of one period can sit side by side.
#   TIMING_MARGIN=m  the estimate flags endpoints above (requirement - m) ns, default 0.
#   TIMING_GATE=1    stop before place-and-route when the estimate flags any endpoint (the default is to report only).
#   LEVELS_FORCE=1   run place-and-route even if TIMING_GATE=1 would stop it.
#   LEVELS_ONLY=1    synthesis and the estimate only, same as the argument "syn".
#   PY=cmd     the Python interpreter for the two tools (settings.ini or environment, default "python").
#
# Exit status: 2 when synthesis failed, 1 when the gate stopped the build, otherwise the status of the last command.
# A synthesis run that prints "SSRAM: n" with n above 0 is worth stopping by hand: SSRAM (distributed RAM made from
# LUTs) is disabled on device version B of this chip.
# Stop on an unset variable (typing mistakes in names would otherwise expand to nothing, silently).
set -u
# Gowin's command line tool, the synthesis and place-and-route driver. The location comes from settings.ini (GOWIN_SH), else gw_sh on the PATH.
# root = the repository (this script's folder, one level up); p = the period from the first argument; out = the build
# folder, which is removed and created again so that no file of an older run is mixed with the new one.
root="$(cd "$(dirname "$0")/.." && pwd)"; . "$root/tools/env.sh"; G="$GOWIN_SH"; p=$1; out="$root/build/p$p${TAG:-}"; rm -rf "$out"; mkdir -p "$out"
# Half the period: the clock waveform below is high for the first half and low for the second (50 percent duty cycle).
half=$(awk "BEGIN{print $p/2}")
# Pick the RTL files: the old sample layout when SRC is set, otherwise every .v file of rtl/core (the .vh include
# files are found through the include path set in run.tcl).
if [ -n "${SRC:-}" ]; then inc="$root/$SRC/sample"; files="$inc/p68_core.v $inc/p68_top.v"   # old layout, only with SRC
else inc="$root/rtl/core"; files=$(ls "$inc"/*.v); fi
# With the FPU: copy the whole core folder into the build folder and switch the FPU on in the copy, so that the
# repository stays untouched and a build can be reproduced from its own folder.
if [ "${FPU:-0}" = 1 ]; then   # default is the integer core alone
  mkdir -p "$out/core"; cp "$inc"/*.v "$inc"/*.vh "$out/core/"
  # The FPU switch is a `define in p68_uop.vh (P68_FPU 0 or 1); sed rewrites that one line in the copy.
  sed -i 's/^`define P68_FPU 0/`define P68_FPU 1/' "$out/core/p68_uop.vh"
  # The same for the FPU engine's step interval P68_FPN (every FPN clocks), then point the build at the copy.
  sed -i "s/^\`define P68_FPN 2/\`define P68_FPN ${FPN:-2}/" "$out/core/p68_fpu.vh"; inc="$out/core"; files=$(ls "$inc"/*.v); fi
# The timing constraints file (SDC, Synopsys Design Constraints, the format both Gowin and the estimate read). First the
# one clock: name clk, the period, rise at 0 ns and fall at half the period, on the input port clk.
{ echo "create_clock -name clk -period $p -waveform {0 $half} [get_ports {clk}]"
  # The FPU engine (instance i_fpe) only changes every FPN clocks, so a path inside it may take FPN clocks instead of
  # one. The multicycle setup line gives it FPN clocks to settle, and the hold line (FPN-1) keeps the hold check at the
  # original edge, which is the usual pair of lines for a clock enable (SUG940 5.4.3).
  n=${FPN:-2}; if [ "${FPU:-0}" = 1 ] && [ "$n" -gt 1 ] && [ "${MCP:-1}" = 1 ]; then
    echo "set_multicycle_path -from [get_regs {*i_fpe/*}] -to [get_regs {*i_fpe/*}] -setup -end $n"
    echo "set_multicycle_path -from [get_regs {*i_fpe/*}] -to [get_regs {*i_fpe/*}] -hold -start $((n - 1))"; fi
  # Ask Gowin's analyzer to print the 200 worst setup paths. Those reports are what tools/timing_est.py is fitted on
  # and what tools/postbuild.py reads.
  echo "report_timing -setup -max_paths 200 -max_common_paths 1"
  # And every path with more than 8 logic levels, up to 3000 of them (SUG940 5.5.1).
  echo "report_timing -setup -min_logic_level 8 -max_paths 3000 -max_common_paths 1"; } > "$out/t.sdc"   # end of the sdc file
# The Tcl script gw_sh runs. The unquoted heredoc lets the shell fill in the $(...) and ${...} parts; \$ is a Tcl
# variable that the shell must not touch. It creates the project for the GW5AST-138C in the FPG676 package, adds
# the Verilog files, the pin constraints and the timing constraints, and sets the options: the spare configuration pins
# are released as ordinary I/O (use_*_as_gpio), read-write checks on block RAM are off, hold violations are repaired by
# the router (correct_hold_violation), placement and routing are timing driven, and the netlist is hierarchical unless
# HIER=0. After "run syn" it prints the SSRAM count from the synthesis report, writes syn.done to tell this script that
# the netlist exists, and then waits for pnr.go before starting "run pnr".
cat > "$out/run.tcl" <<T
create_project -name t -dir {$(cygpath -m "$out")} -pn GW5AST-LV138FPG676AC1/I0 -device_version B -force
set_device -name GW5AST-138C GW5AST-LV138FPG676AC1/I0
$(for f in $files; do echo "add_file -type verilog {$(cygpath -m "$f")}"; done)
add_file -type cst {$(cygpath -m "$out/p.cst")}
add_file -type sdc {$(cygpath -m "$out/t.sdc")}
set_option -top_module p68_top
set_option -verilog_std sysv2017
set_option -include_path {$(cygpath -m "$inc")}
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -rw_check_on_ram 0
set_option -correct_hold_violation 1
set_option -place_option ${PLACE:-3}
set_option -route_option ${ROUTE:-2}
set_option -replicate_resources ${REPL:-0}
set_option -timing_driven 1
set_option -gen_posp 1
set_option -output_base_name t
set_option -netlist_hierarchy ${HIER:-1}
run syn
set r [open {$(cygpath -m "$out")/t/impl/gwsynthesis/t_syn.rpt.html}]; set d [read \$r]; close \$r
set n 0; regexp {<b>SSRAM </b></td>\s*<td>([0-9]+)} \$d -> n; puts "SSRAM: \$n"
# the level check (tools/levels.py) runs between synthesis and place-and-route: wait for build.sh's verdict
set f [open {$(cygpath -m "$out")/syn.done} w]; close \$f
if {"${2:-all}" != "syn" && "${LEVELS_ONLY:-0}" != "1"} {
  for {set i 0} {\$i < 1800 && ![file exists {$(cygpath -m "$out")/pnr.go}] && ![file exists {$(cygpath -m "$out")/pnr.stop}]} {incr i} { after 2000 }
  if {[file exists {$(cygpath -m "$out")/pnr.go}]} { run pnr }
}
T
# Copy the pin constraints into the build folder. Lines for the FPU are appended to this copy below, so the file in hw/ is
# never changed.
cp "$root/${CST:-hw/p68.cst}" "$out/p.cst"                     # a copy, so hw/ stays unchanged
# Extra constraints for the FPU build. The FPU is instance i_fpu inside the execute unit i_ex of the core.
if [ "${FPU:-0}" = 1 ]; then
  i="core/i_ex/g_fpu.i_fpu"
  # Put the FPU's step enable "ce" on a global clock line, as a clock enable (CE|LOGIC), unless CEG=0. Gowin never splits a
  # net with such a fan-out by itself, and on ordinary wiring it was a slow net.
  [ "${CEG:-1}" = 1 ] && [ "${FPN:-2}" -gt 1 ] && echo "CLOCK_LOC \"$i/ce\" BUFG = CE|LOGIC;" >> "$out/p.cst"
  if [ -n "${FGRP:-}" ]; then
    # Optional region box (see FGRP above).
    { echo "GROUP g_fpe = {\"$i/i_fpe/*\"};"; echo "GRP_LOC g_fpe $FGRP;"; } >> "$out/p.cst"; fi
fi
# What happens from here. Start gw_sh in the background and note its process id (gw_sh.pid) so that a hung run can be
# ended by that id and not by name. Then wait until synthesis has written syn.done, or until gw_sh has died.
cd "$out"; "$G" run.tcl > gw_sh.log 2>&1 &
pid=$!; echo $pid > gw_sh.pid
# Poll every 3 seconds: leave the loop when the netlist is ready or when gw_sh has exited.
while [ ! -f syn.done ] && kill -0 $pid 2>/dev/null; do sleep 3; done
# gw_sh died before the netlist existed: synthesis failed. Let the Tcl script finish, mark the log, report, exit 2.
if [ ! -f syn.done ]; then wait; echo done >> gw_sh.log; echo "synthesis failed, see $out/gw_sh.log"; exit 2; fi
# Pass --mcp i_fpe/ to levels.py when the engine's multicycle paths are in force, so that paths inside the engine are
# not counted as long paths.
mcpo=""; [ "${FPU:-0}" = 1 ] && [ "${FPN:-2}" -gt 1 ] && [ "${MCP:-1}" = 1 ] && mcpo="--mcp i_fpe/"   # for levels.py
# The synthesized netlist that both tools read.
vg="$out/t/impl/gwsynthesis/t.vg"
# The timing estimate: delay of the worst path into every register, block RAM and DSP input, per clock, using the SDC
# written above. Result in timing.txt (and warnings in timing.err).
$PY "$root/tools/timing_est.py" "$vg" --sdc "$out/t.sdc" --margin "${TIMING_MARGIN:-0}" > timing.txt 2> timing.err
# Gowin-style logic level count above 8, for information only (levels do not predict the clock well here).
$PY "$root/tools/levels.py" "$vg" $mcpo > levels.txt 2>> timing.err
# The number after "TIMING:" on the last line of the estimate is the count of endpoints estimated above their
# requirement; 0 when the line is missing.
n=$(awk '/^TIMING:/{print $2}' timing.txt); n=${n:-0}
# Show the head of the estimate (everything before its "worst nets" part, at most 40 lines), the per-clock summary lines
# and the last line of the levels file.
sed -n '/^# worst nets/q;p' timing.txt | head -40; grep '^CLOCK:' timing.txt; tail -1 levels.txt
# Optional gate: stop here, without place-and-route, when endpoints are above the requirement.
if [ "$n" -gt 0 ] && [ "${TIMING_GATE:-0}" = 1 ] && [ "${LEVELS_FORCE:-0}" != 1 ]; then touch pnr.stop; wait; echo done >> gw_sh.log
  echo "TIMING: $n endpoints estimated above their requirement - ${TIMING_MARGIN:-0} ns: no place-and-route (LEVELS_FORCE=1 overrides). Full list: $out/timing.txt"; exit 1; fi
# No gate stopped it (or the gate is off): release place-and-route, wait for gw_sh to finish and mark the log as done.
touch pnr.go; wait; echo done >> gw_sh.log
