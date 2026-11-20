#!/usr/bin/env bash
# usage: hw/ssramtest/build.sh   Gowin V1.9.12.04. Output in hw/ssramtest/build (delete after reading).
set -u
d="$(cd "$(dirname "$0")" && pwd)"; root="$d/../.."; . "$root/tools/env.sh"; G="$GOWIN_SH"; out="$d/build"; rm -rf "$out"; mkdir -p "$out"
cat > "$out/run.tcl" <<T
create_project -name t -dir {$(cygpath -m "$out")} -pn GW5AST-LV138FPG676AC1/I0 -device_version B -force
set_device -name GW5AST-138C GW5AST-LV138FPG676AC1/I0
$(for f in ssr_top ssr_dom ssr_grp; do echo "add_file -type verilog {$(cygpath -m "$d/$f.v")}"; done)
add_file -type cst {$(cygpath -m "$d/ssr.cst")}
add_file -type sdc {$(cygpath -m "$d/ssr.sdc")}
set_option -top_module ssr_top
set_option -verilog_std sysv2017
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -rw_check_on_ram 0
set_option -correct_hold_violation 1
set_option -place_option 3
set_option -route_option 2
set_option -timing_driven 1
set_option -gen_posp 1
set_option -output_base_name t
set_option -netlist_hierarchy 1
run all
T
cd "$out"; "$G" run.tcl > gw_sh.log 2>&1 &
echo $! > gw_sh.pid; wait; echo done >> gw_sh.log
