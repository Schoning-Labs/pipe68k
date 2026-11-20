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
# Builds and runs the WinUAE cputester runner (sim/lockstep/cputest.cpp): the test data that WinUAE's CPU tester generates
# (millions of one-instruction tests with exact expected registers, flags, memory writes and exception frames) is replayed on
# the RTL core in Verilator. tools/cputest_build.sh makes the data; this script only builds the runner and runs it.
#
# There are three commands. "build" compiles the runner. "run" replays the 68020 integer data one instruction folder at a
# time (a folder is one mnemonic and size, for example ADD.L). "run-fpu" replays the 68882 data sets FBASIC, FCPX and FINT,
# several folders at once, and adds up the per-shard summaries with tools/cputest_sum.awk.
# usage: [FPU=0] [FPN=n] tools/cputest.sh build        builds build/cputest/cputest.exe (Verilator and the Microsoft compiler, no Musashi)
#        tools/cputest.sh run [max_groups] [NAME...]   runs the 68020 data made by tools/cputest_build.sh gen; max_groups 0 = all,
#                                                      NAME e.g. ADD.L (default: every folder); summary on stdout
#        tools/cputest.sh run-fpu [max_groups] [SET...] the FPU sets FBASIC FCPX FINT made by tools/cputest_build.sh gen-fpu
# environment: CPUTEST_DATA (default build/cputest-gen/data/2_BASIC), CPUTEST_STRIDE (run every Nth group only), CPUTEST_SHOW
#   (failures printed per folder), CPUTEST_JOBS (parallel processes for run-fpu, default 4), CPUTEST_ONLY="A|B" (only those
#   folders of run-fpu: opcode words or values), CPUTEST_FAILLOG=prefix (writes prefix.NAME.SHARD with one line per failing test).
# The compiler environment is set up through a temporary .cmd file made by this script itself, because tools/msvc.sh used one
# shared file that parallel builds from other runs overwrote.
# Stop at the first failing command.
set -e
# root = repository folder, out = build folder of the runner, data = the generated 68020 data (CPUTEST_DATA overrides it).
root="$(cd "$(dirname "$0")/.." && pwd)"; out="$root/build/cputest"; data="${CPUTEST_DATA:-$root/build/cputest-gen/data/2_BASIC}"
# env.sh gives the tool locations (VERILATOR, VINC, VCVARS64; see settings.example.ini).
. "$root/tools/env.sh"
# The "build" command.
if [ "$1" = build ]; then
  mkdir -p "$out"
  # The core's Verilog files, without the top-level wrapper.
  RTL=$(ls "$root"/rtl/core/*.v | grep -v p68_top.v)
  # Verilator translates the core to C++ with the FPU on (FPU=0 turns it off) and the FPU step rate FPN. No Musashi is involved:
  # the expected results are in the data.
  $VERILATOR --cc $RTL -DP68_FPU=${FPU:-1} -DP68_FPN=${FPN:-2} -I"$root/rtl/core" --top-module p68_core -Mdir "$out/obj" \
    --x-assign 0 --x-initial 0 -O3 -Wno-fatal -Wno-WIDTH -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME 2>&1 | grep -E "%Error" || true
  # Verilator runtime folder as a Windows path, and W() converts Git Bash paths for the Windows tools.
  VI=$(cygpath -w "$VINC"); W() { cygpath -w "$1"; }
  # A temporary batch file with the compiler setup, one per call.
  f="$(mktemp --suffix=.cmd)"
  # Batch lines: silence echo, set up the 64-bit compiler (vcvars64.bat), change to the build folder, ...
  { echo '@echo off'; [ -z "$VCVARS64" ] || echo "call \"$VCVARS64\" >nul"; echo "cd /d \"$(W "$out")\"";
    # ... compile Verilator's output, its runtime and cputest.cpp with the optimizing C++17 settings, ...
    echo "cl /nologo /O2 /EHsc /std:c++17 /w /I$(W "$out/obj") /I$VI /I$VI\vltstd /c $(W "$out/obj")\*.cpp $VI\verilated.cpp $VI\verilated_threads.cpp $(W "$root/sim/lockstep/cputest.cpp")";
    # ... and link them into cputest.exe. The sed adds the carriage return that cmd.exe wants at every line end.
    echo "link /nologo /OUT:cputest.exe *.obj"; } | sed 's/$/\r/' > "$f"
  # Run the batch file, show only error lines (first 20), then delete it.
  cmd //c "$(W "$f")" 2>&1 | grep -E "error|Error" | head -20 || true; rm -f "$f"
  exit 0
fi
if [ "$1" = run-fpu ]; then  # run-fpu: the FPU data sets, folders run in parallel
  # fd = generated FPU data folder; max = maximum number of groups (0 = all); then shift the command and the limit away so that
  # the remaining arguments are the set names. Without names the three sets FBASIC, FCPX and FINT run.
  fd="${CPUTEST_GEN_FPU:-$root/build/cputest-gen-fpu}/data"; max="${2:-0}"; shift; shift || true; sets="$*"; [ -n "$sets" ] || sets="FBASIC FCPX FINT"
  # tmp = a temporary folder for the per-shard result files; st = stride (every Nth group, CPUTEST_STRIDE, default 1).
  tmp="$(mktemp -d)"; st="${CPUTEST_STRIDE:-1}"
  for set in $sets; do  # folders over 2 MB run as 4 shards
    # A folder whose data file is larger than 2 MB is split into 4 shards (each shard takes every 4th group, starting at group k),
    # so that one large folder does not keep one process busy while the others are finished.
    # List the folders of the set (everything except the .dat files), filtered by CPUTEST_ONLY when it is set.
    for n in $(ls "$fd/2_$set" | grep -v '\.dat$' | grep -E "^(${CPUTEST_ONLY:-.*})$"); do
      # Print one line per job: set, folder, shard number and shard count.
      if [ "$(stat -c %s "$fd/2_$set/$n/0000.dat")" -gt 2000000 ]; then for k in 0 1 2 3; do echo "$set $n $k 4"; done; else echo "$set $n 0 1"; fi
    done
  # One runner process per line, CPUTEST_JOBS at a time. In the command $0 = set, $1 = folder, $2 = shard, $3 = shard count; the shard
  # offset and stride go to the runner through CPUTEST_OFFSET and the stride argument, and the output goes to a file per job.
  done | xargs -P "${CPUTEST_JOBS:-4}" -L 1 bash -c 'k=$2; sh=$3; CPUTEST_OFFSET=$((k*'"$st"')) "'"$out"'/cputest.exe" "'"$root"'/tools/cputest_skip.txt" "'"$fd"'/2_$0" "$1" "'"$max"'" "${CPUTEST_SHOW:-3}" 0 $(('"$st"'*sh)) > "'"$tmp"'/$0.$1.$k.txt" 2>&1'
  # Add up all job files with the awk script (one summary line per folder, plus the skip and cause lines).
  cat "$tmp"/*.txt | awk -f "$root/tools/cputest_sum.awk"
  # Delete the temporary folder and stop here (the plain "run" below is not for this command).
  rm -rf "$tmp"; exit 0
fi
# Anything except "run" prints the usage line.
[ "$1" = run ] || { echo "usage: $0 build | run [max_groups] [NAME...] | run-fpu [max_groups] [SET...]"; exit 2; }
# max = maximum number of groups (0 = all); drop the command and the limit so that the rest are folder names.
max="${2:-0}"; shift; shift || true
# Without folder names, run every folder of the 68020 data (every entry that is not a .dat file).
names="$*"; [ -n "$names" ] || names=$(ls "$data" | grep -v '\.dat$')
# One runner call per folder: skip list, data, folder, group limit, number of failures to print (CPUTEST_SHOW, default 3), no single
# group selected (0), and the stride (every Nth group, CPUTEST_STRIDE).
for n in $names; do "$out/cputest.exe" "$root/tools/cputest_skip.txt" "$data" "$n" "$max" "${CPUTEST_SHOW:-3}" 0 "${CPUTEST_STRIDE:-1}"; done
