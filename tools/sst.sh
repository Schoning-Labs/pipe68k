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
# Builds and runs the SingleStepTests runner (sim/lockstep/sst.cpp): about one million one-instruction tests for the 68000, in
# JSON files from Tom Harte's SingleStepTests project, each run on the RTL core in Verilator.
#
# usage: tools/sst.sh build                      builds build/sst/sst.exe (Verilator and the Microsoft C++ compiler, no Musashi)
#        tools/sst.sh run [max_tests] [NAME...]  runs the data (fetched by tools/fetch_tests.sh); max_tests 0 means all,
#                                                NAME is one data file such as ADD.b (default: every file); summary on stdout
#
# Unlike the lockstep bench there is no reference CPU here: the expected final state is in the data. Each data file is
# compressed, so the run step unpacks one file at a time and pipes it into the runner, which reads standard input.
set -e
# root = repository folder, out = build folder of this runner, data = folder of the .json.gz files (SST_DATA overrides it).
root="$(cd "$(dirname "$0")/.." && pwd)"; out="$root/build/sst"; data="${SST_DATA:-$root/build/sst-data/68000}"
# env.sh gives the tool locations (VERILATOR, VINC, VCVARS64; see settings.example.ini).
. "$root/tools/env.sh"
# build step: make sst.exe
if [ "$1" = build ]; then
  mkdir -p "$out"
  # The core's Verilog files, without the top-level wrapper.
  RTL=$(ls "$root"/rtl/core/*.v | grep -v p68_top.v)
  # Verilator translates the core to C++. No -DP68_FPU is given, so the default of the Verilog source applies (the 68000 tests contain no FPU instructions).
  $VERILATOR --cc $RTL -I"$root/rtl/core" --top-module p68_core -Mdir "$out/obj" \
    --x-assign 0 --x-initial 0 -O3 -Wno-fatal -Wno-WIDTH -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME 2>&1 | grep -E "%Error" || true
  # Verilator runtime folder as a Windows path, and W() to convert Git Bash paths for the Windows compiler.
  VI=$(cygpath -w "$VINC"); W() { cygpath -w "$1"; }
  # C++ sources: Verilator's output, its runtime files and the runner.
  SRCS="$(W "$out/obj")\*.cpp $VI\verilated.cpp $VI\verilated_threads.cpp $(W "$root/sim/lockstep/sst.cpp")"
  # Compile and link with the Microsoft compiler; only lines with error in them are shown.
  "$root/tools/msvc.sh" "$out" "cl /nologo /O2 /EHsc /std:c++17 /w /I$(W "$out/obj") /I$VI /I$VI\vltstd /c $SRCS && link /nologo /OUT:sst.exe *.obj" 2>&1 | grep -E "error|Error" | head -20 || true
  exit 0
fi
# Anything but "run" prints the usage line.
[ "$1" = run ] || { echo "usage: $0 build | run [max_tests] [NAME...]"; exit 2; }
# max = first argument after run (0 = all tests); then drop "run" and max so that "$@" keeps only the file names.
max="${2:-0}"; shift; shift || true
# Without file names, take every .json.gz file in the data folder and remove the extension.
names="$*"; [ -n "$names" ] || names=$(ls "$data" | sed 's/\.json\.gz$//')
# For each file: unpack to standard output, feed the runner with the skip list, the file name, the test limit and the number of
# failures to print (SST_SHOW, default 3).
for n in $names; do gzip -dc "$data/$n.json.gz" | "$out/sst.exe" "$root/tools/sst_skip.txt" "$n" "$max" "${SST_SHOW:-3}"; done
