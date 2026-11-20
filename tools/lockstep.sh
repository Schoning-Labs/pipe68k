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
# Builds and runs the lockstep bench: the RTL core, simulated cycle by cycle in Verilator, runs side by side with the
# Musashi 68020 emulator, and the two are compared after every instruction (the bench itself is sim/lockstep/lockstep.cpp).
#
# usage: [FPU=0] [FPN=n] tools/lockstep.sh [programs] [seed] [length] [trace] [groups]
#   programs, seed, length, trace, groups are the bench's own arguments (see the top of lockstep.cpp). FPU=0 builds the core
#   and the reference without the FPU (only for stand-alone tests of the integer core; every real build has the FPU).
#   FPN is the number of clocks per step of the core's FPU engine, which runs at half rate (2) on a clock enable.
#
# The script does five things in this order. First it makes a patched copy of Musashi's opcode source (once, and again
# whenever tools/musashi_patch.py is newer). Second it makes a copy of Musashi's main source file that includes our FPU
# reference. Third it turns the core's Verilog into C++ with Verilator. Fourth it compiles everything with the Microsoft C++
# compiler. Fifth it runs the program. The tool locations come from tools/env.sh (Verilator from the OSS CAD Suite, Perl from MSYS2,
# the compiler through tools/msvc.sh, see settings.example.ini).
# Stop at the first failing command (set -e), so a broken build never leaves a run that looks like a pass.
set -e
# root = the repository folder (one level above tools); out = the build folder for this bench, created if missing.
root="$(cd "$(dirname "$0")/.." && pwd)"; out="$root/build/lockstep"; mkdir -p "$out"
# env.sh sets the tool locations (see settings.example.ini). MU = the Musashi submodule (never edited), GEN = the generated, patched Musashi copy.
. "$root/tools/env.sh"; "$root/tools/get_musashi.sh" >/dev/null; MU="$root/third_party/musashi"; GEN="$root/build/musashi_gen"
# Rebuild the patched Musashi when it does not exist yet or when the patch script changed after the last copy.
if [ ! -f "$GEN/m68k_in.c" ] || [ "$root/tools/musashi_patch.py" -nt "$GEN/m68k_in.c" ]; then
  # Remove the old generated opcode table so that it is certainly made again from the new patched source.
  mkdir -p "$GEN"; rm -f "$GEN/m68kops.c"
  # Musashi's opcode handlers live in m68k_in.c; the patcher changes the ones where Musashi differs from a real 68020.
  python "$root/tools/musashi_patch.py" "$MU/m68k_in.c" "$GEN/m68k_in.c"
  # m68kmake is Musashi's own generator: it reads the patched m68k_in.c and writes m68kops.c (the table of instruction handlers).
  "$root/tools/msvc.sh" "$GEN" "cl /nologo /O2 /Fe:m68kmake.exe $(cygpath -w "$MU/m68kmake.c") && .\m68kmake.exe . $(cygpath -w "$GEN/m68k_in.c")" >/dev/null
fi
# Musashi's m68kcpu.c includes its own FPU file, m68kfpu.c, which has no precision control and no exception bits; the sed
# command writes a copy that includes our reference FPU (sim/lockstep/fpu_ref.c) in its place. The original stays unchanged.
sed 's/#include "m68kfpu.c"/#include "fpu_ref.c"/' "$MU/m68kcpu.c" > "$GEN/m68kcpu.c"
# The core's Verilog files: everything in rtl/core except the top-level wrapper p68_top.v (the bench drives p68_core directly).
RTL=$(ls "$root"/rtl/core/*.v | grep -v p68_top.v)
# Fresh Verilator and object output on every run. A stale object file once stayed in the link of a smaller build, and a failed
# link once left the old executable behind, which then ran and printed PASS. Deleting everything first rules both out.
rm -rf "$out/obj" "$out"/*.obj "$out/lockstep.exe"
# Verilator translates the core to C++ in obj/. -DP68_FPU and -DP68_FPN select the FPU and its step rate in the Verilog source;
# --top-module p68_core makes the core itself the simulated top.
# --x-assign 0 and --x-initial 0 turn every unknown (X) value into 0, so that a run is repeatable (real flip-flops never hold X);
# -O3 optimizes the generated C++ for speed; the -Wno options silence lint warnings that the sources are known to trigger,
# and only lines containing %Error reach the screen.
$VERILATOR --cc $RTL -DP68_FPU=${FPU:-1} -DP68_FPN=${FPN:-2} -I"$root/rtl/core" --top-module p68_core -Mdir "$out/obj" \
  --x-assign 0 --x-initial 0 -O3 -Wno-fatal -Wno-WIDTH -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME 2>&1 | grep -E "%Error" || true
# Verilator's runtime library folder, converted to a Windows path because the compiler is a Windows program.
VI=$(cygpath -w "$VINC")
# W() converts a Git Bash path into the Windows form that cl.exe understands.
W() { cygpath -w "$1"; }
# C++ sources: everything Verilator generated, Verilator's own runtime files, and the bench.
SRCS="$(W "$out/obj")\*.cpp $VI\verilated.cpp $VI\verilated_threads.cpp $(W "$root/sim/lockstep/lockstep.cpp")"
# C sources: the patched Musashi (CPU, opcode handlers, disassembler, SoftFloat floating point library) and the glue file.
CS="$(W "$GEN/m68kcpu.c") $(W "$GEN/m68kops.c") $(W "$MU/m68kdasm.c") $(W "$MU/softfloat/softfloat.c") $(W "$root/sim/lockstep/musashi_glue.c")"
# Compile the C++ (std C++17, optimized, no warnings), compile the C files with /TC (treat as C) and the Musashi options
# M68K_SEPARATE_READS=1 (instruction words and PC-relative reads go through their own callbacks, which lets the bench tell
# them from data reads) and M68K_EMULATE_PMMU=0 (no memory management unit; the two features together do not compile),
# add P68_NOFPU for FPU=0, then link all objects into lockstep.exe. The grep drops the file-name lines the compiler prints.
"$root/tools/msvc.sh" "$out" "cl /nologo /O2 /EHsc /std:c++17 /w /I$(W "$out/obj") /I$VI /I$VI\vltstd /I$(W "$MU") /I$(W "$GEN") /DM68K_SEPARATE_READS=1 /DM68K_EMULATE_PMMU=0 /c $SRCS && cl /nologo /O2 /w /TC /I$(W "$GEN") /I$(W "$MU") /I$(W "$root/sim/lockstep") /DM68K_SEPARATE_READS=1 /DM68K_EMULATE_PMMU=0 $( [ "${FPU:-1}" = 0 ] && echo /DP68_NOFPU ) /c $CS && link /nologo /OUT:lockstep.exe *.obj" 2>&1 | grep -vE "^(verilated|Vp68|lockstep|m68k|softfloat|m68kops|musashi_glue).*\.(c|cpp)$|vswhere|Generating Code|^$" | head -40
# If the link failed there is no executable: stop here with a message and exit code 1.
[ -f "$out/lockstep.exe" ] || { echo "lockstep: build failed, no lockstep.exe" >&2; exit 1; }
# Run the bench with the arguments the caller gave.
"$out/lockstep.exe" "$@"
