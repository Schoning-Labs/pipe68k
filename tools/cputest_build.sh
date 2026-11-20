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
# Builds WinUAE's cputester generator and runs it to produce the test data for the RTL core. WinUAE is a well known Amiga
# emulator whose 68000 to 68060 core has been checked against real chips, so test data made from it settles every case where
# Musashi and the 68000 test data disagree with a real 68020.
#
# Only the generator is built and run. It is a separate program, never part of pipe68k, and its source (GPL-2.0) stays in
# build/winuae, outside this repository. Two programs are built. gencpu is WinUAE's instruction table generator; with
# CPU_TESTER set to 1 it writes tester versions of the CPU cores (cpuemu_9x_test.cpp), which are then compiled together with the
# generator's main program (cputest.cpp) into cputestgen.exe. That program runs random instructions on the emulated CPU and
# records, per test, the inputs and every register, flag and memory change.
#
# usage: tools/cputest_build.sh clone      shallow clone of github.com/tonioni/WinUAE (one pinned commit) into build/winuae
#        tools/cputest_build.sh build      gencpu, then cpuemu_9x_test.cpp, then cputestgen.exe (VS 2019 Build Tools, about 1 minute)
#        tools/cputest_build.sh gen [MODE] writes cputestgen.ini for the 68020 and runs the generator (data in build/cputest-gen/data)
#        tools/cputest_build.sh gen-fpu    68020 and 68882: the WinUAE presets FBASIC, FINT and FCPX (only fscale, fgetexp, fgetman),
#                                          data in build/cputest-gen-fpu/data (FPU_ROUNDS = minimum tests per opcode in FBASIC, default 300)
# env: WINUAE_DIR = the WinUAE checkout to build and use (default build/winuae)
# The compiler environment is set up through a temporary .cmd file made by this script itself (tools/msvc.sh used one shared file
# that parallel builds overwrote).
# Stop at the first failing command.
set -e
# root = repository folder, wu = WinUAE checkout, gd = folder for the generated 68020 data.
root="$(cd "$(dirname "$0")/.." && pwd)"; wu="${WINUAE_DIR:-$root/build/winuae}"; gd="${CPUTEST_GEN:-$root/build/cputest-gen}"
WINUAE_COMMIT=684e52de45fd952af609b9a602f362eb6243ff2b  # the WinUAE revision the data was made with; a fixed revision keeps the data identical from run to run
msvc() {  # msvc DIR COMMAND...: run COMMAND in DIR inside the Visual Studio 2019 x64 environment, through a temporary batch file
  # d = DIR as a Windows path; then drop it so that "$*" is the command.
  local d; d=$(cygpath -w "$1"); shift
  # f = a unique temporary .cmd file.
  local f; f="$(mktemp --suffix=.cmd)"
  # The Visual Studio batch file that sets up the 64-bit compiler (VCVARS64 in settings.ini; empty when cl is already on the PATH).
  . "$root/tools/env.sh"; local VC="$VCVARS64"
  # Write the batch file: silence echo, set up the compiler, change to the folder, run the command (carriage returns added by sed).
  { echo '@echo off'; [ -z "$VC" ] || echo "call \"$VC\" >nul"; echo "cd /d \"$d\""; echo "$*"; } | sed 's/$/\r/' > "$f"
  # Run it, remember the exit code, delete the file, return the code.
  cmd //c "$(cygpath -w "$f")"; local rc=$?; rm -f "$f"; return $rc
}
# The four commands.
case "$1" in
# clone: get the WinUAE source.
clone)
  mkdir -p "$root/build"; cd "$root/build"
  # A shallow clone (latest commit only) keeps the download small.
  [ -d winuae ] || git clone --depth 1 https://github.com/tonioni/WinUAE.git winuae
  # Fetch the pinned commit by its hash and switch to it; if that fails the shallow master stays.
  cd winuae; git fetch --depth 1 origin $WINUAE_COMMIT 2>/dev/null && git checkout -q $WINUAE_COMMIT || true
  # Print the commit that is checked out.
  git log -1 --format=%H ;;
# build: compile the generator.
build)
  # gen = output of gencpu, gt = output of cputestgen; zstub = a stub of zlib.h made below.
  cd "$wu"; mkdir -p gen gt "$root/build/zstub"
  # The generator includes zlib.h for writing compressed data, but the script writes the data uncompressed (feature_gzip=0 in the
  # ini file), so the compression calls are never made. Four empty stub functions in a fake zlib.h replace the real library.
  printf 'typedef void *gzFile;\nstatic gzFile gzdopen(int fd,const char*m){return 0;}\nstatic int gzwrite(gzFile f,const void*b,unsigned n){return 0;}\nstatic int gzclose(gzFile f){return 0;}\n' > "$root/build/zstub/zlib.h"
  sed -i 's/^#define CPU_TESTER 0/#define CPU_TESTER 1/' gencpu.cpp  # gencpu has a switch at the top of its source: 0 writes the emulator cores, 1 writes the tester cores. The sed sets it to 1.
  # The stub header folder as a Windows path.
  Z=$(cygpath -w "$root/build/zstub")
  # Compiler options: no logo, C++ exceptions, C++14, no warnings, static runtime, Unicode, include folders of the checkout and the stub.
  D="/nologo /EHsc /std:c++14 /w /MT /DUNICODE /D_UNICODE /DWIN32 /Iinclude /I. /Iod-win32 /I$Z"
  # Compile gencpu with optimization for size (/O1 compiles faster; the speed of the generator does not matter) and keep errors only.
  msvc "$wu" "cl $D /O1 /Fo:gen\ /Fe:gen\gencpu.exe gencpu.cpp cpudefs.cpp missing.cpp readcpu.cpp od-win32\unicode.cpp user32.lib" | grep -E "error|LNK" || true
  ./gen/gencpu.exe  # Run gencpu: it reads the instruction table and writes the tester cores, the table and a header.
  # Defines for the second compile: the six core files and the tester switch.
  C="/DCPUEMU_90 /DCPUEMU_91 /DCPUEMU_92 /DCPUEMU_93 /DCPUEMU_94 /DCPUEMU_95 /DCPU_TESTER /D_MBCS"
  # Compile the six tester cores, the generator main program, WinUAE's FPU code (fpp.cpp and the SoftFloat files), the disassembler and
  # the support files into cputestgen.exe, using several compiler processes at once (/MP).
  msvc "$wu" "cl /MP $D $C /O1 /Fo:gt\ /Fe:gt\cputestgen.exe cpudefs.cpp cpuemu_90_test.cpp cpuemu_91_test.cpp cpuemu_92_test.cpp cpuemu_93_test.cpp cpuemu_94_test.cpp cpuemu_95_test.cpp cpustbl_test.cpp cputest.cpp cputest_support.cpp disasm.cpp fpp.cpp fpp_softfloat.cpp ini.cpp newcpu_common.cpp readcpu.cpp softfloat\softfloat.cpp softfloat\softfloat_decimal.cpp softfloat\softfloat_fpsp.cpp od-win32\unicode.cpp user32.lib" | grep -E "error|LNK" || true
  # Show the executable (file size and date) as proof that the build worked.
  ls -la gt/cputestgen.exe ;;
# gen: write the configuration for the 68020 integer data and run the generator.
gen)
  # Settings: 68020 with 32-bit addresses, no FPU and no MMU. Low memory is 0 to 0x7fff (reachable by an absolute word address) and
  # test memory 0x500000 to 0x6fffff; there is no high memory.
  # feature_sr_mask 0x3000 adds rounds with the supervisor bit and the master bit set (so that the stack pointer banks are exercised).
  # Flags mode 1 tests each instruction with the condition codes all zero and all one. Undefined flags are checked by the generator.
  # Work in the data folder.
  mkdir -p "$gd/data"; cd "$gd"
  # mode = which instructions to generate (default all); a single mnemonic can be named for a short run.
  mode="${2:-all}"
  # Edit WinUAE's own ini file: quiet output, uncompressed data, no high ROM, enable the BASIC test, set the SR mask and the mode,
  # and use one round per test instead of two.
  sed -e 's/^verbose=1/verbose=0/' -e 's/^feature_gzip=1/feature_gzip=0/' -e 's/^high_rom=/;high_rom=/' \
      -e '/^\[test=BASIC\]/,/^mode=/{ s/^enabled=0/enabled=1/; s/^feature_sr_mask=.*/feature_sr_mask=0x3000/; s/^mode=.*/mode='"$mode"'/ }' \
      -e 's/^test_rounds=2/test_rounds=1/' "$wu/cputest/cputestgen.ini" > ini.tmp
  # The ini file has a [test=BASIC] section for each CPU type, and the generator reads all of them. Only the second one (the 68020)
  # should stay enabled, so the awk script switches the first one off.
  awk 'BEGIN{b=0} /^\[test=BASIC\]/{b++} b==1 && /^enabled=1/{sub(/enabled=1/,"enabled=0")} {print}' ini.tmp > cputestgen.ini; rm -f ini.tmp
  # Run the generator and show only its total line (the number of tests written).
  "$wu/gt/cputestgen.exe" | grep "total tests generated" ;;
# gen-fpu: the same for the three FPU presets.
gen-fpu)
  # FBASIC covers arithmetic and moves without arithmetic exceptions. FINT covers FMOVECR, FMOVEM (also of the control registers),
  # FDBcc, FBcc, FTRAPcc and FScc.
  # FCPX is reduced to fscale, fgetexp and fgetman: the core traps the transcendental functions, fmod and frem to the line F vector
  # by design.
  # Trace rounds (SR mask 0xc000) are removed because the core has no trace mode and the runner would skip them.
  # gd = output folder for the FPU data (CPUTEST_GEN_FPU overrides it).
  gd="${CPUTEST_GEN_FPU:-$root/build/cputest-gen-fpu}"; mkdir -p "$gd/data"; cd "$gd"
  # One generator run per preset: a single run with all three enabled wrote no data for FINT after FCPX.
  # For each preset: ...
  for t in FBASIC FCPX FINT; do
  # ... take WinUAE's ini file with quiet output, uncompressed data and no high ROM, and let one awk script rewrite the section of the
  # preset (the awk text is inside quotes, so it carries no comments of its own): the section of the current preset is enabled and
  # the other two are disabled, the CPU is set to 68020, no SR rounds are added, and the minimum number of tests per opcode is set
  # from FPU_ROUNDS. For FCPX only the first mode line is replaced by fscale, fgetexp and fgetman and further mode lines are dropped.
  sed -e 's/^verbose=1/verbose=0/' -e 's/^feature_gzip=1/feature_gzip=0/' -e 's/^high_rom=/;high_rom=/' "$wu/cputest/cputestgen.ini" | awk -v rounds="${FPU_ROUNDS:-300}" -v only="[test=$t]" '
    /^\[/ { sec = $0; modes = 0 }
    sec == "[test=FBASIC]" || sec == "[test=FCPX]" || sec == "[test=FINT]" {
      if ($0 ~ /^enabled=/) $0 = (sec == only) ? "enabled=1" : "enabled=0"
      if ($0 ~ /^cpu=/) $0 = "cpu=68020"
      if ($0 ~ /^feature_sr_mask=/) $0 = "feature_sr_mask=0x0000"
      if ($0 ~ /^min_opcode_test_rounds=/) $0 = "min_opcode_test_rounds=" rounds
      if (sec == "[test=FCPX]" && $0 ~ /^mode=/) { if (++modes == 1) $0 = "mode=fscale,fgetexp,fgetman"; else next }
    }
    { print }' > cputestgen.ini
  # Run the generator and print its total line with the preset name in front.
  "$wu/gt/cputestgen.exe" | grep "total tests generated" | sed "s/^/$t: /"
  done ;;
# Anything else prints the usage line.
*) echo "usage: $0 clone|build|gen [mode]|gen-fpu"; exit 2 ;;
esac
