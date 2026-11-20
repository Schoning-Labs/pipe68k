# Tools used to build and test pipe68k

Everything that was installed, downloaded or built to make and check the CPU, with what it did for the project and the
runs made with it. The project was developed on Windows 11 with Git Bash as the shell; the scripts in `tools/` are bash
and read the install locations of the tools from `settings.ini` in the repository root (copy `settings.example.ini` and edit
it); anything not set there is taken from the PATH. The paths named below are the ones used during development.

## FPGA tools

### Gowin EDA (IDE, gw_sh)

- **What it is:** the vendor tool chain for Gowin FPGAs: GowinSynthesis, place and route, timing analysis, and
  `gw_sh.exe`, the Tcl command-line front end that runs all of them without the IDE.
- **Where from, license:** Gowin Semiconductor (gowinsemi.com). Proprietary; the tools need a free license from Gowin.
- **Version:** V1.9.12.04 only, installed in a folder of its own (set `GOWIN_SH` in `settings.ini` to its `gw_sh.exe`). Other versions were not tried, and the
  measurements in HISTORY.md (route option behavior, region-box limits, incremental flow) belong to this version.
- **Role:** `tools/build.sh PERIOD` writes a Tcl script and runs `gw_sh` on it: synthesis of `rtl/core` with
  `p68_top.v` as the top (hierarchical netlist), then `tools/timing_est.py` on the netlist, then place and route
  (default place option 3, route option 2) and the timing report with
  `report_timing -setup -min_logic_level 8 -max_paths 3000`. The full Amiga 1200 system is built by the system
  repository's own script, which calls the same tools on a snapshot of this core.
- **Runs:** core-alone builds after every work package and most timing steps (the first at 57.4 MHz with 8,300 logic,
  71.0-73.4 MHz for the dual-issue integer core, 42-60 MHz with the FPU beside it), and 67 recorded full-system builds
  (44 routed, 23 failed in routing or on a constraint error), with the CPU clock between 50.9 and 69.35 MHz and 66.70 MHz
  with every clock met as the release state. HISTORY.md sections 15 and 16 have the details. The Gowin manuals used most: SUG113 (timing closure order), SUG940 (SDC), SUG1541 (physical constraints),
  SUG100 and SUG1220 (options), SUG550 (synthesis attributes), UG306 (clock resources). They ship with the IDE in
  `IDE/doc/EN`.

### Gowin programmer (programmer_cli)

- **What it is:** Gowin's command-line programmer for loading a bitstream over the board's USB-JTAG.
- **Where from, license:** ships with Gowin EDA; proprietary.
- **Role:** load bitstreams into the FPGA's SRAM only (`--run 2`), never into flash. For the A1200 system this goes
  through the pipemig repository's loader script.
  It crashes unless `PYTHONIOENCODING=utf-8` is set when called from Python.
- **Runs:** the LUT RAM board test (`hw/ssramtest`): 1,512 RAM16SDP4 blocks at 50 MHz for about 100 seconds, about
  7.6e12 reads, 0 errors, and the canary group with a wrong checker on purpose did report errors. All AROS boots of the
  A1200 system with pipe68k were loaded the same way.

## Simulators and synthesis helpers

### Verilator

- **What it is:** a compiler from Verilog to C++; the fastest way to run the whole core cycle by cycle in software.
- **Where from, license:** verilator.org, LGPL-3.0 or Artistic-2.0. Taken from the YosysHQ OSS CAD Suite (build
  20260912 (github.com/YosysHQ/oss-cad-suite-build), unpacked to a folder named by `OSS_CAD_SUITE` in `settings.ini`). Version 5.053 (devel).
- **Install note:** on Windows the suite's own Perl lacks `Pod::Usage`, so the scripts start `verilator` through MSYS2's
  Perl (below). Nothing else is installed.
- **Role:** runs the core in the lockstep bench, the SingleStepTests runner and the cputester runner. Every script
  deletes its old Verilator and compiler output first, because a stale object once linked into a smaller build and a
  stale executable once printed PASS after a failed link.

### Icarus Verilog

- **What it is:** an event-driven Verilog simulator.
- **Where from, license:** github.com/steveicarus/iverilog, GPL-2.0; from the same OSS CAD Suite (version 14.0 devel).
- **Role:** the smoke test of the first pipeline sample (`sim/tb_smoke.v`); `tools/gen_pdrom.py`, which evaluates the
  fill predecode functions over all 65,536 words (with and without the FPU) to write the predecode ROM contents; and the
  exhaustive decoder checks, where a changed decoder was compared with the old one over all 65,536 opcodes, several sets
  of extension words, both privilege states and the FPU states (bit-identical uops required).

### Yosys

- **What it is:** an open synthesis tool; `synth_gowin` maps to Gowin cells.
- **Where from, license:** yosyshq.net, ISC license; from the same OSS CAD Suite.
- **Role:** quick relative size measurements of single units without a Gowin run, for example the FPU unit alone
  (LUT 9,235 to 6,668 after its first timing step) and a what-if on the decoder (with every control field free, DS
  would shrink by only about 700 LUT, which decided against an opcode ROM). Gowin's own synthesis gives the real numbers.

## Compilers and scripting

### Microsoft Visual Studio 2019 Build Tools (C++)

- **What it is:** the MSVC compiler and linker.
- **Where from, license:** visualstudio.microsoft.com, proprietary, free Build Tools license.
- **Role:** compiles the Verilator output together with Musashi and the bench C++ (`/O2 /std:c++17`), and builds
  WinUAE's cputester generator. `VCVARS64` in `settings.ini` names its `vcvars64.bat`. `tools/msvc.sh` runs a command inside its environment and gives every call its own temp
  file (parallel runs overwrote each other's before).

### MSYS2 Perl

- **What it is:** Perl 5.42.2 from MSYS2 (msys2.org; Perl's Artistic or GPL license), installed in its own folder (`PERL` in `settings.ini`).
- **Role:** only to start Verilator's Perl front end.

### Python

- **What it is:** Python 3.10.11 (python.org, PSF license) with numpy (numpy.org, BSD license).
- **Role:** all project tools in `tools/*.py`: the timing estimate and its fit (numpy only for the least-squares fit),
  the level count, the after-build checklist, the size per unit, the predecode ROM generator, and the Musashi patcher.
  No other libraries.

### Git

- **Role:** version control, and the download of Musashi and the test data.

## Reference CPUs

### Musashi

- **What it is:** a 68000-68040 emulator in C, used as the reference CPU of the lockstep bench.
- **Where from, license:** github.com/kstenerud/Musashi by Karl Stenerud, MIT license. Version 4.10. Not part of this repository:
  `tools/get_musashi.sh` (run by `tools/lockstep.sh` when needed) downloads the exact revision into `third_party/musashi`,
  where it is never edited.
- **Role:** the bench builds a patched copy in `build/musashi_gen`: `tools/musashi_patch.py` changes the handlers where
  Musashi differs from a real 68020 (BFTST on memory, BFFFO register offsets, MULU.L/MULS.L with Dh = Dl, MOVEM and
  MOVES with the base in the list, ABCD/SBCD/NBCD, PACK/UNPK memory forms, the CACR read), and `m68kfpu.c` is replaced by
  the project's own FPU reference. Built with separate immediate/PC-relative reads and without its PMMU.

### SoftFloat (inside the Musashi tree)

- **What it is:** John R. Hauser's SoftFloat release 2 (MAME variant), IEEE floating point in software.
- **License:** its own permissive license (`third_party/musashi/softfloat/README.txt`).
- **Role:** compiled into the bench because Musashi's sources reference it. The FPU results the bench compares come
  from the project's own reference below.

### The 68881/68882 reference (`sim/lockstep/fpu_ref.c`, written for this project)

- **What it is:** a replacement for Musashi's FPU, written from the 68881/68882 manual and WinUAE's 68882 behavior,
  with its own 128-bit helpers: precision 24/53/64 with their own exponent limits, 68882 NaN rules, the exception and
  accrued bits, pending traps, FSAVE/FRESTORE frames, the undefined FMOVECR offsets. Musashi's own FPU had no precision
  control, no exception bits and wrong FMOD/FINT/FGETEXP results, so it could not be the reference.
- **License:** project code (Apache-2.0 with the rest).

## CPU test packs

### The lockstep bench (`sim/lockstep/lockstep.cpp`, written for this project)

- **What it is:** the core in Verilator and Musashi in the same process; after every retired instruction (two per clock
  with dual issue, applied in program order) it compares PC, D0-D7, A0-A7, SR, the stack pointer banks, the control
  registers, the FPU state and every stored byte. A random program generator with 40 instruction groups (integer,
  system, interrupts, calls and loops, multiply/divide, bit fields, DMA writes behind the CPU's back, self-modifying
  code, full-format EAs, CAS, FPU, polling loops) writes the programs; a fixed perf test measures clocks per instruction;
  an AROS mode runs a Kickstart replacement ROM from reset against Musashi.
- **Usage:** `tools/lockstep.sh [programs] [seed] [length] [trace] [groups]`, with `FPU=0` for the core without the FPU.
- **Runs:**
  - First run on the pipeline sample: 5,000 programs x 400 plus 1,000 x 200, 2.15 million instructions, no difference
    after six RTL faults were fixed.
  - Quick check on every change: 300 programs x 200 instructions, seeds 1 and 7000 with the FPU and seed 1 without.
  - Check before every merge: 2000 programs x 400 instructions with all groups, about 2.1 million instructions (5.24
    million in the runs with the FPU groups), no differences on the release state. One such run found the polling-loop
    interrupt fault (program 1876 of 2000).
  - Directed polling-loop run: 200 programs x 300, 15.1 million instructions, 179,224 interrupts, no differences.
  - Perf test: CPI 2.029 for the first single-issue core, 1.007 for the dual-issue core, 1.031 for the release state.

### SingleStepTests 680x0 (68000 set)

- **What it is:** about 1 million one-instruction tests for the 68000 in JSON (start state, two prefetch words, memory,
  one instruction, end state), by Tom Harte's SingleStepTests project.
- **Where from, license:** github.com/SingleStepTests/680x0, folder `68000/v1`; no license stated in the repository, so
  the data is downloaded by `tools/fetch_tests.sh` (124 files, about 203 MB gzip) and never copied into this repository.
- **Role:** `sim/lockstep/sst.cpp` runs each test on the core through a generated prologue and compares registers, SR,
  PC and stores. 68000-only behavior (address error frames, the 6-byte exception frame, LINK A7) is skipped by rules
  listed in `tools/sst_skip.txt`.
- **Runs:** the full set once per merge (about 20 minutes, one process per file in parallel): 745,646 run, 3 fail (two
  ASL.b entries whose data is wrong, one DIVU divide-by-zero flag case where the 68000 and 68020 differ), 7,162 undefined
  (BCD flags), 6,500 where the data disagrees with Musashi, WinUAE and the core alike.

### WinUAE cputester (68020 integer data)

- **What it is:** WinUAE's CPU tester: a generator (`gencpu` with `CPU_TESTER=1`, then `cputestgen`) that runs WinUAE's
  hardware-verified CPU emulation and writes test data, normally replayed by an Amiga program.
- **Where from, license:** github.com/tonioni/WinUAE, GPL-2.0. Only the generator is built and run (shallow clone into
  `build/winuae` by `tools/cputest_build.sh`); none of its code goes into pipe68k.
- **Data set:** 68020, 32-bit addresses, no FPU or MMU, all modes, undefined flags checked, extra rounds with S and M
  set: 178 instruction folders, 5,136,622 tests, 83 MB, generated in 25 seconds.
- **Role:** `sim/lockstep/cputest.cpp` decodes the data the way the Amiga program does and runs every test on the core,
  comparing registers, the stacked SR and PC, frame format and vector, and every memory write. This pack settles every
  case where Musashi and the 68000 data disagree with a real 68020.
- **Runs:** before the fixes: 4,043,119 pass, 30,541 fail (illegal-encoding cases and places where the core had followed
  Musashi). On the release state: 5,134,638 run, 5,128,108 pass, 0 fail, 6,530 undefined (WinUAE's CHK/CHK2 flags are not
  silicon values), 1,984 skipped (trace). A full run takes about 40 minutes with `tools/cputest_par.sh`.

### WinUAE cputester (FPU data)

- **Data set:** the presets FBASIC, FINT and FCPX (FCPX only FSCALE, FGETEXP, FGETMAN), CPU 68020 with a 68882: 98 folders,
  917,360 tests.
- **Runs:** first run: every arithmetic and move folder passed, and six folders failed on forms the core still trapped
  (predicates 32-63, FMOVECR offsets 1-8, unusual FMOVEM.X lists); after those were added: 864,016 run, all pass, 53,344
  skipped by design (packed decimal, immediate .D/.X/.P, dynamic FMOVEM.X, FMOVEM.L #imm with two or three registers).

## ROM images and disks

- **AROS ROM images** (the open Kickstart replacement, aros.org, AROS Public License): used by the lockstep bench's AROS
  mode (`P68_AROS=rom,ext`) and by every boot of the A1200 system. The images are built and kept in the system
  repository, not here.
- Aminet disks and the Amiga test disks (coptest, loadtest, memtest1200 and others) belong to the pipemig repository's
  board tests and are not needed to build or test the CPU alone.

## Not used

- Moira (a second reference CPU) was named in the study as a second opinion but never needed: WinUAE's data settled every
  disagreement.
