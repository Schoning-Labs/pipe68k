# pipe68k

pipe68k is a 68020-compatible CPU core with a 68882-compatible floating-point unit, written in Verilog for one FPGA, the
Gowin GW5AST-138C on the Sipeed Tang Mega 138K Pro board. It is pipelined (six stages) and dual issue, which means that
two instructions can start in every clock when the second one is simple and does not need a result of the first. It
was written to be the CPU of an Amiga 1200 rebuilt on that board, and the Amiga system around the core is a separate
project, pipemig (github.com/Schoning-Labs/pipemig).

## Status

The core runs AROS, the open Kickstart replacement, in the pipemig system at a CPU clock of 66.7 MHz, with the FPU, both
caches and every clock of the design meeting timing in Gowin's place-and-route. The measured average is about 1.03 clocks
per instruction on the performance test of the lockstep bench, which is a little over one clock for each
instruction, because pairs are found only in part of the code and cache misses and multi-step instructions cost clocks.

The core is checked against three independent references. The lockstep bench runs the RTL and the Musashi emulator side by
side on random programs and compares every instruction (2,000 programs of 400 instructions is the usual final check, about
2.1 million instructions, and 5.2 million with the FPU groups). The SingleStepTests 680x0 data (about one million tests) and
WinUAE's cputester data (5.1 million tests for the 68020 integer instructions, more for the 68882) are replayed on the core
as well. The remaining known differences are listed with their reasons in `tools/sst_skip.txt` and `tools/cputest_skip.txt`.

Keep the limits in mind before you pick this up for something of your own:

- It is built and measured on one chip with one tool version, Gowin EDA V1.9.12.04. Other versions were not tried, and the
  timing results in the documentation belong to that version.
- The scripts were written on Windows 11 with Git Bash, and the simulation benches use the Microsoft C++ compiler. They are
  not tested on Linux or macOS.
- It is a 68020, so there is no memory management unit. The 68882 transcendental instructions (FSIN, FLOGN and the like) are
  not in hardware; they take the line F exception, as on the 68040 and 68060, and need a software package.
- The core alone is only a timing harness (`rtl/core/p68_top.v`). To see it do something useful you need the system around
  it, which is pipemig.

## The board and what is in the tree

| Folder | Content |
|---|---|
| `rtl/core` | the CPU core and the FPU (Verilog files and headers) |
| `rtl/sample` | the first small pipeline sketch, a good place to start reading |
| `hw` | pin constraints of the core-alone timing build, and a board test of the FPGA's LUT RAM |
| `sim/lockstep` | the lockstep bench, the SingleStepTests and cputester runners, and the 68882 reference model |
| `tools` | build script, timing estimate, after-build checks, test scripts |
| `docs` | the memory port description and `docs/learn`, which explains the whole design |

The board is the Sipeed Tang Mega 138K Pro (FPGA GW5AST-LV138FPG676AC1/I0).

## Quick start

You need Gowin EDA V1.9.12.04 for builds, and Verilator, Perl, Python 3, Git and the Microsoft Visual Studio 2019 Build
Tools for the simulation benches. `tools/` finds them in this order: the file `settings.ini` in the repository root, then
the PATH. Copy `settings.example.ini` to `settings.ini` and edit the lines you need (the folder of the OSS CAD Suite,
the Perl, `gw_sh` and `vcvars64.bat`). The file is ignored by git and never holds keys.

Run the lockstep bench, with 300 random programs of 200 instructions and seed 1:

    tools/lockstep.sh 300 1 200

The first run downloads Musashi into `third_party/musashi` (`tools/get_musashi.sh`). The test data of the two external
packs is downloaded too, and not shipped: `tools/fetch_tests.sh` for the SingleStepTests data, then `tools/sst.sh build`
and `tools/sst.sh run`, and `tools/cputest_build.sh` (see the header of the script for its commands) followed by
`tools/cputest.sh build` and `tools/cputest.sh run`.

Build the core alone for 80 MHz (a period of 12.5 ns) with the FPU:

    FPU=1 PLACE=3 ROUTE=2 tools/build.sh 12.5

The script runs synthesis, then the timing estimate (`tools/timing_est.py`), then place-and-route, and leaves everything
in `build/`. `tools/postbuild.py BUILD_DIR` goes through the results afterwards. Keep in mind that a core-alone build only
shows what the core can do on its own, because in the full system the other units compete for the same wiring, and a
change that looks good alone can lose several MHz there.

## Where to learn

`docs/learn/OUTLINE.md` is the reading order for someone who has never designed a CPU: sixteen chapters, from what an
FPGA is to timing closure, each naming the source files to read. `docs/learn/HISTORY.md` records every variant that was
tried for each part and what was measured, which is the quickest way to find out why a piece of the RTL looks the way it
does. `docs/learn/TOOLS.md` lists every tool used, and `docs/MEMORY_PORT.md` describes the port you attach the core to.

## License

Apache License 2.0, see `LICENSE`. The parts that come from other projects, and the test data that is downloaded and not
shipped, are listed in `NOTICE`.
