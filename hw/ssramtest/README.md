# SSRAM board test (Tang Mega 138K Pro, GW5AST-LV138FPG676A)

Question: does the FPGA's SSRAM (RAM16SDP4, LUT RAM) work on the real board? Our FPU register file uses 40 of them.

## Result: SSRAM works
- 1,512 RAM16SDP4 (63 groups x 24; 62 real groups, 1 canary), 50 MHz board clock, device version B build.
- About 100 s run: epoch 0x12D00 (77,056 x 65,536, about 5.05 billion cycles).
  Every RAM was read and compared on every clock, about 7.6e12 RAM reads in all. 0 errors in all 62 real groups.
  The canary group (checker wrong on purpose) counted 4+ errors, so the compare path does detect faults.
- Data patterns: random, all-0, all-1, walking-1, walking-0, 5/A, writes and reads at random addresses every clock,
  asynchronous read, compared with a flat register model (not SSRAM).
- Build: synthesis and place-and-route both report RAM16SDP4 = 1512, none removed or replaced. Only EX3791 truncation
  warnings. Timing 50 MHz met, Fmax 91.6 MHz. Gowin's timing report header says "Device Version: C" even though the
  project is created with -device_version B (same in tools/build.sh).
- Programmer scan: ID 0x0001081B, "GW5AST-138B GW5AST-138C GW5AS-138B GW5AS-138C (One of them)". It does not tell B from C.
  SRAM load: rc 0, status 0x00026230.

## Limits
- Room temperature only, 50 MHz only. A first design with a 100 MHz PLL domain did not finish routing in 32 min and was
  dropped. Core voltage not changed. Where the placer put the RAMs was not constrained.
- The chip's silicon version (B or C) is still unknown; the test shows SSRAM works in this build on this board.

## Lesson
The 50 MHz board clock is pin P16. M21 is an unused USB pin: the first build used M21
and loaded fine but printed nothing. hw/p68.cst uses M21 because it is for timing-only builds that are never loaded.

## Files
ssr_top.v (UART report, LEDs), ssr_dom.v (epoch and data mode), ssr_grp.v (24 RAMs, model, compare), ssr.cst, ssr.sdc,
build.sh (writes hw/ssramtest/build, delete after reading). UART: P15, 115200 8N1, one line per second:
`S <epochs hex> <63 digits> PASS`, digit = errors per group (0-4), last digit = canary, must be 4.
Load: `programmer_cli --device GW5AST-138C --run 2 --fsFile t.fs` (SRAM only).
