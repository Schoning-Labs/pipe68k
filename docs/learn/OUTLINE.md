# Reading order

This outline is for someone who knows a little programming but has never designed a CPU or used an FPGA. The chapters
go from the basics to the deepest units, and each one names the source files to read with it and the sections of
HISTORY.md that tell how that part got its shape. Reading the chapters in order matters more than usual here, because
almost every unit in the core leans on rules that were set by an earlier one (the uop format, the replay instead of a
stall, the rule that anything steering the pipeline comes from a register).

## Part 1: the ground to stand on

**Chapter 1. What an FPGA is, and what this one is like.** LUTs, flip-flops, carry chains, block RAM, DSP blocks and
global clock lines, explained on the GW5AST-138C, and why on this chip a wire between two far-apart units costs more
than five LUTs (which is the reason for most of the strange-looking code later on). Also how a build goes from Verilog to
a bitstream with Gowin's tools, and what Fmax, a timing report and a logic level are.
Files: `hw/p68.cst`, `tools/build.sh`. HISTORY: section 1, section 15 (place and route options, netlists, LUT copies).
TOOLS: Gowin EDA.

**Chapter 2. What a 68k is.** The 68000 family from a programmer's view: data and address registers, the status
register and its flags, the addressing modes, how an instruction is encoded in 16-bit words with extension words behind
it, user and supervisor mode, exceptions and the stack frame. Then what the 68020 added (32-bit everything, scaled index,
the full extension format, bit fields, CAS, the instruction cache) and why the Amiga 1200 cares. No source files yet;
the reference is Motorola's M68000 Family Programmer's Reference Manual.

**Chapter 3. What a pipeline is.** One instruction per clock by working on six instructions at once, with the stages
IF, DS, AG, OC, EX, WB of this core, and the three problems every pipeline has (an instruction needs a result that is
not written yet, a branch changes the PC after later instructions were fetched, a cache miss). How pipe68k answers each:
forwarding, prediction with redirect, and replay. The feasibility sample is the best first code to read, because it is
one file that does all of this for a small instruction subset.
Files: `rtl/sample/p68_core.v`, `rtl/sample/p68_top.v`, `sim/tb_smoke.v`. HISTORY: section 2, section 3.

**Chapter 4. Proving it is right: the lockstep bench.** Before reading the real core, how it is tested: the core and a
reference CPU run side by side and are compared after every instruction, with random programs. This chapter explains the
retire port (the core telling the bench what each instruction did), the random generator and its groups, and why
Musashi had to be patched.
Files: `sim/lockstep/lockstep.cpp` (structure only, not every line), `sim/lockstep/musashi_glue.c`, `tools/lockstep.sh`,
`tools/musashi_patch.py`. HISTORY: section 17. TOOLS: Verilator, Musashi, the lockstep bench.

## Part 2: the real core, front to back

**Chapter 5. The uop and the core top.** Every instruction becomes one or more micro-ops of a fixed format that travel
down the pipeline; this chapter reads that format field by field and then the top file that connects all stages, so the
reader has a map before going into the units. It also covers the register file with its temporaries and the committed
copy used after a redirect.
Files: `rtl/core/p68_uop.vh`, `rtl/core/p68_core.v`, `rtl/core/p68_top.v`, `rtl/core/p68_regfile.v`. HISTORY: sections 3 and 9.

**Chapter 6. Memory: the port, the caches, coherency.** The core's only memory interface, the store buffer and why
reads never overtake writes, the 8 KB I-cache and D-cache, the line buffers, uncached areas for the Amiga chip RAM,
snooping DMA writes, and self-modifying code.
Files: `rtl/core/p68_mem.v`, `rtl/core/p68_icache.v`, `rtl/core/p68_dcache.v`. HISTORY: sections 11 and 14, and the
uncached-areas part of section 7.

**Chapter 7. Fetch.** Eight bytes per clock into a queue that is also the decode window, the predecode stored with each
cache line, the branch target cache and return prediction, and the extractor that moves whole instructions into the
instruction buffer. This is the part with the most variants in HISTORY, so read that section first.
Files: `rtl/core/p68_fetch.v`, `rtl/core/p68_pdf.vh`, `rtl/core/p68_pdrom.v` (and a look at the generated
`p68_pdrom.vh` and `tools/gen_pdrom.py`), `rtl/core/p68_ib.vh`, `rtl/core/p68_pdec.v`. HISTORY: section 4.

**Chapter 8. Decode.** The instruction buffer, the group decoders and their OR merge, the data codes, illegal
instructions, and how a decoder is added. One or two decoders are read in full (`dec_move.v`, `dec_alu.v`), the rest by
pattern.
Files: `rtl/core/p68_decode.v`, `rtl/core/p68_dec.vh`, `rtl/core/p68_dr.vh`, all `rtl/core/dec_*.v` except `dec_fpu.v`.
HISTORY: section 5.

**Chapter 9. Long instructions: sequencers.** How one instruction becomes several uops (MOVEM first), the empty step 0,
the full-format address pre-sequence, multiply and divide sequences, bit fields, CAS, BCD, calls and returns.
Files: `rtl/core/p68_seq.v`, `seq_int.v`, `seq_flow.v`, `seq_cas.v`, `seq_bf.v`, `p68_mdseq.v`, `p68_fx.v`, and the
small headers `p68_flowf.vh`, `p68_p7f.vh`, `p68_bff.vh`, `p68_md.vh`, `p68_mdf.vh`. HISTORY: section 6.

**Chapter 10. Exceptions and supervisor mode.** The system unit, the exception sequencer that writes the stack frame,
interrupts, RTE, and the rule that a raise condition always comes from a register.
Files: `rtl/core/p68_sys.v`, `p68_sys.vh`, `p68_sysf.vh`, `p68_sseq.v`, `p68_xseq.v`, `dec_sys.v` (read again here).
HISTORY: section 7.

**Chapter 11. Addresses and operands.** The DR queue in front of AG, address generation with the 3:2 compressor, An
updates, the interlock, and then OC in two stages: D-cache access, store forwarding, misaligned splits, replays.
Files: `rtl/core/p68_ag.v`, `rtl/core/p68_oc.v`. HISTORY: sections 10 and 11.

**Chapter 12. Execute and commit.** The adder, logic and rotator, flags by flag mode, branch resolve and redirect, the
extra units (multiply/divide, bit fields, Scc/BCD/bit operations), and write-back with the retire port.
Files: `rtl/core/p68_ex.v`, `p68_exi.v`, `p68_md.v`, `p68_bf.v`, `p68_wb.v`. HISTORY: section 12.

**Chapter 13. Dual issue.** Two instructions per clock: lane B, the pairing rules, swapped pairs, the branch fold pair
and the skip pair, and what each one bought in CPI. This chapter reads parts of files already seen (fetch's pair
candidate, decode's second instruction, AG's pairing decision) plus lane B's own ALU.
Files: `rtl/core/p68_exb.v`, plus the pairing parts of `p68_decode.v`, `p68_ag.v`, `p68_ex.v`. HISTORY: section 8.

**Chapter 14. The FPU.** The 68881/68882 model, what is in hardware and what traps, the registered boundary, the engine
on a half-rate clock enable, and how it is checked against its own reference model.
Files: `rtl/core/p68_fpu.v`, `p68_fpe.v`, `p68_fpu.vh`, `dec_fpu.v`, `seq_fpu.v`, `sim/lockstep/fpu_ref.c`.
HISTORY: section 13.

## Part 3: making it fast and keeping it right

**Chapter 15. Timing closure on Gowin.** How a timing report is read, why logic levels alone do not predict the clock,
the timing estimate and how it is fitted, the after-build checklist, register copies that synthesis merges, region boxes
and place/route options, and the story of the CPU clock in the full Amiga system.
Files: `tools/timing_est.py`, `tools/levels.py`, `tools/postbuild.py`, `tools/modsize.py`, the JSON models,
`hw/ssramtest/*` (as the example of a board question answered with a small test design). HISTORY: sections 15 and 16.

**Chapter 16. The external test packs.** SingleStepTests and WinUAE's cputester, how their data is turned into runs on
the core, and what their failures taught.
Files: `sim/lockstep/sst.cpp`, `sim/lockstep/cputest.cpp`, the `tools/sst*`, `tools/cputest*`, `tools/fetch_tests.sh`
scripts and skip lists. HISTORY: section 17. TOOLS: the test pack sections.
