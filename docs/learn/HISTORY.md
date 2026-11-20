# How pipe68k got its shape: every variant that was tried

This file is the record of the design variants behind each part of pipe68k, what was measured on each one, and why it
stayed in the design or was taken out again. The source code only shows the version that won, so this is the place to
look when a piece of the RTL seems more complicated than it needs to be (usually it is that way because the simple
version was built first and measured, and it was too slow on this chip).

A few words that come up everywhere:

- **MHz** is the clock the Gowin place-and-route reached (Fmax from its timing report). A "core-alone build" is the CPU
  with a small timing top; a "full build" is the whole Amiga 1200 system on the Tang Mega 138K Pro (chipset, DDR3
  memory, HDMI, RTG card, Ethernet and the CPU), where the CPU has a fixed target clock.
- **CPI** is clocks per instruction on the fixed performance test of the lockstep bench (an integer loop, a memory copy
  and a loop with calls and branches, caches warm). Lower is better, and below 1.0 means two instructions per clock
  some of the time.
- **Logic** is LUT plus ALU cells as Gowin counts them. The chip has 138,240.
- **Lockstep** is the bench that runs the RTL and a reference CPU side by side and compares them after every
  instruction (see "The test benches" at the end).

Where a reason was not written down at the time, the text says "reason not recorded".

## 1. How the work was done

The project started with a feasibility study and a small pipeline sample, then a lockstep bench, and only then the full
instruction set. The instruction set was built in work packages that each added one group (integer completion,
exceptions, program flow, multiply and divide, bit fields, addressing, caches), several of them in parallel and merged
one after the other, with the bench green after every merge. After that every round of work was a timing round, and
almost every one of those rounds followed the same loop: build, read Gowin's worst paths, cut those paths, check the
cut with the bench and the CPI test, build again.

Two things the project learned early and kept relearning are worth knowing before reading the rest. First, on this
chip the wires cost much more than the logic: about 85 % of a slow path's delay is routing, a LUT costs 0.26-0.52 ns and
a net between two big units 1.3-3.1 ns, so moving logic closer together or computing a signal a clock earlier into a
register often helps more than removing a LUT level. Second, the same RTL with the same options gives the same result
(71.569 MHz twice), but a small unrelated edit moves the placement and can shift the clock by several MHz, so one build
never proves a change on its own.

## 2. The feasibility sample

Before any real core existed, a one-file pipeline sample was written (`rtl/sample/p68_core.v`): the six stages IF, DS,
AG, OC, EX, WB, real 4 KB caches in block RAM, a 16-word fetch queue, forwarding, An updates in AG with a committed copy
for restore, and a subset of the instructions (MOVEQ, MOVE, the ALU operations, ADDQ/SUBQ, Bcc, LEA, shifts, all 68000
addressing modes plus the 68020 brief index format). It was built four times:

| Version | Logic | Fmax | Worst path |
|---|---|---|---|
| As first written | 6,738 + 387 ALU | 66 MHz (synthesis estimate only) | queue pointer, 16:1 word select, length decode (19 levels) |
| Instruction length predecoded when a word enters the queue | 6,853 + 407 | 54 MHz | D-cache tag compare driving the stall of every earlier stage |
| Replay on a D-cache miss instead of a stall | 6,830 + 412 | 67.5 MHz | An read, add, An write (the (An)+ loop) |
| AG forwarding select one clock early | 6,858 + 411 | 64.0 MHz | placement noise against the run before |

Three lessons from the sample shaped everything after it. The length of an instruction is predecoded per word as it
enters the queue (it was the first slow path). The core never stalls on a cache result; a miss replays the instruction
instead, because a stall that depends on a compare in the same clock fans out to every stage and cost 13 MHz here. And
the router matters: route option 1 without a global clock line did not route at all (70 minutes, 1,129 unrouted nets),
route option 2 with the clock on a global line routed in 4 minutes and was 13 MHz faster than route option 1 on the same
netlist. From the sample the study predicted 20,000-23,000 LUT and 75-85 MHz for the full integer core; the real
integer core ended up at about 33,000 logic with dual issue, and the clock story is the rest of this file.

## 3. The pipeline as a whole

**Six stages first.** The first real core split the sample into modules with a common micro-op (uop) format: IF, DS
(decode and sequencer), AG, OC, EX, WB, with temporaries T0-T3 in the register file for multi-step instructions. With
the integer instruction set complete, exceptions and caches in, it ran at 50.9-55 MHz (core alone, 16 ns constraint,
23,538 logic).

**First timing round (registers in front of everything that steers).** Twenty-one small changes made every value that
feeds a redirect, a stall or a clock enable come from registers through few LUT levels: the OC split decode and the
D-cache compare address as registers, the DS window as a register, the interrupt-pending decision one clock old, AG
reading its base and index registers one clock early, branches without a register redirecting from AG with their own
adder, the EX result as an AND-OR of one-hot selects. Result: 66.5 MHz (best single build 67.9) at 26,447 logic. The
price was one clock per EX redirect, because the front end followed the redirect one clock later (CPI 2.029 to 2.150).
Two of the 21 steps (aligning the forwarded store word in OC itself and comparing each store term separately) were
reverted because the build with them was slower (57.5 MHz, worst path through exactly that logic). A blanket
`syn_maxfan 10` on all modules was tried too and lost badly (45.5 MHz, +2,500 logic).

**Deeper pipeline.** The next round added a DR register stage between DS and AG (a two-entry queue that merges the
decoder and sequencer parts), split OC into OC1 (cache, forwarding, misaligned split) and OC2 (load alignment, operand
selects), and completed the flags in WB one clock after the result. Every flush now reads its target from a register
in the flush clock, which won back the extra redirect clock (CPI 2.150 to 2.029, then 2.017 with DR). Build: 67.5 MHz,
26,282 logic. This round was stopped half way when dual issue became the main goal, and every step of it was kept,
because dual issue needed each one (the pairing checks live in DR, OC2 reads lane B's operands, two flag writers per
clock need flags completed in WB).

**A DS-to-AG stage that was rejected.** Much later, one more register stage between DS and AG was tried to cut the
decode paths in the full system. It cost CPI 1.035 (1.031 with a bypass for an empty DR, which also failed a lockstep
run), so it was dropped.

## 4. Fetch and the instruction queue

**Queue with predecode, window as a register.** From the sample on, fetch reads 8 bytes per clock from the I-cache
into a 16-word queue, and each word carries its predecoded length and sequence flag. In the first timing round the DS
window (the first five words of the head instruction) became a register with the block words bypassed, so DS had no
16:1 select behind a head pointer, and the predecode moved into the I-cache itself: it is computed from the fill data
and stored with the line (36 more bits per entry, 2 more block RAMs), so it is never on the path from the cache to the
queue.

**Rotator window for dual issue.** For pairs the window grew to eight words. The first version used two 16:1 selects
per window word and did not route (576 unrouted nets, 34,754 logic). It was rebuilt as one rotator: the queue after this
clock rotated by head plus length in four 2:1 stages, with the pair taking it shifted by the second instruction's
length. That routed at 57.9 MHz and became the base for the dual-issue work.

**Third fetch stage and a shift-register queue (tried twice, not kept).** Twice a third fetch stage was built, with the
I-cache block, its predecode and the hit as registers before the queue write, once together with moving the issue
decision into fetch, and once together with replacing the queue by a 20-word shift register whose head is the window
(no head pointer, no rotator). The first attempt measured 57.9, 57.6 and 61.1 MHz against 71 MHz before: the registers
around the window loop spread the window logic 30-60 columns away from DS, so the same 13 levels took 17 ns instead of
14. The third stage also costs CPI (1.007 to 1.021, or 1.063 without an early read of the return target). The second
attempt was folded into the level-reduction work described in section 15 and its line of work was not merged; its
ideas (relative cut positions, the decision in fetch, the early return-target read) came back later in other forms.

**Instruction buffer (kept).** The structure that stayed is a 68060-style fixed-format instruction buffer. Fetch's
extractor moves up to two whole instructions per clock from the window into a 16-entry buffer in DS (two LUT-RAM banks,
the two oldest entries as register copies), and DS consumes 0, 1 or 2 entries per clock. DS's issue no longer drives
the window, so the window loop dropped from 24.2 to 19.9 ns in the timing estimate, and pairs no longer needed a head
of one or two words. It cost a little CPI (1.007 to 1.021, the calls loop 1.216 to 1.227) and about 4,400 more logic.

**The queue is the window.** In the first full builds the worst CPU path was the 16:1 select from the queue into the
window (17.7 ns). The 16-word queue plus the eight window registers were replaced by one 14-word queue that is itself
the window, moved each clock by the consumption with an AND-OR per position. Fetch dropped from 10,750 to 9,515 LUT and
the estimated endpoints above 15 ns from 1,065 to 812. Fourteen words is the minimum: the longest instruction the
full-format walker reads is 11 words, and a 12-word queue hung.

**Fill predecode from block ROM (reverted, then kept).** The fill predecode of the four words of each fill beat is about
1,100 LUT of logic. It was turned into two levels of ROM (the top 10 bits of a word give one of 75 row classes, then the
class and the low 6 bits give the 12-bit predecode), generated by `tools/gen_pdrom.py` over all 65,536 words, which
saved 1,150 LUT in fetch for 28 small ROM blocks. The first time it was reverted because those 28 blocks pushed the full
A1200 system to 367 block RAMs of 340. It was put back later (saving 1,055 LUT) once the system's block RAM count
allowed it (the change that freed the blocks is in the pipemig repository). A version with 14 true dual-port block RAMs
instead of 28 ROMs was written, but Gowin's simulation model of that primitive could not be run in reasonable time (25
minutes for 1,100 words without finishing, and the first compared output did not match), so the inferred ROMs stayed.

**Branch target cache.** A 256-entry BTC in block RAM, one entry per 8-byte fetch block, read with the I-cache. At first
it had one prediction outstanding, predicted only Bcc/BRA inside one block, and checked the opcode at the cut. A perf
trace showed the copy loop's DBF crossed a block and paid a redirect every iteration, so the BTC learned DBcc, BSR and
branches that cross a block (copy loop CPI 2.515 to 1.591). Two outstanding predictions came with the branch fold pair
(the integer loop was fetch-bound with one). The opcode check was dropped for timing: a wrong cut is marked and replays,
and a wrong target is caught by OC's compare.

**Return prediction.** A 4-entry return stack in DS from the start. RTS was first a two-uop sequence (move A7, then
jump); making it one uop predicted at its issue saved two clocks per return (CPI 1.091 to 1.007, calls 1.446 to
1.216). With the instruction buffer the RTS is predicted from the buffer registers and fetch takes the prediction one
clock later as a flush with a registered target.

**I-cache fill wait (kept, costs CPI).** In the full 720p system the last slow fetch path ran from the predecode ROM
output straight into the queue length one-hots (142 failing endpoints). The bypass was removed: a read of the line in
the clock its last beat arrives waits one clock. CPI went from 1.021 to 1.031 (the integer loop 224 to 232 clocks), and
the project author accepted that for the clock it bought; this build was the first to meet 66.70 MHz in the full system.

**A reset bug the board found.** A CPU reset while running left the queue's valid thermometer set, so the new words went
in behind stale ones and nothing was ever extracted; the CPU hung at address 0. Only a flush cleared it before. The
system simulation reproduced it with a second reset, and the reset now clears the thermometer and decode's return flags.

## 5. Decode

**Group decoders OR-merged.** DS has one decoder per instruction group (`dec_*.v`), each giving a uop and a hit, and the
uops are OR-merged; no hit means illegal instruction. This kept the parallel work packages out of each other's files.
The price is depth: twenty decoders plus the OR were 16 levels from the window word to the DR entry with two module
crossings. A select between decoder and sequencer uops was measured to cost 20 MHz early on, so sequences start with an
empty step 0 and are ORed in.

**Making the decode shallower, step by step.** In order: DS ends at the decoder and sequencer ORs and DR merges the
parts (one more stage, no CPI cost); every decoder gives its uop ungated and DS ANDs the hits in its final OR; the
immediate and displacement values became 14-bit data codes that DS turns into values once instead of twenty decoders
each building 32-bit values (measured first: an ideal opcode ROM for the control fields would save only about 700 LUT,
far less than its block RAMs cost, so the data fields were the target); the value selects come straight from the opcode;
and finally the decoders run once when an entry is written into the instruction buffer (`p68_pdec.v`) and store their
gates and the legality in the entry (20 bits), so DS gates the uops with registered bits. Each of these was checked
exhaustively against the old decoders over all 65,536 opcodes with several sets of extension words.

**Not kept.** Predecoding the decoders' code constants into the buffer as well was built and reverted (reason not
recorded). Giving each sub-sequencer its own preserved copies of class, step and parameters was built and reverted
(reason not recorded). A larger rework that added about 2,000 LUT to fetch and decode (block positions from registers,
data-value selects registered per entry) improved the estimate (worst decode path 18.74 to 16.96 ns) but made the full
system unroutable (306 unrouted nets at place option 2, while the same system with the previous CPU routed at 62.5 MHz),
so it was reverted. In the level-reduction work (section 15) a version with decoder sub-hits predecoded at the I-cache
fill (54 bits per word in the cache and the queue) was built; it reduced the fetch-to-DS paths above 8 levels from 43 to
17 but its line of work was not merged.

## 6. Sequencers and the long instructions

- **MOVEM** was the first sequence: step 0 empty, control modes compute the EA into T0, then one uop per register. The
  remaining-register mask after the next transfer later became a register to take the carry chain out of `last`.
- **Full-format EAs** (memory indirect, base and index suppress): the choice was between a window of 11 words with a
  length decode over several words, or a pre-sequence. The pre-sequence (`p68_fx.v`) won because the full format is rare
  and a wide window would lengthen the queue-to-DS path for every instruction. It walks the extension words and issues at
  most two uops per EA into T1/T2, then the decoders see the opcode with the EA changed to (An). The walk later read its
  queue port into a register (two clocks per extension word) for timing.
- **Multiply and divide.** The plan was a DSP-based divider (reciprocal estimate and Newton-Raphson, about 8-10 clocks).
  Worked out in detail, it needed about 20 clocks with DSP pipelines and an error proof, no faster than a radix-4
  restoring divider of 16 clocks, so the radix-4 divider was built (exact, three subtractors per clock; multiply on four
  DSP products, 6 clocks). The unit sits behind a fixed-latency interface, so a faster divider can replace it alone.
- **Bit fields** first used two barrel shifters on a 64-bit window (1,769 LUT, with a 19-level path). The unit was
  rewritten with one 64-bit rotator used in two passes (1,067 LUT, one more empty uop, perf unchanged).
- **BCD** (ABCD, SBCD, NBCD) chained three additions in one clock (about 10 levels). It became three uops, each one
  ten-bit add and a select, since these instructions are rare.
- **CAS/CAS2** are sequences with a bus lock; their loads are uncached. A first lock that also held off the core's own
  instruction refills deadlocked a replay, so refills are not held.

## 7. Exceptions, interrupts and the system unit

The rule that came out of the exception work and never changed: a condition that raises an exception must come from a
register, because it feeds the global flush. CHK, CHK2 and divide by zero latch their compare and raise in a following
uop. Another early rule (a uop that redirects at commit must not update An in AG) became unnecessary once the register
file restore moved one clock after the redirect, which is what allowed RTS to become a single uop.

**Interrupts in polling loops.** A 2000-program lockstep run found a loop (MOVEQ, a load from uncached memory, DBcs)
that could never take an interrupt: the uncached load holds interrupts until it commits, and the head instruction ran
in lane B, where no interrupt is taken. The integer core without the FPU had the same fault. The fix forms no pairs
while an unmasked interrupt is requested; it costs nothing without interrupts (CPI unchanged at 1.007), and a new
generator group tests exactly this case (179,224 interrupts in the directed run).

**CPU speed mode for games.** For old games the system can lock the CPU to about 14 MHz equivalent. The first version
was a fixed throttle of N clocks per instruction with dual issue off; the second charged each instruction its 68EC020
cache-case cycles times 4.7 from the opcode instead of a fixed 28 clocks; then the cost was registered (opcode to base
and EA cost to sum to counter), pairs were blocked through a registered copy instead of an AND in the pair terms, and
the cost tables were scaled to the real core clock.

**Uncached areas.** The data cache must not cache chip RAM or I/O. A 16-bit map of 1 MB granules (`unc_map`) did that,
but it matched A23-A20 in every 16 MB, so Zorro III RAM at $40000000 was uncached in 7 of every 16 MB; the map was limited
to the 24-bit area. A second input made the RTG board's 32 MB window uncached. A third (`cac_hi`) caches data above
16 MB only inside the RAM board window, because an AROS memory probe of empty space read its own stores back from the
D-cache, added RAM that did not exist and overwrote ExecBase.

## 8. Dual issue

The model is the 68060: pipe A does everything, pipe B only simple single-uop instructions, one memory operand per pair.
It was built in this order, each step green on the bench, and measured on core-alone builds at a 12.5 ns constraint:

| Step | What it did | Fmax | Logic | CPI |
|---|---|---|---|---|
| Single issue, deeper pipeline | baseline | 67.5 | 26,282 | 2.017 |
| Two retire records, lane B through all stages, register file ports, pairs of register instructions | first dual-issue core | did not route | 34,754 | 1.589 |
| One rotator for the window | routing fix | 57.9 | 32,353 | 1.589 |
| Memory in either slot; predicted branches in lane B | | 56.5 | 33,555 | 1.429 |
| N of both lanes registered, store checks without the CAS condition | timing | 64.5 | 33,651 | 1.429 |
| MOVEM mask as a register | timing | 64.1 | 33,564 | 1.429 |
| Pairs only behind heads of 1-2 words; shifter, CHK and similar take no WB forwarding | timing; the fold and skip pairs (below) were added in the same round | 71.8 | 33,041 | 1.091 |
| Bit field unit takes no WB forwarding; RTS one uop | | 73.4 | 33,122 | 1.007 |
| Register copies for AG's interlock; window rotator turns stored words only | | 69.4-71.6 | about 33,650 | 1.007 |
| Bus lock dropped one clock after a redirect | | did not route twice (reverted) | 33,431 | 1.007 |
| Final of this round | | 71.0 | 33,347 | 1.007 |

MIPS (Fmax divided by CPI) went from 33.5 for the single-issue core to 70.5. The pairing pieces that made the difference:

- **Swapped pairs.** A memory instruction behind a simple head runs in lane A with the head in lane B as the older one.
- **Branch fold pair.** A BTC-predicted branch issues together with the first instruction at its target in lane B; the
  branch costs no clock, also behind its own flag writer (integer loop 0.759 to 0.625, all 1.429 to 1.325).
- **Skip pair.** A forward Bcc over exactly the next instruction runs with it in lane B and never redirects; when taken,
  lane B writes its destination back unchanged (CPI 1.325 to 1.091, calls 2.090 to 1.446).
- **MOVE.L Rm,Rx rename** from the 68060: the second instruction reads Rm where it reads Rx.

Not done, with the reasons recorded: a real zero-slot fold for a branch behind its flag writer in the same pair (needs
the condition from the pair's own flags, which are completed in WB, so it would put an unregistered path in); making
DS wait one clock for a fold target (tried, CPI 1.007 to 1.010, dropped); pairing the memory copy loop, which is bound
by the memory port at 3 clocks per write in the bench, so only a wider or burst write port would help.

Lockstep 2000 x 400 on the final core: 2,084,566 instructions, 214,534 of them in lane B (10 %). Pairs behind heads of
any length came back with the instruction buffer.

## 9. Register file and forwarding

The register file is flip-flops with packed vectors (D0-D7, A0-A7, T0-T3 and the three stack pointer banks) and a
committed copy of A and T registers that is restored after a redirect. Its history is mostly about where the reads
happen:

- AG reads base and index one clock early at DS's register numbers, with AG's own previous writes forwarded from OC's
  registers. Kept from the first timing round on.
- AG's writes enter the register file from OC's registers one clock later. Kept.
- The restore moved one clock after the redirect (costs nothing once flush targets are registered). Kept.
- Reading the register file in OC1 into registers, with OC2 only bypassing WB writes, was built twice and not kept: the
  first time it was not among the slow paths in the reports, the second time (made exact one clock early, plus
  preserved copies of the WB value for EX and AG) the full system's place-and-route showed WB value nets of 9-11 ns, so
  both parts were reverted.
- What stayed in the end: AG's early reads as a held value ORed with the write data under simple selects, and OC2's
  operand reads by registered one-hot register numbers, which cut the WB-value-to-AG and OC2-to-EX path groups of the
  full system (68.20 to 68.70 MHz at the 70.6 MHz target).

## 10. Address generation

- **LUT adder against DSP.** A DSP multiply-add for base + index x scale + displacement was measured alone at 114.7 MHz
  (2 DSP) against 200.5 MHz for the LUT 3:2 compressor and carry chain, so AG keeps the LUT adder.
- **Interlock and stall.** The change/use interlock was decided one clock early from registers (`hz_r`), and then the
  whole AG stall became a register computed from next-clock values, checked against the old combinational form over
  410,000 instructions with no difference.
- **Into the D-cache index.** The D-cache index bits come from 13-bit adders in AG. The forwarding of a WB value into
  that index was the longest full-system path family for a while; it was cut by scaling the index per forwarding part,
  then folding the WB forward selects into the scale selects, and by taking OC's split decode from AG's low address bits.

## 11. Operand fetch and the D-cache

- **Replay instead of stall** from the sample on. A miss replays; the replayed load starts the fill into a line buffer
  and then hits it.
- **Misaligned accesses** are split by OC (one more clock per extra read), not by the sequencer.
- **Bugs the benches found** in this area: a refill writing a stale word over a store to the same word (found when the
  exception and integer work were merged; OC now drops such a refill), and a livelock where a line-crossing misaligned
  load in a sequence replayed faster than its second word's single read (found by the cputester data with caches off).
- **Install enable from registers.** The D-cache line install decision came through the CAS condition; it was first taken
  out of that condition and finally made one register formed a clock early as a superset of the store checks, which
  removed 38 failing endpoints in the full system.
- **Own snoop registers.** Synthesis had merged the D-cache's snoop address registers with the I-cache's (a 3.4 ns net);
  each cache keeps its own now, with one `syn_preserve` attribute per register (on a declaration list the attribute only
  applies to the last name).

## 12. Execute

The EX history is a long list of small cuts, all kept unless noted: Z of the adder from the operands without the carry
chain in front, N from the sum, the result as one flat AND-OR of twelve terms (a merged form tried once cost a level and
was undone), operand selects as AND-ORs of one-hot registered controls (two levels instead of four), final flags of both
lanes registered at commit so the condition logic is two levels, the branch condition as a 16-entry truth table
registered on entry to EX, the shifter's count, masks and first rotator stages registered one stage earlier.

Not kept: a round of EX register copies for the full system grew the CPU by 6-8 % and cost about 4 MHz in the full
build, so it was reverted while the fetch and decode parts of the same round stayed. Carry-chain primitives for small
adders and zero tests and hand-placed LUT4 primitives for the flag terms were built in the level-reduction work (section
15) and not merged.

## 13. FPU

**Scope.** The 68881/68882 programming model in hardware for everything except the transcendentals, FMOD/FREM, packed
decimal and a few immediate and dynamic forms, which trap to software as on the 68040/68060. The reasons: packed decimal
needs a 64-digit scaling with exact rounding (several thousand LUTs for a format Amiga software hardly uses), FMOD/FREM
loop up to 16,000 clocks, and the immediate forms need instruction words the decoders do not see. Motorola's 68060 FPSP
was checked and may be redistributed, but a port is not part of the core.

**First version and its timing.** One unit in EX lane A, every FPU instruction a DS sequence with fixed latencies. The
first build ran 53.3 MHz. Three timing steps followed: no interrupt term into the unit, comparisons and adder carries
registered, one shared normaliser and shifter, FP0-FP7 in distributed RAM (synthesis of the unit alone: LUT 9,235 to
6,668, flip-flops 2,690 to 1,507), then the sequencer terms registered, then the FPU terms taken out of the issue and
fetch logic (builds 50.2, 46.9 and 55.7 MHz). All WinUAE cputester FPU sets passed (864,016 tests run, 0 fail).

**The FPU made the whole core slower.** The integer core ran 71.6 MHz alone and 42-56 MHz with the FPU beside it (six
builds of the same RTL), with the same logic depth: the extra third of design size spread the placement. So the FPU was
off by default for a while, and two ways to take it out of the core's timing were built:

- **Own asynchronous clock** with a two-phase handshake: built and benched (lockstep passes at clock ratios 0.63 and
  1.37, CPI unchanged), never place-and-routed; stopped by the author in favor of the next one.
- **Half-rate clock enable** (kept): the engine steps every second clock and `set_multicycle_path` gives its internal
  paths two clocks, following Gowin's own guides. The boundary is registered on both sides and the step enable runs on
  a global line. Result: 52.1 MHz at place option 3 and 60.0 MHz at place option 2, with no FPU path among the 200 worst;
  the worst paths were now the integer front end.

**Region boxes for the FPU** were tried at the same time and failed: a box partly outside the chip was refused, and a
27 x 51 box for the engine (about 76 % fill) failed routing with about 3,500 unrouted nets.

**Small engine (kept).** The engine was rewritten with the same algorithms and bit-exact results: per-step next-value
codes for each wide register, step shifters (by 32, 8 or 1-7 per step) instead of barrel shifters and a 66-bit
leading-zero counter, one DSP product per step summed by columns, FMOVECR constants in block ROM. The engine went from
8,188 LUT and 4 DSP to 4,100 LUT, 1 DSP and 3 block RAMs, at the cost of longer waits (FMUL about 48 clocks more).

**The rule afterwards:** every build is the complete chip with the FPU, and timing is never judged without it.
Measured on the full system, a build without the FPU was even slower than with it (54.0 against 56.9 MHz), which shows
how much one build depends on placement.

**SSRAM on the real board.** Because FP0-FP7 live in 40 LUT RAMs (RAM16SDP4), and those are disabled on one revision of
the chip, a board test ran 1,512 of them for about 100 seconds at 50 MHz: about 7.6e12 reads, 0 errors, and a canary
group with a deliberately wrong checker did report errors. The LUT RAMs stay.

## 14. Memory port and caches

The memory port was fixed at the start so the core never sees the Amiga bus: a 64-bit line fill and a 32-bit write port
with byte enables, one transaction at a time, reads only behind an empty store buffer. The reasons written down: a chip
bus adapter needs program order anyway, and the caches already hold every store a later load could hit, so forwarding
from the store buffer into fills would buy nothing. Both caches are 8 KB direct mapped (I 32-byte lines, D 16-byte
lines, write-through, no allocation on a store miss). External snoops invalidate lines but do not flush the fetch queue;
own stores into fetched code do (self-modifying code works without a CACR clear, unlike a real 68020, because old Amiga
games depend on it). A perf trace later showed the memory port lost a clock after every write, and starting the next
transaction in the clock a write ends was part of the copy loop improvement.

## 15. Timing method and tools

**Logic levels first.** After the half-rate FPU the plan was to bring every path down to 8 LUT levels (Gowin's SUG113
figure for 100 MHz). `tools/levels.py` counted levels in the synthesized netlist and the build stopped above 8. Twelve
batches of cuts took the integer core from 262 path groups above 8 levels (worst 19) to 0, and the core with the FPU to
0 as well (77,618 logic in synthesis with the FPU at that point). But the build of that work ran only 49.0 MHz with the FPU, and the
place-and-route reports showed why: paths of exactly 8 levels ran anywhere from 13.3 to 20.1 ns in the same build, and
the home-made level counter undercounted Gowin's own count by 1-5 levels. This whole line of work was not merged; the
instruction buffer work had started from an earlier point and went on instead.

**Timing estimate (kept).** `tools/timing_est.py` replaced the level count. It estimates each path's delay from the
netlist: Gowin cell delays plus net delays from fanout, the size of the module both ends share, the number of modules
among the loads, and the size of the whole design, with clocks, generated clocks, false paths and multicycle paths read
from the build's SDC. The first model was fitted on one report's nets and estimated 24.2 ns against a real 14.0 ns
(report paths are the worst ones, and timing-driven placement pulls their cells together, so their nets are shorter than
the average net of their kind). The second model was fitted per endpoint on three reports with a design-size term:
worst path within 0.07-0.7 ns for two of them and 2.3 ns pessimistic for the third. Two more models were fitted on full
A1200 builds (one build, then four, with a separate clock-to-out for block RAM outputs). GowinSynthesis's own "Max
Frequency" was checked as an alternative and was 4-9 ns optimistic. The estimate is a screen and a ranking of the paths
to cut; the clock is only ever judged from a real place-and-route. At first it stopped the build, later it only reports.

**Other tools.** `tools/postbuild.py` runs the after-build checklist in Gowin's own order (errors, the EX1998 synthesis
trap, resources, clock lines, Fmax, worst-path anatomy, estimate against real). `tools/modsize.py` gives the size of each
unit. `tools/fplan.py` turned a plan file and the synthesis module tree into region boxes.

**Region boxes (dropped).** A floor plan put the core, the FPU, the chipset and video each in their own box. In the full
system, boxes over about 45,000 cells failed with a constraint memory error (four builds), half the core alone parsed,
and a build with boxes only around the front end and the FPU ran over 7 hours and failed with 37,489 unrouted nets.

**Place and route options.** On the same full-system RTL: place option 1 61.2 MHz, 2 65.5, 3 62.6, 4 58.5; route option 2
instead of 0 gave 60.5 and 62.3; replicate resources 63.5 at place 2 and no route at place 3. Over many builds neither place
2 nor place 3 won reliably (each was the better one about half the time), so every change is judged at both. Hierarchical netlists route where flat ones
did not (on the early core a flat netlist failed 5 times with 180 unrouted nets, hierarchical gave 62.5 MHz; on the small sample flat was faster, 70.7 against 64.0 MHz). Incremental place-and-route was measured
and is useless here: it only works with a flat netlist, reading the old routing took 27 minutes against 4.6 for a full
build, and a placement reused after a one-line change lost 14 MHz.

**LUT copies get merged.** GowinSynthesis merges LUTs with the same function and inputs even when each output carries
`syn_keep`, so every hand copy in the core had silently become one net (for example the five AG stall copies, a 7.5 ns
enable net). A copy only stays apart when one of its inputs is its own preserved register copy. All copies were redone
this way, which, together with the other cuts of that round, took the estimated worst CPU path from 18.37 to 17.15 ns.

## 16. The CPU clock in the full A1200 system

| Stage of the work | CPU clock reached (target) |
|---|---|
| First full builds with the instruction buffer | 62.5 MHz at place 2, 56.8 at place 3 (66.7 target, failed) |
| Cuts on the failing CPU path groups of that build, -949 LUT | 66.68 MHz (66.67), every clock met for the first time |
| Copies made real, decoder gates predecoded | 65.5 MHz with the new memory controller, 66.68 with the older one |
| 720p HDMI added to the system | 61.2-65.2 MHz; one variant needed 353 block RAMs of 340 |
| Cut attempts aimed at 70 MHz, most reverted | 56.9 MHz at 70.6 target |
| I-cache fill wait, D-cache install from registers, shorter forward select | 66.70 MHz (66.67), all clocks met |
| The same CPU at a 70.6 MHz target, then two rounds on its real failing path groups | 68.20, then 68.70, then 69.35 MHz |
| Same CPU in the 720p system with USB input | 66.70 MHz (66.67), all clocks met |

Several builds in between failed for reasons outside the CPU (SDC lines naming registers that did not exist, a clock
constraint on an input pin, routing failures with "0 unrouted nets", which Gowin reports for reasons it does not
explain). A system build takes between half an hour and two hours.

## 17. The test benches

- **Lockstep bench** (Verilator and Musashi in one process, compared after every instruction): the first run on the
  sample compared 2.15 million random instructions and found six RTL faults (ROL/ROR changing X, ASL not setting V,
  shift counts above the size, ADDA.W from memory loading a long, two (An)+ ordering cases). It grew to 40 generator
  groups (DMA writes, self-modifying code, FPU, polling loops among them), a fixed perf test and an AROS boot mode. The
  final check before every merge is 2000 programs x 400 instructions, about 2.1 million instructions, 5.24 million with
  the FPU groups. Two bench bugs are worth knowing: a smaller build once linked a stale object, and a failed link left an
  old executable that printed PASS, so the script now deletes all old output first.
- **Musashi was wrong in places.** Where it disagrees with a real 68020 (BCD, BFFFO, MOVEM with the base in the list,
  PACK/UNPK memory forms, CACR read-back, BFTST on memory), the core follows the 68020 and `tools/musashi_patch.py`
  patches Musashi the same way. Its FPU was too weak (no precision control, no exception bits), so the bench has its own
  68881/68882 reference, `fpu_ref.c`.
- **SingleStepTests 68000** (about 1 million one-instruction tests): 745,646 run, 3 fail, and all three are known (two
  ASL.b entries with wrong data, one DIVU divide-by-zero flag case where the 68000 and 68020 differ).
- **WinUAE cputester 68020 data** (5.1 million tests): the first run had 30,541 failures (illegal-encoding cases and
  places where the core followed Musashi); after the fixes 5,128,108 pass and 0 fail, with 6,530 undefined-flag cases of
  CHK/CHK2 counted apart.
- **WinUAE cputester FPU data**: 917,360 tests, 864,016 run, all pass, 53,344 skipped by design (the trapped forms).
