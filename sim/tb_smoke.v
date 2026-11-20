// Copyright 2026 Franz Schoning
// Project: https://github.com/Schoning-Labs/pipe68k
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Smoke test for the feasibility sample core (rtl/sample/p68_core.v), run in the Icarus Verilog simulator. This is the first and
// simplest test in the project: a Verilog testbench with a tiny program, a fake memory and a check of a few results at the end.
//
// Where it sits: a testbench is a Verilog module that is not part of the chip. It generates the clock and reset, plays the role of
// the memory the CPU talks to, and looks at the CPU's registers when the run is over. The module under test (the "dut", device
// under test) is the pipelined sample core with its six stages (IF, DS, AG, OC, EX, WB). The sample core has a small memory
// port: it raises bus_req with a word address, and the testbench answers with bus_ack and the 32-bit word a few clocks later; stores
// leave on a separate write port (bw_en, bw_addr, bw_data and the four byte enables bw_be).
//
// The program (12 instructions in 6 memory words) adds 5+4+3+2+1 in a loop, stores each running sum through the (An)+ addressing
// mode, reads the last two sums back with -(An) and (An), shifts the result left by two and stops in a one-instruction loop. Every
// piece of the sample was written for exactly these instructions: MOVEQ, LEA, ADD, MOVE with post-increment and pre-decrement,
// SUBQ, BNE and LSL. If the final registers are D0=0, D1=15, D2=120 and A0=$1010 and the first and last stored words are 5 and 15,
// the testbench prints PASS. The check proves very little about the CPU in general. It only shows that the pipeline, the caches and
// the forwarding do the right thing for the simplest loop, which was enough to start from. Everything after the sample was checked
// with the lockstep bench instead (sim/lockstep/lockstep.cpp), which compares against a reference CPU after every instruction.
// This file is not used by the lockstep scripts and is run by hand with Icarus Verilog together with rtl/sample/p68_core.v.
// One nanosecond per time unit, with picosecond resolution.
`timescale 1ns/1ps
// The testbench has no ports; it is the top module of the simulation.
module tb;
  reg clk = 0; always #5 clk = ~clk;  // clock: starts low and toggles every 5 ns, which is a 10 ns period (100 MHz); the sample was only ever simulated, so the rate has no timing meaning
  // Reset is held active (high) from time 0 and released by the second initial block below.
  reg rst = 1;
  // Wires from the core: memory request and address (word address, bits 31 to 2), write port (enable, address, data, byte enables), and
  // observation outputs (oa, ob, ccr) that this test does not use.
  wire bus_req, bw_en; wire [31:2] bus_addr, bw_addr; wire [31:0] bw_data, oa, ob; wire [3:0] bw_be; wire [4:0] ccr;
  // mem = 2048 words of 32 bits (8 KB) of fake memory. rdata and ack answer the core's read; lat counts the clocks of the read latency.
  reg [31:0] mem [0:2047]; reg [31:0] rdata; reg ack = 0; reg [1:0] lat = 0;
  // Loop counter for the initial blocks (the simulator allows an integer in a testbench; it would not be synthesizable in the chip).
  integer k;
  // First initial block: fill the memory.
  initial begin
    // Every word starts as two NOPs (0x4E71 is the 68k NOP), so the core runs off the end of the program harmlessly.
    for (k = 0; k < 2048; k = k + 1) mem[k] = 32'h4E714E71;
    mem[0] = 32'h70057200;  // MOVEQ #5,D0 ; MOVEQ #0,D1  // MOVEQ #5,D0 (0x7005: 0111 000 0 + data byte 5, sign extended to 32 bits) and MOVEQ #0,D1 (0x7200)
    mem[1] = 32'h41F81000;  // LEA $1000.W,A0  // LEA $1000.W,A0 (0x41F8 = LEA with an absolute-short source address, A0 as destination, address word 0x1000)
    mem[2] = 32'hD28020C1;  // loop: ADD.L D0,D1 ; MOVE.L D1,(A0)+  // loop: ADD.L D0,D1 (0xD280) and MOVE.L D1,(A0)+ (0x20C1): the sum is stored and A0 moves up by 4
    mem[3] = 32'h538066F8;  // SUBQ.L #1,D0 ; BNE loop  // SUBQ.L #1,D0 (0x5380) and BNE loop (0x66F8: condition not-equal, displacement -8 bytes back to the ADD)
    mem[4] = 32'h2420D490;  // MOVE.L -(A0),D2 ; ADD.L (A0),D2  // MOVE.L -(A0),D2 (0x2420: A0 steps back by 4 first) and ADD.L (A0),D2 (0xD490)
    mem[5] = 32'hE58A60FE;  // LSL.L #2,D2 ; BRA *  // LSL.L #2,D2 (0xE58A) and BRA * (0x60FE: a branch to itself, which ends the program)
  end
  // Memory model, one clock domain: everything happens on the rising edge of the clock.
  always @(posedge clk) begin
    // Default: no acknowledge this clock (ack is a register, so it lasts one clock).
    ack <= 0;
    // A read request that has not been answered yet: wait three clocks in total and then answer.
    if (bus_req && !ack) begin
      // When lat reaches 2 the word at the requested address is returned and ack is raised for one clock; until then lat counts 0, 1, 2.
      // bus_addr is a word address, so bits 12 down to 2 of the byte address select one of the 2048 words.
      if (lat == 2) begin ack <= 1; lat <= 0; rdata <= mem[bus_addr[12:2]]; end else lat <= lat + 1;
    end
    // Writes take effect at once (no latency): the byte enables decide which of the four bytes of the word change.
    if (bw_en) begin
      // bw_be[3] is the byte at the lowest address (the 68k is big endian, so the most significant byte comes first in memory).
      if (bw_be[3]) mem[bw_addr[12:2]][31:24] <= bw_data[31:24];
      if (bw_be[2]) mem[bw_addr[12:2]][23:16] <= bw_data[23:16];
      if (bw_be[1]) mem[bw_addr[12:2]][15:8]  <= bw_data[15:8];
      if (bw_be[0]) mem[bw_addr[12:2]][7:0]   <= bw_data[7:0];
    end
  end
  // The core under test, connected to the clock, reset, the read port and the write port.
  p68_core dut(.clk(clk), .rst(rst), .bus_req(bus_req), .bus_addr(bus_addr), .bus_rdata(rdata), .bus_ack(ack),
    .bw_en(bw_en), .bw_addr(bw_addr), .bw_data(bw_data), .bw_be(bw_be), .obs_a(oa), .obs_b(ob), .obs_ccr(ccr));
  // Second initial block: the test sequence.
  initial begin
    // The sample core's registers have no reset value, and Icarus would start them as unknown (X), so the testbench clears the packed
    // register file rf (D0-D7 and A0-A7, 16 values of 32 bits) by hierarchical reference into the core.
    for (k = 0; k < 16; k = k + 1) dut.rf[k*32 +: 32] = 0;
    // Clear iql (the predecoded instruction length of every instruction queue word), the instruction cache tags (512) and the two copies of the data cache tags (1024 each),
    // so that no X value makes a cache look valid or invalid at random.
    dut.iql = 0;
    for (k = 0; k < 512; k = k + 1) dut.ic_tag[k] = 0;
    for (k = 0; k < 1024; k = k + 1) begin dut.dct[k] = 0; dut.dct2[k] = 0; end
    // After 25 ns, release reset.
    #25 rst = 0;
    // Run for 4000 ns (400 clocks), far more than the 12 instructions need even with the 3-clock memory latency and cache fills.
    #4000;
    // Print D0, D1, D2 and A0 (rf holds D0 in bits 31 to 0, D1 in 63 to 32, and so on; A0 is register number 8).
    $display("D0=%h D1=%h D2=%h A0=%h", dut.rf[31:0], dut.rf[63:32], dut.rf[95:64], dut.rf[8*32 +: 32]);
    // Print the words at byte addresses $1000 to $1010 (word indexes 1024 to 1028 of the memory).
    $display("mem 1000..1010: %h %h %h %h %h", mem[1024], mem[1025], mem[1026], mem[1027], mem[1028]);
    // Expected: D0 = 0 (the counter ran out), D1 = 15 (5+4+3+2+1), D2 = 120 ((15+15) shifted left by 2), A0 = $1010, the first stored
    // word is 5 and the fifth is 15.
    if (dut.rf[31:0] == 0 && dut.rf[63:32] == 15 && dut.rf[95:64] == 120 && dut.rf[8*32 +: 32] == 32'h1010 &&
        mem[1024] == 5 && mem[1028] == 15) $display("PASS"); else $display("FAIL");
    // End the simulation.
    $finish;
  end
endmodule
