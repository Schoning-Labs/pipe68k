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
// ==== p68_mem.v: the memory port unit ================================================================================
//
// What it is for. This is the core's only connection to memory (docs/MEMORY_PORT.md describes the port for
// someone who wants to attach the core to another system). Three clients inside the core want memory:
//   - the store buffer, fed by the write-back stage (WB, p68_wb.v), which writes every store through to memory (both
//     caches are write-through),
//   - the D-side, the operand fetch stage with its D-cache (OC, p68_oc.v and p68_dcache.v), which reads a 16-byte line
//     on a D-cache miss (2 beats of 64 bits) or one uncached longword (for chip RAM and I/O on the Amiga),
//   - the I-side, the I-cache in fetch (p68_icache.v), which reads a 32-byte line on a miss (4 beats).
// This unit puts one of them on the port at a time and returns the read beats to the right one.
//
// Why it is this simple. The port was fixed at the start so that the core never sees the Amiga bus: a 64-bit line fill,
// a 32-bit write with byte enables, one transaction at a time. Two reasons were written down then. An adapter for the
// Amiga chip bus has to keep program order anyway, so letting reads overtake writes here would gain nothing there. And
// the caches already hold every store a later load could hit (the D-cache is updated by every store that hits it), so
// forwarding data from the store buffer into a line fill would buy nothing either.
//
// Ordering rules. Writes leave in program order. A read is issued only when the store buffer is empty and no store
// enters it in the same clock, so a read never overtakes a write, and a line fill always sees every older store in
// memory. Priority when several want the port: store buffer, then D read, then I read. The store buffer has 4 entries,
// is filled from WB (one store per clock) and drains one write per transaction. OC replays a store when the buffer could
// be full by the time that store reaches WB (it counts the entries with sb_cnt plus the stores in flight), so this unit
// never has to refuse a store.
//
// One improvement measured on the bench: a performance trace showed that the port lost one clock after every write,
// because a new transaction could only start once the unit was idle. Now the next transaction may start in the very
// clock a write ends (m_done), which was part of the work that sped up the memory copy loop of the performance test.
//
// Clock: the CPU clock. The port's fields are registers (m_*), so the memory controller sees clean outputs.
`default_nettype none
module p68_mem (
  input  wire        clk,
  input  wire        rst,
  // the uncached areas for writes (the D-side decides its reads itself and says so with dr_unc)
  input  wire [15:0] unc_map,        // bit k: addresses with A23-A20 = k are uncached (data); in the 24-bit area only
  input  wire [7:0]  unc_hi,         // and A31-A25 = unc_hi[6:0] when unc_hi[7] (a 32 MB board, e.g. the RTG card)
  input  wire [6:0]  cac_hi,         // and above 16 MB outside the RAM board window (described at p68_core.v's port)
  // store buffer input (WB stage)
  input  wire        sb_we,          // WB writes a store into the buffer this clock
  input  wire [31:2] sb_addr,        // its longword address
  input  wire [31:0] sb_data,        // its data, already aligned to the longword (big-endian byte lanes)
  input  wire [3:0]  sb_be,          // its byte enables (bit 3 = byte at address +0)
  output reg  [2:0]  sb_cnt,         // entries in the buffer (0-4); OC uses it to replay a store that might not fit
  // read requests: a level, held until the grant (dr_go / ir_go)
  input  wire        dr_req,         // the D-side wants a read
  input  wire [31:2] dr_addr,        // ... at this address
  input  wire        dr_unc,         // one uncached word (else a 2-beat line, dr_addr line aligned)
  input  wire [3:0]  dr_be,          // byte enables of the uncached word (the exact bytes the instruction reads)
  input  wire        ir_req,         // the I-side wants a line
  input  wire [31:2] ir_addr,        // 4-beat line, aligned
  output wire        dr_go,          // grant: the D read starts now
  output wire        ir_go,          // grant: the I read starts now
  output wire        r_d,            // read beat for the D side
  output wire        r_i,            // read beat for the I side
  output wire [63:0] r_data,         // the beat (shared by both sides)
  output wire        r_done,         // last beat of the read (with r_d or r_i)
  // memory port (the core's outputs, see p68_core.v)
  output reg         m_req,          // one clock: start a transaction (fields below stay valid until m_done)
  output reg         m_wr,           // write (else read)
  output reg  [31:2] m_addr,         // longword address
  output reg  [1:0]  m_len,          // beats - 1 for reads: 0 (uncached word), 1 (D line), 3 (I line)
  output reg         m_unc,          // uncached access: exactly these bytes, no line
  output reg         m_ins,          // instruction fetch
  output reg  [3:0]  m_be,           // byte enables
  output reg  [31:0] m_wdata,        // write data
  input  wire        m_rdy,          // read beat valid on m_rdata
  input  wire [63:0] m_rdata,        // read beat, big-endian (byte +0 in bits 63:56)
  input  wire        m_done          // transaction ends (reads: with the last beat)
) /* synthesis syn_maxfan = 16 */;
// The store buffer: four entries as separate registers. Writing a packed array element inside a loop would be the
// usual way, but plain named registers and a case statement give the synthesizer nothing to misread.
reg  [31:2] q_a0, q_a1, q_a2, q_a3;  // store buffer (packed registers, no arrays written in loops)
reg  [31:0] q_d0, q_d1, q_d2, q_d3;  // data of each entry
reg  [3:0]  q_b0, q_b1, q_b2, q_b3;  // byte enables of each entry
reg  [1:0]  q_wp, q_rp;              // write pointer (next free entry) and read pointer (oldest entry), wrap at 4
reg         busy;                    // a transaction is on the port
reg  [1:0]  own;                     // whose transaction: 0 write, 1 D read, 2 I read

// The next transaction may start in the clock a write ends (m_done), not only when idle.
wire        pop   = busy && (own == 2'd0) && m_done;   // a write ends: its entry leaves the buffer
wire        idle  = !busy || pop;                       // the port is free for a new start this clock
wire [2:0]  sb_r  = sb_cnt - {2'b00, pop};    // entries left after this clock's pop
wire [1:0]  q_rn  = q_rp + {1'b0, pop};       // the entry to write next
wire        sb_go = idle && (sb_r != 3'd0);   // start a write: the buffer has priority
// a read only with an empty buffer and no store entering it now, so it never overtakes a write
assign      dr_go = idle && (sb_r == 3'd0) && !sb_we && dr_req;
assign      ir_go = idle && (sb_r == 3'd0) && !sb_we && !dr_req && ir_req;   // the I-side waits for the D-side
assign      r_d    = busy && (own == 2'd1) && m_rdy;   // a beat for the D-side
assign      r_i    = busy && (own == 2'd2) && m_rdy;   // a beat for the I-side
assign      r_data = m_rdata;                          // both sides see the data; r_d / r_i say whose it is
assign      r_done = busy && own[1:0] != 2'd0 && m_done;   // a read ends

// the entry to write next: a 4:1 select by q_rn
reg  [31:2] h_a; reg [31:0] h_d; reg [3:0] h_b;
always @* case (q_rn)
  2'd0: begin h_a = q_a0; h_d = q_d0; h_b = q_b0; end
  2'd1: begin h_a = q_a1; h_d = q_d1; h_b = q_b1; end
  2'd2: begin h_a = q_a2; h_d = q_d2; h_b = q_b2; end
  default: begin h_a = q_a3; h_d = q_d3; h_b = q_b3; end
endcase

// store buffer writes (no reset needed: an entry is only read after it was written)
always @(posedge clk) begin
  if (sb_we) case (q_wp)
    2'd0: begin q_a0 <= sb_addr; q_d0 <= sb_data; q_b0 <= sb_be; end
    2'd1: begin q_a1 <= sb_addr; q_d1 <= sb_data; q_b1 <= sb_be; end
    2'd2: begin q_a2 <= sb_addr; q_d2 <= sb_data; q_b2 <= sb_be; end
    default: begin q_a3 <= sb_addr; q_d3 <= sb_data; q_b3 <= sb_be; end
  endcase
end

// pointers, count and the port's request registers
always @(posedge clk) begin
  if (rst) begin
    sb_cnt <= 3'd0; q_wp <= 2'd0; q_rp <= 2'd0; busy <= 1'b0; m_req <= 1'b0; own <= 2'd0;
  end else begin
    m_req <= 1'b0;                                       // m_req is a one-clock pulse
    if (sb_we) q_wp <= q_wp + 2'd1;                      // an entry written
    if (pop) q_rp <= q_rp + 2'd1;                        // an entry drained
    sb_cnt <= sb_cnt + (sb_we ? 3'd1 : 3'd0) - (pop ? 3'd1 : 3'd0);
    if (busy && m_done) busy <= 1'b0;                    // the transaction ends (a start below may set busy again)
    if (sb_go) begin
      // a write from the buffer. m_unc is worked out here for writes, from the same three maps the D-side uses for
      // its reads: (1) the 1 MB granules of the 24-bit area (A31-A24 all zero or all one, i.e. the first 16 MB and its
      // mirror at the top), (2) the 32 MB window unc_hi, (3) when cac_hi[6] is set, everything above 16 MB outside
      // the RAM board window, which is cac_hi[2:0] x 64 MB from the start of the 512 MB region A31-A29 = cac_hi[5:3]
      m_req <= 1'b1; busy <= 1'b1; own <= 2'd0; m_wr <= 1'b1; m_addr <= h_a; m_be <= h_b; m_wdata <= h_d; m_len <= 2'd0;
      m_unc <= (unc_map[h_a[23:20]] && (&h_a[31:24] || ~|h_a[31:24])) || (unc_hi[7] && h_a[31:25] == unc_hi[6:0]) || (cac_hi[6] && !(&h_a[31:24] || ~|h_a[31:24]) && !((h_a[31:29] == cac_hi[5:3]) && (h_a[28:26] < cac_hi[2:0]))); m_ins <= 1'b0;
    end else if (dr_go) begin
      // a D read: an uncached word with its exact byte enables, or a 2-beat line with all bytes
      m_req <= 1'b1; busy <= 1'b1; own <= 2'd1; m_wr <= 1'b0; m_addr <= dr_addr; m_be <= dr_unc ? dr_be : 4'b1111;
      m_len <= dr_unc ? 2'd0 : 2'd1; m_unc <= dr_unc; m_ins <= 1'b0;
    end else if (ir_go) begin
      // an I read: always a 4-beat cached line
      m_req <= 1'b1; busy <= 1'b1; own <= 2'd2; m_wr <= 1'b0; m_addr <= ir_addr; m_be <= 4'b1111; m_len <= 2'd3; m_unc <= 1'b0;
      m_ins <= 1'b1;
    end
  end
end
endmodule
`default_nettype wire
