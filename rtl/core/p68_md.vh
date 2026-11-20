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
// ---------------------------------------------------------------------------------------------------------------------
// p68_md.vh: codes for multiply and divide. Operation codes of the UN_MUL / UN_DIV uops (the multi-clock unit p68_md.v
// in EX, the execute stage) and the two sequence classes. Included at file level.
//
// CC field of every UN_MUL / UN_DIV uop: [0] signed, [1] 16-bit operands (W forms), [2] 64-bit (MUL: 64-bit product to
// Dh:Dl; DIV: 64-bit dividend Dr:Dq). The unit captures its operands from the start uops and keeps the result until the
// result uops (MD_GLO, MD_GHI) read it; the sequence in p68_mdseq.v puts empty uops in between to cover the latency.
// ---------------------------------------------------------------------------------------------------------------------
`ifndef P68_MD_VH
`define P68_MD_VH
`define MD_MUL   5'd0    // UN_MUL: A = source operand, B = other factor; starts the multiply
`define MD_DLD   5'd1    // UN_DIV: A = divisor, B = dividend high (Dr; unused unless CC[2])
`define MD_DGO   5'd2    // UN_DIV: B = dividend low (Dq); starts the divide
`define MD_CHK   5'd3    // UN_DIV with EXC = 5: raises divide by zero (the unit's latched divisor == 0)
`define MD_GLO   5'd5    // result low: product low, quotient (W form: remainder:quotient); flags
`define MD_GHI   5'd6    // result high: product high, remainder
`define SQ_MUL   4'd4    // sequence class: multiply (p68_mdseq.v)
`define SQ_DIV   4'd5    // sequence class: divide (p68_mdseq.v)
`endif
