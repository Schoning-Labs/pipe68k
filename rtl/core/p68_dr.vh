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
// ==== p68_dr.vh: the override fields DS hands to the DR queue ========================================================
//
// DR is a small two-entry register queue at the front of p68_ag.v, between the decode stage (DS, p68_decode.v) and
// address generation (AG). DS does not build the finished uop. It hands DR the parts: the OR of all group decoders'
// uops (o_dec), the OR of all sequencers' uops (o_seq), one bit that says a sequence is running (o_act, so DR knows which
// of the two counts), and the override fields listed here (o_ov). DR then merges them in its own clock (function drm in
// p68_ag.v) into the uop that AG works on.
//
// The reason for the split is timing. The full decode, twenty group decoders ORed together plus the select between a
// decoder uop and a sequencer uop plus the fields that depend on the instruction's position (PC, length, prediction
// state), was about 16 LUT levels with two module crossings, too deep for one clock at the target. Ending DS at the
// two ORs and moving the merge into DR's clock added a pipeline register but cost no CPI, because DR also had to exist
// for dual issue (the pairing checks live there). The fields below are exactly the ones that are not simply ORed.
`ifndef P68_DR_VH                        // include guard
`define P68_DR_VH
`define DV_LAB   0  +: 5                 // LA base register of a decoder uop (An from the opcode, or T1 for a converted full-format EA)
`define DV_SAB   5  +: 5                 // SA base register of a decoder uop (An from the opcode, or T2, A7 for BSR, T0 for FPU LEAs)
`define DV_LXR   10 +: 4                 // index register of a decoder uop (bits 15-12 of the brief extension word)
`define DV_ADJ0  14                      // clear SA_ADJ (a MOVE destination converted by p68_fx: T2 already holds the source's step)
`define DV_PFIX  15                      // the instruction was cut wrongly by a fetch prediction (becomes F_PFIX)
`define DV_PBTC  16                      // the instruction is the BTC-predicted branch (becomes F_PBTC)
`define DV_ISEL  17                      // replace IMM by DV_IMM (the BTC target, or the predicted return address)
`define DV_PRET  18                      // predicted return (becomes F_PRET)
`define DV_IMM   19 +: 32                // the replacement IMM value
`define DV_PC    51 +: 32                // the instruction address (from DS's PC register, or from the exception sequencer)
`define DV_LEN   83 +: 4                 // the instruction length in words
`define DV_FIRST 87                      // F_FIRST of this uop
`define DV_LAST  88                      // F_LAST of this uop
`define DV_STEP  89 +: 5                 // F_STEP of this uop
`define DV_FXP   94                      // the uop belongs to the full-format pre-sequence (U_FXP)
`define DV_FXM   95                      // the uop belongs to the main part of a p68_fx instruction (U_FXM)
`define DVW      96                      // width of the override vector
`endif
