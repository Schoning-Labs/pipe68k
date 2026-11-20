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
// Bench helper for the lockstep test: gives the C++ bench access to one function inside the Musashi emulator that is
// not part of Musashi's public interface.
//
// The lockstep bench (sim/lockstep/lockstep.cpp) runs the RTL core and Musashi side by side, and when the RTL takes an
// interrupt, Musashi has to take the same interrupt at the same instruction boundary. Musashi's public call for running
// code, m68k_execute(1), would first take the pending interrupt and then also run the first instruction of the
// interrupt handler, and the RTL has not retired that instruction yet at that moment. The comparison would then be one
// instruction ahead on the Musashi side. The internal function m68ki_check_interrupts() only takes the interrupt (it
// builds the exception frame and jumps to the handler) and runs nothing else, which is the behavior the bench needs.
// The function is declared inside Musashi's private header m68kcpu.h, so a small wrapper compiled together with
// Musashi (tools/lockstep.sh compiles this file with the same options as Musashi) makes it callable from the bench.
// Musashi itself stays unchanged in third_party/musashi.
// m68kcpu.h is Musashi's internal header; it declares m68ki_check_interrupts() and the register macros.
#include "m68kcpu.h"
// Wrapper called from the bench as m68k_glue_check_irq(): take the pending interrupt now, run no instruction.
void m68k_glue_check_irq(void) { m68ki_check_interrupts(); }
