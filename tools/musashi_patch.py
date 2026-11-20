#!/usr/bin/env python3
# Copyright 2026 Franz Schoning
# Project: https://github.com/Schoning-Labs/pipe68k
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Makes a patched copy of Musashi's opcode source file (m68k_in.c) for the lockstep bench. Musashi is an emulator for the whole 68000
# family, written in C and used here as the reference CPU. It is very good, but in a few places it does not do what a real 68020 does,
# and the bench would then report a "failure" that is really a flaw of the reference. Each patch below makes Musashi do what a real
# 68020 does, using WinUAE's cputester data (which comes from an emulator checked against real chips) as the authority.
#
# usage: musashi_patch.py IN OUT      IN is third_party/musashi/m68k_in.c and OUT is build/musashi_gen/m68k_in.c (tools/lockstep.sh calls it).
# The submodule itself stays unchanged. How the file works: m68k_in.c is not plain C. It is a table of handlers that start with a
# header line M68KMAKE_OP(name, size, operand 1, operand 2) and a body in braces, and Musashi's generator m68kmake expands every header
# into the real opcode functions for all addressing mode combinations. The script finds handlers by header with a regular
# expression, changes their text with plain string replacement, and asserts that every pattern was found, so that a new Musashi
# version that moved the code makes the script stop and not silently skip a patch.
# The patches, in file order:
#   BFTST on memory       the field test uses only the field bits of the last byte
#   BFFFO Dn              with a register offset the result is the full offset plus the bit position, not reduced modulo 32
#   ABCD, SBCD, NBCD      WinUAE's arithmetic, which gives results for operands that are not valid BCD, N = bit 7 of the result, V = 0
#   PACK, UNPK            memory forms: one word read (PACK) or one word write (UNPK), source step before destination step, A7 by 2
#   MOVEM regs,-(An)      with An in the register list the stored value is the initial An minus the operand size
#   MULU.L, MULS.L        64-bit result with Dh = Dl: the high half stays in the register
#   cpSAVE, cpRESTORE     privilege violation in user mode (and the FPU hooks below)
#   MOVEC CACR,Rn         the bits C and CE read as zero
#   F2xx and F3xx opcodes the FPU of the bench (fpu_ref.c) is called for the 68020, which Musashi only did for the 68040
import re, sys
# re = regular expressions, sys = the command line. The whole source is read as one string; latin-1 maps every byte to one character,
# so no byte is changed by the round trip.
src = open(sys.argv[1], 'rb').read().decode('latin-1')

# Generator over all handlers whose header matches name_re, from the header line to the closing brace at the start of a line.
def handlers(name_re):
    # One handler: M68KMAKE_OP( name ) at line start, then the opening brace line, then everything up to the first line that starts with }.
    """yield (start, end) of every M68KMAKE_OP handler whose header matches name_re (header line to the closing brace)"""
    for m in re.finditer(r'^M68KMAKE_OP\(' + name_re + r'\)\r?\n\{.*?^\}', src, re.S | re.M):
        yield m.start(), m.end()

# rewrite(name_re, fn): replace every matching handler with fn(its text); fail if none matched. Returns how many were changed.
def rewrite(name_re, fn):
    global src
    out, pos = [], 0
    # Collect the pieces: untouched text before the handler, the new handler text, and the position after it.
    n = 0
    for s, e in list(handlers(name_re)):
        out.append(src[pos:s]); out.append(fn(src[s:e])); pos = e; n += 1
    out.append(src[pos:]); src = ''.join(out)
    # A pattern that matches nothing means Musashi's source changed: stop.
    assert n > 0, name_re
    return n

# sub1(text, a, b): replace the first occurrence of a by b and make sure a is there.
def sub1(text, a, b):
    assert a in text, a
    return text.replace(a, b, 1)

# BFTST on memory: Musashi tests the whole fifth byte of the field with the mask of the first byte. A bit field can start in the
# middle of a byte, so the mask has to move to the left by (8 - offset) to select only the bits that belong to the field.
a = 'mask_byte = MASK_OUT_ABOVE_8(mask_base);'
assert a in src
src = src.replace(a, 'mask_byte = MASK_OUT_ABOVE_8(mask_base << (8 - offset));')

# BFFFO with a data register as operand: the offset register is a full 32 bit signed number. The bit position inside a register
# field is taken modulo 32, but the 68020 returns offset plus position, so the multiple of 32 that was cut off is added back.
def bfffo(t):
    # The three string replacements declare a helper variable, save the full offset before it is reduced, and add the cut off part back.
    t = sub1(t, 'uint bit;', 'uint bit;\n\t\tuint ofull_;')
    t = sub1(t, 'offset &= 31;', 'ofull_ = offset; offset &= 31;')
    t = sub1(t, 'REG_D[(word2>>12)&7] = offset;', 'REG_D[(word2>>12)&7] = offset + (ofull_ - (ofull_ & 31));')
    return t
rewrite(r'bfffo, 32, \., d', bfffo)

# ABCD and SBCD (binary coded decimal add and subtract with extend): replace the arithmetic block of the handlers. Musashi gives
# a different result than the 68020 when an operand is not a valid BCD number (a nibble above 9), a case the cputester data covers
# with the values of the real chip. The new code follows WinUAE: compute low and high nibbles separately, correct by 6
# after a carry out of the low nibble and by 0x60 after a carry out of the high one, N = bit 7 of the result, V cleared.
abcd_old = re.compile(r'\tuint res = LOW_NIBBLE\(src\) \+ LOW_NIBBLE\(dst\) \+ XFLAG_AS_1\(\);.*?\tFLAG_Z \|= res;\r?\n', re.S)
abcd_new = '''\tuint res, cf_;
\t{ uint lo_ = LOW_NIBBLE(src) + LOW_NIBBLE(dst) + XFLAG_AS_1(); uint hi_ = HIGH_NIBBLE(src) + HIGH_NIBBLE(dst);
\t  res = hi_ + lo_; if(lo_ > 9) res += 6; cf_ = (res & 0x3f0) > 0x90; if(cf_) res += 0x60; }
\tFLAG_X = FLAG_C = cf_ << 8; FLAG_V = VFLAG_CLEAR; FLAG_N = NFLAG_8(res);
\tres = MASK_OUT_ABOVE_8(res);
\tFLAG_Z |= res;
'''
# The same for SBCD, with borrows in place of carries.
sbcd_old = re.compile(r'\tuint res = LOW_NIBBLE\(dst\) - LOW_NIBBLE\(src\) - XFLAG_AS_1\(\);.*?\tFLAG_Z \|= res;\r?\n', re.S)
sbcd_new = '''\tuint res, cf_;
\t{ uint lo_ = LOW_NIBBLE(dst) - LOW_NIBBLE(src) - XFLAG_AS_1(); uint hi_ = (dst & 0xf0) - (src & 0xf0); uint bcd_ = 0;
\t  res = hi_ + lo_; if(lo_ & 0xf0) { res -= 6; bcd_ = 6; }
\t  if((((dst & 0xff) - (src & 0xff) - XFLAG_AS_1()) & 0x100) > 0xff) res -= 0x60;
\t  cf_ = ((((dst & 0xff) - (src & 0xff) - bcd_ - XFLAG_AS_1()) & 0x300) > 0xff); }
\tFLAG_X = FLAG_C = cf_ << 8; FLAG_V = VFLAG_CLEAR; FLAG_N = NFLAG_8(res);
\tres = MASK_OUT_ABOVE_8(res);
\tFLAG_Z |= res;
'''
# bcd(old, new): make a function that replaces the first match of the pattern old inside a handler text with new.
def bcd(old, new):
    def f(t):
        assert old.search(t)
        return old.sub(lambda m: new, t, count=1)
    return f
# Apply to every register-to-register (rr) and memory-to-memory (mm) form, and to the three forms with A7 as one of the operands.
rewrite(r'abcd, 8, (rr|mm), \.', bcd(abcd_old, abcd_new))
rewrite(r'abcd, 8, mm, (ax7|ay7|axy7)', bcd(abcd_old, abcd_new))
rewrite(r'sbcd, 8, (rr|mm), \.', bcd(sbcd_old, sbcd_new))
rewrite(r'sbcd, 8, mm, (ax7|ay7|axy7)', bcd(sbcd_old, sbcd_new))

# NBCD (negate decimal with extend): the whole arithmetic is replaced the same way, and the result is always written back.
nb_arith = '''\tuint res, cf_;
\t{ uint lo_ = (0 - (dst & 0xf) - XFLAG_AS_1()); uint hi_ = (0 - (dst & 0xf0));
\t  if(lo_ > 9) lo_ -= 6;
\t  res = hi_ + lo_; cf_ = (res & 0x1f0) > 0x90; if(cf_) res -= 0x60; }
\tres = MASK_OUT_ABOVE_8(res);
\tFLAG_N = NFLAG_8(res); FLAG_V = VFLAG_CLEAR; FLAG_C = FLAG_X = cf_ << 8; FLAG_Z |= res;
'''
# The register form keeps Musashi's text up to the first "uint res" and appends the new arithmetic and the register write.
def nbcd_d(t):
    return t[:t.index('\tuint res')] + nb_arith + '\t*r_dst = MASK_OUT_BELOW_8(*r_dst) | res;\n}'
# The memory form appends the arithmetic and a byte write of the result to the effective address.
def nbcd_m(t):
    return t[:t.index('\tuint res')] + nb_arith + '\tm68ki_write_8(ea, res);\n}'
rewrite(r'nbcd, 8, \., d', nbcd_d)
rewrite(r'nbcd, 8, \., \.', nbcd_m)

# PACK (adjust and pack two unpacked BCD digits into one byte): the memory form reads one 16-bit word at the source address minus 2
# (not two separate bytes), adds the adjustment word, and writes one byte. For A7 the step is 2 (the stack pointer stays even). The source
# pre-decrement happens before the destination pre-decrement.
def pack_h(srca7, dsta7):
    def f(t):
        head = t[:t.index('\t\t/* Note') if '\t\t/* Note' in t else t.index('\t\tuint ea_src')]
        sa = 'REG_A[7] -= 2' if srca7 else 'AY -= 2'
        da = 'EA_A7_PD_8()' if dsta7 else 'EA_AX_PD_8()'
        return head + '\t\tuint ea_src = (%s);\n\t\tuint src = m68ki_read_16(ea_src) + OPER_I_16();\n\n\t\tm68ki_write_8(%s, ((src >> 4) & 0x00f0) | (src & 0x000f));\n\t\treturn;\n\t}\n\tm68ki_exception_illegal();\n}' % (sa, da)
    return f
rewrite(r'pack, 16, mm, ax7', pack_h(False, True))
rewrite(r'pack, 16, mm, ay7', pack_h(True, False))
rewrite(r'pack, 16, mm, axy7', pack_h(True, True))
rewrite(r'pack, 16, mm, \.', pack_h(False, False))
# UNPK (the reverse): read one byte, spread its digits into a word, add the adjustment and write one word at the destination minus 2.
def unpk_h(srca7, dsta7):
    def f(t):
        head = t[:t.index('\t\t/* Note') if '\t\t/* Note' in t else t.index('\t\tuint src')]
        sr = 'OPER_A7_PD_8()' if srca7 else 'OPER_AY_PD_8()'
        dd = 'REG_A[7] -= 2' if dsta7 else 'AX -= 2'
        return head + '\t\tuint src = %s;\n\t\tuint ea_dst;\n\n\t\tsrc = (((src << 4) & 0x0f00) | (src & 0x000f)) + OPER_I_16();\n\t\tea_dst = (%s);\n\t\tm68ki_write_16(ea_dst, src & 0xffff);\n\t\treturn;\n\t}\n\tm68ki_exception_illegal();\n}' % (sr, dd)
    return f
rewrite(r'unpk, 16, mm, ax7', unpk_h(False, True))
rewrite(r'unpk, 16, mm, ay7', unpk_h(True, False))
rewrite(r'unpk, 16, mm, axy7', unpk_h(True, True))
rewrite(r'unpk, 16, mm, \.', unpk_h(False, False))

# MOVEM registers to memory with pre-decrement: on a 68020 an address register in the list is stored with its initial value minus
# the operand size (2 or 4), and not with the final value. 15-i is the register index in the pre-decrement mask order, 8 + (IR & 7)
# is the base register, and ea0_ is the initial address.
def movem_pd(size):
    def f(t):
        t = sub1(t, 'uint ea = AY;', 'uint ea = AY;\n\tuint ea0_ = AY;')
        return t.replace('REG_DA[15-i]', '((15-i) == (8 + (REG_IR & 7)) ? ea0_ - %d : REG_DA[15-i])' % size)
    return f
rewrite(r'movem, 16, re, pd', movem_pd(2))
rewrite(r'movem, 32, re, pd', movem_pd(4))

# MULU.L and MULS.L with a 64-bit result and the same register for the high and low half: the 68020 writes the low half first and the
# high half second, so the high half stays. The text is converted to LF line ends first so that multi-line patterns match; it is
# converted back to CR LF before writing.
src = src.replace('\r\n', '\n')
# The handlers appear with two indentation depths, one for each form; swap the order of the two register writes in both.
for ind in ('\t\t\t', '\t\t'):
    old = ind + 'REG_D[word2 & 7] = (res >> 32);\n' + ind + 'REG_D[(word2 >> 12) & 7] = MASK_OUT_ABOVE_32(res);'
    new = ind + 'REG_D[(word2 >> 12) & 7] = MASK_OUT_ABOVE_32(res);\n' + ind + 'REG_D[word2 & 7] = (res >> 32);'
    assert old in src
    src = src.replace(old, new)
# The third form keeps the intermediate values in the variables hi and lo; swap those writes too.
old = '\t\t\tREG_D[word2 & 7] = hi;\n\t\t\tREG_D[(word2 >> 12) & 7] = lo;'
assert old in src
src = src.replace(old, '\t\t\tREG_D[(word2 >> 12) & 7] = lo;\n\t\t\tREG_D[word2 & 7] = hi;')

# F-line (coprocessor) instructions: cpSAVE and cpRESTORE are privileged. In user mode they must raise a privilege violation, in
# supervisor mode they are legal for the coprocessor. The condition below checks bits 8-6 (100 = save, 101 = restore) and the
# effective address modes that are legal for each, and the FPU hooks reuse the same check.
cp_chk = ('\tif(!FLAG_S && (((REG_IR & 0x1c0) == 0x100 && (((REG_IR >> 3) & 7) == 2 || ((REG_IR >> 3) & 7) == 4 || ((REG_IR >> 3) & 7) == 5 || ((REG_IR >> 3) & 7) == 6 || (((REG_IR >> 3) & 7) == 7 && (REG_IR & 7) <= 1))) ||\n'
          '\t               ((REG_IR & 0x1c0) == 0x140 && (((REG_IR >> 3) & 7) == 2 || ((REG_IR >> 3) & 7) == 3 || ((REG_IR >> 3) & 7) == 5 || ((REG_IR >> 3) & 7) == 6 || (((REG_IR >> 3) & 7) == 7 && (REG_IR & 7) <= 3)))))\n'
          '\t{\n\t\tm68ki_exception_privilege_violation();\n\t\treturn;\n\t}\n')
# Insert the check before the last statement of a handler (the line F exception call).
def fline(t):
    i = t.rindex('\tm68ki_exception_1111();')
    return t[:i] + cp_chk + t[i:]
# Add it to the generic line F handler and to the 040 FPU handlers.
rewrite(r'1111, 0, \., \.', fline)
rewrite(r'040fpu0, 32, \., \.', fline)
rewrite(r'040fpu1, 32, \., \.', fline)
rewrite(r'pmmu, 32, \., \.', fline)

# The FPU for the 68020: Musashi only called its FPU code for the 68040 handlers. The script makes the two handlers (fpu0 for the
# general instructions, fpu1 for FSAVE and FRESTORE) call m68040_fpu_op0() and m68040_fpu_op1(), which sim/lockstep/fpu_ref.c defines.
# The privilege check above stays in front of them.
def fpuop(n):
    def f(t):
        i = t.index('{') + 1
        return t[:i] + chr(10) + cp_chk + chr(9) + 'm68040_fpu_op%d();' % n + chr(10) + '}'
    return f
# Patch the handlers for the F2xx and F3xx opcodes (coprocessor id 1).
rewrite(r'040fpu0, 32, \., \.', fpuop(0))
rewrite(r'040fpu1, 32, \., \.', fpuop(1))
# FDBcc and FTRAPcc have a more specific opcode pattern than the general handler and ran as no-ops in Musashi. For coprocessor id 1
# (the FPU) they call the FPU; every other coprocessor id takes the line F exception, as on a 68020 without those coprocessors.
# Replace the start of a handler (after the opening brace) with the id test and the line F fallback.
def cpop(t):
    i = t.index('{') + 1
    return t[:i] + chr(10) + chr(9) + 'if((REG_IR & 0x0e00) == 0x0200) { m68040_fpu_op0(); return; }' + chr(10) + chr(9) + 'm68ki_exception_1111();' + chr(10) + '}'
# Apply to the five coprocessor handlers.
for n in ('cpdbcc', 'cptrapcc', 'cpgen', 'cpscc', 'cpbcc'):
    rewrite(n + r', 32, \., \.', cpop)

# MOVEC CACR,Rn: a real 68020 returns zeros in the cache clear bits (C and CE are write-only strobes); keep only the two enable bits.
a = 'REG_DA[(word2 >> 12) & 15] = REG_CACR;'
assert a in src
src = src.replace(a, 'REG_DA[(word2 >> 12) & 15] = REG_CACR & 3;')

# Write the CR LF line ends back (Musashi's original file has them) and save the patched file.
src = src.replace('\r\n', '\n').replace('\n', '\r\n')   # the original has CRLF line ends
open(sys.argv[2], 'wb').write(src.encode('latin-1'))
