// -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
// vim: set ts=8 sts=4 et sw=4 tw=99:
//
// Copyright 2013, ARM Limited
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//   * Neither the name of ARM Limited nor the names of its contributors may be
//     used to endorse or promote products derived from this software without
//     specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "jit/arm64/vixl/Assembler-vixl.h"

#include <cmath>

namespace js {
namespace jit {

// CPURegList utilities.
CPURegister
CPURegList::PopLowestIndex()
{
    if (IsEmpty())
      return NoCPUReg;

    int index = CountTrailingZeros(list_, kRegListSizeInBits);
    VIXL_ASSERT((1 << index) & list_);
    Remove(index);
    return CPURegister(index, size_, type_);
}

CPURegister
CPURegList::PopHighestIndex()
{
    VIXL_ASSERT(IsValid());
    if (IsEmpty())
      return NoCPUReg;

    int index = CountLeadingZeros(list_, kRegListSizeInBits);
    index = kRegListSizeInBits - 1 - index;
    VIXL_ASSERT((1 << index) & list_);
    Remove(index);
    return CPURegister(index, size_, type_);
}

bool
CPURegList::IsValid() const
{
    if ((type_ == CPURegister::kARMRegister) || (type_ == CPURegister::kARMFPRegister)) {
        bool is_valid = true;
        // Try to create a CPURegister for each element in the list.
        for (int i = 0; i < kRegListSizeInBits; i++) {
            if (((list_ >> i) & 1) != 0)
                is_valid &= CPURegister(i, size_, type_).IsValid();
        }
        return is_valid;
    }
    
    if (type_ == CPURegister::kNoRegister) {
      // We can't use IsEmpty here because that asserts IsValid().
      return list_ == 0;
    }

    return false;
}

void
CPURegList::RemoveCalleeSaved()
{
    if (type() == CPURegister::kARMRegister) {
        Remove(GetCalleeSaved(RegisterSizeInBits()));
    } else if (type() == CPURegister::kARMFPRegister) {
        Remove(GetCalleeSavedFP(RegisterSizeInBits()));
    } else {
        VIXL_ASSERT(type() == CPURegister::kNoRegister);
        VIXL_ASSERT(IsEmpty());
        // The list must already be empty, so do nothing.
    }
}

CPURegList
CPURegList::GetCalleeSaved(unsigned size)
{
    return CPURegList(CPURegister::kARMRegister, size, 19, 29);
}

CPURegList
CPURegList::GetCalleeSavedFP(unsigned size)
{
    return CPURegList(CPURegister::kARMFPRegister, size, 8, 15);
}

CPURegList
CPURegList::GetCallerSaved(unsigned size)
{
    // ARMRegisters x0-x18 and lr (x30) are caller-saved.
    CPURegList list = CPURegList(CPURegister::kARMRegister, size, 0, 18);
    list.Combine(lr);
    return list;
}

CPURegList
CPURegList::GetCallerSavedFP(unsigned size)
{
    // ARMRegisters d0-d7 and d16-d31 are caller-saved.
    CPURegList list = CPURegList(CPURegister::kARMFPRegister, size, 0, 7);
    list.Combine(CPURegList(CPURegister::kARMFPRegister, size, 16, 31));
    return list;
}

const CPURegList kCalleeSaved = CPURegList::GetCalleeSaved();
const CPURegList kCalleeSavedFP = CPURegList::GetCalleeSavedFP();
const CPURegList kCallerSaved = CPURegList::GetCallerSaved();
const CPURegList kCallerSavedFP = CPURegList::GetCallerSavedFP();

// Registers.
#define WREG(n) w##n,
const ARMRegister ARMRegister::wregisters[] = {
REGISTER_CODE_LIST(WREG)
};
#undef WREG

#define XREG(n) x##n,
const ARMRegister ARMRegister::xregisters[] = {
REGISTER_CODE_LIST(XREG)
};
#undef XREG

#define SREG(n) s##n,
const ARMFPRegister ARMFPRegister::sregisters[] = {
REGISTER_CODE_LIST(SREG)
};
#undef SREG

#define DREG(n) d##n,
const ARMFPRegister ARMFPRegister::dregisters[] = {
REGISTER_CODE_LIST(DREG)
};
#undef DREG

const ARMRegister&
ARMRegister::WRegFromCode(unsigned code)
{
    if (code == kSPRegInternalCode)
        return wsp;
    VIXL_ASSERT(code < kNumberOfRegisters);
    return wregisters[code];
}

const ARMRegister&
ARMRegister::XRegFromCode(unsigned code)
{
    if (code == kSPRegInternalCode)
      return sp;
    VIXL_ASSERT(code < kNumberOfRegisters);
    return xregisters[code];
}

const ARMFPRegister&
ARMFPRegister::SRegFromCode(unsigned code)
{
    VIXL_ASSERT(code < kNumberOfFloatRegisters);
    return sregisters[code];
}

const ARMFPRegister&
ARMFPRegister::DRegFromCode(unsigned code)
{
    VIXL_ASSERT(code < kNumberOfFloatRegisters);
    return dregisters[code];
}

const ARMRegister&
CPURegister::W() const
{
    VIXL_ASSERT(IsValidRegister());
    return ARMRegister::WRegFromCode(code_);
}

const ARMRegister&
CPURegister::X() const
{
    VIXL_ASSERT(IsValidRegister());
    return ARMRegister::XRegFromCode(code_);
}

const ARMFPRegister&
CPURegister::S() const
{
    VIXL_ASSERT(IsValidARMFPRegister());
    return ARMFPRegister::SRegFromCode(code_);
}

const ARMFPRegister&
CPURegister::D() const
{
    VIXL_ASSERT(IsValidARMFPRegister());
    return ARMFPRegister::DRegFromCode(code_);

}

// Operand.
Operand::Operand(int64_t immediate)
  : immediate_(immediate),
    reg_(NoReg),
    shift_(NO_SHIFT),
    extend_(NO_EXTEND),
    shift_amount_(0)
{
}

Operand::Operand(ARMRegister reg, Shift shift, unsigned shift_amount)
  : reg_(reg),
    shift_(shift),
    extend_(NO_EXTEND),
    shift_amount_(shift_amount)
{
    VIXL_ASSERT(reg.Is64Bits() || (shift_amount < kWRegSize));
    VIXL_ASSERT(reg.Is32Bits() || (shift_amount < kXRegSize));
    VIXL_ASSERT(!reg.IsSP());
}

Operand::Operand(ARMRegister reg, Extend extend, unsigned shift_amount)
  : reg_(reg),
    shift_(NO_SHIFT),
    extend_(extend),
    shift_amount_(shift_amount)
{
    VIXL_ASSERT(reg.IsValid());
    VIXL_ASSERT(shift_amount <= 4);
    VIXL_ASSERT(!reg.IsSP());

    // Extend modes SXTX and UXTX require a 64-bit register.
    VIXL_ASSERT(reg.Is64Bits() || ((extend != SXTX) && (extend != UXTX)));
}

bool
Operand::IsImmediate() const
{
    return reg_.Is(NoReg);
}

bool
Operand::IsShiftedRegister() const
{
    return reg_.IsValid() && (shift_ != NO_SHIFT);
}

bool
Operand::IsExtendedRegister() const
{
    return reg_.IsValid() && (extend_ != NO_EXTEND);
}

bool
Operand::IsZero() const
{
    if (IsImmediate())
        return immediate() == 0;
    return reg().IsZero();
}

Operand
Operand::ToExtendedRegister() const
{
    VIXL_ASSERT(IsShiftedRegister());
    VIXL_ASSERT((shift_ == LSL) && (shift_amount_ <= 4));
    return Operand(reg_, reg_.Is64Bits() ? UXTX : UXTW, shift_amount_);
}

// MemOperand
MemOperand::MemOperand(ARMRegister base, ptrdiff_t offset, AddrMode addrmode)
  : base_(base), regoffset_(NoReg), offset_(offset), addrmode_(addrmode)
{
    VIXL_ASSERT(base.Is64Bits() && !base.IsZero());
}

MemOperand::MemOperand(ARMRegister base, ARMRegister regoffset,
                       Extend extend, unsigned shift_amount)
  : base_(base), regoffset_(regoffset), offset_(0), addrmode_(Offset),
    shift_(NO_SHIFT), extend_(extend), shift_amount_(shift_amount)
{
    VIXL_ASSERT(base.Is64Bits() && !base.IsZero());
    VIXL_ASSERT(!regoffset.IsSP());
    VIXL_ASSERT((extend == UXTW) || (extend == SXTW) || (extend == SXTX));

    // SXTX extend mode requires a 64-bit offset register.
    VIXL_ASSERT(regoffset.Is64Bits() || (extend != SXTX));
}

MemOperand::MemOperand(ARMRegister base, ARMRegister regoffset,
                       Shift shift, unsigned shift_amount)
  : base_(base), regoffset_(regoffset), offset_(0), addrmode_(Offset),
    shift_(shift), extend_(NO_EXTEND), shift_amount_(shift_amount)
{
    VIXL_ASSERT(base.Is64Bits() && !base.IsZero());
    VIXL_ASSERT(regoffset.Is64Bits() && !regoffset.IsSP());
    VIXL_ASSERT(shift == LSL);
}

MemOperand::MemOperand(ARMRegister base, const Operand& offset, AddrMode addrmode)
  : base_(base), regoffset_(NoReg), addrmode_(addrmode)
{
    VIXL_ASSERT(base.Is64Bits() && !base.IsZero());

    if (offset.IsImmediate()) {
        offset_ = offset.immediate();
    } else if (offset.IsShiftedRegister()) {
        VIXL_ASSERT(addrmode == Offset);

        regoffset_ = offset.reg();
        shift_= offset.shift();
        shift_amount_ = offset.shift_amount();

        extend_ = NO_EXTEND;
        offset_ = 0;

        // These assertions match those in the shifted-register constructor.
        VIXL_ASSERT(regoffset_.Is64Bits() && !regoffset_.IsSP());
        VIXL_ASSERT(shift_ == LSL);
    } else {
        VIXL_ASSERT(offset.IsExtendedRegister());
        VIXL_ASSERT(addrmode == Offset);

        regoffset_ = offset.reg();
        extend_ = offset.extend();
        shift_amount_ = offset.shift_amount();

        shift_= NO_SHIFT;
        offset_ = 0;

        // These assertions match those in the extended-register constructor.
        VIXL_ASSERT(!regoffset_.IsSP());
        VIXL_ASSERT((extend_ == UXTW) || (extend_ == SXTW) || (extend_ == SXTX));
        VIXL_ASSERT((regoffset_.Is64Bits() || (extend_ != SXTX)));
    }
}

bool
MemOperand::IsImmediateOffset() const
{
    return (addrmode_ == Offset) && regoffset_.Is(NoReg);
}

bool
MemOperand::IsRegisterOffset() const
{
    return (addrmode_ == Offset) && !regoffset_.Is(NoReg);
}

bool
MemOperand::IsPreIndex() const
{
    return addrmode_ == PreIndex;
}

bool
MemOperand::IsPostIndex() const
{
    return addrmode_ == PostIndex;
}

// Assembler
void
AssemblerVIXL::Reset()
{
#ifdef DEBUG
    // TODO: VIXL_ASSERT((pc_ >= buffer_) && (pc_ < buffer_ + buffer_size_));
    // TODO: memset(buffer_, 0, pc_ - buffer_);
    finalized_ = false;
#endif
    // TODO: pc_ = buffer_;
    pc_ = nullptr;
}

void
AssemblerVIXL::FinalizeCode()
{
#ifdef DEBUG
    finalized_ = true;
#endif
}

void
AssemblerVIXL::bind(Label* label)
{
#if 0
    label->is_bound_ = true;
    label->target_ = pc_;
    while (label->IsLinked()) {
      // Get the address of the following instruction in the chain.
      Instruction* next_link = label->link_->ImmPCOffsetTarget();
      // Update the instruction target.
      label->link_->SetImmPCOffsetTarget(label->target_);
      // Update the label's link.
      // If the offset of the branch we just updated was 0 (kEndOfChain) we are
      // done.
      label->link_ = (label->link_ != next_link) ? next_link : NULL;
    }
#else
    JS_ASSERT(0 && "bind");
#endif
}

int
AssemblerVIXL::UpdateAndGetByteOffsetTo(Label* label)
{
#if 0
    int offset;
    VIXL_STATIC_ASSERT(sizeof(*pc_) == 1);
    if (label->IsBound()) {
      offset = label->target() - pc_;
    } else if (label->IsLinked()) {
      offset = label->link() - pc_;
    } else {
      offset = Label::kEndOfChain;
    }
    label->set_link(pc_);
    return offset;
#else
    JS_ASSERT(0 && "UpdateAndGetByteOffsetTo");
    return 0;
#endif
}

// Code generation.
void
AssemblerVIXL::br(const ARMRegister& xn)
{
    VIXL_ASSERT(xn.Is64Bits());
    Emit(BR | Rn(xn));
}

void
AssemblerVIXL::blr(const ARMRegister& xn)
{
    VIXL_ASSERT(xn.Is64Bits());
    Emit(BLR | Rn(xn));
}

void
AssemblerVIXL::ret(const ARMRegister& xn)
{
    VIXL_ASSERT(xn.Is64Bits());
    Emit(RET | Rn(xn));
}

void
AssemblerVIXL::b(int imm26)
{
    Emit(B | ImmUncondBranch(imm26));
}

void
AssemblerVIXL::b(int imm19, Condition cond)
{
    Emit(B_cond | ImmCondBranch(imm19) | cond);
}

void
AssemblerVIXL::b(Label* label)
{
    b(UpdateAndGetInstructionOffsetTo(label));
}

void
AssemblerVIXL::b(Label* label, Condition cond)
{
    b(UpdateAndGetInstructionOffsetTo(label), cond);
}

void
AssemblerVIXL::bl(int imm26)
{
    Emit(BL | ImmUncondBranch(imm26));
}

void
AssemblerVIXL::bl(Label* label)
{
    bl(UpdateAndGetInstructionOffsetTo(label));
}

void
AssemblerVIXL::cbz(const ARMRegister& rt, int imm19)
{
    Emit(SF(rt) | CBZ | ImmCmpBranch(imm19) | Rt(rt));
}

void
AssemblerVIXL::cbz(const ARMRegister& rt, Label* label)
{
    cbz(rt, UpdateAndGetInstructionOffsetTo(label));
}

void
AssemblerVIXL::cbnz(const ARMRegister& rt, int imm19)
{
    Emit(SF(rt) | CBNZ | ImmCmpBranch(imm19) | Rt(rt));
}

void
AssemblerVIXL::cbnz(const ARMRegister& rt, Label* label)
{
    cbnz(rt, UpdateAndGetInstructionOffsetTo(label));
}

void
AssemblerVIXL::tbz(const ARMRegister& rt, unsigned bit_pos, int imm14)
{
    VIXL_ASSERT(rt.Is64Bits() || (rt.Is32Bits() && (bit_pos < kWRegSize)));
    Emit(TBZ | ImmTestBranchBit(bit_pos) | ImmTestBranch(imm14) | Rt(rt));
}

void
AssemblerVIXL::tbz(const ARMRegister& rt, unsigned bit_pos, Label* label)
{
    tbz(rt, bit_pos, UpdateAndGetInstructionOffsetTo(label));
}

void
AssemblerVIXL::tbnz(const ARMRegister& rt, unsigned bit_pos, int imm14)
{
    VIXL_ASSERT(rt.Is64Bits() || (rt.Is32Bits() && (bit_pos < kWRegSize)));
    Emit(TBNZ | ImmTestBranchBit(bit_pos) | ImmTestBranch(imm14) | Rt(rt));
}

void
AssemblerVIXL::tbnz(const ARMRegister& rt, unsigned bit_pos, Label* label)
{
    tbnz(rt, bit_pos, UpdateAndGetInstructionOffsetTo(label));
}

void
AssemblerVIXL::adr(const ARMRegister& rd, int imm21)
{
    VIXL_ASSERT(rd.Is64Bits());
    Emit(ADR | ImmPCRelAddress(imm21) | Rd(rd));
}

void
AssemblerVIXL::adr(const ARMRegister& rd, Label* label)
{
    adr(rd, UpdateAndGetByteOffsetTo(label));
}

void
AssemblerVIXL::add(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSub(rd, rn, operand, LeaveFlags, ADD);
}

void
AssemblerVIXL::adds(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSub(rd, rn, operand, SetFlags, ADD);
}

void
AssemblerVIXL::cmn(const ARMRegister& rn, const Operand& operand)
{
    ARMRegister zr = AppropriateZeroRegFor(rn);
    adds(zr, rn, operand);
}

void
AssemblerVIXL::sub(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSub(rd, rn, operand, LeaveFlags, SUB);
}

void
AssemblerVIXL::subs(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSub(rd, rn, operand, SetFlags, SUB);
}

void
AssemblerVIXL::cmp(const ARMRegister& rn, const Operand& operand)
{
    ARMRegister zr = AppropriateZeroRegFor(rn);
    subs(zr, rn, operand);
}

void
AssemblerVIXL::neg(const ARMRegister& rd, const Operand& operand)
{
    ARMRegister zr = AppropriateZeroRegFor(rd);
    sub(rd, zr, operand);
}

void
AssemblerVIXL::negs(const ARMRegister& rd, const Operand& operand)
{
    ARMRegister zr = AppropriateZeroRegFor(rd);
    subs(rd, zr, operand);
}

void
AssemblerVIXL::adc(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSubWithCarry(rd, rn, operand, LeaveFlags, ADC);
}

void
AssemblerVIXL::adcs(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSubWithCarry(rd, rn, operand, SetFlags, ADC);
}

void
AssemblerVIXL::sbc(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSubWithCarry(rd, rn, operand, LeaveFlags, SBC);
}

void
AssemblerVIXL::sbcs(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    AddSubWithCarry(rd, rn, operand, SetFlags, SBC);
}

void
AssemblerVIXL::ngc(const ARMRegister& rd, const Operand& operand)
{
    ARMRegister zr = AppropriateZeroRegFor(rd);
    sbc(rd, zr, operand);
}

void
AssemblerVIXL::ngcs(const ARMRegister& rd, const Operand& operand)
{
    ARMRegister zr = AppropriateZeroRegFor(rd);
    sbcs(rd, zr, operand);
}

// Logical instructions.
void
AssemblerVIXL::and_(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, AND);
}

void
AssemblerVIXL::ands(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, ANDS);
}

void
AssemblerVIXL::tst(const ARMRegister& rn, const Operand& operand)
{
    ands(AppropriateZeroRegFor(rn), rn, operand);
}

void
AssemblerVIXL::bic(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, BIC);
}

void
AssemblerVIXL::bics(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, BICS);
}

void
AssemblerVIXL::orr(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, ORR);
}

void
AssemblerVIXL::orn(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, ORN);
}

void
AssemblerVIXL::eor(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, EOR);
}

void
AssemblerVIXL::eon(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    Logical(rd, rn, operand, EON);
}

void
AssemblerVIXL::lslv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | LSLV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::lsrv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | LSRV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::asrv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | ASRV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::rorv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | RORV | Rm(rm) | Rn(rn) | Rd(rd));
}

// Bitfield operations.
void
AssemblerVIXL::bfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms)
{
    VIXL_ASSERT(rd.size() == rn.size());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | BFM | N |
         ImmR(immr, rd.size()) | ImmS(imms, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::sbfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms)
{
    VIXL_ASSERT(rd.Is64Bits() || rn.Is32Bits());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | SBFM | N |
         ImmR(immr, rd.size()) | ImmS(imms, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::ubfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms)
{
    VIXL_ASSERT(rd.size() == rn.size());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | UBFM | N |
         ImmR(immr, rd.size()) | ImmS(imms, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::extr(const ARMRegister& rd, const ARMRegister& rn,
                    const ARMRegister& rm, unsigned lsb)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | EXTR | N | Rm(rm) | ImmS(lsb, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::csel(const ARMRegister& rd, const ARMRegister& rn,
                    const ARMRegister& rm, Condition cond)
{
    ConditionalSelect(rd, rn, rm, cond, CSEL);
}

void
AssemblerVIXL::csinc(const ARMRegister& rd, const ARMRegister& rn,
                     const ARMRegister& rm, Condition cond)
{
    ConditionalSelect(rd, rn, rm, cond, CSINC);
}

void
AssemblerVIXL::csinv(const ARMRegister& rd, const ARMRegister& rn,
                     const ARMRegister& rm, Condition cond)
{
    ConditionalSelect(rd, rn, rm, cond, CSINV);
}

void
AssemblerVIXL::csneg(const ARMRegister& rd, const ARMRegister& rn,
                     const ARMRegister& rm, Condition cond)
{
    ConditionalSelect(rd, rn, rm, cond, CSNEG);
}

void
AssemblerVIXL::cset(const ARMRegister &rd, Condition cond)
{
    VIXL_ASSERT((cond != al) && (cond != nv));
    ARMRegister zr = AppropriateZeroRegFor(rd);
    csinc(rd, zr, zr, InvertCondition(cond));
}

void
AssemblerVIXL::csetm(const ARMRegister &rd, Condition cond)
{
    VIXL_ASSERT((cond != al) && (cond != nv));
    ARMRegister zr = AppropriateZeroRegFor(rd);
    csinv(rd, zr, zr, InvertCondition(cond));
}

void
AssemblerVIXL::cinc(const ARMRegister &rd, const ARMRegister &rn, Condition cond)
{
    VIXL_ASSERT((cond != al) && (cond != nv));
    csinc(rd, rn, rn, InvertCondition(cond));
}

void
AssemblerVIXL::cinv(const ARMRegister &rd, const ARMRegister &rn, Condition cond)
{
    VIXL_ASSERT((cond != al) && (cond != nv));
    csinv(rd, rn, rn, InvertCondition(cond));
}

void
AssemblerVIXL::cneg(const ARMRegister &rd, const ARMRegister &rn, Condition cond)
{
    VIXL_ASSERT((cond != al) && (cond != nv));
    csneg(rd, rn, rn, InvertCondition(cond));
}

void
AssemblerVIXL::ConditionalSelect(const ARMRegister& rd, const ARMRegister& rn,
                                 const ARMRegister& rm, Condition cond,
                                 ConditionalSelectOp op)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | op | Rm(rm) | Cond(cond) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::ccmn(const ARMRegister& rn, const Operand& operand, StatusFlags nzcv, Condition cond)
{
    ConditionalCompare(rn, operand, nzcv, cond, CCMN);
}

void
AssemblerVIXL::ccmp(const ARMRegister& rn, const Operand& operand, StatusFlags nzcv, Condition cond)
{
    ConditionalCompare(rn, operand, nzcv, cond, CCMP);
}

void
AssemblerVIXL::DataProcessing3Source(const ARMRegister& rd, const ARMRegister& rn,
                                     const ARMRegister& rm, const ARMRegister& ra,
                                     DataProcessing3SourceOp op)
{
    Emit(SF(rd) | op | Rm(rm) | Ra(ra) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::mul(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(AreSameSizeAndType(rd, rn, rm));
    DataProcessing3Source(rd, rn, rm, AppropriateZeroRegFor(rd), MADD);
}

void
AssemblerVIXL::madd(const ARMRegister& rd, const ARMRegister& rn,
                    const ARMRegister& rm, const ARMRegister& ra)
{
    DataProcessing3Source(rd, rn, rm, ra, MADD);
}

void
AssemblerVIXL::mneg(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(AreSameSizeAndType(rd, rn, rm));
    DataProcessing3Source(rd, rn, rm, AppropriateZeroRegFor(rd), MSUB);
}

void
AssemblerVIXL::msub(const ARMRegister& rd, const ARMRegister& rn,
                    const ARMRegister& rm, const ARMRegister& ra)
{
    DataProcessing3Source(rd, rn, rm, ra, MSUB);
}

void
AssemblerVIXL::umaddl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    VIXL_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    VIXL_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, UMADDL_x);
}

void
AssemblerVIXL::smaddl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    VIXL_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    VIXL_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, SMADDL_x);
}

void
AssemblerVIXL::umsubl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    VIXL_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    VIXL_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, UMSUBL_x);
}

void
AssemblerVIXL::smsubl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    VIXL_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    VIXL_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, SMSUBL_x);
}

void
AssemblerVIXL::smull(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.Is64Bits());
    VIXL_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, xzr, SMADDL_x);
}

void
AssemblerVIXL::sdiv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | SDIV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::smulh(const ARMRegister& xd, const ARMRegister& xn, const ARMRegister& xm)
{
    VIXL_ASSERT(xd.Is64Bits() && xn.Is64Bits() && xm.Is64Bits());
    DataProcessing3Source(xd, xn, xm, xzr, SMULH_x);
}

void
AssemblerVIXL::udiv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | UDIV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::rbit(const ARMRegister& rd, const ARMRegister& rn)
{
    DataProcessing1Source(rd, rn, RBIT);
}

void
AssemblerVIXL::rev16(const ARMRegister& rd, const ARMRegister& rn)
{
    DataProcessing1Source(rd, rn, REV16);
}

void
AssemblerVIXL::rev32(const ARMRegister& rd, const ARMRegister& rn)
{
    VIXL_ASSERT(rd.Is64Bits());
    DataProcessing1Source(rd, rn, REV);
}

void
AssemblerVIXL::rev(const ARMRegister& rd, const ARMRegister& rn)
{
    DataProcessing1Source(rd, rn, rd.Is64Bits() ? REV_x : REV_w);
}

void
AssemblerVIXL::clz(const ARMRegister& rd, const ARMRegister& rn)
{
    DataProcessing1Source(rd, rn, CLZ);
}

void
AssemblerVIXL::cls(const ARMRegister& rd, const ARMRegister& rn)
{
    DataProcessing1Source(rd, rn, CLS);
}

void
AssemblerVIXL::ldp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& src)
{
    LoadStorePair(rt, rt2, src, LoadPairOpFor(rt, rt2));
}

void
AssemblerVIXL::stp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& dst)
{
    LoadStorePair(rt, rt2, dst, StorePairOpFor(rt, rt2));
}

void
AssemblerVIXL::ldpsw(const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& src)
{
    VIXL_ASSERT(rt.Is64Bits());
    LoadStorePair(rt, rt2, src, LDPSW_x);
}

void
AssemblerVIXL::LoadStorePair(const CPURegister& rt, const CPURegister& rt2,
                             const MemOperand& addr, LoadStorePairOp op)
{
    // 'rt' and 'rt2' can only be aliased for stores.
    VIXL_ASSERT(((op & LoadStorePairLBit) == 0) || !rt.Is(rt2));
    VIXL_ASSERT(AreSameSizeAndType(rt, rt2));

    Instr memop = op | Rt(rt) | Rt2(rt2) | RnSP(addr.base()) |
                  ImmLSPair(addr.offset(), CalcLSPairDataSize(op));

    Instr addrmodeop;
    if (addr.IsImmediateOffset()) {
        addrmodeop = LoadStorePairOffsetFixed;
    } else {
        VIXL_ASSERT(addr.offset() != 0);
        if (addr.IsPreIndex()) {
            addrmodeop = LoadStorePairPreIndexFixed;
        } else {
            VIXL_ASSERT(addr.IsPostIndex());
            addrmodeop = LoadStorePairPostIndexFixed;
        }
    }
    Emit(addrmodeop | memop);
}

void
AssemblerVIXL::ldnp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& src)
{
    LoadStorePairNonTemporal(rt, rt2, src, LoadPairNonTemporalOpFor(rt, rt2));
}

void
AssemblerVIXL::stnp(const CPURegister& rt, const CPURegister& rt2, const MemOperand& dst)
{
    LoadStorePairNonTemporal(rt, rt2, dst, StorePairNonTemporalOpFor(rt, rt2));
}

void
AssemblerVIXL::LoadStorePairNonTemporal(const CPURegister& rt, const CPURegister& rt2,
                                        const MemOperand& addr, LoadStorePairNonTemporalOp op)
{
    VIXL_ASSERT(!rt.Is(rt2));
    VIXL_ASSERT(AreSameSizeAndType(rt, rt2));
    VIXL_ASSERT(addr.IsImmediateOffset());

    LSDataSize size = CalcLSPairDataSize(static_cast<LoadStorePairOp>(op & LoadStorePairMask));
    Emit(op | Rt(rt) | Rt2(rt2) | RnSP(addr.base()) | ImmLSPair(addr.offset(), size));
}

// Memory instructions.
void
AssemblerVIXL::ldrb(const ARMRegister& rt, const MemOperand& src)
{
    LoadStore(rt, src, LDRB_w);
}

void
AssemblerVIXL::strb(const ARMRegister& rt, const MemOperand& dst)
{
    LoadStore(rt, dst, STRB_w);
}

void
AssemblerVIXL::ldrsb(const ARMRegister& rt, const MemOperand& src)
{
    LoadStore(rt, src, rt.Is64Bits() ? LDRSB_x : LDRSB_w);
}

void
AssemblerVIXL::ldrh(const ARMRegister& rt, const MemOperand& src)
{
    LoadStore(rt, src, LDRH_w);
}

void
AssemblerVIXL::strh(const ARMRegister& rt, const MemOperand& dst)
{
    LoadStore(rt, dst, STRH_w);
}

void
AssemblerVIXL::ldrsh(const ARMRegister& rt, const MemOperand& src)
{
    LoadStore(rt, src, rt.Is64Bits() ? LDRSH_x : LDRSH_w);
}

void
AssemblerVIXL::ldr(const CPURegister& rt, const MemOperand& src)
{
    LoadStore(rt, src, LoadOpFor(rt));
}

void
AssemblerVIXL::str(const CPURegister& rt, const MemOperand& src)
{
    LoadStore(rt, src, StoreOpFor(rt));
}

void
AssemblerVIXL::ldrsw(const ARMRegister& rt, const MemOperand& src)
{
    VIXL_ASSERT(rt.Is64Bits());
    LoadStore(rt, src, LDRSW_x);
}

void
AssemblerVIXL::ldr(const ARMRegister& rt, uint64_t imm)
{
    LoadLiteral(rt, imm, rt.Is64Bits() ? LDR_x_lit : LDR_w_lit);
}

void
AssemblerVIXL::ldr(const ARMFPRegister& ft, double imm)
{
    VIXL_ASSERT(ft.Is64Bits());
    LoadLiteral(ft, double_to_rawbits(imm), LDR_d_lit);
}

void
AssemblerVIXL::ldr(const ARMFPRegister& ft, float imm)
{
    VIXL_ASSERT(ft.Is32Bits());
    LoadLiteral(ft, float_to_rawbits(imm), LDR_s_lit);
}

void
AssemblerVIXL::mov(const ARMRegister& rd, const ARMRegister& rm)
{
    // Moves involving the stack pointer are encoded as add immediate with
    // second operand of zero. Otherwise, orr with first operand zr is
    // used.
    if (rd.IsSP() || rm.IsSP()) {
        add(rd, rm, 0);
    } else {
        orr(rd, AppropriateZeroRegFor(rd), rm);
    }
}

void
AssemblerVIXL::mvn(const ARMRegister& rd, const Operand& operand)
{
    orn(rd, AppropriateZeroRegFor(rd), operand);
}

void
AssemblerVIXL::mrs(const ARMRegister& rt, SystemRegister sysreg)
{
    VIXL_ASSERT(rt.Is64Bits());
    Emit(MRS | ImmSystemRegister(sysreg) | Rt(rt));
}

void
AssemblerVIXL::msr(SystemRegister sysreg, const ARMRegister& rt)
{
    VIXL_ASSERT(rt.Is64Bits());
    Emit(MSR | Rt(rt) | ImmSystemRegister(sysreg));
}

void
AssemblerVIXL::hint(SystemHint code)
{
    Emit(HINT | ImmHint(code) | Rt(xzr));
}

void
AssemblerVIXL::dmb(BarrierDomain domain, BarrierType type)
{
    Emit(DMB | ImmBarrierDomain(domain) | ImmBarrierType(type));
}

void
AssemblerVIXL::dsb(BarrierDomain domain, BarrierType type)
{
    Emit(DSB | ImmBarrierDomain(domain) | ImmBarrierType(type));
}

void
AssemblerVIXL::isb()
{
    Emit(ISB | ImmBarrierDomain(FullSystem) | ImmBarrierType(BarrierAll));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, double imm)
{
    VIXL_ASSERT(fd.Is64Bits());
    VIXL_ASSERT(IsImmFP64(imm));
    Emit(FMOV_d_imm | Rd(fd) | ImmFP64(imm));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, float imm)
{
    VIXL_ASSERT(fd.Is32Bits());
    VIXL_ASSERT(IsImmFP32(imm));
    Emit(FMOV_s_imm | Rd(fd) | ImmFP32(imm));
}

void
AssemblerVIXL::fmov(const ARMRegister& rd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(rd.size() == fn.size());
    FPIntegerConvertOp op = rd.Is32Bits() ? FMOV_ws : FMOV_xd;
    Emit(op | Rd(rd) | Rn(fn));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, const ARMRegister& rn)
{
    VIXL_ASSERT(fd.size() == rn.size());
    FPIntegerConvertOp op = fd.Is32Bits() ? FMOV_sw : FMOV_dx;
    Emit(op | Rd(fd) | Rn(rn));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.size() == fn.size());
    Emit(FPType(fd) | FMOV | Rd(fd) | Rn(fn));
}

void
AssemblerVIXL::fadd(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FADD);
}

void
AssemblerVIXL::fsub(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FSUB);
}

void
AssemblerVIXL::fmul(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FMUL);
}

void
AssemblerVIXL::fmadd(const ARMFPRegister& fd, const ARMFPRegister& fn,
                     const ARMFPRegister& fm, const ARMFPRegister& fa)
{
    FPDataProcessing3Source(fd, fn, fm, fa, fd.Is32Bits() ? FMADD_s : FMADD_d);
}

void
AssemblerVIXL::fmsub(const ARMFPRegister& fd, const ARMFPRegister& fn,
                     const ARMFPRegister& fm, const ARMFPRegister& fa)
{
    FPDataProcessing3Source(fd, fn, fm, fa, fd.Is32Bits() ? FMSUB_s : FMSUB_d);
}

void
AssemblerVIXL::fnmadd(const ARMFPRegister& fd, const ARMFPRegister& fn,
                      const ARMFPRegister& fm, const ARMFPRegister& fa)
{
    FPDataProcessing3Source(fd, fn, fm, fa, fd.Is32Bits() ? FNMADD_s : FNMADD_d);
}

void
AssemblerVIXL::fnmsub(const ARMFPRegister& fd, const ARMFPRegister& fn,
                      const ARMFPRegister& fm, const ARMFPRegister& fa)
{
    FPDataProcessing3Source(fd, fn, fm, fa, fd.Is32Bits() ? FNMSUB_s : FNMSUB_d);
}

void
AssemblerVIXL::fdiv(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FDIV);
}

void
AssemblerVIXL::fmax(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FMAX);
}

void
AssemblerVIXL::fmaxnm(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FMAXNM);
}

void
AssemblerVIXL::fmin(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FMIN);
}

void
AssemblerVIXL::fminnm(const ARMFPRegister& fd, const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    FPDataProcessing2Source(fd, fn, fm, FMINNM);
}

void
AssemblerVIXL::fabs(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FABS);
}

void
AssemblerVIXL::fneg(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FNEG);
}

void
AssemblerVIXL::fsqrt(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FSQRT);
}

void
AssemblerVIXL::frinta(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FRINTA);
}

void
AssemblerVIXL::frintm(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FRINTM);
}

void
AssemblerVIXL::frintn(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FRINTN);
}

void
AssemblerVIXL::frintz(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    VIXL_ASSERT(fd.SizeInBits() == fn.SizeInBits());
    FPDataProcessing1Source(fd, fn, FRINTZ);
}

void
AssemblerVIXL::fcmp(const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    VIXL_ASSERT(fn.size() == fm.size());
    Emit(FPType(fn) | FCMP | Rm(fm) | Rn(fn));
}

void
AssemblerVIXL::fcmp(const ARMFPRegister& fn, double value)
{
    USEARG(value);
    // Although the fcmp instruction can strictly only take an immediate value of
    // +0.0, we don't need to check for -0.0 because the sign of 0.0 doesn't
    // affect the result of the comparison.
    VIXL_ASSERT(value == 0.0);
    Emit(FPType(fn) | FCMP_zero | Rn(fn));
}

void
AssemblerVIXL::fccmp(const ARMFPRegister& fn, const ARMFPRegister& fm,
                     StatusFlags nzcv, Condition cond)
{
    VIXL_ASSERT(fn.size() == fm.size());
    Emit(FPType(fn) | FCCMP | Rm(fm) | Cond(cond) | Rn(fn) | Nzcv(nzcv));
}

void
AssemblerVIXL::fcsel(const ARMFPRegister& fd, const ARMFPRegister& fn,
                     const ARMFPRegister& fm, Condition cond)
{
    VIXL_ASSERT(fd.size() == fn.size());
    VIXL_ASSERT(fd.size() == fm.size());
    Emit(FPType(fd) | FCSEL | Rm(fm) | Cond(cond) | Rn(fn) | Rd(fd));
}

void
AssemblerVIXL::FPConvertToInt(const ARMRegister& rd, const ARMFPRegister& fn, FPIntegerConvertOp op)
{
    Emit(SF(rd) | FPType(fn) | op | Rn(fn) | Rd(rd));
}

void
AssemblerVIXL::fcvt(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    if (fd.Is64Bits()) {
      // Convert float to double.
      VIXL_ASSERT(fn.Is32Bits());
      FPDataProcessing1Source(fd, fn, FCVT_ds);
    } else {
      // Convert double to float.
      VIXL_ASSERT(fn.Is64Bits());
      FPDataProcessing1Source(fd, fn, FCVT_sd);
    }
}

void
AssemblerVIXL::fcvtau(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTAU);
}

void
AssemblerVIXL::fcvtas(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTAS);
}

void
AssemblerVIXL::fcvtmu(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTMU);
}

void
AssemblerVIXL::fcvtms(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTMS);
}

void
AssemblerVIXL::fcvtnu(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTNU);
}

void
AssemblerVIXL::fcvtns(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTNS);
}

void
AssemblerVIXL::fcvtzu(const ARMRegister& rd, const ARMFPRegister& fn)
{
    FPConvertToInt(rd, fn, FCVTZU);
}

void
AssemblerVIXL::fcvtzs(const ARMRegister& rd, const ARMFPRegister& fn)
{

    FPConvertToInt(rd, fn, FCVTZS);
}

void
AssemblerVIXL::scvtf(const ARMFPRegister& fd, const ARMRegister& rn, unsigned fbits)
{
    if (fbits == 0)
        Emit(SF(rn) | FPType(fd) | SCVTF | Rn(rn) | Rd(fd));
    else
        Emit(SF(rn) | FPType(fd) | SCVTF_fixed | FPScale(64 - fbits) | Rn(rn) | Rd(fd));
}

void
AssemblerVIXL::ucvtf(const ARMFPRegister& fd, const ARMRegister& rn, unsigned fbits)
{
    if (fbits == 0)
        Emit(SF(rn) | FPType(fd) | UCVTF | Rn(rn) | Rd(fd));
    else
        Emit(SF(rn) | FPType(fd) | UCVTF_fixed | FPScale(64 - fbits) | Rn(rn) | Rd(fd));
}

// Note:
// Below, a difference in case for the same letter indicates a
// negated bit.
// If b is 1, then B is 0.
Instr
AssemblerVIXL::ImmFP32(float imm)
{
    VIXL_ASSERT(IsImmFP32(imm));
    // bits: aBbb.bbbc.defg.h000.0000.0000.0000.0000
    uint32_t bits = float_to_rawbits(imm);
    // bit7: a000.0000
    uint32_t bit7 = ((bits >> 31) & 0x1) << 7;
    // bit6: 0b00.0000
    uint32_t bit6 = ((bits >> 29) & 0x1) << 6;
    // bit5_to_0: 00cd.efgh
    uint32_t bit5_to_0 = (bits >> 19) & 0x3f;

    return (bit7 | bit6 | bit5_to_0) << ImmFP_offset;
}

Instr
AssemblerVIXL::ImmFP64(double imm)
{
    VIXL_ASSERT(IsImmFP64(imm));
    // bits: aBbb.bbbb.bbcd.efgh.0000.0000.0000.0000
    //       0000.0000.0000.0000.0000.0000.0000.0000
    uint64_t bits = double_to_rawbits(imm);
    // bit7: a000.0000
    uint32_t bit7 = ((bits >> 63) & 0x1) << 7;
    // bit6: 0b00.0000
    uint32_t bit6 = ((bits >> 61) & 0x1) << 6;
    // bit5_to_0: 00cd.efgh
    uint32_t bit5_to_0 = (bits >> 48) & 0x3f;

    return (bit7 | bit6 | bit5_to_0) << ImmFP_offset;
}

// Code generation helpers.
void
AssemblerVIXL::MoveWide(const ARMRegister& rd, uint64_t imm, int shift, MoveWideImmediateOp mov_op)
{
    if (shift >= 0) {
        // Explicit shift specified.
        VIXL_ASSERT((shift == 0) || (shift == 16) ||
                    (shift == 32) || (shift == 48));
        VIXL_ASSERT(rd.Is64Bits() || (shift == 0) || (shift == 16));
        shift /= 16;
    } else {
        // Calculate a new immediate and shift combination to encode the immediate
        // argument.
        shift = 0;
        if ((imm & 0xffffffffffff0000) == 0) {
            // Nothing to do.
        } else if ((imm & 0xffffffff0000ffff) == 0) {
            imm >>= 16;
            shift = 1;
        } else if ((imm & 0xffff0000ffffffff) == 0) {
            VIXL_ASSERT(rd.Is64Bits());
            imm >>= 32;
            shift = 2;
        } else if ((imm & 0x0000ffffffffffff) == 0) {
            VIXL_ASSERT(rd.Is64Bits());
            imm >>= 48;
            shift = 3;
        }
    }

    VIXL_ASSERT(is_uint16(imm));

    Emit(SF(rd) | MoveWideImmediateFixed | mov_op |
         Rd(rd) | ImmMoveWide(imm) | ShiftMoveWide(shift));
}

void
AssemblerVIXL::AddSub(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand,
                      FlagsUpdate S, AddSubOp op)
{
    VIXL_ASSERT(rd.size() == rn.size());

    if (operand.IsImmediate()) {
        int64_t immediate = operand.immediate();
        VIXL_ASSERT(IsImmAddSub(immediate));
        Instr dest_reg = (S == SetFlags) ? Rd(rd) : RdSP(rd);
        Emit(SF(rd) | AddSubImmediateFixed | op | Flags(S) |
            ImmAddSub(immediate) | dest_reg | RnSP(rn));
    } else if (operand.IsShiftedRegister()) {
        VIXL_ASSERT(operand.reg().size() == rd.size());
        VIXL_ASSERT(operand.shift() != ROR);

        // For instructions of the form:
        //   add/sub   wsp, <Wn>, <Wm> [, LSL #0-3 ]
        //   add/sub   <Wd>, wsp, <Wm> [, LSL #0-3 ]
        //   add/sub   wsp, wsp, <Wm> [, LSL #0-3 ]
        //   adds/subs <Wd>, wsp, <Wm> [, LSL #0-3 ]
        // or their 64-bit register equivalents, convert the operand from shifted to
        // extended register mode, and emit an add/sub extended instruction.
        if (rn.IsSP() || rd.IsSP()) {
            VIXL_ASSERT(!(rd.IsSP() && (S == SetFlags)));
            DataProcExtendedRegister(rd, rn, operand.ToExtendedRegister(), S,
                                     AddSubExtendedFixed | op);
        } else {
            DataProcShiftedRegister(rd, rn, operand, S, AddSubShiftedFixed | op);
        }
    } else {
        VIXL_ASSERT(operand.IsExtendedRegister());
        DataProcExtendedRegister(rd, rn, operand, S, AddSubExtendedFixed | op);
    }
}

void
AssemblerVIXL::AddSubWithCarry(const ARMRegister& rd, const ARMRegister& rn,
                               const Operand& operand, FlagsUpdate S, AddSubWithCarryOp op)
{
    VIXL_ASSERT(rd.size() == rn.size());
    VIXL_ASSERT(rd.size() == operand.reg().size());
    VIXL_ASSERT(operand.IsShiftedRegister() && (operand.shift_amount() == 0));
    Emit(SF(rd) | op | Flags(S) | Rm(operand.reg()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::hlt(int code)
{
    VIXL_ASSERT(is_uint16(code));
    Emit(HLT | ImmException(code));
}

void
AssemblerVIXL::brk(int code)
{
    VIXL_ASSERT(is_uint16(code));
    Emit(BRK | ImmException(code));
}

void
AssemblerVIXL::Logical(const ARMRegister& rd, const ARMRegister& rn,
                       const Operand& operand, LogicalOp op)
{
    VIXL_ASSERT(rd.size() == rn.size());
    if (operand.IsImmediate()) {
        int64_t immediate = operand.immediate();
        unsigned reg_size = rd.size();

        VIXL_ASSERT(immediate != 0);
        VIXL_ASSERT(immediate != -1);
        VIXL_ASSERT(rd.Is64Bits() || is_uint32(immediate));

        // If the operation is NOT, invert the operation and immediate.
        if ((op & NOT) == NOT) {
            op = static_cast<LogicalOp>(op & ~NOT);
            immediate = rd.Is64Bits() ? ~immediate : (~immediate & kWRegMask);
        }

        unsigned n, imm_s, imm_r;
        if (IsImmLogical(immediate, reg_size, &n, &imm_s, &imm_r)) {
            // Immediate can be encoded in the instruction.
            LogicalImmediate(rd, rn, n, imm_s, imm_r, op);
        } else {
            // This case is handled in the macro assembler.
            VIXL_UNREACHABLE();
        }
    } else {
        VIXL_ASSERT(operand.IsShiftedRegister());
        VIXL_ASSERT(operand.reg().size() == rd.size());
        Instr dp_op = static_cast<Instr>(op | LogicalShiftedFixed);
        DataProcShiftedRegister(rd, rn, operand, LeaveFlags, dp_op);
    }
}

void
AssemblerVIXL::LogicalImmediate(const ARMRegister& rd, const ARMRegister& rn,
                                unsigned n, unsigned imm_s, unsigned imm_r, LogicalOp op)
{
    unsigned reg_size = rd.size();
    Instr dest_reg = (op == ANDS) ? Rd(rd) : RdSP(rd);
    Emit(SF(rd) | LogicalImmediateFixed | op | BitN(n, reg_size) |
         ImmSetBits(imm_s, reg_size) | ImmRotate(imm_r, reg_size) | dest_reg | Rn(rn));
}

void
AssemblerVIXL::ConditionalCompare(const ARMRegister& rn, const Operand& operand,
                                  StatusFlags nzcv, Condition cond, ConditionalCompareOp op)
{
    Instr ccmpop;
    if (operand.IsImmediate()) {
        int64_t immediate = operand.immediate();
        VIXL_ASSERT(IsImmConditionalCompare(immediate));
        ccmpop = ConditionalCompareImmediateFixed | op | ImmCondCmp(immediate);
    } else {
        VIXL_ASSERT(operand.IsShiftedRegister() && (operand.shift_amount() == 0));
        ccmpop = ConditionalCompareRegisterFixed | op | Rm(operand.reg());
    }
    Emit(SF(rn) | ccmpop | Cond(cond) | Rn(rn) | Nzcv(nzcv));
}

void
AssemblerVIXL::DataProcessing1Source(const ARMRegister& rd, const ARMRegister& rn,
                                     DataProcessing1SourceOp op)
{
    VIXL_ASSERT(rd.size() == rn.size());
    Emit(SF(rn) | op | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::FPDataProcessing1Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                       FPDataProcessing1SourceOp op)
{
    Emit(FPType(fn) | op | Rn(fn) | Rd(fd));
}

void
AssemblerVIXL::FPDataProcessing2Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                       const ARMFPRegister& fm, FPDataProcessing2SourceOp op)
{
    VIXL_ASSERT(fd.size() == fn.size());
    VIXL_ASSERT(fd.size() == fm.size());
    Emit(FPType(fd) | op | Rm(fm) | Rn(fn) | Rd(fd));
}

void
AssemblerVIXL::FPDataProcessing3Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                       const ARMFPRegister& fm, const ARMFPRegister& fa,
                                       FPDataProcessing3SourceOp op)
{
    VIXL_ASSERT(AreSameSizeAndType(fd, fn, fm, fa));
    Emit(FPType(fd) | op | Rm(fm) | Rn(fn) | Rd(fd) | Ra(fa));
}

void
AssemblerVIXL::EmitShift(const ARMRegister& rd, const ARMRegister& rn,
                         Shift shift, unsigned shift_amount)
{
    switch (shift) {
      case LSL:
        lsl(rd, rn, shift_amount);
        break;
      case LSR:
        lsr(rd, rn, shift_amount);
        break;
      case ASR:
        asr(rd, rn, shift_amount);
        break;
      case ROR:
        ror(rd, rn, shift_amount);
        break;
      default:
        VIXL_UNREACHABLE();
    }
}

void
AssemblerVIXL::EmitExtendShift(const ARMRegister& rd, const ARMRegister& rn,
                               Extend extend, unsigned left_shift)
{
    VIXL_ASSERT(rd.size() >= rn.size());
    unsigned reg_size = rd.size();
    // Use the correct size of register.
    ARMRegister rn_ = ARMRegister(rn.code(), rd.size());
    // Bits extracted are high_bit:0.
    unsigned high_bit = (8 << (extend & 0x3)) - 1;
    // Number of bits left in the result that are not introduced by the shift.
    unsigned non_shift_bits = (reg_size - left_shift) & (reg_size - 1);

    if ((non_shift_bits > high_bit) || (non_shift_bits == 0)) {
        switch (extend) {
          case UXTB:
          case UXTH:
          case UXTW: ubfm(rd, rn_, non_shift_bits, high_bit); break;
          case SXTB:
          case SXTH:
          case SXTW: sbfm(rd, rn_, non_shift_bits, high_bit); break;
          case UXTX:
          case SXTX: {
              VIXL_ASSERT(rn.size() == kXRegSize);
              // Nothing to extend. Just shift.
              lsl(rd, rn_, left_shift);
              break;
          }
          default: VIXL_UNREACHABLE();
        }
    } else {
        // No need to extend as the extended bits would be shifted away.
        lsl(rd, rn_, left_shift);
    }
}

void
AssemblerVIXL::DataProcShiftedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                       const Operand& operand, FlagsUpdate S, Instr op)
{
    VIXL_ASSERT(operand.IsShiftedRegister());
    VIXL_ASSERT(rn.Is64Bits() || (rn.Is32Bits() &&
                is_uint5(operand.shift_amount())));
    Emit(SF(rd) | op | Flags(S) |
         ShiftDP(operand.shift()) | ImmDPShift(operand.shift_amount()) |
         Rm(operand.reg()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::DataProcExtendedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                        const Operand& operand, FlagsUpdate S, Instr op)
{
    Instr dest_reg = (S == SetFlags) ? Rd(rd) : RdSP(rd);
    Emit(SF(rd) | op | Flags(S) | Rm(operand.reg()) |
         ExtendMode(operand.extend()) | ImmExtendShift(operand.shift_amount()) |
         dest_reg | RnSP(rn));
}

bool
AssemblerVIXL::IsImmAddSub(int64_t immediate)
{
    return is_uint12(immediate) || (is_uint12(immediate >> 12) && ((immediate & 0xfff) == 0));
}

void
AssemblerVIXL::LoadStore(const CPURegister& rt, const MemOperand& addr, LoadStoreOp op)
{
    Instr memop = op | Rt(rt) | RnSP(addr.base());
    ptrdiff_t offset = addr.offset();

    if (addr.IsImmediateOffset()) {
        LSDataSize size = CalcLSDataSize(op);
        if (IsImmLSScaled(offset, size)) {
            // Use the scaled addressing mode.
            Emit(LoadStoreUnsignedOffsetFixed | memop |
                ImmLSUnsigned(offset >> size));
        } else if (IsImmLSUnscaled(offset)) {
            // Use the unscaled addressing mode.
            Emit(LoadStoreUnscaledOffsetFixed | memop | ImmLS(offset));
        } else {
            // This case is handled in the macro assembler.
            VIXL_UNREACHABLE();
        }
    } else if (addr.IsRegisterOffset()) {
        Extend ext = addr.extend();
        Shift shift = addr.shift();
        unsigned shift_amount = addr.shift_amount();

        // LSL is encoded in the option field as UXTX.
        if (shift == LSL) {
            ext = UXTX;
        }

        // Shifts are encoded in one bit, indicating a left shift by the memory
        // access size.
        VIXL_ASSERT((shift_amount == 0) ||
                (shift_amount == static_cast<unsigned>(CalcLSDataSize(op))));
        Emit(LoadStoreRegisterOffsetFixed | memop | Rm(addr.regoffset()) |
             ExtendMode(ext) | ImmShiftLS((shift_amount > 0) ? 1 : 0));
    } else {
        if (IsImmLSUnscaled(offset)) {
            if (addr.IsPreIndex()) {
            Emit(LoadStorePreIndexFixed | memop | ImmLS(offset));
            } else {
            VIXL_ASSERT(addr.IsPostIndex());
            Emit(LoadStorePostIndexFixed | memop | ImmLS(offset));
            }
        } else {
            // This case is handled in the macro assembler.
            VIXL_UNREACHABLE();
        }
    }
}

bool
AssemblerVIXL::IsImmLSUnscaled(ptrdiff_t offset)
{
    return is_int9(offset);
}

bool
AssemblerVIXL::IsImmLSScaled(ptrdiff_t offset, LSDataSize size)
{
    bool offset_is_size_multiple = (((offset >> size) << size) == offset);
    return offset_is_size_multiple && is_uint12(offset >> size);
}

void
AssemblerVIXL::LoadLiteral(const CPURegister& rt, uint64_t imm, LoadLiteralOp op)
{
    VIXL_ASSERT(is_int32(imm) || is_uint32(imm) || (rt.Is64Bits()));

    //FIXME: BlockLiteralPoolScope scope(this);
    JS_ASSERT(0 && "LoadLiteral");
    RecordLiteral(imm, rt.SizeInBytes());
    Emit(op | ImmLLiteral(0) | Rt(rt));
}

// Test if a given value can be encoded in the immediate field of a logical
// instruction.
// If it can be encoded, the function returns true, and values pointed to by n,
// imm_s and imm_r are updated with immediates encoded in the format required
// by the corresponding fields in the logical instruction.
// If it can not be encoded, the function returns false, and the values pointed
// to by n, imm_s and imm_r are undefined.
bool
AssemblerVIXL::IsImmLogical(uint64_t value, unsigned width, unsigned* n,
                            unsigned* imm_s, unsigned* imm_r)
{
    VIXL_ASSERT((n != NULL) && (imm_s != NULL) && (imm_r != NULL));
    VIXL_ASSERT((width == kWRegSize) || (width == kXRegSize));

    // Logical immediates are encoded using parameters n, imm_s and imm_r using
    // the following table:
    //
    //  N   imms    immr    size        S             R
    //  1  ssssss  rrrrrr    64    UInt(ssssss)  UInt(rrrrrr)
    //  0  0sssss  xrrrrr    32    UInt(sssss)   UInt(rrrrr)
    //  0  10ssss  xxrrrr    16    UInt(ssss)    UInt(rrrr)
    //  0  110sss  xxxrrr     8    UInt(sss)     UInt(rrr)
    //  0  1110ss  xxxxrr     4    UInt(ss)      UInt(rr)
    //  0  11110s  xxxxxr     2    UInt(s)       UInt(r)
    // (s bits must not be all set)
    //
    // A pattern is constructed of size bits, where the least significant S+1
    // bits are set. The pattern is rotated right by R, and repeated across a
    // 32 or 64-bit value, depending on destination register width.
    //
    // To test if an arbitrary immediate can be encoded using this scheme, an
    // iterative algorithm is used.
    //
    // TODO: This code does not consider using X/W register overlap to support
    // 64-bit immediates where the top 32-bits are zero, and the bottom 32-bits
    // are an encodable logical immediate.

    // 1. If the value has all set or all clear bits, it can't be encoded.
    if ((value == 0) || (value == kXRegMask) || ((width == kWRegSize) && (value == kWRegMask)))
      return false;

    unsigned lead_zero = CountLeadingZeros(value, width);
    unsigned lead_one = CountLeadingZeros(~value, width);
    unsigned trail_zero = CountTrailingZeros(value, width);
    unsigned trail_one = CountTrailingZeros(~value, width);
    unsigned set_bits = CountSetBits(value, width);

    // The fixed bits in the immediate s field.
    // If width == 64 (X reg), start at 0xFFFFFF80.
    // If width == 32 (W reg), start at 0xFFFFFFC0, as the iteration for 64-bit
    // widths won't be executed.
    int imm_s_fixed = (width == kXRegSize) ? -128 : -64;
    int imm_s_mask = 0x3F;

    for (;;) {
        // 2. If the value is two bits wide, it can be encoded.
        if (width == 2) {
            *n = 0;
            *imm_s = 0x3C;
            *imm_r = (value & 3) - 1;
            return true;
        }

        *n = (width == 64) ? 1 : 0;
        *imm_s = ((imm_s_fixed | (set_bits - 1)) & imm_s_mask);

        if ((lead_zero + set_bits) == width)
            *imm_r = 0;
        else
            *imm_r = (lead_zero > 0) ? (width - trail_zero) : lead_one;

        // 3. If the sum of leading zeros, trailing zeros and set bits is equal to
        //    the bit width of the value, it can be encoded.
        if (lead_zero + trail_zero + set_bits == width)
            return true;

        // 4. If the sum of leading ones, trailing ones and unset bits in the
        //    value is equal to the bit width of the value, it can be encoded.
        if (lead_one + trail_one + (width - set_bits) == width)
            return true;

        // 5. If the most-significant half of the bitwise value is equal to the
        //    least-significant half, return to step 2 using the least-significant
        //    half of the value.
        uint64_t mask = (UINT64_C(1) << (width >> 1)) - 1;
        if ((value & mask) == ((value >> (width >> 1)) & mask)) {
            width >>= 1;
            set_bits >>= 1;
            imm_s_fixed >>= 1;
            continue;
        }

        // 6. Otherwise, the value can't be encoded.
        return false;
    }
}

bool
AssemblerVIXL::IsImmConditionalCompare(int64_t immediate)
{
    return is_uint5(immediate);
}

bool
AssemblerVIXL::IsImmFP32(float imm)
{
    // Valid values will have the form:
    // aBbb.bbbc.defg.h000.0000.0000.0000.0000
    uint32_t bits = float_to_rawbits(imm);
    // bits[19..0] are cleared.
    if ((bits & 0x7ffff) != 0)
        return false;

    // bits[29..25] are all set or all cleared.
    uint32_t b_pattern = (bits >> 16) & 0x3e00;
    if (b_pattern != 0 && b_pattern != 0x3e00)
        return false;

    // bit[30] and bit[29] are opposite.
    if (((bits ^ (bits << 1)) & 0x40000000) == 0)
        return false;

    return true;
}

bool
AssemblerVIXL::IsImmFP64(double imm)
{
    // Valid values will have the form:
    // aBbb.bbbb.bbcd.efgh.0000.0000.0000.0000
    // 0000.0000.0000.0000.0000.0000.0000.0000
    uint64_t bits = double_to_rawbits(imm);
    // bits[47..0] are cleared.
    if ((bits & 0x0000ffffffffffff) != 0)
        return false;

    // bits[61..54] are all set or all cleared.
    uint32_t b_pattern = (bits >> 48) & 0x3fc0;
    if ((b_pattern != 0) && (b_pattern != 0x3fc0))
        return false;

    // bit[62] and bit[61] are opposite.
    if (((bits ^ (bits << 1)) & (UINT64_C(1) << 62)) == 0)
        return false;

    return true;
}

LoadStoreOp
AssemblerVIXL::LoadOpFor(const CPURegister& rt)
{
    VIXL_ASSERT(rt.IsValid());
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDR_x : LDR_w;

    VIXL_ASSERT(rt.IsARMFPRegister());
    return rt.Is64Bits() ? LDR_d : LDR_s;
}

LoadStorePairOp
AssemblerVIXL::LoadPairOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    VIXL_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDP_x : LDP_w;

    VIXL_ASSERT(rt.IsARMFPRegister());
    return rt.Is64Bits() ? LDP_d : LDP_s;
}

LoadStoreOp
AssemblerVIXL::StoreOpFor(const CPURegister& rt)
{
    VIXL_ASSERT(rt.IsValid());
    if (rt.IsRegister())
        return rt.Is64Bits() ? STR_x : STR_w;

    VIXL_ASSERT(rt.IsARMFPRegister());
    return rt.Is64Bits() ? STR_d : STR_s;
}

LoadStorePairOp
AssemblerVIXL::StorePairOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    VIXL_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? STP_x : STP_w;

    VIXL_ASSERT(rt.IsARMFPRegister());
    return rt.Is64Bits() ? STP_d : STP_s;
}

LoadStorePairNonTemporalOp
AssemblerVIXL::LoadPairNonTemporalOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    VIXL_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDNP_x : LDNP_w;

    VIXL_ASSERT(rt.IsARMFPRegister());
    return rt.Is64Bits() ? LDNP_d : LDNP_s;
}

LoadStorePairNonTemporalOp
AssemblerVIXL::StorePairNonTemporalOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    VIXL_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? STNP_x : STNP_w;

    VIXL_ASSERT(rt.IsARMFPRegister());
    return rt.Is64Bits() ? STNP_d : STNP_s;
}

void
AssemblerVIXL::RecordLiteral(int64_t imm, unsigned size)
{
    //FIXME: literals_.push_front(new Literal(pc_, imm, size));
    JS_ASSERT(0 && "RecordLiteral");
}

bool
AreAliased(const CPURegister& reg1, const CPURegister& reg2,
           const CPURegister& reg3, const CPURegister& reg4,
           const CPURegister& reg5, const CPURegister& reg6,
           const CPURegister& reg7, const CPURegister& reg8)
{
    int number_of_valid_regs = 0;
    int number_of_valid_fpregs = 0;

    RegList unique_regs = 0;
    RegList unique_fpregs = 0;

    const CPURegister regs[] = {reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8};

    for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        if (regs[i].IsRegister()) {
            number_of_valid_regs++;
            unique_regs |= regs[i].Bit();
        } else if (regs[i].IsARMFPRegister()) {
            number_of_valid_fpregs++;
            unique_fpregs |= regs[i].Bit();
        } else {
            VIXL_ASSERT(!regs[i].IsValid());
        }
    }

    int number_of_unique_regs = CountSetBits(unique_regs, sizeof(unique_regs) * 8);
    int number_of_unique_fpregs = CountSetBits(unique_fpregs, sizeof(unique_fpregs) * 8);

    VIXL_ASSERT(number_of_valid_regs >= number_of_unique_regs);
    VIXL_ASSERT(number_of_valid_fpregs >= number_of_unique_fpregs);

    return (number_of_valid_regs != number_of_unique_regs) ||
           (number_of_valid_fpregs != number_of_unique_fpregs);
}

bool
AreSameSizeAndType(const CPURegister& reg1, const CPURegister& reg2,
                   const CPURegister& reg3, const CPURegister& reg4,
                   const CPURegister& reg5, const CPURegister& reg6,
                   const CPURegister& reg7, const CPURegister& reg8)
{
    VIXL_ASSERT(reg1.IsValid());
    bool match = true;
    match &= !reg2.IsValid() || reg2.IsSameSizeAndType(reg1);
    match &= !reg3.IsValid() || reg3.IsSameSizeAndType(reg1);
    match &= !reg4.IsValid() || reg4.IsSameSizeAndType(reg1);
    match &= !reg5.IsValid() || reg5.IsSameSizeAndType(reg1);
    match &= !reg6.IsValid() || reg6.IsSameSizeAndType(reg1);
    match &= !reg7.IsValid() || reg7.IsSameSizeAndType(reg1);
    match &= !reg8.IsValid() || reg8.IsSameSizeAndType(reg1);
    return match;
}

} // namespace jit
} // namespace js
