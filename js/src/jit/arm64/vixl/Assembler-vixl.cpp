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

#include "jsutil.h"

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
    MOZ_ASSERT((1 << index) & list_);
    Remove(index);
    return CPURegister(index, size_, type_);
}

CPURegister
CPURegList::PopHighestIndex()
{
    MOZ_ASSERT(IsValid());
    if (IsEmpty())
      return NoCPUReg;

    int index = CountLeadingZeros(list_, kRegListSizeInBits);
    index = kRegListSizeInBits - 1 - index;
    MOZ_ASSERT((1 << index) & list_);
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
        MOZ_ASSERT(type() == CPURegister::kNoRegister);
        MOZ_ASSERT(IsEmpty());
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
    list.Combine(lr_64);
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
    MOZ_ASSERT(code < kNumberOfRegisters);
    return wregisters[code];
}

const ARMRegister&
ARMRegister::XRegFromCode(unsigned code)
{
    if (code == kSPRegInternalCode)
      return sp;
    MOZ_ASSERT(code < kNumberOfRegisters);
    return xregisters[code];
}

const ARMFPRegister&
ARMFPRegister::SRegFromCode(unsigned code)
{
    MOZ_ASSERT(code < kNumberOfFloatRegisters);
    return sregisters[code];
}

const ARMFPRegister&
ARMFPRegister::DRegFromCode(unsigned code)
{
    MOZ_ASSERT(code < kNumberOfFloatRegisters);
    return dregisters[code];
}

const ARMRegister&
CPURegister::W() const
{
    MOZ_ASSERT(IsValidRegister());
    return ARMRegister::WRegFromCode(code_);
}

const ARMRegister&
CPURegister::X() const
{
    MOZ_ASSERT(IsValidRegister());
    return ARMRegister::XRegFromCode(code_);
}

const ARMFPRegister&
CPURegister::S() const
{
    MOZ_ASSERT(IsValidFPRegister());
    return ARMFPRegister::SRegFromCode(code_);
}

const ARMFPRegister&
CPURegister::D() const
{
    MOZ_ASSERT(IsValidFPRegister());
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
    MOZ_ASSERT(reg.Is64Bits() || (shift_amount < kWRegSize));
    MOZ_ASSERT(reg.Is32Bits() || (shift_amount < kXRegSize));
    MOZ_ASSERT(!reg.IsSP());
}

Operand::Operand(ARMRegister reg, Extend extend, unsigned shift_amount)
  : reg_(reg),
    shift_(NO_SHIFT),
    extend_(extend),
    shift_amount_(shift_amount)
{
    MOZ_ASSERT(reg.IsValid());
    MOZ_ASSERT(shift_amount <= 4);
    MOZ_ASSERT(!reg.IsSP());

    // Extend modes SXTX and UXTX require a 64-bit register.
    MOZ_ASSERT(reg.Is64Bits() || ((extend != SXTX) && (extend != UXTX)));
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
    MOZ_ASSERT(IsShiftedRegister());
    MOZ_ASSERT((shift_ == LSL) && (shift_amount_ <= 4));
    return Operand(reg_, reg_.Is64Bits() ? UXTX : UXTW, shift_amount_);
}

// MemOperand
MemOperand::MemOperand(ARMRegister base, ptrdiff_t offset, AddrMode addrmode)
  : base_(base), regoffset_(NoReg), offset_(offset), addrmode_(addrmode)
{
    MOZ_ASSERT(base.Is64Bits() && !base.IsZero());
}

MemOperand::MemOperand(ARMRegister base, ARMRegister regoffset,
                       Extend extend, unsigned shift_amount)
  : base_(base), regoffset_(regoffset), offset_(0), addrmode_(Offset),
    shift_(NO_SHIFT), extend_(extend), shift_amount_(shift_amount)
{
    MOZ_ASSERT(base.Is64Bits() && !base.IsZero());
    MOZ_ASSERT(!regoffset.IsSP());
    MOZ_ASSERT((extend == UXTW) || (extend == SXTW) || (extend == SXTX));

    // SXTX extend mode requires a 64-bit offset register.
    MOZ_ASSERT(regoffset.Is64Bits() || (extend != SXTX));
}

MemOperand::MemOperand(ARMRegister base, ARMRegister regoffset,
                       Shift shift, unsigned shift_amount)
  : base_(base), regoffset_(regoffset), offset_(0), addrmode_(Offset),
    shift_(shift), extend_(NO_EXTEND), shift_amount_(shift_amount)
{
    MOZ_ASSERT(base.Is64Bits() && !base.IsZero());
    MOZ_ASSERT(regoffset.Is64Bits() && !regoffset.IsSP());
    MOZ_ASSERT(shift == LSL);
}

MemOperand::MemOperand(ARMRegister base, const Operand& offset, AddrMode addrmode)
  : base_(base), regoffset_(NoReg), addrmode_(addrmode)
{
    MOZ_ASSERT(base.Is64Bits() && !base.IsZero());

    if (offset.IsImmediate()) {
        offset_ = offset.immediate();
    } else if (offset.IsShiftedRegister()) {
        MOZ_ASSERT(addrmode == Offset);

        regoffset_ = offset.reg();
        shift_= offset.shift();
        shift_amount_ = offset.shift_amount();

        extend_ = NO_EXTEND;
        offset_ = 0;

        // These assertions match those in the shifted-register constructor.
        MOZ_ASSERT(regoffset_.Is64Bits() && !regoffset_.IsSP());
        MOZ_ASSERT(shift_ == LSL);
    } else {
        MOZ_ASSERT(offset.IsExtendedRegister());
        MOZ_ASSERT(addrmode == Offset);

        regoffset_ = offset.reg();
        extend_ = offset.extend();
        shift_amount_ = offset.shift_amount();

        shift_= NO_SHIFT;
        offset_ = 0;

        // These assertions match those in the extended-register constructor.
        MOZ_ASSERT(!regoffset_.IsSP());
        MOZ_ASSERT((extend_ == UXTW) || (extend_ == SXTW) || (extend_ == SXTX));
        MOZ_ASSERT((regoffset_.Is64Bits() || (extend_ != SXTX)));
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
    // TODO: MOZ_ASSERT((pc_ >= buffer_) && (pc_ < buffer_ + buffer_size_));
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

// Assembly buffer.
void
AssemblerVIXL::InsertIndexIntoTag(uint8_t *load, uint32_t index)
{
    // Store the PoolEntry index into the instruction.
    // finishPool() will walk over all literal load instructions
    // and use PatchConstantPoolLoad() to patch to the final relative offset.
    *((uint32_t*)load) |= ImmLLiteral(index);
}

bool
AssemblerVIXL::PatchConstantPoolLoad(void *loadAddr, void *constPoolAddr)
{
    Instruction *load = reinterpret_cast<Instruction*>(loadAddr);

    // The load currently contains the PoolEntry's index,
    // as written by InsertIndexIntoTag().
    uint32_t index = load->ImmLLiteral();

    // Each entry in the literal pool is uint32_t-sized.
    uint32_t *constPool = reinterpret_cast<uint32_t*>(constPoolAddr);
    Instruction *source = reinterpret_cast<Instruction *>(&constPool[index]);

    load->SetImmLLiteral(source);
    return false; // FIXME: Nothing actually uses the return value.
}

// A common implementation for the LinkAndGet<Type>OffsetTo helpers.
//
// If the label is bound, returns the offset as a multiple of element_size.
// Otherwise, links the instruction to the label and returns the offset to encode
// as a multiple of kInstructionSize.
//
// The offset is calculated by aligning the PC and label addresses down to a
// multiple of element_size, then calculating the (scaled) offset between them.
// This matches the semantics of adrp, for example.
template <int element_size>
ptrdiff_t
AssemblerVIXL::LinkAndGetOffsetTo(BufferOffset branch, Label *label)
{
    if (armbuffer_.oom())
        return LabelBase::INVALID_OFFSET;

    // The label is bound: all uses are already linked.
    if (label->bound()) {
        ptrdiff_t branch_offset = ptrdiff_t(branch.getOffset() / element_size);
        ptrdiff_t label_offset = ptrdiff_t(label->offset() / element_size);
        return label_offset - branch_offset;
    }

    // The label is unbound and unused: store the offset in the label itself
    // for patching by bind().
    if (!label->used()) {
        label->use(branch.getOffset());
        return LabelBase::INVALID_OFFSET;
    }

    // The label is unbound but used. Create an implicit linked list between
    // the branches, and update the linked list head in the label struct.
    ptrdiff_t prevHeadOffset = static_cast<ptrdiff_t>(label->offset());
    label->use(branch.getOffset());
    MOZ_ASSERT(prevHeadOffset - branch.getOffset() != LabelBase::INVALID_OFFSET);
    return prevHeadOffset - branch.getOffset();
}

ptrdiff_t
AssemblerVIXL::LinkAndGetByteOffsetTo(BufferOffset branch, Label *label)
{
    return LinkAndGetOffsetTo<1>(branch, label);
}

ptrdiff_t
AssemblerVIXL::LinkAndGetInstructionOffsetTo(BufferOffset branch, Label *label)
{
    return LinkAndGetOffsetTo<kInstructionSize>(branch, label);
}

ptrdiff_t
AssemblerVIXL::LinkAndGetPageOffsetTo(BufferOffset branch, Label *label)
{
    return LinkAndGetOffsetTo<kPageSize>(branch, label);
}

// Code generation.
void
AssemblerVIXL::br(const ARMRegister& xn)
{
    MOZ_ASSERT(xn.Is64Bits());
    // No need for EmitBranch(): no immediate offset needs fixing.
    Emit(BR | Rn(xn));
}
// Code generation.
void
AssemblerVIXL::br(Instruction * at, const ARMRegister& xn)
{
    MOZ_ASSERT(xn.Is64Bits());
    // No need for EmitBranch(): no immediate offset needs fixing.
    Emit(at, BR | Rn(xn));
}

void
AssemblerVIXL::blr(const ARMRegister& xn)
{
    MOZ_ASSERT(xn.Is64Bits());
    // No need for EmitBranch(): no immediate offset needs fixing.
    Emit(BLR | Rn(xn));
}
void
AssemblerVIXL::blr(Instruction *at, const ARMRegister& xn)
{
    MOZ_ASSERT(xn.Is64Bits());
    // No need for EmitBranch(): no immediate offset needs fixing.
    Emit(at, BLR | Rn(xn));
}

void
AssemblerVIXL::ret(const ARMRegister& xn)
{
    MOZ_ASSERT(xn.Is64Bits());
    // No need for EmitBranch(): no immediate offset needs fixing.
    Emit(RET | Rn(xn));
}

BufferOffset
AssemblerVIXL::b(int imm26)
{
    return EmitBranch(B | ImmUncondBranch(imm26));
}

void
AssemblerVIXL::b(Instruction *at, int imm26)
{
    return EmitBranch(at, B | ImmUncondBranch(imm26));
}

BufferOffset
AssemblerVIXL::b(int imm19, Condition cond)
{
    return EmitBranch(B_cond | ImmCondBranch(imm19) | cond);
}

void
AssemblerVIXL::b(Instruction *at, int imm19, Condition cond)
{
    EmitBranch(at, B_cond | ImmCondBranch(imm19) | cond);
}

BufferOffset
AssemblerVIXL::b(Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0);
    Instruction *ins = getInstructionAt(branch);
    MOZ_ASSERT(ins->IsUncondBranchImm());

    // Encode the relative offset.
    b(ins, LinkAndGetInstructionOffsetTo(branch, label));
    return branch;
}

BufferOffset
AssemblerVIXL::b(Label* label, Condition cond)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0, Always);
    Instruction *ins = getInstructionAt(branch);
    MOZ_ASSERT(ins->IsCondBranchImm());

    // Encode the relative offset.
    b(ins, LinkAndGetInstructionOffsetTo(branch, label), cond);
    return branch;
}

void
AssemblerVIXL::bl(int imm26)
{
    EmitBranch(BL | ImmUncondBranch(imm26));
}

void
AssemblerVIXL::bl(Instruction *at, int imm26)
{
    EmitBranch(at, BL | ImmUncondBranch(imm26));
}

void
AssemblerVIXL::bl(Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0);
    Instruction *ins = getInstructionAt(branch);

    // Encode the relative offset.
    bl(ins, LinkAndGetInstructionOffsetTo(branch, label));
}

void
AssemblerVIXL::cbz(const ARMRegister& rt, int imm19)
{
    EmitBranch(SF(rt) | CBZ | ImmCmpBranch(imm19) | Rt(rt));
}

void
AssemblerVIXL::cbz(Instruction *at, const ARMRegister& rt, int imm19)
{
    EmitBranch(at, SF(rt) | CBZ | ImmCmpBranch(imm19) | Rt(rt));
}

void
AssemblerVIXL::cbz(const ARMRegister& rt, Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0);
    Instruction *ins = getInstructionAt(branch);

    // Encode the relative offset.
    cbz(ins, rt, LinkAndGetInstructionOffsetTo(branch, label));
}

void
AssemblerVIXL::cbnz(const ARMRegister& rt, int imm19)
{
    EmitBranch(SF(rt) | CBNZ | ImmCmpBranch(imm19) | Rt(rt));
}

void
AssemblerVIXL::cbnz(Instruction *at, const ARMRegister& rt, int imm19)
{
    EmitBranch(at, SF(rt) | CBNZ | ImmCmpBranch(imm19) | Rt(rt));
}

void
AssemblerVIXL::cbnz(const ARMRegister& rt, Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0);
    Instruction *ins = getInstructionAt(branch);

    // Encode the relative offset.
    cbnz(ins, rt, LinkAndGetInstructionOffsetTo(branch, label));
}

void
AssemblerVIXL::tbz(const ARMRegister& rt, unsigned bit_pos, int imm14)
{
    MOZ_ASSERT(rt.Is64Bits() || (rt.Is32Bits() && (bit_pos < kWRegSize)));
    EmitBranch(TBZ | ImmTestBranchBit(bit_pos) | ImmTestBranch(imm14) | Rt(rt));
}

void
AssemblerVIXL::tbz(Instruction *at, const ARMRegister& rt, unsigned bit_pos, int imm14)
{
    MOZ_ASSERT(rt.Is64Bits() || (rt.Is32Bits() && (bit_pos < kWRegSize)));
    EmitBranch(at, TBZ | ImmTestBranchBit(bit_pos) | ImmTestBranch(imm14) | Rt(rt));
}

void
AssemblerVIXL::tbz(const ARMRegister& rt, unsigned bit_pos, Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0);
    Instruction *ins = getInstructionAt(branch);

    // Encode the relative offset.
    tbz(ins, rt, bit_pos, LinkAndGetInstructionOffsetTo(branch, label));
}

void
AssemblerVIXL::tbnz(const ARMRegister& rt, unsigned bit_pos, int imm14)
{
    MOZ_ASSERT(rt.Is64Bits() || (rt.Is32Bits() && (bit_pos < kWRegSize)));
    EmitBranch(TBNZ | ImmTestBranchBit(bit_pos) | ImmTestBranch(imm14) | Rt(rt));
}

void
AssemblerVIXL::tbnz(Instruction *at, const ARMRegister& rt, unsigned bit_pos, int imm14)
{
    MOZ_ASSERT(rt.Is64Bits() || (rt.Is32Bits() && (bit_pos < kWRegSize)));
    EmitBranch(at, TBNZ | ImmTestBranchBit(bit_pos) | ImmTestBranch(imm14) | Rt(rt));
}

void
AssemblerVIXL::tbnz(const ARMRegister& rt, unsigned bit_pos, Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    BufferOffset branch = b(0);
    Instruction *ins = getInstructionAt(branch);

    // Encode the relative offset.
    tbnz(ins, rt, bit_pos, LinkAndGetInstructionOffsetTo(branch, label));
}

void
AssemblerVIXL::adr(const ARMRegister& rd, int imm21)
{
    MOZ_ASSERT(rd.Is64Bits());
    Emit(ADR | ImmPCRelAddress(imm21) | Rd(rd));
}

void
AssemblerVIXL::adr(Instruction *at, const ARMRegister& rd, int imm21)
{
    MOZ_ASSERT(rd.Is64Bits());
    Emit(at, ADR | ImmPCRelAddress(imm21) | Rd(rd));
}

void
AssemblerVIXL::adr(const ARMRegister& rd, Label* label)
{
    // Flush the instruction buffer before calculating relative offset.
    // ADR is not a branch.
    BufferOffset offset = Emit(0);
    Instruction *ins = getInstructionAt(offset);

    // Encode the relative offset.
    // TODO: This is probably incorrect -- ADR needs patching
    // during finalization to take constant pools into account.
    adr(ins, rd, LinkAndGetByteOffsetTo(offset, label));
}

void
AssemblerVIXL::adrp(const ARMRegister& rd, int imm21)
{
    MOZ_ASSERT(rd.Is64Bits());
    Emit(ADRP | ImmPCRelAddress(imm21) | Rd(rd));
}


void
AssemblerVIXL::adrp(const ARMRegister& rd, Label* label)
{
    MOZ_ASSERT(AllowPageOffsetDependentCode());
    MOZ_CRASH("adrp()");
    //adrp(rd, LinkAndGetPageOffsetTo(label));
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

BufferOffset
AssemblerVIXL::ands(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand)
{
    return Logical(rd, rn, operand, ANDS);
}

BufferOffset
AssemblerVIXL::tst(const ARMRegister& rn, const Operand& operand)
{
    return ands(AppropriateZeroRegFor(rn), rn, operand);
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
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | LSLV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::lsrv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | LSRV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::asrv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | ASRV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::rorv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | RORV | Rm(rm) | Rn(rn) | Rd(rd));
}

// Bitfield operations.
void
AssemblerVIXL::bfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms)
{
    MOZ_ASSERT(rd.size() == rn.size());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | BFM | N |
         ImmR(immr, rd.size()) | ImmS(imms, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::sbfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms)
{
    MOZ_ASSERT(rd.Is64Bits() || rn.Is32Bits());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | SBFM | N |
         ImmR(immr, rd.size()) | ImmS(imms, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::ubfm(const ARMRegister& rd, const ARMRegister& rn, unsigned immr, unsigned imms)
{
    MOZ_ASSERT(rd.size() == rn.size());
    Instr N = SF(rd) >> (kSFOffset - kBitfieldNOffset);
    Emit(SF(rd) | UBFM | N |
         ImmR(immr, rd.size()) | ImmS(imms, rn.size()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::extr(const ARMRegister& rd, const ARMRegister& rn,
                    const ARMRegister& rm, unsigned lsb)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
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
    MOZ_ASSERT((cond != al) && (cond != nv));
    ARMRegister zr = AppropriateZeroRegFor(rd);
    csinc(rd, zr, zr, InvertCondition(cond));
}

void
AssemblerVIXL::csetm(const ARMRegister &rd, Condition cond)
{
    MOZ_ASSERT((cond != al) && (cond != nv));
    ARMRegister zr = AppropriateZeroRegFor(rd);
    csinv(rd, zr, zr, InvertCondition(cond));
}

void
AssemblerVIXL::cinc(const ARMRegister &rd, const ARMRegister &rn, Condition cond)
{
    MOZ_ASSERT((cond != al) && (cond != nv));
    csinc(rd, rn, rn, InvertCondition(cond));
}

void
AssemblerVIXL::cinv(const ARMRegister &rd, const ARMRegister &rn, Condition cond)
{
    MOZ_ASSERT((cond != al) && (cond != nv));
    csinv(rd, rn, rn, InvertCondition(cond));
}

void
AssemblerVIXL::cneg(const ARMRegister &rd, const ARMRegister &rn, Condition cond)
{
    MOZ_ASSERT((cond != al) && (cond != nv));
    csneg(rd, rn, rn, InvertCondition(cond));
}

void
AssemblerVIXL::ConditionalSelect(const ARMRegister& rd, const ARMRegister& rn,
                                 const ARMRegister& rm, Condition cond,
                                 ConditionalSelectOp op)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
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
    MOZ_ASSERT(AreSameSizeAndType(rd, rn, rm));
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
    MOZ_ASSERT(AreSameSizeAndType(rd, rn, rm));
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
    MOZ_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    MOZ_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, UMADDL_x);
}

void
AssemblerVIXL::smaddl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    MOZ_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    MOZ_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, SMADDL_x);
}

void
AssemblerVIXL::umsubl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    MOZ_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    MOZ_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, UMSUBL_x);
}

void
AssemblerVIXL::smsubl(const ARMRegister& rd, const ARMRegister& rn,
                      const ARMRegister& rm, const ARMRegister& ra)
{
    MOZ_ASSERT(rd.Is64Bits() && ra.Is64Bits());
    MOZ_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, ra, SMSUBL_x);
}

void
AssemblerVIXL::smull(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    MOZ_ASSERT(rd.Is64Bits());
    MOZ_ASSERT(rn.Is32Bits() && rm.Is32Bits());
    DataProcessing3Source(rd, rn, rm, xzr, SMADDL_x);
}

void
AssemblerVIXL::sdiv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
    Emit(SF(rd) | SDIV | Rm(rm) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::smulh(const ARMRegister& xd, const ARMRegister& xn, const ARMRegister& xm)
{
    MOZ_ASSERT(xd.Is64Bits() && xn.Is64Bits() && xm.Is64Bits());
    DataProcessing3Source(xd, xn, xm, xzr, SMULH_x);
}

void
AssemblerVIXL::udiv(const ARMRegister& rd, const ARMRegister& rn, const ARMRegister& rm)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == rm.size());
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
    MOZ_ASSERT(rd.Is64Bits());
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
    MOZ_ASSERT(rt.Is64Bits());
    LoadStorePair(rt, rt2, src, LDPSW_x);
}

void
AssemblerVIXL::LoadStorePair(const CPURegister& rt, const CPURegister& rt2,
                             const MemOperand& addr, LoadStorePairOp op)
{
    // 'rt' and 'rt2' can only be aliased for stores.
    MOZ_ASSERT(((op & LoadStorePairLBit) == 0) || !rt.Is(rt2));
    MOZ_ASSERT(AreSameSizeAndType(rt, rt2));

    Instr memop = op | Rt(rt) | Rt2(rt2) | RnSP(addr.base()) |
                  ImmLSPair(addr.offset(), CalcLSPairDataSize(op));

    Instr addrmodeop;
    if (addr.IsImmediateOffset()) {
        addrmodeop = LoadStorePairOffsetFixed;
    } else {
        MOZ_ASSERT(addr.offset() != 0);
        if (addr.IsPreIndex()) {
            addrmodeop = LoadStorePairPreIndexFixed;
        } else {
            MOZ_ASSERT(addr.IsPostIndex());
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
    MOZ_ASSERT(!rt.Is(rt2));
    MOZ_ASSERT(AreSameSizeAndType(rt, rt2));
    MOZ_ASSERT(addr.IsImmediateOffset());

    LSDataSize size = CalcLSPairDataSize(static_cast<LoadStorePairOp>(op & LoadStorePairMask));
    Emit(op | Rt(rt) | Rt2(rt2) | RnSP(addr.base()) | ImmLSPair(addr.offset(), size));
}

// Memory instructions.
void
AssemblerVIXL::ldrb(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, src, LDRB_w, option);
}

void
AssemblerVIXL::strb(const ARMRegister& rt, const MemOperand& dst, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, dst, STRB_w, option);
}

void
AssemblerVIXL::ldrsb(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, src, rt.Is64Bits() ? LDRSB_x : LDRSB_w, option);
}

void
AssemblerVIXL::ldrh(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, src, LDRH_w, option);
}

void
AssemblerVIXL::strh(const ARMRegister& rt, const MemOperand& dst, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, dst, STRH_w, option);
}

void
AssemblerVIXL::ldrsh(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, src, rt.Is64Bits() ? LDRSH_x : LDRSH_w, option);
}

void
AssemblerVIXL::ldr(const CPURegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, src, LoadOpFor(rt), option);
}

void
AssemblerVIXL::str(const CPURegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    LoadStore(rt, src, StoreOpFor(rt), option);
}

void
AssemblerVIXL::ldrsw(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireUnscaledOffset);
    MOZ_ASSERT(option != PreferUnscaledOffset);
    MOZ_ASSERT(rt.Is64Bits());
    LoadStore(rt, src, LDRSW_x, option);
}

void
AssemblerVIXL::ldurb(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, src, LDRB_w, option);
}

void
AssemblerVIXL::sturb(const ARMRegister& rt, const MemOperand& dst, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, dst, STRB_w, option);
}

void
AssemblerVIXL::ldursb(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, src, rt.Is64Bits() ? LDRSB_x : LDRSB_w, option);
}

void
AssemblerVIXL::ldurh(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, src, LDRH_w, option);
}

void
AssemblerVIXL::sturh(const ARMRegister& rt, const MemOperand& dst, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, dst, STRH_w, option);
}

void
AssemblerVIXL::ldursh(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, src, rt.Is64Bits() ? LDRSH_x : LDRSH_w, option);
}

void
AssemblerVIXL::ldur(const CPURegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, src, LoadOpFor(rt), option);
}

void
AssemblerVIXL::stur(const CPURegister& rt, const MemOperand& dst, LoadStoreScalingOption option)
{
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, dst, StoreOpFor(rt), option);
}

void
AssemblerVIXL::ldursw(const ARMRegister& rt, const MemOperand& src, LoadStoreScalingOption option)
{
    MOZ_ASSERT(rt.Is64Bits());
    MOZ_ASSERT(option != RequireScaledOffset);
    MOZ_ASSERT(option != PreferScaledOffset);
    LoadStore(rt, src, LDRSW_x, option);
}

void
AssemblerVIXL::ldr(const CPURegister &rt, int imm19)
{
    LoadLiteralOp op = LoadLiteralOpFor(rt);
    Emit(op | ImmLLiteral(imm19) | Rt(rt));
}

void
AssemblerVIXL::ldr(Instruction *at, const CPURegister &rt, int imm19)
{
    LoadLiteralOp op = LoadLiteralOpFor(rt);
    Emit(at, op | ImmLLiteral(imm19) | Rt(rt));
}

void
AssemblerVIXL::ldrsw(const ARMRegister &rt, int imm19)
{
    Emit(LDRSW_x_lit | ImmLLiteral(imm19) | Rt(rt));
}

// Exclusive-access instructions.
void
AssemblerVIXL::stxrb(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    Emit(STXRB_w | Rs(rs) | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::stxrh(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    Emit(STXRH_w | Rs(rs) | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::stxr(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? STXR_x : STXR_w;
    Emit(op | Rs(rs) | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::ldxrb(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    Emit(LDXRB_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::ldxrh(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    Emit(LDXRH_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::ldxr(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? LDXR_x : LDXR_w;
    Emit(op | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::stxp(const ARMRegister& rs, const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& dst)
{
    MOZ_ASSERT(rt.size() == rt2.size());
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? STXP_x : STXP_w;
    Emit(op | Rs(rs) | Rt(rt) | Rt2(rt2) | RnSP(dst.base()));
}

void
AssemblerVIXL::ldxp(const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& src)
{
    MOZ_ASSERT(rt.size() == rt2.size());
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? LDXP_x : LDXP_w;
    Emit(op | Rs_mask | Rt(rt) | Rt2(rt2) | RnSP(src.base()));
}

void
AssemblerVIXL::stlxrb(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    Emit(STLXRB_w | Rs(rs) | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::stlxrh(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    Emit(STLXRH_w | Rs(rs) | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::stlxr(const ARMRegister& rs, const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? STLXR_x : STLXR_w;
    Emit(op | Rs(rs) | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::ldaxrb(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    Emit(LDAXRB_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::ldaxrh(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    Emit(LDAXRH_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::ldaxr(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? LDAXR_x : LDAXR_w;
    Emit(op | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::stlxp(const ARMRegister& rs, const ARMRegister& rt,
                     const ARMRegister& rt2, const MemOperand& dst)
{
    MOZ_ASSERT(rt.size() == rt2.size());
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? STLXP_x : STLXP_w;
    Emit(op | Rs(rs) | Rt(rt) | Rt2(rt2) | RnSP(dst.base()));
}

void AssemblerVIXL::ldaxp(const ARMRegister& rt, const ARMRegister& rt2, const MemOperand& src)
{
    MOZ_ASSERT(rt.size() == rt2.size());
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? LDAXP_x : LDAXP_w;
    Emit(op | Rs_mask | Rt(rt) | Rt2(rt2) | RnSP(src.base()));
}

void
AssemblerVIXL::stlrb(const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    Emit(STLRB_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::stlrh(const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    Emit(STLRH_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::stlr(const ARMRegister& rt, const MemOperand& dst)
{
    MOZ_ASSERT(dst.IsImmediateOffset() && (dst.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? STLR_x : STLR_w;
    Emit(op | Rs_mask | Rt(rt) | Rt2_mask | RnSP(dst.base()));
}

void
AssemblerVIXL::ldarb(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    Emit(LDARB_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::ldarh(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    Emit(LDARH_w | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
}

void
AssemblerVIXL::ldar(const ARMRegister& rt, const MemOperand& src)
{
    MOZ_ASSERT(src.IsImmediateOffset() && (src.offset() == 0));
    LoadStoreExclusive op = rt.Is64Bits() ? LDAR_x : LDAR_w;
    Emit(op | Rs_mask | Rt(rt) | Rt2_mask | RnSP(src.base()));
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
    MOZ_ASSERT(rt.Is64Bits());
    Emit(MRS | ImmSystemRegister(sysreg) | Rt(rt));
}

void
AssemblerVIXL::msr(SystemRegister sysreg, const ARMRegister& rt)
{
    MOZ_ASSERT(rt.Is64Bits());
    Emit(MSR | Rt(rt) | ImmSystemRegister(sysreg));
}

BufferOffset
AssemblerVIXL::hint(SystemHint code)
{
    return Emit(HINT | ImmHint(code) | Rt(xzr));
}
void
AssemblerVIXL::hint(Instruction *at, SystemHint code)
{
    Emit(at, HINT | ImmHint(code) | Rt(xzr));
}

void
AssemblerVIXL::clrex(int imm4)
{
    Emit(CLREX | CRm(imm4));
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
    MOZ_ASSERT(fd.Is64Bits());
    MOZ_ASSERT(IsImmFP64(imm));
    Emit(FMOV_d_imm | Rd(fd) | ImmFP64(imm));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, float imm)
{
    MOZ_ASSERT(fd.Is32Bits());
    MOZ_ASSERT(IsImmFP32(imm));
    Emit(FMOV_s_imm | Rd(fd) | ImmFP32(imm));
}

void
AssemblerVIXL::fmov(const ARMRegister& rd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(rd.size() == fn.size());
    FPIntegerConvertOp op = rd.Is32Bits() ? FMOV_ws : FMOV_xd;
    Emit(op | Rd(rd) | Rn(fn));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, const ARMRegister& rn)
{
    MOZ_ASSERT(fd.size() == rn.size());
    FPIntegerConvertOp op = fd.Is32Bits() ? FMOV_sw : FMOV_dx;
    Emit(op | Rd(fd) | Rn(rn));
}

void
AssemblerVIXL::fmov(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
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
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FABS);
}

void
AssemblerVIXL::fneg(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FNEG);
}

void
AssemblerVIXL::fsqrt(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FSQRT);
}

void
AssemblerVIXL::frinta(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FRINTA);
}

void
AssemblerVIXL::frintm(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FRINTM);
}

void
AssemblerVIXL::frintn(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FRINTN);
}

void
AssemblerVIXL::frintz(const ARMFPRegister& fd, const ARMFPRegister& fn)
{
    MOZ_ASSERT(fd.size() == fn.size());
    FPDataProcessing1Source(fd, fn, FRINTZ);
}

void
AssemblerVIXL::fcmp(const ARMFPRegister& fn, const ARMFPRegister& fm)
{
    MOZ_ASSERT(fn.size() == fm.size());
    Emit(FPType(fn) | FCMP | Rm(fm) | Rn(fn));
}

void
AssemblerVIXL::fcmp(const ARMFPRegister& fn, double value)
{
    USEARG(value);
    // Although the fcmp instruction can strictly only take an immediate value of
    // +0.0, we don't need to check for -0.0 because the sign of 0.0 doesn't
    // affect the result of the comparison.
    MOZ_ASSERT(value == 0.0);
    Emit(FPType(fn) | FCMP_zero | Rn(fn));
}

void
AssemblerVIXL::fccmp(const ARMFPRegister& fn, const ARMFPRegister& fm,
                     StatusFlags nzcv, Condition cond)
{
    MOZ_ASSERT(fn.size() == fm.size());
    Emit(FPType(fn) | FCCMP | Rm(fm) | Cond(cond) | Rn(fn) | Nzcv(nzcv));
}

void
AssemblerVIXL::fcsel(const ARMFPRegister& fd, const ARMFPRegister& fn,
                     const ARMFPRegister& fm, Condition cond)
{
    MOZ_ASSERT(fd.size() == fn.size());
    MOZ_ASSERT(fd.size() == fm.size());
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
      MOZ_ASSERT(fn.Is32Bits());
      FPDataProcessing1Source(fd, fn, FCVT_ds);
    } else {
      // Convert double to float.
      MOZ_ASSERT(fn.Is64Bits());
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
    MOZ_ASSERT(IsImmFP32(imm));
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
    MOZ_ASSERT(IsImmFP64(imm));
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
    // Ignore the top 32 bits of an immediate if we're moving to a W register.
    if (rd.Is32Bits()) {
        // Check that the top 32 bits are zero (a positive 32-bit number) or top
        // 33 bits are one (a negative 32-bit number, sign extended to 64 bits).
        MOZ_ASSERT(((imm >> kWRegSize) == 0) ||
                    ((imm >> (kWRegSize - 1)) == 0x1ffffffff));
        imm &= kWRegMask;
    }

    if (shift >= 0) {
        // Explicit shift specified.
        MOZ_ASSERT((shift == 0) || (shift == 16) ||
                    (shift == 32) || (shift == 48));
        MOZ_ASSERT(rd.Is64Bits() || (shift == 0) || (shift == 16));
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
            MOZ_ASSERT(rd.Is64Bits());
            imm >>= 32;
            shift = 2;
        } else if ((imm & 0x0000ffffffffffff) == 0) {
            MOZ_ASSERT(rd.Is64Bits());
            imm >>= 48;
            shift = 3;
        }
    }

    MOZ_ASSERT(is_uint16(imm));

    Emit(SF(rd) | MoveWideImmediateFixed | mov_op |
         Rd(rd) | ImmMoveWide(imm) | ShiftMoveWide(shift));
}

void
AssemblerVIXL::AddSub(const ARMRegister& rd, const ARMRegister& rn, const Operand& operand,
                      FlagsUpdate S, AddSubOp op)
{
    MOZ_ASSERT(rd.size() == rn.size());

    if (operand.IsImmediate()) {
        int64_t immediate = operand.immediate();
        MOZ_ASSERT(IsImmAddSub(immediate));
        Instr dest_reg = (S == SetFlags) ? Rd(rd) : RdSP(rd);
        Emit(SF(rd) | AddSubImmediateFixed | op | Flags(S) |
            ImmAddSub(immediate) | dest_reg | RnSP(rn));
    } else if (operand.IsShiftedRegister()) {
        MOZ_ASSERT(operand.reg().size() == rd.size());
        MOZ_ASSERT(operand.shift() != ROR);

        // For instructions of the form:
        //   add/sub   wsp, <Wn>, <Wm> [, LSL #0-3 ]
        //   add/sub   <Wd>, wsp, <Wm> [, LSL #0-3 ]
        //   add/sub   wsp, wsp, <Wm> [, LSL #0-3 ]
        //   adds/subs <Wd>, wsp, <Wm> [, LSL #0-3 ]
        // or their 64-bit register equivalents, convert the operand from shifted to
        // extended register mode, and emit an add/sub extended instruction.
        if (rn.IsSP() || rd.IsSP()) {
            MOZ_ASSERT(!(rd.IsSP() && (S == SetFlags)));
            DataProcExtendedRegister(rd, rn, operand.ToExtendedRegister(), S,
                                     AddSubExtendedFixed | op);
        } else {
            DataProcShiftedRegister(rd, rn, operand, S, AddSubShiftedFixed | op);
        }
    } else {
        MOZ_ASSERT(operand.IsExtendedRegister());
        DataProcExtendedRegister(rd, rn, operand, S, AddSubExtendedFixed | op);
    }
}

void
AssemblerVIXL::AddSubWithCarry(const ARMRegister& rd, const ARMRegister& rn,
                               const Operand& operand, FlagsUpdate S, AddSubWithCarryOp op)
{
    MOZ_ASSERT(rd.size() == rn.size());
    MOZ_ASSERT(rd.size() == operand.reg().size());
    MOZ_ASSERT(operand.IsShiftedRegister() && (operand.shift_amount() == 0));
    Emit(SF(rd) | op | Flags(S) | Rm(operand.reg()) | Rn(rn) | Rd(rd));
}

void
AssemblerVIXL::hlt(int code)
{
    MOZ_ASSERT(is_uint16(code));
    Emit(HLT | ImmException(code));
}

void
AssemblerVIXL::brk(int code)
{
    MOZ_ASSERT(is_uint16(code));
    Emit(BRK | ImmException(code));
}

void
AssemblerVIXL::svc(int code)
{
    MOZ_ASSERT(is_uint16(code));
    Emit(SVC | ImmException(code));
}

void
AssemblerVIXL::svc(Instruction *at, int code)
{
    MOZ_ASSERT(is_uint16(code));
    Emit(at, SVC | ImmException(code));
}

void
AssemblerVIXL::nop(Instruction *at)
{
    hint(at, NOP);
}

BufferOffset
AssemblerVIXL::Logical(const ARMRegister& rd, const ARMRegister& rn,
                       const Operand& operand, LogicalOp op)
{
    MOZ_ASSERT(rd.size() == rn.size());
    if (operand.IsImmediate()) {
        int64_t immediate = operand.immediate();
        unsigned reg_size = rd.size();

        MOZ_ASSERT(immediate != 0);
        MOZ_ASSERT(immediate != -1);
        MOZ_ASSERT(rd.Is64Bits() || is_uint32(immediate));

        // If the operation is NOT, invert the operation and immediate.
        if ((op & NOT) == NOT) {
            op = static_cast<LogicalOp>(op & ~NOT);
            immediate = rd.Is64Bits() ? ~immediate : (~immediate & kWRegMask);
        }

        unsigned n, imm_s, imm_r;
        if (IsImmLogical(immediate, reg_size, &n, &imm_s, &imm_r)) {
            // Immediate can be encoded in the instruction.
            return LogicalImmediate(rd, rn, n, imm_s, imm_r, op);
        } else {
            // This case is handled in the macro assembler.
            VIXL_UNREACHABLE();
        }
    } else {
        MOZ_ASSERT(operand.IsShiftedRegister());
        MOZ_ASSERT(operand.reg().size() == rd.size());
        Instr dp_op = static_cast<Instr>(op | LogicalShiftedFixed);
        return DataProcShiftedRegister(rd, rn, operand, LeaveFlags, dp_op);
    }
}

BufferOffset
AssemblerVIXL::LogicalImmediate(const ARMRegister& rd, const ARMRegister& rn,
                                unsigned n, unsigned imm_s, unsigned imm_r, LogicalOp op)
{
    unsigned reg_size = rd.size();
    Instr dest_reg = (op == ANDS) ? Rd(rd) : RdSP(rd);
    return Emit(SF(rd) | LogicalImmediateFixed | op | BitN(n, reg_size) |
                ImmSetBits(imm_s, reg_size) | ImmRotate(imm_r, reg_size) | dest_reg | Rn(rn));
}

void
AssemblerVIXL::ConditionalCompare(const ARMRegister& rn, const Operand& operand,
                                  StatusFlags nzcv, Condition cond, ConditionalCompareOp op)
{
    Instr ccmpop;
    if (operand.IsImmediate()) {
        int64_t immediate = operand.immediate();
        MOZ_ASSERT(IsImmConditionalCompare(immediate));
        ccmpop = ConditionalCompareImmediateFixed | op | ImmCondCmp(immediate);
    } else {
        MOZ_ASSERT(operand.IsShiftedRegister() && (operand.shift_amount() == 0));
        ccmpop = ConditionalCompareRegisterFixed | op | Rm(operand.reg());
    }
    Emit(SF(rn) | ccmpop | Cond(cond) | Rn(rn) | Nzcv(nzcv));
}

void
AssemblerVIXL::DataProcessing1Source(const ARMRegister& rd, const ARMRegister& rn,
                                     DataProcessing1SourceOp op)
{
    MOZ_ASSERT(rd.size() == rn.size());
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
    MOZ_ASSERT(fd.size() == fn.size());
    MOZ_ASSERT(fd.size() == fm.size());
    Emit(FPType(fd) | op | Rm(fm) | Rn(fn) | Rd(fd));
}

void
AssemblerVIXL::FPDataProcessing3Source(const ARMFPRegister& fd, const ARMFPRegister& fn,
                                       const ARMFPRegister& fm, const ARMFPRegister& fa,
                                       FPDataProcessing3SourceOp op)
{
    MOZ_ASSERT(AreSameSizeAndType(fd, fn, fm, fa));
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
    MOZ_ASSERT(rd.size() >= rn.size());
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
              MOZ_ASSERT(rn.size() == kXRegSize);
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

BufferOffset
AssemblerVIXL::DataProcShiftedRegister(const ARMRegister& rd, const ARMRegister& rn,
                                       const Operand& operand, FlagsUpdate S, Instr op)
{
    MOZ_ASSERT(operand.IsShiftedRegister());
    MOZ_ASSERT(rn.Is64Bits() || (rn.Is32Bits() &&
                is_uint5(operand.shift_amount())));
    return Emit(SF(rd) | op | Flags(S) |
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

BufferOffset
AssemblerVIXL::LoadStore(const CPURegister& rt, const MemOperand& addr,
                         LoadStoreOp op, LoadStoreScalingOption option)
{
    Instr memop = op | Rt(rt) | RnSP(addr.base());
    ptrdiff_t offset = addr.offset();
    LSDataSize size = CalcLSDataSize(op);

    if (addr.IsImmediateOffset()) {
        bool prefer_unscaled = (option == PreferUnscaledOffset) ||
                               (option == RequireUnscaledOffset);
        if (prefer_unscaled && IsImmLSUnscaled(offset)) {
            // Use the unscaled addressing mode.
            return Emit(LoadStoreUnscaledOffsetFixed | memop | ImmLS(offset));
        }
 
        if ((option != RequireUnscaledOffset) && IsImmLSScaled(offset, size)) {
            // Use the scaled addressing mode.
            return Emit(LoadStoreUnsignedOffsetFixed | memop |
                        ImmLSUnsigned(offset >> size));
        }

        if ((option != RequireScaledOffset) && IsImmLSUnscaled(offset)) {
            // Use the unscaled addressing mode.
            return Emit(LoadStoreUnscaledOffsetFixed | memop | ImmLS(offset));
        }
    }

    // All remaining addressing modes are register-offset, pre-indexed or
    // post-indexed modes.
    MOZ_ASSERT((option != RequireUnscaledOffset) &&
                (option != RequireScaledOffset));

    if (addr.IsRegisterOffset()) {
        Extend ext = addr.extend();
        Shift shift = addr.shift();
        unsigned shift_amount = addr.shift_amount();

        // LSL is encoded in the option field as UXTX.
        if (shift == LSL) {
            ext = UXTX;
        }

        // Shifts are encoded in one bit, indicating a left shift by the memory
        // access size.
        MOZ_ASSERT((shift_amount == 0) ||
                    (shift_amount == static_cast<unsigned>(CalcLSDataSize(op))));
        return Emit(LoadStoreRegisterOffsetFixed | memop | Rm(addr.regoffset()) |
                    ExtendMode(ext) | ImmShiftLS((shift_amount > 0) ? 1 : 0));
    }

    if (addr.IsPreIndex() && IsImmLSUnscaled(offset)) {
        return Emit(LoadStorePreIndexFixed | memop | ImmLS(offset));
    }

    if (addr.IsPostIndex() && IsImmLSUnscaled(offset)) {
        return Emit(LoadStorePostIndexFixed | memop | ImmLS(offset));
    }

    // If this point is reached, the MemOperand (addr) cannot be encoded.
    VIXL_UNREACHABLE();
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
    MOZ_ASSERT(is_int32(imm) || is_uint32(imm) || (rt.Is64Bits()));

    //FIXME: BlockLiteralPoolScope scope(this);
    MOZ_CRASH("LoadLiteral");
    RecordLiteral(imm, rt.SizeInBytes());
    Emit(op | ImmLLiteral(0) | Rt(rt));
}

void
AssemblerVIXL::LoadPCLiteral(const CPURegister &rt, ptrdiff_t pcInsOffset, LoadLiteralOp op)
{
    MOZ_ASSERT(is_int19(pcInsOffset));

    // The PCInsOffset is in units of Instruction.
    Emit(op | ImmLLiteral(pcInsOffset) | Rt(rt));
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
    MOZ_ASSERT((width == kWRegSize) || (width == kXRegSize));

    bool negate = false;

    // Logical immediates are encoded using parameters n, imm_s and imm_r using
    // the following table:
    //
    //    N   imms    immr    size        S             R
    //    1  ssssss  rrrrrr    64    UInt(ssssss)  UInt(rrrrrr)
    //    0  0sssss  xrrrrr    32    UInt(sssss)   UInt(rrrrr)
    //    0  10ssss  xxrrrr    16    UInt(ssss)    UInt(rrrr)
    //    0  110sss  xxxrrr     8    UInt(sss)     UInt(rrr)
    //    0  1110ss  xxxxrr     4    UInt(ss)      UInt(rr)
    //    0  11110s  xxxxxr     2    UInt(s)       UInt(r)
    // (s bits must not be all set)
    //
    // A pattern is constructed of size bits, where the least significant S+1 bits
    // are set. The pattern is rotated right by R, and repeated across a 32 or
    // 64-bit value, depending on destination register width.
    //
    // Put another way: the basic format of a logical immediate is a single
    // contiguous stretch of 1 bits, repeated across the whole word at intervals
    // given by a power of 2. To identify them quickly, we first locate the
    // lowest stretch of 1 bits, then the next 1 bit above that; that combination
    // is different for every logical immediate, so it gives us all the
    // information we need to identify the only logical immediate that our input
    // could be, and then we simply check if that's the value we actually have.
    //
    // (The rotation parameter does give the possibility of the stretch of 1 bits
    // going 'round the end' of the word. To deal with that, we observe that in
    // any situation where that happens the bitwise NOT of the value is also a
    // valid logical immediate. So we simply invert the input whenever its low bit
    // is set, and then we know that the rotated case can't arise.)

    if (value & 1) {
        // If the low bit is 1, negate the value, and set a flag to remember that we
        // did (so that we can adjust the return values appropriately).
        negate = true;
        value = ~value;
    }

    if (width == kWRegSize) {
        // To handle 32-bit logical immediates, the very easiest thing is to repeat
        // the input value twice to make a 64-bit word. The correct encoding of that
        // as a logical immediate will also be the correct encoding of the 32-bit
        // value.

        // Avoid making the assumption that the most-significant 32 bits are zero by
        // shifting the value left and duplicating it.
        value <<= kWRegSize;
        value |= value >> kWRegSize;
    }

    // The basic analysis idea: imagine our input word looks like this.
    //
    //    0011111000111110001111100011111000111110001111100011111000111110
    //                                                          c  b    a
    //                                                          |<--d-->|
    //
    // We find the lowest set bit (as an actual power-of-2 value, not its index)
    // and call it a. Then we add a to our original number, which wipes out the
    // bottommost stretch of set bits and replaces it with a 1 carried into the
    // next zero bit. Then we look for the new lowest set bit, which is in
    // position b, and subtract it, so now our number is just like the original
    // but with the lowest stretch of set bits completely gone. Now we find the
    // lowest set bit again, which is position c in the diagram above. Then we'll
    // measure the distance d between bit positions a and c (using CLZ), and that
    // tells us that the only valid logical immediate that could possibly be equal
    // to this number is the one in which a stretch of bits running from a to just
    // below b is replicated every d bits.
    uint64_t a = LowestSetBit(value);
    uint64_t value_plus_a = value + a;
    uint64_t b = LowestSetBit(value_plus_a);
    uint64_t value_plus_a_minus_b = value_plus_a - b;
    uint64_t c = LowestSetBit(value_plus_a_minus_b);

    int d, clz_a, out_n;
    uint64_t mask;

    if (c != 0) {
        // The general case, in which there is more than one stretch of set bits.
        // Compute the repeat distance d, and set up a bitmask covering the basic
        // unit of repetition (i.e. a word with the bottom d bits set). Also, in all
        // of these cases the N bit of the output will be zero.
        clz_a = CountLeadingZeros(a, kXRegSize);
        int clz_c = CountLeadingZeros(c, kXRegSize);
        d = clz_a - clz_c;
        mask = ((UINT64_C(1) << d) - 1);
        out_n = 0;
    } else {
        // Handle degenerate cases.
        //
        // If any of those 'find lowest set bit' operations didn't find a set bit at
        // all, then the word will have been zero thereafter, so in particular the
        // last lowest_set_bit operation will have returned zero. So we can test for
        // all the special case conditions in one go by seeing if c is zero.
        if (a == 0) {
            // The input was zero (or all 1 bits, which will come to here too after we
            // inverted it at the start of the function), for which we just return
            // false.
            return false;
        } else {
            // Otherwise, if c was zero but a was not, then there's just one stretch
            // of set bits in our word, meaning that we have the trivial case of
            // d == 64 and only one 'repetition'. Set up all the same variables as in
            // the general case above, and set the N bit in the output.
            clz_a = CountLeadingZeros(a, kXRegSize);
            d = 64;
            mask = ~UINT64_C(0);
            out_n = 1;
        }
    }

    // If the repeat period d is not a power of two, it can't be encoded.
    if (!IsPowerOfTwo(d)) {
        return false;
    }

    if (((b - a) & ~mask) != 0) {
        // If the bit stretch (b - a) does not fit within the mask derived from the
        // repeat period, then fail.
        return false;
    }

    // The only possible option is b - a repeated every d bits. Now we're going to
    // actually construct the valid logical immediate derived from that
    // specification, and see if it equals our original input.
    //
    // To repeat a value every d bits, we multiply it by a number of the form
    // (1 + 2^d + 2^(2d) + ...), i.e. 0x0001000100010001 or similar. These can
    // be derived using a table lookup on CLZ(d).
    static const uint64_t multipliers[] = {
        0x0000000000000001UL,
        0x0000000100000001UL,
        0x0001000100010001UL,
        0x0101010101010101UL,
        0x1111111111111111UL,
        0x5555555555555555UL,
    };
    uint64_t multiplier = multipliers[CountLeadingZeros(d, kXRegSize) - 57];
    uint64_t candidate = (b - a) * multiplier;
  
    if (value != candidate) {
        // The candidate pattern doesn't match our input value, so fail.
        return false;
    }
  
    // We have a match! This is a valid logical immediate, so now we have to
    // construct the bits and pieces of the instruction encoding that generates
    // it.
  
    // Count the set bits in our basic stretch. The special case of clz(0) == -1
    // makes the answer come out right for stretches that reach the very top of
    // the word (e.g. numbers like 0xffffc00000000000).
    int clz_b = (b == 0) ? -1 : CountLeadingZeros(b, kXRegSize);
    int s = clz_a - clz_b;
  
    // Decide how many bits to rotate right by, to put the low bit of that basic
    // stretch in position a.
    int r;
    if (negate) {
        // If we inverted the input right at the start of this function, here's
        // where we compensate: the number of set bits becomes the number of clear
        // bits, and the rotation count is based on position b rather than position
        // a (since b is the location of the 'lowest' 1 bit after inversion).
        s = d - s;
        r = (clz_b + 1) & (d - 1);
    } else {
        r = (clz_a + 1) & (d - 1);
    }
  
    // Now we're done, except for having to encode the S output in such a way that
    // it gives both the number of set bits and the length of the repeated
    // segment. The s field is encoded like this:
    //
    //     imms    size        S
    //    ssssss    64    UInt(ssssss)
    //    0sssss    32    UInt(sssss)
    //    10ssss    16    UInt(ssss)
    //    110sss     8    UInt(sss)
    //    1110ss     4    UInt(ss)
    //    11110s     2    UInt(s)
    //
    // So we 'or' (-d << 1) with our computed s to form imms.
    if ((n != NULL) || (imm_s != NULL) || (imm_r != NULL)) {
        *n = out_n;
        *imm_s = ((-d << 1) | (s - 1)) & 0x3f;
        *imm_r = r;
    }
  
    return true;
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
    MOZ_ASSERT(rt.IsValid());
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDR_x : LDR_w;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? LDR_d : LDR_s;
}

LoadStorePairOp
AssemblerVIXL::LoadPairOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    MOZ_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDP_x : LDP_w;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? LDP_d : LDP_s;
}

LoadStoreOp
AssemblerVIXL::StoreOpFor(const CPURegister& rt)
{
    MOZ_ASSERT(rt.IsValid());
    if (rt.IsRegister())
        return rt.Is64Bits() ? STR_x : STR_w;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? STR_d : STR_s;
}

LoadStorePairOp
AssemblerVIXL::StorePairOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    MOZ_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? STP_x : STP_w;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? STP_d : STP_s;
}

LoadStorePairNonTemporalOp
AssemblerVIXL::LoadPairNonTemporalOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    MOZ_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDNP_x : LDNP_w;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? LDNP_d : LDNP_s;
}

LoadStorePairNonTemporalOp
AssemblerVIXL::StorePairNonTemporalOpFor(const CPURegister& rt, const CPURegister& rt2)
{
    MOZ_ASSERT(AreSameSizeAndType(rt, rt2));
    USEARG(rt2);
    if (rt.IsRegister())
        return rt.Is64Bits() ? STNP_x : STNP_w;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? STNP_d : STNP_s;
}

LoadLiteralOp
AssemblerVIXL::LoadLiteralOpFor(const CPURegister &rt)
{
    if (rt.IsRegister())
        return rt.Is64Bits() ? LDR_x_lit : LDR_w_lit;

    MOZ_ASSERT(rt.IsFPRegister());
    return rt.Is64Bits() ? LDR_d_lit : LDR_s_lit;
}

// FIXME: Share with arm/Assembler-arm.cpp
struct PoolHeader
{
    uint32_t data;

    struct Header
    {
        // The size should take into account the pool header.
        // The size is in units of Instruction (4bytes), not byte.
        union {
            struct {
                uint32_t size : 15;
                bool isNatural : 1;
                uint32_t ONES : 16;
            };
            uint32_t data;
        };

        Header(int size_, bool isNatural_)
          : size(size_),
            isNatural(isNatural_),
            ONES(0xffff)
        { }

        Header(uint32_t data)
          : data(data)
        {
            JS_STATIC_ASSERT(sizeof(Header) == sizeof(uint32_t));
            MOZ_ASSERT(ONES == 0xffff);
        }

        uint32_t raw() const {
            JS_STATIC_ASSERT(sizeof(Header) == sizeof(uint32_t));
            return data;
        }
    };

    PoolHeader(int size_, bool isNatural_)
      : data(Header(size_, isNatural_).raw())
    { }

    uint32_t size() const {
        Header tmp(data);
        return tmp.size;
    }
    uint32_t isNatural() const {
        Header tmp(data);
        return tmp.isNatural;
    }

    // FIXME: Remove?
    /*
    static bool isTHIS(const Instruction &i) {
        return (*i.raw() & 0xffff0000) == 0xffff0000;
    }
    static const PoolHeader *asTHIS(const Instruction &i) {
        if (!isTHIS(i))
            return nullptr;
        return static_cast<const PoolHeader*>(&i);
    }
    */
};

// FIXME: Share with Assembler-arm.cpp
void
AssemblerVIXL::WritePoolHeader(uint8_t *start, Pool *p, bool isNatural)
{
    JS_STATIC_ASSERT(sizeof(PoolHeader) == 4);

    // Get the total size of the pool.
    uint8_t *pool = start + sizeof(PoolHeader) + p->getPoolSize();

    uintptr_t size = pool - start;
    MOZ_ASSERT((size & 3) == 0);
    size = size >> 2;
    MOZ_ASSERT(size < (1 << 15));

    PoolHeader header(size, isNatural);
    *(PoolHeader *)start = header;
}

// FIXME: Share with Assembler-arm.cpp
void
AssemblerVIXL::WritePoolFooter(uint8_t *start, Pool *p, bool isNatural)
{
    return;
}

void
AssemblerVIXL::WritePoolGuard(BufferOffset branch, Instruction *inst, BufferOffset dest)
{
    int byteOffset = dest.getOffset() - branch.getOffset();
    MOZ_ASSERT(byteOffset % kInstructionSize == 0);

    int instOffset = byteOffset >> kInstructionSizeLog2;
    b(inst, instOffset);
}

ptrdiff_t
AssemblerVIXL::GetBranchOffset(const Instruction *ins)
{
    MOZ_ASSERT_IF(!ins->IsBranchLinkImm(), ins->BranchType() != UnknownBranchType);
    // Convert from instruction offset to byte offset.
    return ins->ImmPCRawOffset() * kInstructionSize;
}

void
AssemblerVIXL::RetargetNearBranch(Instruction *i, int offset, Condition cond, bool final)
{
    if (i->IsCondBranchImm()) {
        MOZ_ASSERT(i->IsCondB());
        b(i, offset, cond);
        return;
    }

    MOZ_CRASH("Unsupported branch type");
}

void
AssemblerVIXL::RetargetNearBranch(Instruction *i, int byteOffset, bool final)
{
    // We expect the offset in instructions, the buffer gives it in bytes.
    JS_STATIC_ASSERT(kInstructionSize == 4);
    MOZ_ASSERT(byteOffset % kInstructionSize == 0);
    int instOffset = byteOffset >> kInstructionSizeLog2;

    // The only valid conditional instruction is B.
    if (i->IsCondBranchImm()) {
        MOZ_ASSERT(i->IsCondB());
        Condition cond = static_cast<Condition>(i->ConditionBranch());
        b(i, instOffset, cond);
        return;
    }

    // Valid unconditional branches are B and BL.
    if (i->IsUncondBranchImm()) {
        if (i->IsUncondB()) {
            b(i, instOffset);
        } else {
            MOZ_ASSERT(i->IsBL());
            bl(i, instOffset);
        }

        MOZ_ASSERT(i->ImmUncondBranch() == instOffset);
        return;
    }

    // Valid compare branches are CBZ and CBNZ.
    if (i->IsCompareBranch()) {
        ARMRegister rt = i->SixtyFourBits() ? ARMRegister::XRegFromCode(i->Rt())
                                            : ARMRegister::WRegFromCode(i->Rt());

        if (i->IsCBZ()) {
            cbz(i, rt, instOffset);
        } else {
            MOZ_ASSERT(i->IsCBNZ());
            cbnz(i, rt, instOffset);
        }

        MOZ_ASSERT(i->ImmCmpBranch() == instOffset);
        return;
    }

    // Valid test branches are TBZ and TBNZ.
    if (i->IsTestBranch()) {
        // Opposite of ImmTestBranchBit(): MSB in bit 5, 0:5 at bit 40.
        unsigned bit_pos = (i->ImmTestBranchBit5() << 5) | (i->ImmTestBranchBit40());
        MOZ_ASSERT(is_uint6(bit_pos));

        // Register size doesn't matter for the encoding.
        ARMRegister rt = ARMRegister::XRegFromCode(i->Rt());

        //unsigned bit_pos = i->ImmTestBranchBit();
        if (i->IsTBZ()) {
            tbz(i, rt, bit_pos, instOffset);
        } else {
            MOZ_ASSERT(i->IsTBNZ());
            tbnz(i, rt, bit_pos, instOffset);
        }

        MOZ_ASSERT(i->ImmTestBranch() == instOffset);
        return;
    }

    MOZ_CRASH("Unsupported branch type");
}

void
AssemblerVIXL::RetargetFarBranch(Instruction *i, uint8_t **slot, uint8_t *dest, Condition cond)
{
    MOZ_CRASH("RetargetFarBranch()");
}

void
AssemblerVIXL::RecordLiteral(int64_t imm, unsigned size)
{
    //FIXME: literals_.push_front(new Literal(pc_, imm, size));
    MOZ_CRASH("RecordLiteral");
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
        } else if (regs[i].IsFPRegister()) {
            number_of_valid_fpregs++;
            unique_fpregs |= regs[i].Bit();
        } else {
            MOZ_ASSERT(!regs[i].IsValid());
        }
    }

    int number_of_unique_regs = CountSetBits(unique_regs, sizeof(unique_regs) * 8);
    int number_of_unique_fpregs = CountSetBits(unique_fpregs, sizeof(unique_fpregs) * 8);

    MOZ_ASSERT(number_of_valid_regs >= number_of_unique_regs);
    MOZ_ASSERT(number_of_valid_fpregs >= number_of_unique_fpregs);

    return (number_of_valid_regs != number_of_unique_regs) ||
           (number_of_valid_fpregs != number_of_unique_fpregs);
}

bool
AreSameSizeAndType(const CPURegister& reg1, const CPURegister& reg2,
                   const CPURegister& reg3, const CPURegister& reg4,
                   const CPURegister& reg5, const CPURegister& reg6,
                   const CPURegister& reg7, const CPURegister& reg8)
{
    MOZ_ASSERT(reg1.IsValid());
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
