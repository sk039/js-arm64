/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/arm64/Assembler-arm64.h"

#include "mozilla/DebugOnly.h"
#include "mozilla/MathAlgorithms.h"

#include "jscompartment.h"
#include "jsutil.h"

#include "gc/Marking.h"

#include "jit/arm64/MacroAssembler-arm64.h"
#include "jit/ExecutableAllocator.h"
#include "jit/JitCompartment.h"

using namespace js;
using namespace js::jit;

using mozilla::CountLeadingZeroes32;

// Note this is used for inter-AsmJS calls and may pass arguments and results
// in floating point registers even if the system ABI does not.

ABIArg
ABIArgGenerator::next(MIRType type)
{
    switch (type) {
      case MIRType_Int32:
      case MIRType_Pointer:
        if (intRegIndex_ == numIntArgRegs) {
            current_ = ABIArg(stackOffset_);
            stackOffset_ += sizeof(uintptr_t);
            break;
        }
        current_ = ABIArg(Register::FromCode(intRegIndex_));
        intRegIndex_++;
        break;
      case MIRType_Float32:
      case MIRType_Double:
        if (floatRegIndex_ == NumFloatArgRegs) {
            current_ = ABIArg(stackOffset_);
            stackOffset_ += sizeof(double);
            break;
        }
        current_ = ABIArg(FloatRegister::FromCode(floatRegIndex_));
        floatRegIndex_++;
        break;
      default:
        MOZ_CRASH("Unexpected argument type");
    }
    return current_;
}

const Register ABIArgGenerator::NonArgReturnReg0 = r4;
const Register ABIArgGenerator::NonArgReturnReg1 = r5;
const Register ABIArgGenerator::NonVolatileReg = r1;
const Register ABIArgGenerator::NonArg_VolatileReg = r13;
const Register ABIArgGenerator::NonReturn_VolatileReg0 = r2;
const Register ABIArgGenerator::NonReturn_VolatileReg1 = r3;

namespace js {
namespace jit {

void
Assembler::finish()
{
    armbuffer_.flushPool();

    // The extended jump table is part of the code buffer.
    emitExtendedJumpTable();

    AssemblerVIXL::FinalizeCode();

    for (unsigned int i = 0; i < tmpDataRelocations_.length(); i++) {
        int offset = tmpDataRelocations_[i].getOffset();
        int real_offset = offset + armbuffer_.poolSizeBefore(offset);
        dataRelocations_.writeUnsigned(real_offset);
    }

    for (unsigned int i = 0; i < tmpJumpRelocations_.length(); i++) {
        int offset = tmpJumpRelocations_[i].jump.getOffset();
        int real_offset = offset + armbuffer_.poolSizeBefore(offset);
        jumpRelocations_.writeUnsigned(real_offset);
    }

    for (unsigned int i = 0; i < tmpPreBarriers_.length(); i++) {
        int offset = tmpPreBarriers_[i].getOffset();
        int real_offset = offset + armbuffer_.poolSizeBefore(offset);
        preBarriers_.writeUnsigned(real_offset);
    }
}

void
Assembler::emitExtendedJumpTable()
{
    if (!pendingJumps_.length() || oom())
        return;

    armbuffer_.align(SizeOfJumpTableEntry);

    for (size_t i = 0; i < pendingJumps_.length(); i++) {
        // Each jump entry is of the form:
        //   LDR ip0 [PC, 8]
        //   BR ip0
        //   [Patchable 8-byte constant low bits]
        //   [Patchable 8-byte constant high bits]
        ldr(ip0_64, ptrdiff_t(2 * sizeof(Instruction)));
        br(ip0_64);
        brk(0x0);
        brk(0x0);
    }
}

BufferOffset
Assembler::immPool(ARMRegister dest, uint8_t *value, LoadLiteralOp op)
{
    uint32_t inst = op | Rt(dest);
    const size_t numInst = 1;
    const unsigned numPoolEntries = 2;
    return armbuffer_.allocEntry(numInst, numPoolEntries, (uint8_t*)&inst, value);
}

BufferOffset
Assembler::immPool64(ARMRegister dest, uint64_t value)
{
    return immPool(dest, (uint8_t*)&value, LDR_x_lit);
}

void
Assembler::bind(Label *label, BufferOffset targetOffset)
{
    // Nothing has seen the label yet: just mark the location.
    if (!label->used()) {
        label->bind(targetOffset.getOffset());
        return;
    }

    Instruction *target = getInstructionAt(targetOffset);

    // Get the most recent instruction that used the label, as stored in the label.
    // This instruction is the head of an implicit linked list of label uses.
    uint32_t branchOffset = label->offset();

    while (branchOffset != LabelBase::INVALID_OFFSET) {
        Instruction *link = getInstructionAt(BufferOffset(branchOffset));

        // Before overwriting the offset in this instruction, get the offset of
        // the next link in the implicit branch list.
        uint32_t nextLinkOffset = uint32_t(link->ImmPCRawOffset());

        // Write a new relative offset into the instruction.
        link->SetImmPCOffsetTarget(target);
        branchOffset = nextLinkOffset;
    }

    // Bind the label, so that future uses may encode the offset immediately.
    label->bind(targetOffset.getOffset());
}

void
Assembler::bind(RepatchLabel *label)
{
    // Nothing has seen the label yet: just mark the location.
    if (!label->used()) {
        label->bind(nextOffset().getOffset());
        return;
    }

    MOZ_ASSERT(0 && "bind (RepatchLabel)");
}

// FIXME: Share with ARM?
void
Assembler::trace(JSTracer *trc)
{
    for (size_t i = 0; i < pendingJumps_.length(); i++) {
        RelativePatch &rp = pendingJumps_[i];
        if (rp.kind == Relocation::JITCODE) {
            JitCode *code = JitCode::FromExecutable((uint8_t *)rp.target);
            MarkJitCodeUnbarriered(trc, &code, "masmrel32");
            MOZ_ASSERT(code == JitCode::FromExecutable((uint8_t *)rp.target));
        }
    }

    //FIXME
#if 0
    if (tmpDataRelocations_.length())
        ::TraceDataRelocations(trc, &armbuffer_, &tmpDataRelocations_);
#endif
}

void
Assembler::addJumpRelocation(BufferOffset src, Relocation::Kind reloc)
{
    // Only JITCODE relocations are patchable at runtime.
    MOZ_ASSERT(reloc == Relocation::JITCODE);

    // Each relocation requires an entry in the extended jump table.
    tmpJumpRelocations_.append(JumpRelocation(src, pendingJumps_.length()));
}

void
Assembler::addPendingJump(BufferOffset src, ImmPtr target, Relocation::Kind reloc)
{
    MOZ_ASSERT(target.value != nullptr);

    if (reloc == Relocation::JITCODE)
        addJumpRelocation(src, reloc);

    // This jump is not patchable at runtime. Extended jump table entry requirements
    // cannot be known until finalization, so to be safe, give each jump and entry.
    // This also causes GC tracing of the target.
    enoughMemory_ &= pendingJumps_.append(RelativePatch(src, target.value, reloc));
}

size_t
Assembler::addPatchableJump(BufferOffset src, Relocation::Kind reloc)
{
    if (reloc == Relocation::JITCODE)
        addJumpRelocation(src, reloc);

    size_t extendedTableIndex = pendingJumps_.length();
    enoughMemory_ &= pendingJumps_.append(RelativePatch(src, nullptr, reloc));
    return extendedTableIndex;
}

// FIXME: Shouldn't this be a static method of Assembler?
void
PatchJump(CodeLocationJump &jump_, CodeLocationLabel label) {
    MOZ_ASSERT(0 && "PatchJump()");
}

void
Assembler::PatchDataWithValueCheck(CodeLocationLabel label, PatchedImmPtr newValue,
                                   PatchedImmPtr expected)
{
    Instruction *i = (Instruction *)label.raw();
    void** pValue = reinterpret_cast<void**>(i->LiteralAddress());
    MOZ_ASSERT(*pValue == expected.value);
    *pValue = newValue.value;
}

void
Assembler::PatchDataWithValueCheck(CodeLocationLabel label, ImmPtr newValue, ImmPtr expected)
{
    PatchDataWithValueCheck(label, PatchedImmPtr(newValue.value), PatchedImmPtr(expected.value));
}

void
Assembler::ToggleToJmp(CodeLocationLabel inst_)
{
    Instruction *i = (Instruction *)inst_.raw();
    MOZ_ASSERT(i->IsAddSubImmediate());

    // Refer to instruction layout in ToggleToCmp().
    int imm19 = (int)i->Bits(23, 5);
    MOZ_ASSERT(is_int19(imm19));

    b(i, imm19, Always);
}

void
Assembler::ToggleToCmp(CodeLocationLabel inst_)
{
    Instruction *i = (Instruction *)inst_.raw();
    MOZ_ASSERT(i->IsCondB());

    int imm19 = i->ImmCondBranch();
    MOZ_ASSERT(is_int19(imm19));

    // 31 - 64-bit if set, 32-bit if unset. (OK!)
    // 30 - sub if set, add if unset. (OK!)
    // 29 - SetFlagsBit. Must be set.
    // 22:23 - ShiftAddSub. (OK!)
    // 10:21 - ImmAddSub. (OK!)
    // 5:9 - First source register (Rn). (OK!)
    // 0:4 - Destination Register. Must be xzr.

    // From the above, there is a safe 19-bit contiguous region from 5:23.
    Emit(i, ThirtyTwoBits | AddSubImmediateFixed | SUB | Flags(SetFlags) |
            Rd(xzr) | (imm19 << Rn_offset));
}

void
Assembler::ToggleCall(CodeLocationLabel inst_, bool enabled)
{
    MOZ_CRASH("ToggleCall()");
}

void
Assembler::TraceJumpRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader)
{
    MOZ_CRASH("TraceJumpRelocations()");
}

void
Assembler::TraceDataRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader)
{
    MOZ_CRASH("TraceDataRelocations()");
}

int32_t
Assembler::ExtractCodeLabelOffset(uint8_t *code)
{
    MOZ_CRASH("ExtractCodeLabelOffset()");
}

void
Assembler::PatchInstructionImmediate(uint8_t *code, PatchedImmPtr imm)
{
    MOZ_CRASH("PatchInstructionImmediate()");
}

} // namespace jit
} // namespace js
