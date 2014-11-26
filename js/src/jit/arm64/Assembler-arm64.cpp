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
using mozilla::DebugOnly;

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
    BufferOffset extendedJumpTable = emitExtendedJumpTable();
    AssemblerVIXL::FinalizeCode();

    // The jump relocation table starts with a fixed-width integer pointing
    // to the start of the extended jump table.
    if (tmpJumpRelocations_.length())
        jumpRelocations_.writeFixedUint32_t(toFinalOffset(extendedJumpTable));

    for (unsigned int i = 0; i < tmpJumpRelocations_.length(); i++) {
        JumpRelocation &reloc = tmpJumpRelocations_[i];

        // Each entry in the relocations table is an (offset, extendedTableIndex) pair.
        jumpRelocations_.writeUnsigned(toFinalOffset(reloc.jump));
        jumpRelocations_.writeUnsigned(reloc.extendedTableIndex);
    }

    for (unsigned int i = 0; i < tmpDataRelocations_.length(); i++)
        dataRelocations_.writeUnsigned(toFinalOffset(tmpDataRelocations_[i]));

    for (unsigned int i = 0; i < tmpPreBarriers_.length(); i++)
        preBarriers_.writeUnsigned(toFinalOffset(tmpPreBarriers_[i]));
}

BufferOffset
Assembler::emitExtendedJumpTable()
{
    if (!pendingJumps_.length() || oom())
        return BufferOffset();

    armbuffer_.flushPool();
    armbuffer_.align(SizeOfJumpTableEntry);

    BufferOffset tableOffset = armbuffer_.nextOffset();

    for (size_t i = 0; i < pendingJumps_.length(); i++) {
        // Each jump entry is of the form:
        //   LDR ip0 [PC, 8]
        //   BR ip0
        //   [Patchable 8-byte constant low bits]
        //   [Patchable 8-byte constant high bits]
        DebugOnly<size_t> preOffset = size_t(armbuffer_.nextOffset().getOffset());

        ldr(ip0_64, ptrdiff_t(2 * kInstructionSize));
        br(ip0_64);

        DebugOnly<size_t> prePointer = size_t(armbuffer_.nextOffset().getOffset());
        MOZ_ASSERT(prePointer - preOffset == OffsetOfJumpTableEntryPointer);

        brk(0x0);
        brk(0x0);

        DebugOnly<size_t> postOffset = size_t(armbuffer_.nextOffset().getOffset());

        MOZ_ASSERT(postOffset - preOffset == SizeOfJumpTableEntry);
    }

    return tableOffset;
}

void
Assembler::executableCopy(uint8_t *buffer)
{
    // Copy the code and all constant pools into the output buffer.
    armbuffer_.executableCopy(buffer);

    // Patch any relative jumps that target code outside the buffer.
    // The extended jump table may be used for distant jumps.
    for (size_t i = 0; i < pendingJumps_.length(); i++) {
        RelativePatch &rp = pendingJumps_[i];

        if (!rp.target) {
            // The patch target is nullptr for jumps that have been linked to
            // a label within the same code block, but may be repatched later
            // to jump to a different code block.
            continue;
        }

        Instruction *target = (Instruction *)rp.target;
        Instruction *branch = (Instruction *)(buffer + toFinalOffset(rp.offset));
        if(branch->BranchType() != UnknownBranchType) {
            if (branch->IsTargetReachable(target)) {
                branch->SetImmPCOffsetTarget(target);
            } else {
                MOZ_CRASH("Implement extended jump table patching");
            }
        } else {
            // currently a nop, get the offset, and stick it in a cmp instruction
            bl(branch, 0);
            branch->SetImmPCOffsetTarget(target);
            // Turn it off, don't bother duplicating the logic here.
            ToggleCall(CodeLocationLabel((uint8_t*)branch), false);
        }
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

BufferOffset
Assembler::fImmPool(ARMFPRegister dest, uint8_t *value, LoadLiteralOp op)
{
    uint32_t inst = op | Rt(dest);
    const size_t numInst = 1;
    const unsigned numPoolEntries = dest.size() / 32;
    return armbuffer_.allocEntry(numInst, numPoolEntries, (uint8_t*)&inst, value);
}

BufferOffset
Assembler::fImmPool64(ARMFPRegister dest, double value)
{
    return fImmPool(dest, (uint8_t*)&value, LDR_d_lit);
}
BufferOffset
Assembler::fImmPool32(ARMFPRegister dest, float value)
{
    return fImmPool(dest, (uint8_t*)&value, LDR_s_lit);
}

void
Assembler::bind(Label *label, BufferOffset targetOffset)
{
    // Nothing has seen the label yet: just mark the location.
    if (!label->used()) {
        label->bind(targetOffset.getOffset());
        return;
    }

    // Get the most recent instruction that used the label, as stored in the label.
    // This instruction is the head of an implicit linked list of label uses.
    uint32_t branchOffset = label->offset();

    while ((int32_t)branchOffset != LabelBase::INVALID_OFFSET) {
        Instruction *link = getInstructionAt(BufferOffset(branchOffset));

        // Before overwriting the offset in this instruction, get the offset of
        // the next link in the implicit branch list.
        uint32_t nextLinkOffset = uint32_t(link->ImmPCRawOffset());

        // Linking against the actual (Instruction *) would be invalid,
        // since that Instruction could be anywhere in memory.
        // Instead, just link against the correct relative offset, assuming
        // no constant pools, which will be taken into consideration
        // during finalization.
        ptrdiff_t relativeByteOffset = targetOffset.getOffset() - branchOffset;
        Instruction *target = (Instruction *)(((uint8_t *)link) + relativeByteOffset);

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

    MOZ_CRASH("bind (RepatchLabel)");
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
    MOZ_CRASH("PatchJump()");
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
    // bit 23 is reserved, and the simulator throws an assertion when this happens
    // It'll be messy to decode, but we can steal bit 30 or bit 31.
    MOZ_ASSERT(is_int18(imm19));

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
    Instruction *i = (Instruction *)inst_.raw();
    if (i->IsBL() == enabled)
        return;

    if (i->IsBL()) {
        // There is no way that 26 bits are fitting into a nop
        // The destination should be stored out of line, but there is presently
        // no infrastructure for this.
        MOZ_ASSERT( i->SignedBits(25, 0) ==  i->SignedBits(17, 0));
        // now that the offset definitely fits into 18 bits, grab those 18 bits.
        int imm26 = i->Bits(17, 0);
        // 31 - 64-bit if set, 32-bit if unset. (OK!)
        // 30 - sub if set, add if unset. (OK!)
        // 29 - SetFlagsBit. Must be set.
        // 22:23 - ShiftAddSub. (OK!)
        // 10:21 - ImmAddSub. (OK!)
        // 5:9 - First source register (Rn). (OK!)
        // 0:4 - Destination Register. Must be xzr.

        // From the above, there is a safe 19-bit contiguous region from 5:23.
        Emit(i, ThirtyTwoBits | AddSubImmediateFixed | SUB | Flags(SetFlags) |
             Rd(xzr) | (imm26 << Rn_offset));
    } else {
        // Refer to instruction layout in ToggleToCmp().
        MOZ_ASSERT(i->IsAddSubImmediate());
        
        int imm19 = (int)i->SignedBits(22, 5);
        MOZ_ASSERT(is_int19(imm19));
        bl(i, imm19);
    }
}

class RelocationIterator
{
    CompactBufferReader reader_;
    uint32_t tableStart_;
    uint32_t offset_;
    uint32_t extOffset_;

  public:
    explicit RelocationIterator(CompactBufferReader &reader)
      : reader_(reader)
    {
        // The first uint32_t stores the extended table offset.
        tableStart_ = reader_.readFixedUint32_t();
    }

    bool read() {
        if (!reader_.more())
            return false;
        offset_ = reader_.readUnsigned();
        extOffset_ = reader_.readUnsigned();
        return true;
    }

    uint32_t offset() const {
        return offset_;
    }
    uint32_t extendedOffset() const {
        return extOffset_;
    }
};

static JitCode *
CodeFromJump(JitCode *code, uint8_t *jump)
{
    Instruction *branch = (Instruction *)jump;
    uint8_t *target = (uint8_t *)branch->ImmPCOffsetTarget();

    // If the jump is within the code buffer, it uses the extended jump table.
    if (target >= code->raw() && target < code->raw() + code->instructionsSize()) {
        MOZ_ASSERT(target + Assembler::SizeOfJumpTableEntry <= code->raw() + code->instructionsSize());

        uint8_t **patchablePtr = (uint8_t **)(target + Assembler::OffsetOfJumpTableEntryPointer);
        target = *patchablePtr;
    }

    return JitCode::FromExecutable(target);
}

void
Assembler::TraceJumpRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader)
{
    RelocationIterator iter(reader);
    while (iter.read()) {
        JitCode *child = CodeFromJump(code, code->raw() + iter.offset());
        MarkJitCodeUnbarriered(trc, &child, "rel32");
        MOZ_ASSERT(child == CodeFromJump(code, code->raw() + iter.offset()));
    }
}

static void
TraceDataRelocations(JSTracer *trc, uint8_t *buffer, CompactBufferReader &reader)
{
    while (reader.more()) {
        size_t offset = reader.readUnsigned();
        Instruction *load = (Instruction *)&buffer[offset];

        // The only valid traceable operation is a 64-bit load to an ARMRegister.
        // Refer to movePatchablePtr() for generation.
        MOZ_ASSERT(load->Mask(LoadLiteralMask) == LDR_x_lit);

        uint32_t pcOffset = load->ImmLLiteral();
        uint8_t *literalAddr = ((uint8_t *)load) + (pcOffset << kLiteralEntrySizeLog2);

        // All pointers on AArch64 will have the top bits cleared.
        // If those bits are not cleared, this must be a Value.
        uintptr_t *word = reinterpret_cast<uintptr_t *>(literalAddr);
        if (*word >> JSVAL_TAG_SHIFT) {
            jsval_layout layout;
            layout.asBits = *word;
            Value v = IMPL_TO_JSVAL(layout);
            gc::MarkValueUnbarriered(trc, &v, "ion-masm-value");
            *word = JSVAL_TO_IMPL(v).asBits; // TODO: Need to flush caches?
            continue;
        }

        // No barriers needed since the pointers are constants.
        gc::MarkGCThingUnbarriered(trc, reinterpret_cast<void **>(literalAddr), "ion-masm-ptr");
    }
}

void
Assembler::TraceDataRelocations(JSTracer *trc, JitCode *code, CompactBufferReader &reader)
{
    ::TraceDataRelocations(trc, code->raw(), reader);
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
