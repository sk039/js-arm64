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

#include "assembler/jit/ExecutableAllocator.h"
#include "gc/Marking.h"
#include "jit/arm64/MacroAssembler-arm64.h"
#include "jit/JitCompartment.h"

using namespace js;
using namespace js::jit;

using mozilla::CountLeadingZeroes32;

// Note this is used for inter-AsmJS calls and may pass arguments and results
// in floating point registers even if the system ABI does not.

ABIArg
ABIArgGenerator::next(MIRType type)
{
    JS_ASSERT(0 && "ABIArgGenerator::next");
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

    JS_ASSERT(0 && "bind (RepatchLabel)");
}

// FIXME: Shouldn't this be a static method of Assembler?
void
PatchJump(CodeLocationJump &jump_, CodeLocationLabel label) {
    JS_ASSERT(0 && "PatchJump()");
}

void
Assembler::PatchDataWithValueCheck(CodeLocationLabel label, PatchedImmPtr newValue,
                                   PatchedImmPtr expected)
{
    Instruction *i = (Instruction *)label.raw();

    // FIXME: Just emits a breakpoint for now until we can test it.
    AssemblerVIXL::Emit(i, BRK | AssemblerVIXL::ImmException(0x7777));
}

void
Assembler::PatchDataWithValueCheck(CodeLocationLabel label, ImmPtr newValue, ImmPtr expected)
{
    PatchDataWithValueCheck(label, PatchedImmPtr(newValue.value), PatchedImmPtr(expected.value));
}

} // namespace jit
} // namespace js
