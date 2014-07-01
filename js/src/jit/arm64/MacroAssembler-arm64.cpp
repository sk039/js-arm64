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

#include "jit/arm64/MacroAssembler-arm64.h"

#include "jit/IonMacroAssembler.h"

namespace js {
namespace jit {

void
MacroAssembler::PushRegsInMask(RegisterSet set)
{
    // FIXME: Are we storing the full 128 bits or what?
    int32_t diffF = set.fpus().size() * sizeof(double);
    int32_t diffG = set.gprs().size() * sizeof(intptr_t);

    // TODO: Clean up this function using helpers. Should be easy.
    for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); ) {
        CPURegister src0 = NoCPUReg;
        CPURegister src1 = NoCPUReg;
        CPURegister src2 = NoCPUReg;
        CPURegister src3 = NoCPUReg;

        src0 = ARMRegister(*iter, 64);
        ++iter;

        if (iter.more()) {
            src1 = ARMRegister(*iter, 64);
            ++iter;
        }

        if (iter.more()) {
            src2 = ARMRegister(*iter, 64);
            ++iter;
        }

        if (iter.more()) {
            src3 = ARMRegister(*iter, 64);
            ++iter;
        }

        MacroAssemblerVIXL::Push(src0, src1, src2, src3);
    }

    for (FloatRegisterBackwardIterator iter(set.fpus()); iter.more(); ) {
        CPURegister src0 = NoCPUReg;
        CPURegister src1 = NoCPUReg;
        CPURegister src2 = NoCPUReg;
        CPURegister src3 = NoCPUReg;

        src0 = ARMFPRegister(*iter, 64);
        ++iter;

        if (iter.more()) {
            src1 = ARMFPRegister(*iter, 64);
            ++iter;
        }

        if (iter.more()) {
            src2 = ARMFPRegister(*iter, 64);
            ++iter;
        }

        if (iter.more()) {
            src3 = ARMFPRegister(*iter, 64);
            ++iter;
        }

        MacroAssemblerVIXL::Push(src0, src1, src2, src3);
    }
}

void
MacroAssembler::PopRegsInMaskIgnore(RegisterSet set, RegisterSet ignore) {
    JS_ASSERT(0 && "PopRegsInMaskIgnore()");
}

void
MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output) {
    JS_ASSERT(0 && "clampDoubleToUint8()");
}

void
MacroAssemblerCompat::handleFailureWithHandlerTail()
{
    brk(0x13);
}

void
MacroAssemblerCompat::setupABICall(uint32_t args)
{
    JS_ASSERT(!inCall_);
    inCall_ = true;

    args_ = args;
    passedIntArgs_ = 0;
    passedFloatArgs_ = 0;
    stackForCall_ = ShadowStackSpace;
}

// FIXME: This could probably be shared code. Looks like all the arches are the same,
// just using arch-specific functions for no reason.
void
MacroAssemblerCompat::setupUnalignedABICall(uint32_t args, Register scratch)
{
    setupABICall(args);
    dynamicAlignment_ = true;

    int32_t alignment = ~(StackAlignment - 1);
    And(ARMRegister(scratch, 64), GetStackPointer(), Operand(alignment));
    push(scratch);
}

void
MacroAssemblerCompat::passABIArg(const MoveOperand &from, MoveOp::Type type)
{
    if (!enoughMemory_)
        return;

    Register activeSP = Register::FromCode(GetStackPointer().code());

    if (type == MoveOp::GENERAL) {
        Register dest;
        if (GetIntArgReg(passedIntArgs_++, passedFloatArgs_, &dest)) {
            if (!from.isGeneralReg() || from.reg() != dest)
                enoughMemory_ = moveResolver_.addMove(from, MoveOperand(dest), type);
            return;
        }

        enoughMemory_ = moveResolver_.addMove(from, MoveOperand(activeSP, stackForCall_), type);
        stackForCall_ += sizeof(int64_t);
        return;
    }

    JS_ASSERT(type == MoveOp::FLOAT32 || type == MoveOp::DOUBLE);

    FloatRegister fdest;
    if (GetFloatArgReg(passedIntArgs_, passedFloatArgs_++, &fdest)) {
        if (!from.isFloatReg() || from.floatReg() != fdest) 
            enoughMemory_ = moveResolver_.addMove(from, MoveOperand(fdest), type);
        return;
    }

    enoughMemory_ = moveResolver_.addMove(from, MoveOperand(activeSP, stackForCall_), type);
    switch (type) {
      case MoveOp::FLOAT32: stackForCall_ += sizeof(float);  break;
      case MoveOp::DOUBLE:  stackForCall_ += sizeof(double); break;
      default: MOZ_ASSUME_UNREACHABLE("Unexpected float register class argument type");
    }
}

void
MacroAssemblerCompat::passABIArg(Register reg)
{
    passABIArg(MoveOperand(reg), MoveOp::GENERAL);
}

void
MacroAssemblerCompat::passABIArg(FloatRegister reg, MoveOp::Type type)
{
    passABIArg(MoveOperand(reg), type);
}

void
MacroAssemblerCompat::callWithABIPre(uint32_t *stackAdjust)
{
    // TODO: Implement.
    Brk(0x1111);
}

void
MacroAssemblerCompat::callWithABIPost(uint32_t stackAdjust, MoveOp::Type result)
{
    // TODO: Implement.
    Brk(0x1222);
}

void
MacroAssemblerCompat::callWithABI(void *fun, MoveOp::Type result)
{
    // Load the target into an intra-call-use register.
    loadPtr(ImmWord(fun), ip0);

    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(ip0);
    callWithABIPost(stackAdjust, result);
}

} // namespace jit
} // namespace js
