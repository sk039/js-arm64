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

#include "jit/arm64/BaselineRegisters-arm64.h"
#include "jit/arm64/MoveEmitter-arm64.h"
#include "jit/BaselineFrame.h"
#include "jit/MacroAssembler.h"

namespace js {
namespace jit {

// TODO: What on earth is simdSet?
void
MacroAssembler::PushRegsInMask(RegisterSet set, FloatRegisterSet simdSet)
{
    // TODO: Clean up this function using helpers. Should be easy.
    for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); ) {
        CPURegister src0 = NoCPUReg;
        CPURegister src1 = NoCPUReg;
        CPURegister src2 = NoCPUReg;
        CPURegister src3 = NoCPUReg;

        src0 = ARMRegister(*iter, 64);
        adjustFrame(8);
        ++iter;

        if (iter.more()) {
            src1 = ARMRegister(*iter, 64);
            ++iter;
            adjustFrame(8);
        }

        if (iter.more()) {
            src2 = ARMRegister(*iter, 64);
            ++iter;
            adjustFrame(8);
        }

        if (iter.more()) {
            src3 = ARMRegister(*iter, 64);
            ++iter;
            adjustFrame(8);
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
        adjustFrame(8);

        if (iter.more()) {
            src1 = ARMFPRegister(*iter, 64);
            ++iter;
            adjustFrame(8);
        }

        if (iter.more()) {
            src2 = ARMFPRegister(*iter, 64);
            ++iter;
            adjustFrame(8);
        }

        if (iter.more()) {
            src3 = ARMFPRegister(*iter, 64);
            ++iter;
            adjustFrame(8);
        }

        MacroAssemblerVIXL::Push(src0, src1, src2, src3);
    }
}

// TODO: What on earth is simdSet?
void
MacroAssembler::PopRegsInMaskIgnore(RegisterSet set, RegisterSet ignore, FloatRegisterSet simdSet)
{
    // The offset that we'll be loading from
    uint32_t offset = 0;
    // The offset that offset will be updated to after the current load.
    uint32_t nextOffset = 0;
    for (FloatRegisterIterator iter(set.fpus()); iter.more(); offset = nextOffset) {
        CPURegister src0 = NoCPUReg;
        CPURegister src1 = NoCPUReg;

        while (iter.more() && ignore.has(*iter)) {
            ++iter;
            offset += sizeof(double);
        }

        nextOffset = offset;

        if (!iter.more())
            break;

        src0 = ARMFPRegister(*iter, 64);
        nextOffset += sizeof(double);
        ++iter;

        if (!iter.more() || ignore.has(*iter)) {
            // There is no 'next' that can be loaded, and there is already one
            // element in the queue, just deal with that element.
            Ldr(src0, MemOperand(GetStackPointer64(), offset));
            continue;
        }

        // There is both more, and it isn't being ignored.
        src1 = ARMFPRegister(*iter, 64);
        nextOffset += sizeof(double);
        ++iter;

        MOZ_ASSERT(!src0.Is(src1));
        ldp(src0, src1, MemOperand(GetStackPointer64(), offset));
    }

    FloatRegisterSet frs = set.fpus();
    MOZ_ASSERT(nextOffset <= frs.getPushSizeInBytes());
    nextOffset = set.fpus().getPushSizeInBytes();

    for (GeneralRegisterIterator iter(set.gprs()); iter.more(); offset = nextOffset) {
        CPURegister src0 = NoCPUReg;
        CPURegister src1 = NoCPUReg;
        while (iter.more() && ignore.has(*iter)) {
            ++iter;
            offset += sizeof(double);
        }

        nextOffset = offset;

        if (!iter.more())
            break;

        src0 = ARMRegister(*iter, 64);
        nextOffset += sizeof(double);
        ++iter;
        if (!iter.more() || ignore.has(*iter)) {
            // There is no 'next' that can be loaded, and there is already one
            // element in the queue, just deal with that element.
            Ldr(src0, MemOperand(GetStackPointer64(), offset));
            continue;
        }
        // There is both more, and it isn't being ignored.
        src1 = ARMRegister(*iter, 64);
        ++iter;
        nextOffset += sizeof(double);
        ldp(src0, src1, MemOperand(GetStackPointer64(), offset));
    }
    freeStack(set.gprs().size() * sizeof(int*) + set.fpus().getPushSizeInBytes());
}

void
MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output)
{
    ARMRegister dest(output, 32);
    fcvtns(dest, ARMFPRegister(input, 64));
    Mov(ScratchReg2_32, Operand(255));
    Cmp(dest, ScratchReg2_32);
    csel(dest, dest, ScratchReg2_32, LessThan);
    Cmp(dest, Operand(0));
    csel(dest, wzr, dest, LessThan);
}

BufferOffset
MacroAssemblerCompat::movePatchablePtr(ImmPtr ptr, Register dest)
{
    const size_t numInst = 1; // Inserting one load instruction.
    const unsigned numPoolEntries = 2; // Every pool entry is 4 bytes.
    uint8_t *literalAddr = (uint8_t *)(&ptr.value); // TODO: Should be const.

    // Scratch space for generating the load instruction.
    //
    // allocEntry() will use InsertIndexIntoTag() to store a temporary
    // index to the corresponding PoolEntry in the instruction itself.
    //
    // That index will be fixed up later when finishPool()
    // walks over all marked loads and calls PatchConstantPoolLoad().
    uint32_t instructionScratch = 0;

    // Emit the instruction mask in the scratch space.
    // The offset doesn't matter: it will be fixed up later.
    AssemblerVIXL::ldr((Instruction *)&instructionScratch, ARMRegister(dest, 64), 0);

    // Add the entry to the pool, fix up the LDR imm19 offset,
    // and add the completed instruction to the buffer.
    return armbuffer_.allocEntry(numInst, numPoolEntries,
                                 (uint8_t *)&instructionScratch, literalAddr);
}

BufferOffset
MacroAssemblerCompat::movePatchablePtr(ImmWord ptr, Register dest)
{
    const size_t numInst = 1; // Inserting one load instruction.
    const unsigned numPoolEntries = 2; // Every pool entry is 4 bytes.
    uint8_t *literalAddr = (uint8_t *)(&ptr.value); // TODO: Should be const.

    // Scratch space for generating the load instruction.
    //
    // allocEntry() will use InsertIndexIntoTag() to store a temporary
    // index to the corresponding PoolEntry in the instruction itself.
    //
    // That index will be fixed up later when finishPool()
    // walks over all marked loads and calls PatchConstantPoolLoad().
    uint32_t instructionScratch = 0;

    // Emit the instruction mask in the scratch space.
    // The offset doesn't matter: it will be fixed up later.
    AssemblerVIXL::ldr((Instruction *)&instructionScratch, ARMRegister(dest, 64), 0);

    // Add the entry to the pool, fix up the LDR imm19 offset,
    // and add the completed instruction to the buffer.
    return armbuffer_.allocEntry(numInst, numPoolEntries,
                                 (uint8_t *)&instructionScratch, literalAddr);
}

void
MacroAssemblerCompat::handleFailureWithHandlerTail(void *handler)
{
    // Reserve space for exception information.
    int64_t size = (sizeof(ResumeFromException) + 7) & ~7;
    Sub(GetStackPointer64(), GetStackPointer64(), Operand(size));
    if (!GetStackPointer64().Is(sp))
        Add(sp, GetStackPointer64(), Operand(0));

    Add(x0, GetStackPointer64(), Operand(0));

    // Call the handler.
    setupUnalignedABICall(1, r1);
    passABIArg(r0);
    callWithABI(handler);

    Label entryFrame;
    Label catch_;
    Label finally;
    Label return_;
    Label bailout;

    MOZ_ASSERT(GetStackPointer64().Is(x28)); // Lets the code below be a little cleaner.

    loadPtr(Address(r28, offsetof(ResumeFromException, kind)), r0);
    branch32(Assembler::Equal, r0, Imm32(ResumeFromException::RESUME_ENTRY_FRAME), &entryFrame);
    branch32(Assembler::Equal, r0, Imm32(ResumeFromException::RESUME_CATCH), &catch_);
    branch32(Assembler::Equal, r0, Imm32(ResumeFromException::RESUME_FINALLY), &finally);
    branch32(Assembler::Equal, r0, Imm32(ResumeFromException::RESUME_FORCED_RETURN), &return_);
    branch32(Assembler::Equal, r0, Imm32(ResumeFromException::RESUME_BAILOUT), &bailout);

    breakpoint(); // Invalid kind.

    // No exception handler. Load the error value, load the new stack pointer,
    // and return from the entry frame.
    bind(&entryFrame);
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    loadPtr(Address(r28, offsetof(ResumeFromException, stackPointer)), r28);
    retn(Imm32(1 * sizeof(void *))); // Pop from stack and return.

    // If we found a catch handler, this must be a baseline frame. Restore state
    // and jump to the catch block.
    bind(&catch_);
    loadPtr(Address(r28, offsetof(ResumeFromException, target)), r0);
    loadPtr(Address(r28, offsetof(ResumeFromException, framePointer)), BaselineFrameReg);
    loadPtr(Address(r28, offsetof(ResumeFromException, stackPointer)), r28);
    syncStackPtr();
    Br(x0);

    // If we found a finally block, this must be a baseline frame.
    // Push two values expected by JSOP_RETSUB: BooleanValue(true)
    // and the exception.
    bind(&finally);
    ARMRegister exception = x1;
    Ldr(exception, MemOperand(GetStackPointer64(), offsetof(ResumeFromException, exception)));
    Ldr(x0, MemOperand(GetStackPointer64(), offsetof(ResumeFromException, target)));
    Ldr(ARMRegister(BaselineFrameReg, 64),
        MemOperand(GetStackPointer64(), offsetof(ResumeFromException, framePointer)));
    Ldr(GetStackPointer64(), MemOperand(GetStackPointer64(), offsetof(ResumeFromException, stackPointer)));
    syncStackPtr();
    pushValue(BooleanValue(true));
    push(exception);
    Br(x0);

    // Only used in debug mode. Return BaselineFrame->returnValue() to the caller.
    bind(&return_);
    loadPtr(Address(r28, offsetof(ResumeFromException, framePointer)), BaselineFrameReg);
    loadPtr(Address(r28, offsetof(ResumeFromException, stackPointer)), r28);
    loadValue(Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfReturnValue()),
              JSReturnOperand);
    movePtr(BaselineFrameReg, r28);
    MacroAssemblerVIXL::Pop(ARMRegister(BaselineFrameReg, 64), lr_64);
    syncStackPtr();
    MacroAssemblerVIXL::Ret(lr_64);

    // If we are bailing out to baseline to handle an exception,
    // jump to the bailout tail stub.
    bind(&bailout);
    Ldr(x2, MemOperand(GetStackPointer64(), offsetof(ResumeFromException, bailoutInfo)));
    Ldr(x1, MemOperand(GetStackPointer64(), offsetof(ResumeFromException, target)));
    Mov(x0, BAILOUT_RETURN_OK);
    Br(x1);
}

void
MacroAssemblerCompat::setupABICall(uint32_t args)
{
    MOZ_ASSERT(!inCall_);
    inCall_ = true;

    args_ = args;
    usedOutParam_ = false;
    passedIntArgs_ = 0;
    passedFloatArgs_ = 0;
    passedArgTypes_ = 0;
    stackForCall_ = ShadowStackSpace;
}

void
MacroAssemblerCompat::setupUnalignedABICall(uint32_t args, Register scratch)
{
    setupABICall(args);
    dynamicAlignment_ = true;

    int64_t alignment = ~(int64_t(ABIStackAlignment) - 1);
    ARMRegister scratch64(scratch, 64);

    // Always save LR -- Baseline ICs assume that LR isn't modified.
    push(lr);

    // TODO: Unhandled for sp -- needs slightly different logic.
    MOZ_ASSERT(!GetStackPointer64().Is(sp));

    // Remember the stack address on entry.
    Add(scratch64, GetStackPointer64(), Operand(0));

    // Make alignment, including the effective push of the previous sp.
    Sub(GetStackPointer64(), GetStackPointer64(), Operand(8));
    And(GetStackPointer64(), GetStackPointer64(), Operand(alignment));

    // If the PseudoStackPointer is used, sp must be <= psp before a write is valid.
    syncStackPtr();

    // Store previous sp to the top of the stack, aligned.
    Str(scratch64, MemOperand(GetStackPointer64(), 0));
}

void
MacroAssemblerCompat::passABIArg(const MoveOperand &from, MoveOp::Type type)
{
    if (!enoughMemory_)
        return;

    Register activeSP = Register::FromCode(GetStackPointer64().code());
    if (type == MoveOp::GENERAL) {
        Register dest;
        passedArgTypes_ = (passedArgTypes_ << ArgType_Shift) | ArgType_General;
        if (GetIntArgReg(passedIntArgs_++, passedFloatArgs_, &dest)) {
            if (!from.isGeneralReg() || from.reg() != dest)
                enoughMemory_ = moveResolver_.addMove(from, MoveOperand(dest), type);
            return;
        }

        enoughMemory_ = moveResolver_.addMove(from, MoveOperand(activeSP, stackForCall_), type);
        stackForCall_ += sizeof(int64_t);
        return;
    }

    MOZ_ASSERT(type == MoveOp::FLOAT32 || type == MoveOp::DOUBLE);
    if (type == MoveOp::FLOAT32)
        passedArgTypes_ = (passedArgTypes_ << ArgType_Shift) | ArgType_Float32;
    else
        passedArgTypes_ = (passedArgTypes_ << ArgType_Shift) | ArgType_Double;

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
      default: MOZ_CRASH("Unexpected float register class argument type");
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
MacroAssemblerCompat::passABIOutParam(Register reg)
{
    if (!enoughMemory_)
        return;
    MOZ_ASSERT(!usedOutParam_);
    usedOutParam_ = true;
    if (reg == r8)
        return;
    enoughMemory_ = moveResolver_.addMove(MoveOperand(reg), MoveOperand(r8), MoveOp::GENERAL);

}

void
MacroAssemblerCompat::callWithABIPre(uint32_t *stackAdjust)
{
    *stackAdjust = stackForCall_;
    // ARM64 /really/ wants the stack to always be aligned.  Since we're already tracking it
    // getting it aligned for an abi call is pretty easy.
    *stackAdjust += ComputeByteAlignment(*stackAdjust, StackAlignment);
    reserveStack(*stackAdjust);
    {
        moveResolver_.resolve();
        MoveEmitter emitter(*this);
        emitter.emit(moveResolver_);
        emitter.finish();
    }

    // Call boundaries communicate stack via sp.
    syncStackPtr();
}

void
MacroAssemblerCompat::callWithABIPost(uint32_t stackAdjust, MoveOp::Type result)
{
    // Call boundaries communicate stack via sp.
    if (!GetStackPointer64().Is(sp))
        Add(GetStackPointer64(), sp, Operand(0));

    inCall_ = false;
    freeStack(stackAdjust);

    // Restore the stack pointer from entry.
    if (dynamicAlignment_)
        Ldr(GetStackPointer64(), MemOperand(GetStackPointer64(), 0));

    // Restore LR.
    pop(lr);

    // TODO: This one shouldn't be necessary -- check that callers
    // aren't enforcing the ABI themselves!
    syncStackPtr();

    // If the ABI's return regs are where ION is expecting them, then
    // no other work needs to be done.
}

#if defined(DEBUG) && defined(JS_ARM64_SIMULATOR)
// TODO: This should be shared in IonTypes.h.
static void
AssertValidABIFunctionType(uint32_t passedArgTypes)
{
    switch (passedArgTypes) {
      case Args_General0:
      case Args_General1:
      case Args_General2:
      case Args_General3:
      case Args_General4:
      case Args_General5:
      case Args_General6:
      case Args_General7:
      case Args_General8:
      case Args_Double_None:
      case Args_Int_Double:
      case Args_Float32_Float32:
      case Args_Double_Double:
      case Args_Double_Int:
      case Args_Double_DoubleInt:
      case Args_Double_DoubleDouble:
      case Args_Double_IntDouble:
      case Args_Int_IntDouble:
        break;
      default:
        MOZ_CRASH("Unexpected type");
    }
}
#endif // DEBUG && JS_ARM64_SIMULATOR

void
MacroAssemblerCompat::callWithABI(void *fun, MoveOp::Type result)
{
#ifdef JS_ARM64_SIMULATOR
    MOZ_ASSERT(passedIntArgs_ + passedFloatArgs_ <= 15);
    passedArgTypes_ <<= ArgType_Shift;
    switch (result) {
      case MoveOp::GENERAL: passedArgTypes_ |= ArgType_General; break;
      case MoveOp::DOUBLE:  passedArgTypes_ |= ArgType_Double;  break;
      case MoveOp::FLOAT32: passedArgTypes_ |= ArgType_Float32; break;
      default: MOZ_CRASH("Invalid return type");
    }
# ifdef DEBUG
    AssertValidABIFunctionType(passedArgTypes_);
# endif
    ABIFunctionType type = ABIFunctionType(passedArgTypes_);
    fun = Simulator::RedirectNativeFunction(fun, type);
#endif // JS_ARM64_SIMULATOR

    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(ImmPtr(fun));
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerCompat::callWithABI(Register fun, MoveOp::Type result)
{
    movePtr(fun, ip0);

    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(ip0);
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerCompat::callWithABI(AsmJSImmPtr imm, MoveOp::Type result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(imm);
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerCompat::callWithABI(Address fun, MoveOp::Type result)
{
    loadPtr(fun, ip0);

    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(ip0);
    callWithABIPost(stackAdjust, result);
}

void MacroAssemblerCompat::branchPtrInNurseryRange(Condition cond, Register ptr, Register temp,
                                                   Label *label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    MOZ_ASSERT(ptr != temp);
    MOZ_ASSERT(ptr != ScratchReg && ptr != ScratchReg2); // Both may be used internally.
    MOZ_ASSERT(temp != ScratchReg && temp != ScratchReg2);

    const Nursery &nursery = GetJitContext()->runtime->gcNursery();
    movePtr(ImmWord(-ptrdiff_t(nursery.start())), temp);
    addPtr(ptr, temp);
    branchPtr(cond == Assembler::Equal ? Assembler::Below : Assembler::AboveOrEqual,
              temp, ImmWord(nursery.nurserySize()), label);
}

void
MacroAssemblerCompat::branchValueIsNurseryObject(Condition cond, ValueOperand value, Register temp,
                                                 Label *label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    MOZ_ASSERT(temp != ScratchReg && temp != ScratchReg2); // Both may be used internally.

    // 'Value' representing the start of the nursery tagged as a JSObject
    const Nursery &nursery = GetJitContext()->runtime->gcNursery();
    Value start = ObjectValue(*reinterpret_cast<JSObject *>(nursery.start()));

    movePtr(ImmWord(-ptrdiff_t(start.asRawBits())), temp);
    addPtr(value.valueReg(), temp);
    branchPtr(cond == Assembler::Equal ? Assembler::Below : Assembler::AboveOrEqual,
              temp, ImmWord(nursery.nurserySize()), label);
}

// FIXME: Probably just call Brk() in the header.
void
MacroAssemblerCompat::breakpoint()
{
    static int code = 0xA77;
    // FIXME: Remove this (mjrosenb)
    if (getenv("STOP_BREAK") && strtol(getenv("STOP_BREAK"), 0, 0) == code)
        MOZ_CRASH("You Rang?");
    Brk(code++);
}

void
MacroAssembler::alignFrameForICArguments(MacroAssembler::AfterICSaveLive &aic)
{
    // Exists for MIPS compatibility.
}

void
MacroAssembler::restoreFrameAlignmentForICArguments(MacroAssembler::AfterICSaveLive &aic)
{
    // Exists for MIPS compatibility.
}


} // namespace jit
} // namespace js
