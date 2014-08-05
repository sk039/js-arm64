/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/Bailouts.h"
#include "jit/IonFrames.h"
#include "jit/IonLinker.h"
#include "jit/JitCompartment.h"
#ifdef JS_ION_PERF
# include "jit/PerfSpewer.h"
#endif
#include "jit/VMFunctions.h"
#include "jit/arm64/BaselineHelpers-arm64.h"

using namespace js;
using namespace js::jit;

// All registers to save and restore. This includes the stack pointer, since we
// use the ability to reference register values on the stack by index.
static const RegisterSet AllRegs =
  RegisterSet(GeneralRegisterSet(Registers::AllMask),
              FloatRegisterSet(FloatRegisters::AllMask));

/* This method generates a trampoline on x64 for a c++ function with
 * the following signature:
 *   bool blah(void *code, int argc, Value *argv, JSObject *scopeChain, Value *vp)
 *   ...using standard AArch64 calling convention
 */
JitCode *
JitRuntime::generateEnterJIT(JSContext *cx, EnterJitType type)
{
    MacroAssembler masm(cx);

    const Register reg_code = IntArgReg0;
    const Register reg_argc = IntArgReg1;
    const Register reg_argv = IntArgReg2;
    const Register reg_scope = IntArgReg3;
    const Register reg_vp = IntArgReg4;
    JS_ASSERT(OsrFrameReg == IntArgReg5);

    // TODO: Save old stack frame pointer, set new stack frame pointer.

    // During the pushes below, use the normal stack pointer.
    masm.SetStackPointer(sp);

    // Save callee-save integer registers.
    masm.MacroAssemblerVIXL::Push(x19, x20, x21, x22);
    masm.MacroAssemblerVIXL::Push(x23, x24, x25, x26);
    masm.MacroAssemblerVIXL::Push(x27, x28);

    // Save callee-save floating-point registers.
    // AArch64 ABI specifies that only the lower 64 bits must be saved.
    masm.MacroAssemblerVIXL::Push(d8,  d9,  d10, d11);
    masm.MacroAssemblerVIXL::Push(d12, d13, d14, d15);

    // Common code below attempts to push single registers at a time,
    // which breaks the stack pointer's 16-byte alignment requirement.
    // Note that movePtr() is invalid because StackPointer is treated as xzr.
    masm.Add(ARMRegister(PseudoStackPointer, 64), sp, Operand((int64_t)0));
    masm.SetStackPointer(ARMRegister(PseudoStackPointer, 64));

    // Push the EnterJIT SPS mark.
    masm.spsMarkJit(&cx->runtime()->spsProfiler, PseudoStackPointer, r20);

    // TODO: Remember stack depth without padding and arguments.
    // TODO: Note that we could just push more false registers above...

    // TODO: Remember the number of bytes occupied by argument vector.

    // IonJSFrameLayout is as follows (higher is higher in memory):
    //  N*64  - [ JS argument vector ]
    //  64    - numActualArgs
    //  64    - calleeToken
    //  64    - frameDescriptor
    //  64    - returnAddress

    // Push the argument vector onto the stack.
    // WARNING: destructively modifies reg_argv
    {
        ARMRegister tmp_argc = x19;
        ARMRegister tmp_sp = x28;
        Label noArguments;
        Label loopHead;

        masm.Mov(tmp_argc, ARMRegister(reg_argc, 64));

        // sp -= 8 * argc
        masm.Mov(tmp_sp, sp);
        masm.Sub(tmp_sp, tmp_sp, Operand(tmp_argc, SXTX, 3));

        // Give sp 16-byte alignment.
        masm.And(tmp_sp, tmp_sp, Operand(-15));
        masm.Mov(sp, tmp_sp);

        masm.branchTestPtr(Assembler::Zero, reg_argc, reg_argc, &noArguments);

        // Begin argument-pushing loop.
        // FIXME: Can we use Ldp and Stp to push multiple arguments?
        // FIXME: Do this after the simple thing is well-tested.
        {
            masm.bind(&loopHead);

            // Load an argument from argv, then increment argv by 8.
            masm.Ldr(x24, MemOperand(ARMRegister(reg_argv, 64), Operand(8), PostIndex));

            // Store the argument to tmp_sp, then increment tmp_sp by 8.
            masm.Str(x24, MemOperand(tmp_sp, Operand(8), PostIndex));

            // Set the condition codes for |cmp tmp_argc, 2| (using the old value).
            masm.Subs(tmp_argc, tmp_argc, Operand(1));

            // Branch if arguments remain.
            masm.B(&loopHead, GreaterThanOrEqual_);
        }

        masm.bind(&noArguments);
    }

    // Push numActualArgs and the calleeToken.
    masm.MacroAssemblerVIXL::Push(ARMRegister(reg_argc, 64), ARMRegister(reg_scope, 64));

    // Push the frameDescriptor.
    // TODO: x19 must contain the number of bytes pushed on the frame.
    masm.makeFrameDescriptor(r19, JitFrame_Entry);
    masm.Push(r19);

    if (type == EnterJitBaseline) {
        Label notOsr;
        masm.branchTestPtr(Assembler::Zero, OsrFrameReg, OsrFrameReg, &notOsr);
        masm.breakpoint(); // TODO: Handle Baseline with OSR, which is complicated.
        masm.bind(&notOsr);
    }

    // Call function.
    masm.call(reg_code);

    // TODO: Unwind the EnterJIT SPS mark.
    //masm.spsUnmarkJit(&cx->runtime()->spsProfiler, ???);

    // Restore callee-save floating-point registers.
    masm.MacroAssemblerVIXL::Pop(d15, d14, d13, d12);
    masm.MacroAssemblerVIXL::Pop(d11, d10,  d9,  d8);

    // Restore callee-save integer registers.
    masm.MacroAssemblerVIXL::Pop(x28, x27, x26, x25);
    masm.MacroAssemblerVIXL::Pop(x24, x23, x22, x21);
    masm.MacroAssemblerVIXL::Pop(x20, x19);

    Linker linker(masm);
    JitCode *code = linker.newCode<NoGC>(cx, JSC::OTHER_CODE);

#ifdef JS_ION_PERF
    writePerfSpewerJitCodeProfile(code, "EnterJIT");
#endif

    return code;
}

JitCode *
JitRuntime::generateInvalidator(JSContext *cx)
{
    // FIXME: Actually implement.
    MacroAssembler masm;
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

JitCode *
JitRuntime::generateArgumentsRectifier(JSContext *cx, ExecutionMode mode, void **returnAddrOut)
{
    // FIXME: Actually implement.
    MacroAssembler masm;
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

static void
GenerateBailoutThunk(JSContext *cx, MacroAssembler &masm, uint32_t frameClass)
{
    // FIXME: Actually implement.
}

JitCode *
JitRuntime::generateBailoutTable(JSContext *cx, uint32_t frameClass)
{
    // FIXME: Actually implement.
    MacroAssembler masm;
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

JitCode *
JitRuntime::generateBailoutHandler(JSContext *cx, ExecutionMode mode)
{
    // FIXME: Actually implement.
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

JitCode *
JitRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    JS_ASSERT(functionWrappers_);
    JS_ASSERT(functionWrappers_->initialized());
    VMWrapperMap::AddPtr p = functionWrappers_->lookupForAdd(&f);
    if (p)
        return p->value();

    MacroAssembler masm(cx);
    masm.breakpoint();

    Linker linker(masm);
    JitCode *wrapper = linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
    if (!wrapper)
        return nullptr;

#ifdef JS_ION_PERF
    writePerfSpewerJitCodeProfile(wrapper, "VMWrapper");
#endif

    // linker.newCode may trigger a GC and sweep functionWrappers_ so we have to
    // use relookupOrAdd instead of add.
    if (!functionWrappers_->relookupOrAdd(p, &f, wrapper))
        return nullptr;

    return wrapper;
}

JitCode *
JitRuntime::generatePreBarrier(JSContext *cx, MIRType type)
{
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

typedef bool (*HandleDebugTrapFn)(JSContext *, BaselineFrame *, uint8_t *, bool *);
static const VMFunction HandleDebugTrapInfo = FunctionInfo<HandleDebugTrapFn>(HandleDebugTrap);

JitCode *
JitRuntime::generateDebugTrapHandler(JSContext *cx)
{
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

JitCode *
JitRuntime::generateExceptionTailStub(JSContext *cx)
{
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}

JitCode *
JitRuntime::generateBailoutTailStub(JSContext *cx)
{
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, JSC::OTHER_CODE);
}
