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
#include "jit/arm64/BaselineHelpers-arm64.h"
#include "jit/VMFunctions.h"

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

    const Register reg_code      = IntArgReg0; // EnterJitData::jitcode.
    const Register reg_argc      = IntArgReg1; // EnterJitData::maxArgc.
    const Register reg_argv      = IntArgReg2; // EnterJitData::maxArgv.
    const Register reg_osrFrame  = IntArgReg3; // EnterJitData::osrFrame.
    const Register reg_callee    = IntArgReg4; // EnterJitData::calleeToken.
    const Register reg_scope     = IntArgReg5; // EnterJitData::scopeChain.
    const Register reg_osrNStack = IntArgReg6; // EnterJitData::osrNumStackValues.
    const Register reg_vp        = IntArgReg7; // Address of EnterJitData::result.

    // FIXME: Probably use x8 or something.
    MOZ_ASSERT(OsrFrameReg == IntArgReg3);

    // TODO: Save old stack frame pointer, set new stack frame pointer.

    // During the pushes below, use the normal stack pointer.
    masm.SetStackPointer(sp);

    // Save callee-save integer registers.
    // Also save x7 (reg_vp) and x30 (lr), for use later.
    masm.MacroAssemblerVIXL::Push(x19, x20, x21, x22);
    masm.MacroAssemblerVIXL::Push(x23, x24, x25, x26);
    masm.MacroAssemblerVIXL::Push(x27, x28, x7,  x30);

    // Save callee-save floating-point registers.
    // AArch64 ABI specifies that only the lower 64 bits must be saved.
    masm.MacroAssemblerVIXL::Push(d8,  d9,  d10, d11);
    masm.MacroAssemblerVIXL::Push(d12, d13, d14, d15);

#ifdef DEBUG
    // Emit stack canaries.
    masm.movePtr(ImmWord(0xdeadd00d), r23);
    masm.movePtr(ImmWord(0xdeadd11d), r24);
    masm.MacroAssemblerVIXL::Push(x23, x24);
#endif

    // Common code below attempts to push single registers at a time,
    // which breaks the stack pointer's 16-byte alignment requirement.
    // Note that movePtr() is invalid because StackPointer is treated as xzr.
    //
    // FIXME: After testing, this entire function should be rewritten to not
    // use the PseudoStackPointer: since the amount of data pushed is precalculated,
    // we can just allocate the whole frame header at once and index off sp.
    // This will save a significant number of instructions where Push() updates sp.
    masm.Add(PseudoStackPointer64, sp, Operand((int64_t)0));
    masm.SetStackPointer(PseudoStackPointer64);

    // Push the EnterJIT SPS mark.
    masm.spsMarkJit(&cx->runtime()->spsProfiler, PseudoStackPointer, r20);

    // Remember stack depth without padding and arguments.
    masm.Mov(x19, PseudoStackPointer64);

    // Save stack pointer for pushing in Baseline's emitPrologue().
    if (type == EnterJitBaseline)
        masm.movePtr(PseudoStackPointer, BaselineFrameReg); // x11

    // IonJSFrameLayout is as follows (higher is higher in memory):
    //  N*8  - [ JS argument vector ] (base 16-byte aligned)
    //  8    - numActualArgs
    //  8    - calleeToken (16-byte aligned)
    //  8    - frameDescriptor
    //  8    - returnAddress (16-byte aligned, pushed by callee)

    // Push the argument vector onto the stack.
    // WARNING: destructively modifies reg_argv
    {
        ARMRegister tmp_argc = x16; // ip0 -- no functions below may use scratch registers.
        ARMRegister tmp_sp = x17; // ip1
        Label noArguments;
        Label loopHead;

        masm.Mov(tmp_argc, ARMRegister(reg_argc, 64));

        // sp -= 8
        // Since we're using PostIndex Str below, this is necessary to avoid overwriting
        // the SPS mark pushed above.
        masm.Sub(PseudoStackPointer64, PseudoStackPointer64, Operand(8));

        // sp -= 8 * argc
        masm.Sub(PseudoStackPointer64, PseudoStackPointer64, Operand(tmp_argc, SXTX, 3));

        // Give sp 16-byte alignment.
        masm.And(PseudoStackPointer64, PseudoStackPointer64, Operand(-15));

        // tmp_sp = sp = PseudoStackPointer.
        masm.Sub(sp, PseudoStackPointer64, Operand(0));
        masm.Mov(tmp_sp, PseudoStackPointer64);

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
    masm.checkStackAlignment();

    // Push the number of actual arguments and the calleeToken.
    // The result address is used to store the actual number of arguments
    // without adding an argument to EnterJIT.
    masm.unboxInt32(Address(reg_vp, 0x0), ip0);
    masm.MacroAssemblerVIXL::Push(ip0_64, ARMRegister(reg_callee, 64));
    masm.checkStackAlignment();

    // Calculate the number of bytes pushed so far.
    masm.Sub(x19, x19, PseudoStackPointer64);

    // Push the frameDescriptor.
    masm.makeFrameDescriptor(r19, JitFrame_Entry);
    masm.Push(r19);

    if (type == EnterJitBaseline) {
        Label notOsr;
        masm.branchTestPtr(Assembler::Zero, OsrFrameReg, OsrFrameReg, &notOsr);
        masm.breakpoint(); // TODO: Handle Baseline with OSR, which is complicated.
        masm.bind(&notOsr);
        masm.movePtr(reg_scope, R1_);
    }

    // Call function.
    // Since AArch64 doesn't have the pc register available, the callee must push lr.
    masm.Blr(ARMRegister(reg_code, 64));

    masm.Pop(r19);
    masm.Add(PseudoStackPointer64, PseudoStackPointer64, Operand(x19, LSR, FRAMESIZE_SHIFT));
    masm.spsUnmarkJit(&cx->runtime()->spsProfiler, r20);
    masm.SetStackPointer(sp);
    masm.Add(sp, PseudoStackPointer64, Operand(0));

#ifdef DEBUG
    // Check that canaries placed on function entry are still present.
    // TODO: Once this patch is ready, we can probably remove the canaries.
    masm.MacroAssemblerVIXL::Pop(x24, x23);
    Label x23OK, x24OK;

    masm.branchPtr(Assembler::Equal, r23, ImmWord(0xdeadd00d), &x23OK);
    masm.breakpoint();
    masm.bind(&x23OK);

    masm.branchPtr(Assembler::Equal, r24, ImmWord(0xdeadd11d), &x24OK);
    masm.breakpoint();
    masm.bind(&x24OK);
#endif
    
    // Restore callee-save floating-point registers.
    masm.MacroAssemblerVIXL::Pop(d15, d14, d13, d12);
    masm.MacroAssemblerVIXL::Pop(d11, d10,  d9,  d8);

    // Restore callee-save integer registers.
    // Also restore x7 (reg_vp) and x30 (lr).
    masm.MacroAssemblerVIXL::Pop(x30, x7,  x28, x27);
    masm.MacroAssemblerVIXL::Pop(x26, x25, x24, x23);
    masm.MacroAssemblerVIXL::Pop(x22, x21, x20, x19);

    // Store return value (in JSReturnReg = x2 to just-popped reg_vp).
    masm.storeValue(JSReturnOperand, Address(reg_vp, 0));

    // Return using the value popped into x30.
    masm.ret();

    Linker linker(masm);
    JitCode *code = linker.newCode<NoGC>(cx, OTHER_CODE);

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
    return linker.newCode<NoGC>(cx, OTHER_CODE);
}

JitCode *
JitRuntime::generateArgumentsRectifier(JSContext *cx, ExecutionMode mode, void **returnAddrOut)
{
    MacroAssembler masm;

    // Save the return address for later
    masm.push(lr);
    // Load the information that the rectifier needs from the stack
    masm.Ldr(w0, MemOperand(masm.GetStackPointer(), IonRectifierFrameLayout::offsetOfNumActualArgs()));
    masm.Ldr(x1, MemOperand(masm.GetStackPointer(), IonRectifierFrameLayout::offsetOfCalleeToken()));
    // Extract a JSFunction pointer from the callee token
    masm.And(x6, x1, Operand(CalleeTokenMask));
    // Get the arguments from the function object
    masm.Ldrh(x6, MemOperand(x6, JSFunction::offsetOfNargs()));

    // Calculate the number of undefineds that need to be pushed
    masm.Sub(w2, w6, w8);
    // Put an undefined in a register so it can be pushed
    masm.moveValue(UndefinedValue(), r4);

    // Calculate the position that our arguments are at before sp gets modified
    masm.Add(x3, masm.GetStackPointer(), Operand(x8, LSL, 3));
    masm.Add(x3, x3, Operand(sizeof(IonRectifierFrameLayout)));

    // Push undefined N times
    {
        Label undefLoopTop;
        masm.bind(&undefLoopTop);
        masm.Push(r4);
        masm.Subs(w2, w2, Operand(1));
        masm.B(&undefLoopTop, Assembler::NonZero);
    }

    {
        Label copyLoopTop;
        masm.bind(&copyLoopTop);
        masm.Ldr(x4, MemOperand(x3, -sizeof(Value), PostIndex));
        masm.Push(r4);
        masm.Subs(x8, x8, Operand(1));
        masm.B(&copyLoopTop, Assembler::NotSigned);
    }
    // Fix up the size of the stack frame
    masm.Add(x6, x6, Operand(1));
    masm.Lsl(x6, x6, 3);

    // Make that into a frame descriptor.
    masm.makeFrameDescriptor(r6, JitFrame_Rectifier);

    masm.push(r0); // number of actual arguments
    masm.push(r1); // callee token
    masm.push(r6); // frame descriptor

    // Didn't we just compute this? can't we just stick that value in one of our 30 GPR's?
    // Load the address of the code that is getting called
    masm.And(x1, x1, Operand(CalleeTokenMask));
    masm.Ldr(x3, MemOperand(x1, JSFunction::offsetOfNativeOrScript()));
    masm.loadBaselineOrIonRaw(r3, r3, mode, nullptr);
    masm.call(r3);

    // Clean up!
    // Get the size of the stack frame, and clean up the later fixed frame
    masm.Ldr(x4, MemOperand(masm.GetStackPointer(), 24, PostIndex));
    // Now that the size of the stack frame sans the fixed frame has been loaded,
    // add that onto the stack pointer
    masm.Add(masm.GetStackPointer(), masm.GetStackPointer(), Operand(x4, LSR, FRAMESIZE_SHIFT));
    // and make sure all of these are reflected in the real stack pointer
    masm.syncStackPtr();
    // Do that return thing
    masm.Pop(lr);
    masm.ret();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, OTHER_CODE);
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
    return linker.newCode<NoGC>(cx, OTHER_CODE);
}

JitCode *
JitRuntime::generateBailoutHandler(JSContext *cx, ExecutionMode mode)
{
    // FIXME: Actually implement.
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, OTHER_CODE);
}

JitCode *
JitRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    MOZ_ASSERT(functionWrappers_);
    MOZ_ASSERT(functionWrappers_->initialized());
    VMWrapperMap::AddPtr p = functionWrappers_->lookupForAdd(&f);
    if (p)
        return p->value();

    MacroAssembler masm(cx);

    // Avoid conflicts with argument registers while discarding the result after
    // the function call.
    GeneralRegisterSet regs = GeneralRegisterSet(Register::Codes::WrapperMask);

    // Wrapper register set is a superset of the Volatile register set.
    JS_STATIC_ASSERT((Register::Codes::VolatileMask & ~Register::Codes::WrapperMask) == 0);

    // Unlike on other platforms, it is the responsibility of the VM *callee* to
    // push the return address, while the caller must ensure that the address
    // is stored in lr on entry. This allows the VM wrapper to work with both direct
    // calls and tail calls.
    masm.push(lr);

    // First argument is the JSContext.
    Register reg_cx = IntArgReg0;
    regs.take(reg_cx);

    // Stack is:
    //    ... frame ...
    //  +12 [args]
    //  +8  descriptor
    //  +0  returnAddress (pushed by this function, caller sets as lr)
    //
    //  We're aligned to an exit frame, so link it up.
    masm.enterExitFrameAndLoadContext(&f, reg_cx, regs.getAny(), f.executionMode);

    // Save the current stack pointer as the base for copying arguments.
    Register argsBase = InvalidReg;
    if (f.explicitArgs) {
        // argsBase can't be an argument register. Bad things would happen if
        // the MoveResolver didn't throw an assertion failure first.
        argsBase = r8;
        regs.take(argsBase);
        masm.Add(ARMRegister(argsBase, 64), masm.GetStackPointer(),
                 Operand(IonExitFrameLayout::SizeWithFooter()));
    }

    // Reserve space for any outparameter.
    Register outReg = InvalidReg;
    switch (f.outParam) {
      case Type_Value:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(Value));
        masm.Add(ARMRegister(outReg, 64), masm.GetStackPointer(), Operand(0));
        break;

      case Type_Handle:
        outReg = regs.takeAny();
        masm.PushEmptyRooted(f.outParamRootType);
        masm.Add(ARMRegister(outReg, 64), masm.GetStackPointer(), Operand(0));
        break;

      case Type_Int32:
      case Type_Bool:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(int64_t));
        masm.Add(ARMRegister(outReg, 64), masm.GetStackPointer(), Operand(0));
        break;

      case Type_Double:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(double));
        masm.Add(ARMRegister(outReg, 64), masm.GetStackPointer(), Operand(0));
        break;

      case Type_Pointer:
        outReg = regs.takeAny();
        masm.reserveStack(sizeof(uintptr_t));
        masm.Add(ARMRegister(outReg, 64), masm.GetStackPointer(), Operand(0));
        break;

      default:
        MOZ_ASSERT(f.outParam == Type_Void);
        break;
    }

    masm.setupUnalignedABICall(f.argc(), regs.getAny());
    masm.passABIArg(reg_cx);

    size_t argDisp = 0;

    // Copy arguments.
    for (uint32_t explicitArg = 0; explicitArg < f.explicitArgs; explicitArg++) {
        MoveOperand from;
        switch (f.argProperties(explicitArg)) {
          case VMFunction::WordByValue:
            masm.passABIArg(MoveOperand(argsBase, argDisp),
                            (f.argPassedInFloatReg(explicitArg) ? MoveOp::DOUBLE : MoveOp::GENERAL));
            argDisp += sizeof(void *);
            break;

          case VMFunction::WordByRef:
            masm.passABIArg(MoveOperand(argsBase, argDisp, MoveOperand::EFFECTIVE_ADDRESS),
                            MoveOp::GENERAL);
            argDisp += sizeof(void *);
            break;

          case VMFunction::DoubleByValue:
          case VMFunction::DoubleByRef:
            MOZ_CRASH("NYI: AArch64 callVM should not be used with 128bit values.");
        }
    }

    // Copy the semi-implicit outparam, if any.
    // It is not a C++-abi outparam, which would get passed in the
    // outparam register, but a real parameter to the function, which
    // was stack-allocated above.
    if (outReg != InvalidReg)
        masm.passABIArg(outReg);

    masm.callWithABI(f.wrapped);

    // SP is used to transfer stack across call boundaries.
    masm.Add(masm.GetStackPointer(), sp, Operand(0));

    // Test for failure.
    switch (f.failType()) {
      case Type_Object:
        masm.branchTestPtr(Assembler::Zero, r0, r0, masm.failureLabel(f.executionMode));
        break;
      case Type_Bool:
        masm.branchIfFalseBool(r0, masm.failureLabel(f.executionMode));
        break;
      default:
        MOZ_CRASH("unknown failure kind");
    }

    // Load the outparam and free any allocated stack.
    switch (f.outParam) {
      case Type_Value:
        masm.Ldr(ARMRegister(JSReturnReg, 64), MemOperand(masm.GetStackPointer()));
        masm.freeStack(sizeof(Value));
        break;

      case Type_Handle:
        masm.popRooted(f.outParamRootType, ReturnReg, JSReturnOperand);
        break;

      case Type_Int32:
        masm.Ldr(ARMRegister(ReturnReg, 32), MemOperand(masm.GetStackPointer()));
        masm.freeStack(sizeof(int64_t));
        break;

      case Type_Bool:
        masm.Ldrb(ARMRegister(ReturnReg, 32), MemOperand(masm.GetStackPointer()));
        masm.freeStack(sizeof(int64_t));
        break;

      case Type_Double:
        MOZ_ASSERT(cx->runtime()->jitSupportsFloatingPoint);
        masm.Ldr(ARMFPRegister(ReturnDoubleReg, 64), MemOperand(masm.GetStackPointer()));
        masm.freeStack(sizeof(double));
        break;

      case Type_Pointer:
        masm.Ldr(ARMRegister(ReturnReg, 64), MemOperand(masm.GetStackPointer()));
        masm.freeStack(sizeof(uintptr_t));
        break;

      default:
        MOZ_ASSERT(f.outParam == Type_Void);
        break;
    }

    masm.leaveExitFrame();
    masm.retn(Imm32(sizeof(IonExitFrameLayout) +
              f.explicitStackSlots() * sizeof(void *) +
              f.extraValuesToPop * sizeof(Value)));

    Linker linker(masm);
    JitCode *wrapper = linker.newCode<NoGC>(cx, OTHER_CODE);
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
    return linker.newCode<NoGC>(cx, OTHER_CODE);
}

typedef bool (*HandleDebugTrapFn)(JSContext *, BaselineFrame *, uint8_t *, bool *);
static const VMFunction HandleDebugTrapInfo = FunctionInfo<HandleDebugTrapFn>(HandleDebugTrap);

JitCode *
JitRuntime::generateDebugTrapHandler(JSContext *cx)
{
    MacroAssembler masm(cx);
    masm.breakpoint();
    Linker linker(masm);
    return linker.newCode<NoGC>(cx, OTHER_CODE);
}

JitCode *
JitRuntime::generateExceptionTailStub(JSContext *cx)
{
    MacroAssembler masm(cx);

    masm.handleFailureWithHandlerTail();

    Linker linker(masm);
    JitCode *code = linker.newCode<NoGC>(cx, OTHER_CODE);

#ifdef JS_ION_PERF
    writePerfSpewerJitCodeProfile(code, "ExceptionTailStub");
#endif

    return code;
}

JitCode *
JitRuntime::generateBailoutTailStub(JSContext *cx)
{
    MacroAssembler masm(cx);

    masm.generateBailoutTail(r1, r2);

    Linker linker(masm);
    JitCode *code = linker.newCode<NoGC>(cx, OTHER_CODE);

#ifdef JS_ION_PERF
    writePerfSpewerJitCodeProfile(code, "BailoutTailStub");
#endif

    return code;
}
