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
 *   bool blah(void *code, int argc, Value *argv, JSObject *scopeChain,
 *               Value *vp)
 *   ...using standard x64 fastcall calling convention
 */
JitCode *
JitRuntime::generateEnterJIT(JSContext *cx, EnterJitType type)
{
    MOZ_ASSUME_UNREACHABLE("generateEnterJIT");
}

JitCode *
JitRuntime::generateInvalidator(JSContext *cx)
{
    MOZ_ASSUME_UNREACHABLE("generateInvalidator");
}

JitCode *
JitRuntime::generateArgumentsRectifier(JSContext *cx, ExecutionMode mode, void **returnAddrOut)
{
    MOZ_ASSUME_UNREACHABLE("generateArgumentsRectifier");
}

static void
GenerateBailoutThunk(JSContext *cx, MacroAssembler &masm, uint32_t frameClass)
{
    MOZ_ASSUME_UNREACHABLE("GenerateBailoutThunk");
}

JitCode *
JitRuntime::generateBailoutTable(JSContext *cx, uint32_t frameClass)
{
    MOZ_ASSUME_UNREACHABLE("arm64 does not use bailout tables");
}

JitCode *
JitRuntime::generateBailoutHandler(JSContext *cx)
{
    MOZ_ASSUME_UNREACHABLE("generateBailoutHandler");
}

JitCode *
JitRuntime::generateVMWrapper(JSContext *cx, const VMFunction &f)
{
    MOZ_ASSUME_UNREACHABLE("generateVMWrapper");
}

JitCode *
JitRuntime::generatePreBarrier(JSContext *cx, MIRType type)
{
    MOZ_ASSUME_UNREACHABLE("generatePreBarrier");
}

typedef bool (*HandleDebugTrapFn)(JSContext *, BaselineFrame *, uint8_t *, bool *);
static const VMFunction HandleDebugTrapInfo = FunctionInfo<HandleDebugTrapFn>(HandleDebugTrap);

JitCode *
JitRuntime::generateDebugTrapHandler(JSContext *cx)
{
    MOZ_ASSUME_UNREACHABLE("generateDebugTrapHandler");
}

JitCode *
JitRuntime::generateExceptionTailStub(JSContext *cx)
{
    MOZ_ASSUME_UNREACHABLE("generateExceptionTailStub");
}

JitCode *
JitRuntime::generateBailoutTailStub(JSContext *cx)
{
    MOZ_ASSUME_UNREACHABLE("generateBailoutTailStub");
}
