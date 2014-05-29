/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_arm64_BaselineHelpers_arm64_h
#define jit_arm64_BaselineHelpers_arm64_h

#ifdef JS_ION

#include "jit/BaselineFrame.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineRegisters.h"
#include "jit/IonMacroAssembler.h"

namespace js {
namespace jit {

// Distance from sp to the top Value inside an IC stub (no return address on the stack on ARM).
static const size_t ICStackValueOffset = 0;

inline void
EmitRestoreTailCallReg(MacroAssembler &masm)
{
    // No-op on ARM because link register is always holding the return address.
}

inline void
EmitRepushTailCallReg(MacroAssembler &masm)
{
    // No-op on ARM because link register is always holding the return address.
}

inline void
EmitCallIC(CodeOffsetLabel *patchOffset, MacroAssembler &masm)
{
    JS_ASSERT(0 && "EmitCallIC");
}

inline void
EmitEnterTypeMonitorIC(MacroAssembler &masm,
                       size_t monitorStubOffset = ICMonitoredStub::offsetOfFirstMonitorStub())
{
    JS_ASSERT(0 && "EmitEnterTypeMonitorIC");
}

inline void
EmitReturnFromIC(MacroAssembler &masm)
{
    JS_ASSERT(0 && "EmitReturnFromIC");
}

inline void
EmitChangeICReturnAddress(MacroAssembler &masm, Register reg)
{
    JS_ASSERT(0 && "EmitChangeICReturnAddress");
}

inline void
EmitTailCallVM(JitCode *target, MacroAssembler &masm, uint32_t argSize)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitCreateStubFrameDescriptor(MacroAssembler &masm, Register reg)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitCallVM(JitCode *target, MacroAssembler &masm)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

// Size of vales pushed by EmitEnterStubFrame.
static const uint32_t STUB_FRAME_SIZE = 4 * sizeof(void *);
static const uint32_t STUB_FRAME_SAVED_STUB_OFFSET = sizeof(void *);

inline void
EmitEnterStubFrame(MacroAssembler &masm, Register scratch)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitLeaveStubFrame(MacroAssembler &masm, bool calledIntoIon = false)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitStowICValues(MacroAssembler &masm, int values)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitUnstowICValues(MacroAssembler &masm, int values, bool discard = false)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitCallTypeUpdateIC(MacroAssembler &masm, JitCode *code, uint32_t objectOffset)
{
    JS_ASSERT(R2 == ValueOperand(r0));
    JS_ASSERT(0 && "EmitTailCallVM");
}

template <typename AddrType>
inline void
EmitPreBarrier(MacroAssembler &masm, const AddrType &addr, MIRType type)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}

inline void
EmitStubGuardFailure(MacroAssembler &masm)
{
    JS_ASSERT(0 && "EmitTailCallVM");
}


} // namespace jit
} // namespace js

#endif // JS_ION

#endif /* jit_arm64_BaselineHelpers_arm64_h */
