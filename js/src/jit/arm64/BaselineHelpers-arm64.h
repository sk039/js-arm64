/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_arm64_BaselineHelpers_arm64_h
#define jit_arm64_BaselineHelpers_arm64_h

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
    // Move ICEntry offset into BaselineStubReg
    CodeOffsetLabel offset = masm.movWithPatch(ImmWord(-1), BaselineStubReg);
    *patchOffset = offset;

    // Load stub pointer into BaselineStubReg
    masm.loadPtr(Address(BaselineStubReg, ICEntry::offsetOfFirstStub()), BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // R2 won't be active when we call ICs, so we can use r0.
    JS_ASSERT(R2 == ValueOperand(r0));
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r0);

    // Call the stubcode via a direct branch-and-link.
    masm.Blr(x0);
}

inline void
EmitEnterTypeMonitorIC(MacroAssembler &masm,
                       size_t monitorStubOffset = ICMonitoredStub::offsetOfFirstMonitorStub())
{
    masm.breakpoint();
}

inline void
EmitReturnFromIC(MacroAssembler &masm)
{
    masm.Ret(); // Defaults to lr.
}

inline void
EmitChangeICReturnAddress(MacroAssembler &masm, Register reg)
{
    masm.movePtr(reg, lr);
}

inline void
EmitTailCallVM(JitCode *target, MacroAssembler &masm, uint32_t argSize)
{
    // We assume that R0 and R1 have been pushed, and R2 is unused.
    JS_ASSERT(R2 == ValueOperand(r0));

    // Compute frame size.
    masm.Sub(ScratchReg64, ARMRegister(BaselineFrameReg, 64), masm.GetStackPointer());
    masm.Add(ScratchReg32, ScratchReg32, Operand(BaselineFrame::FramePointerOffset));

    // Store frame size without VMFunction arguments for GC marking.
    masm.Sub(w0, ScratchReg32, Operand(argSize));
    masm.store32(r0, Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Push frame descriptor and perform the tail call.
    // BaselineTailCallReg (lr) already contains the return address (as we keep
    // it there through the stub calls), but the VMWrapper code being called
    // expects the return address to also be pushed on the stack.
    JS_ASSERT(BaselineTailCallReg == lr);
    masm.makeFrameDescriptor(ScratchReg, JitFrame_BaselineJS);
    masm.MacroAssemblerVIXL::Push(ScratchReg64, lr_64);
    masm.branch(target);
}

inline void
EmitCreateStubFrameDescriptor(MacroAssembler &masm, Register reg)
{
    masm.breakpoint();
}

inline void
EmitCallVM(JitCode *target, MacroAssembler &masm)
{
    masm.breakpoint();
}

// Size of vales pushed by EmitEnterStubFrame.
static const uint32_t STUB_FRAME_SIZE = 4 * sizeof(void *);
static const uint32_t STUB_FRAME_SAVED_STUB_OFFSET = sizeof(void *);

inline void
EmitEnterStubFrame(MacroAssembler &masm, Register scratch)
{
    masm.breakpoint();
}

inline void
EmitLeaveStubFrame(MacroAssembler &masm, bool calledIntoIon = false)
{
    masm.breakpoint();
}

inline void
EmitStowICValues(MacroAssembler &masm, int values)
{
    masm.breakpoint();
}

inline void
EmitUnstowICValues(MacroAssembler &masm, int values, bool discard = false)
{
    masm.breakpoint();
}

inline void
EmitCallTypeUpdateIC(MacroAssembler &masm, JitCode *code, uint32_t objectOffset)
{
    JS_ASSERT(R2 == ValueOperand(r0));
    masm.breakpoint();
}

template <typename AddrType>
inline void
EmitPreBarrier(MacroAssembler &masm, const AddrType &addr, MIRType type)
{
    masm.breakpoint();
}

inline void
EmitStubGuardFailure(MacroAssembler &masm)
{
    // NOTE: This routine assumes that the stub guard code left the stack in the
    // same state it was in when it was entered.

    // BaselineStubEntry points to the current stub.

    // Load next stub into BaselineStubReg.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfNext()), BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry into scratch register.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r0);

    // Return address is already loaded, just jump to the next stubcode.
    masm.Br(x0);
}

} // namespace jit
} // namespace js

#endif // jit_arm64_BaselineHelpers_arm64_h
